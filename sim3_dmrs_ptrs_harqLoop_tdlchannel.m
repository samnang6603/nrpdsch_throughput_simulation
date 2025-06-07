%% Simple barebone 5G NR PDSCH End-to-End simulation with HARQ
% Payload > DLSCH Enc > PDSCH Enc > OFDM Mod > TDL > AWGN > OFDM Dem > 
% > Chan Est > EQ > PDSCH Dec > DLSCH Dec > Received Payload -+
% ^                                                           |
% |_________________________ HARQ ____________________________|

%% Data
clear
clc
x = imread('cameraman.tif');
xbin = int8(int2bit(x(:),8));
data_len = length(xbin);
xbin_hat = zeros(size(xbin),'like',xbin);
rng(0);

%% Parameter Settings
SNR = 10;
nlayers = 1;
modulation = '16QAM';
targetcoderate = 1/5;
rv = [0 2 3 1];
numblkerr = 0;
numbiterr = 0;

%% Parameter Configurations

% Set Global Sim Param
sim.PerfectChannelEstimator = true;

% Set Carrier Param
carrier = nrCarrierConfig;
info_waveform = nrOFDMInfo(carrier);

% Set PDSCH Param
pdsch = nrPDSCHConfig;
pdsch.NumLayers = nlayers;
pdsch.Modulation = modulation;
pdsch.NID = carrier.NCellID;
[pdschIndices,pdschIndicesInfo] = nrPDSCHIndices(carrier,pdsch);
q = 0; % single codeword transmission
[pdsch_seq,pdsch_cinit] = nrPDSCHPRBS(pdsch.NID,pdsch.RNTI,q,pdschIndicesInfo.G); % PRBS seq for PDSCH Scrambler TS 5.2.1

% Set DM-RS Param
dmrs = pdsch.DMRS;
dmrssym = nrPDSCHDMRS(carrier,pdsch);
dmrsIndices = nrPDSCHDMRSIndices(carrier,pdsch);

% Set PT-RS Param
pdsch.EnablePTRS = 0;
ptrs = pdsch.PTRS;
ptrssym = nrPDSCHPTRS(carrier,pdsch);
ptrsIndices = nrPDSCHPTRSIndices(carrier,pdsch);

% Set PDSCH Resource Grid
NTx = 1; 
NRx = 1;
pdschGrid = nrResourceGrid(carrier,NTx,OutputDataType='double');

% Calculate Transport Block size
trBlkSizes = nrTBS(pdsch.Modulation,pdsch.NumLayers,numel(pdsch.PRBSet),pdschIndicesInfo.NREPerPRB,targetcoderate);
chunks = data_len/trBlkSizes;
numchunks = ceil(chunks); % or total number of transport block
numzeropad = trBlkSizes - rem(data_len,trBlkSizes);
xbin = [xbin; zeros(numzeropad,1,'int8')];

% Set DLSCH Param
info_dlsch = nrDLSCHInfo(trBlkSizes,targetcoderate);  % info
enc_dlsch = nrDLSCH;       % encode
enc_dlsch.TargetCodeRate = targetcoderate;
enc_dlsch.MultipleHARQProcesses = true;
dec_dlsch = nrDLSCHDecoder; % decode
dec_dlsch.TargetCodeRate = targetcoderate;
dec_dlsch.TransportBlockLength = trBlkSizes;
dec_dlsch.MultipleHARQProcesses = true;

% Set Channel Model Param
channel = nrTDLChannel;
channel.DelayProfile = 'TDL-C';
channel.DelaySpread = 300e-9;
channel.MaximumDopplerShift = 50;
channel.ChannelResponseOutput = 'ofdm-response';
channel.SampleRate = info_waveform.SampleRate;
channel.NumReceiveAntennas = NRx;

% HARQ Handler
NHARQProcesses = 16;
harqEntity = HARQEntity(0:NHARQProcesses-1,rv,pdsch.NumCodewords);
HARQ.maxSlots = 150;
HARQ.slotCounts = 0;
HARQ.maxRetries = 3;
HARQ.passedChunks = 0;
HARQ.discardedChunks = 0;
HARQ.currentIndex = 0;
HARQ.newChunkCount = 0;
for p = 1:NHARQProcesses
    % HARQ.PID(p).PID = p-1;
    % HARQ.PID(p).PIDIndex = p;
    HARQ.PID(p).failedDecodedIndex = 0;
    HARQ.PID(p).timeOut = 0;
    % default (passed) means it can accept new TB, 0 means it still has 
    % previously failed TB to send
    
    HARQ.PID(p).currentStatus = 1; % 1 means passed TX, 0 means failed TX
    HARQ.PID(p).previousStatus = 1; % 1 means passed TX, 0 means failed TX
    HARQ.PID(p).reTx = 0; % 0 means no ReTx, 1 means need to ReTx
    HARQ.PID(p).dutyComplete = false;
end


%% Processes
m = 0; % Chunk index
%while HARQ.passedChunks < numchunks && HARQ.slotCounts < HARQ.maxSlots
while HARQ.slotCounts <= HARQ.maxSlots

    if HARQ.passedChunks == numchunks 
        break
    end

    HARQ.slotCounts = HARQ.slotCounts + 1;
    HARQ.currentIndex = harqEntity.HARQProcessID + 1;
    
    % HARQ Buffer Management
    % Get new transport blocks and flush decoder soft buffer, as required
    for cwIdx = 1:pdsch.NumCodewords % only 1 cw but just to show the idea
        if harqEntity.NewData(cwIdx) && HARQ.newChunkCount < numchunks
            m = m + 1;
            % Allocate new TB
            fprintf('Buffering TB number %d >>> \n',m);
            xbinchunk = xbin((m-1)*trBlkSizes+1:(m-1)*trBlkSizes+trBlkSizes);
            setTransportBlock(enc_dlsch,xbinchunk,cwIdx-1,harqEntity.HARQProcessID);
            HARQ.newChunkCount = HARQ.newChunkCount + 1;

            % If the previous RV sequence ends without successful decoding,
            % flush the soft buffer explicitly
            if harqEntity.SequenceTimeout(cwIdx)
                resetSoftBuffer(dec_dlsch,cwIdx-1,harqEntity.HARQProcessID);
                HARQ.PID(HARQ.currentIndex).timeOut = 1;
            end
        end
    end

    % DLSCH Encoding
    codedTrBlocks = enc_dlsch(pdsch.Modulation,pdsch.NumLayers,...
        pdschIndicesInfo.G,harqEntity.RedundancyVersion,harqEntity.HARQProcessID);

    % PDSCH Encoding
    txsym = nrPDSCH(carrier,pdsch,codedTrBlocks); % Scramble > Modulate > LayerMap

    % Waveform Generation
    pdschGrid(pdschIndices) = txsym;
    if ~sim.PerfectChannelEstimator
        pdschGrid(dmrsIndices) = dmrssym;
    end
    txWaveform = nrOFDMModulate(carrier,pdschGrid);

    % Add channel
    [rxWaveform,ofdmResponse,timingOffset] = channel(txWaveform,carrier);

    % Alternatively if use "path-gain" as channel response output then to
    % get "ofdm-response", use
    % pathFilters = channel.getPathFilters; % channel1 is channel duplicate
    % [rxWaveform1,pathGains,sampleTimes] = channel(txWaveform);
    % ofdmResponse1 = nrPerfectChannelEstimate(carrier,pathGains,pathFilters);

    % Add noise
    %txsymn = awgn(txsym,SNR,'measured');
    %[rxWaveform, noisevar] = awgn(rxWaveform,SNR,'measured');
    SNR_lin = 10^(SNR/10);
    N0 = 1/sqrt(info_waveform.Nfft*SNR_lin);
    nPowerPerRE = N0^2*info_waveform.Nfft;
    noise = N0*randn(size(rxWaveform),'like',rxWaveform);
    rxWaveform = rxWaveform + noise;

    % Channel Estimation
    if sim.PerfectChannelEstimator
        offset = timingOffset;
    else
        offset = nrTimingEstimate(carrier,rxWaveform,pdschGrid);
    end
    rxWaveform = rxWaveform(1+offset:end);
    
    % Waveform Demodulate
    pdschGrid_hat = nrOFDMDemodulate(carrier,rxWaveform);
    [K,L] = size(pdschGrid_hat);
    if L < carrier.SymbolsPerSlot
        pdschGrid_hat = [pdschGrid_hat, zeros(K,1)]; %#ok<AGROW>
    end
    if sim.PerfectChannelEstimator
        % Use OFDM channel response for perfect channel estimator
        estChannelGridAnts = ofdmResponse;

        % Get noise estimation per REs (copied method directly from an
        % example that uses perfect channel estimation
        noiseEst = nPowerPerRE;
       
        % Get PDSCH resource elements from the received grid and
        % channel estimate
        [pdschRx,pdschHest,~,~] = nrExtractResources(pdschIndices,pdschGrid_hat,estChannelGridAnts);
    else
        % Use practical channel estimation
        [estChannelGridAnts,noiseEst] = nrChannelEstimate(pdschGrid_hat,dmrsIndices,dmrssym);
        
        % Get PDSCH resource elements from the received grid and
        % channel estimate
        [pdschRx,pdschHest,~,~] = nrExtractResources(pdschIndices,pdschGrid_hat,estChannelGridAnts);
    end

    % Equalization
    [pdschEq,csi] = nrEqualizeMMSE(pdschRx,pdschHest,noiseEst);
    rxsym = pdschEq;

    % PDSCH Decode
    %rxcodedTrBlocks = nrPDSCHDecode(rxsym,pdsch.Modulation,pdsch.NID,pdsch.RNTI);
    rxcodedTrBlocks = nrPDSCHDecode(carrier,pdsch,pdschEq,noiseEst);
    rxSoftBits = rxcodedTrBlocks{1};

    % Scale LLRs by CSI
    % Qm = length(rxSoftBits)/length(rxsym);
    % csi = repmat(csi.',Qm,1);
    % rxSoftBits = rxSoftBits.*csi(:);

    % DLSCH Decode
    [xbinchunk_hat,blkerr] = dec_dlsch(rxSoftBits,pdsch.Modulation,...
                             pdsch.NumLayers,harqEntity.RedundancyVersion,...
                             harqEntity.HARQProcessID);

    % Update current HARQ Status
    % if there is no blkerr and no reTx pending
    if ~blkerr
        % HARQ PID Tx passed
        HARQ.PID(HARQ.currentIndex).currentStatus = 1;
    else
        % HARQ PID Tx failed
        HARQ.PID(HARQ.currentIndex).currentStatus = 0;
        % HARQ PID pending reTx
        HARQ.PID(HARQ.currentIndex).reTx = 1;
    end

    % if error, create temp indices variable to be stored later in case
    % retransmission passed for the same HARQ PID
    if HARQ.PID(HARQ.currentIndex).reTx && ...
            ~HARQ.PID(HARQ.currentIndex).currentStatus %&& ...
           % ~HARQ.PID(HARQ.currentIndex).previousStatus
        
        mtemp = (m-1)*trBlkSizes+1:(m-1)*trBlkSizes+trBlkSizes;
        HARQ.PID(HARQ.currentIndex).failedDecodedIndex = mtemp;
    end

    % if no error and HARQ PID has no Tx failure and no pending reTX, 
    % store the current correctly decoded TB normally
    if ~blkerr && HARQ.PID(HARQ.currentIndex).currentStatus ...
            && ~HARQ.PID(HARQ.currentIndex).reTx

        xbin_hat((m-1)*trBlkSizes+1:(m-1)*trBlkSizes+trBlkSizes) = xbinchunk_hat;
        % Count number of failed and passed chunks
        HARQ.passedChunks = HARQ.passedChunks + double(~blkerr);
    else

        % If reTx pending, currentstatus is 1 and not timed out
        if HARQ.PID(HARQ.currentIndex).reTx...
            && HARQ.PID(HARQ.currentIndex).currentStatus...
            && ~HARQ.PID(HARQ.currentIndex).timeOut...

            % store the passed data into its respective failed TB indices
            xbin_hat(HARQ.PID(HARQ.currentIndex).failedDecodedIndex) = xbinchunk_hat;
            
            % Reset currentStatus and reTx and update passed chunks
            HARQ.PID(HARQ.currentIndex).currentStatus = 1;
            HARQ.PID(HARQ.currentIndex).reTx = 0;
            HARQ.passedChunks = HARQ.passedChunks + 1;
        elseif HARQ.PID(HARQ.currentIndex).timeOut
            % If retransmission failed and all RV has been exhausted
            % (timeout) then replace failed TB with salt and pepper noise
            discardedTB = randi([0 1],trBlkSizes,1,'like',xbin_hat);
            xbin_hat(HARQ.PID(HARQ.currentIndex).failedDecodedIndex) = discardedTB;
        
            % Reset current PID status to default "passed" so it can be
            % assigned new TB
            HARQ.PID(HARQ.currentIndex).currentStatus = 1;

            % Add one to discarded chunk counter
            HARQ.discardedChunks = HARQ.discardedChunks + 1;

            % Reset TimeOut and ReTx Status
            HARQ.PID(HARQ.currentIndex).timeOut = 0;
            HARQ.PID(HARQ.currentIndex).reTx = 0;
        end
    end

        % HARQ Process Update
    % Update current HARQ process with CRC error, then advance to next
    % process. This step updates info related to the active HARQ process in
    % the HARQ entity
    statusReport = updateAndAdvance(harqEntity,blkerr,trBlkSizes,pdschIndicesInfo.G);

    fprintf("Slot %d). %s\n",HARQ.slotCounts,statusReport)

    % % Calculate error metrics
    % numblkerr = numblkerr + double(blkerr);
    % numbiterr = numbiterr + sum(xbinchunk~=xbinchunk_hat);
end

% Check for expired PID (PID that failed Tx before running out of slot
% resources
for p = 1:NHARQProcesses
    if ~HARQ.PID(p).currentStatus
        % Irrecoverable TBs are marked as discarded per 3GPP TS 38.321. For
        % simulation purposes, synthetic fallback payloads 
        % (e.g., salt & pepper noise) are injected to preserve image 
        % structure.
        fprintf('HARQ Proc %d expired: TB is irrecoverable and replaced by fallback payload \n',p-1)
        discardedTB = randi([0 1],trBlkSizes,1,'like',xbin_hat);
        xbin_hat(HARQ.PID(p).failedDecodedIndex) = discardedTB;
    end
end

% bler =  numblkerr/numchunks; % Always equal to 1 for this case of simplied 5G pipeline
% ber = numbiterr/data_len;



%% Reassemble image
xbin_hat = xbin_hat(1:data_len);
xhat = bit2int(xbin_hat,8);
xhat = uint8(reshape(xhat,size(x)));
figure
set(gca,'FontSize',14)
sgtitle('Barebone 5G NR With DM-RS, PT-RS, HARQ')
subplot(121)
imshow(x), title('Transmitted'),set(gca,'FontSize',14)
subplot(122)
imshow(xhat), title('Received'),set(gca,'FontSize',14)

