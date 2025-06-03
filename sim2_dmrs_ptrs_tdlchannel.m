%% Simple barebone 5G NR PDSCH End-to-End simulation without HARQ, Precode


%% Data
clear
clc
%rng(3)
x = imread('cameraman.tif');
xbin = int8(int2bit(x(:),8));
data_len = length(xbin);
xbin_hat = zeros(size(xbin),'like',xbin);

%% Parameter Settings
SNR = 10;
nlayers = 1;
modulation = '16QAM';
targetcoderate = 1/6;
rv = 0;
numblkerr = 0;
numbiterr = 0;

%% Parameter Configurations

% Set Global Sim Param
sim.PerfectChannelEstimator = false;

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
dec_dlsch = nrDLSCHDecoder; % decode
dec_dlsch.TargetCodeRate = targetcoderate;
dec_dlsch.TransportBlockLength = trBlkSizes;

% Set Channel Model Param
channel = nrTDLChannel;
channel.DelayProfile = 'TDL-C';
channel.DelaySpread = 300e-9;
channel.MaximumDopplerShift = 50;
channel.ChannelResponseOutput = 'ofdm-response';
channel.SampleRate = info_waveform.SampleRate;
channel.NumReceiveAntennas = NRx;

%% Processes

for m = 1:numchunks
    xbinchunk = xbin((m-1)*trBlkSizes+1:(m-1)*trBlkSizes+trBlkSizes);
    setTransportBlock(enc_dlsch,xbinchunk);

    % DLSCH Encoding
    codedTrBlocks = enc_dlsch(pdsch.Modulation,pdsch.NumLayers,...
        pdschIndicesInfo.G,rv);

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
    [xbinchunk_hat,blkerr] = dec_dlsch(rxSoftBits,pdsch.Modulation,pdsch.NumLayers,rv);

    numblkerr = numblkerr + double(blkerr);
    numbiterr = numbiterr + sum(xbinchunk~=xbinchunk_hat);

    xbin_hat((m-1)*trBlkSizes+1:(m-1)*trBlkSizes+trBlkSizes) = xbinchunk_hat;

end
bler =  numblkerr/numchunks; % Always equal to 1 for this case of simplied 5G pipeline
ber = numbiterr/data_len;

%% Reassemble image
xbin_hat = xbin_hat(1:data_len);
xhat = bit2int(xbin_hat,8);
xhat = uint8(reshape(xhat,size(x)));
figure
set(gca,'FontSize',14)
sgtitle('Barebone 5G NR With DM-RS, PT-RS Without HARQ')
subplot(121)
imshow(x), title('Transmitted'),set(gca,'FontSize',14)
subplot(122)
imshow(xhat), title('Received'),set(gca,'FontSize',14)
