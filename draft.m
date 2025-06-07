%% HARQ FSM Handler Script — All-in-One

% Configuration
Q = 100;                        % Total slots allowed
P = 16;                         % Number of HARQ PIDs
rvSeq = [0 2 3 1];              % RV cycle
maxRV = length(rvSeq);         
trBlkSizes = 8448 * ones(P,1); % Example size per TB
numBitsTotal = sum(trBlkSizes);

% Initialize HARQ struct
HARQ.PID = repmat(struct( ...
    'RVIndex', 1, ...
    'Status', 'IDLE', ...       % 'IDLE', 'TX', 'ACK', 'NACK', 'TIMEOUT'
    'RetryCount', 0, ...
    'FailedDecodedIndex', [], ...
    'CurrentData', [], ...
    'FinalBits', []), ...
    P, 1);

xbin_hat = zeros(numBitsTotal,1);    % Final output buffer (binary)
nextFreeBit = 1;                     % Bit index tracker

% Begin slot-by-slot simulation
for slot = 1:Q

    for pid = 1:P
        harq = HARQ.PID(pid);

        % FSM Control
        switch harq.Status

            case 'IDLE'
                % Assign new TB to this HARQ PID
                harq.CurrentData = randi([0 1], trBlkSizes(pid), 1);
                harq.Status = 'TX';

            case 'TX'
                % Simulate decoding
                rv = rvSeq(harq.RVIndex);
                SNRdB = 5; % Simulate low SNR
                passed = simulateDecoding(harq.CurrentData, rv, SNRdB);

                if passed
                    harq.Status = 'ACK';
                    % Store bits
                    harq.FinalBits = harq.CurrentData;
                else
                    harq.RetryCount = harq.RetryCount + 1;
                    harq.RVIndex = mod(harq.RVIndex, maxRV) + 1;

                    if harq.RetryCount >= maxRV
                        harq.Status = 'TIMEOUT';
                        % Salt & pepper fallback
                        harq.FinalBits = randi([0 1], trBlkSizes(pid), 1);
                    else
                        harq.Status = 'TX'; % Retry
                    end
                end

            case {'ACK', 'TIMEOUT'}
                % Already resolved. Do nothing.
        end

        % Update back
        HARQ.PID(pid) = harq;
    end
end

% Reconstruct final xbin_hat
for pid = 1:P
    tb = HARQ.PID(pid).FinalBits;
    idx = nextFreeBit:nextFreeBit+trBlkSizes(pid)-1;
    xbin_hat(idx) = tb;
    nextFreeBit = nextFreeBit + trBlkSizes(pid);
end

disp("HARQ processing complete. Summary:")
fprintf("ACKs: %d\n", sum(arrayfun(@(p) strcmp(p.Status, 'ACK'), HARQ.PID)));
fprintf("Timeouts: %d\n", sum(arrayfun(@(p) strcmp(p.Status, 'TIMEOUT'), HARQ.PID)));

%% Helper Function (can be inside same script or separate)
function passed = simulateDecoding(data, rv, SNRdB)
    % Simulated decoding with SNR effect
    % For demo: low RV, low SNR = low chance of success
    baseP = 0.9 - (rv * 0.2);  % Simulate better chance at lower RV
    snrFactor = SNRdB / 10;    % Increase chance with SNR
    probPass = min(1, baseP + 0.1 * snrFactor);
    passed = rand() < probPass;
end