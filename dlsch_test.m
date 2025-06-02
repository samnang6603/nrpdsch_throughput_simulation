clear
trBlkLen = 16136;
rng(89)
trBlk = randi([0 1],trBlkLen,1,'int8');

targetCodeRate = 1/2; %567/1024;
encDL = nrDLSCH;
encDL.TargetCodeRate = targetCodeRate;

setTransportBlock(encDL,trBlk);

mod = '16QAM';
nLayers = 1;
outlen = 32448;
rv = 0;
codedTrBlock = encDL(mod,nLayers,outlen,rv);

decDL = nrDLSCHDecoder;
decDL.TargetCodeRate = targetCodeRate;
decDL.TransportBlockLength = trBlkLen;

rxSoftBits = 1.0 - 2.0*double(codedTrBlock);
[decbits,blkerr] = decDL(rxSoftBits,mod,nLayers,rv);

isequal(decbits,trBlk)
