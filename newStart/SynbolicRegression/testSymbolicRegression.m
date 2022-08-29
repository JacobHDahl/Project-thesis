WVs = load("../allWVs.mat");
W = WVs(1).allWVs{1};
V = WVs(1).allWVs{2};
minMaxOldus = load("../minMaxOldus.mat");
minOldus = minMaxOldus(1).minMaxOldus{1};
maxOldus = minMaxOldus(1).minMaxOldus{2};
minMaxQs = load("../minMaxQs.mat");
minQs = minMaxQs(1).minMaxs{1};
maxQs = minMaxQs(1).minMaxs{2};

nmid = 10;
amin = 0.01;
amax = 10;

sig = zeros(nmid,1);
sigp = zeros(nmid,nmid);

% precompute activation potentials
a = zeros(nmid,1);
for i=1:nmid-1
    a(i) = tan(atan(amin) + (atan(amax) - atan(amin))*(i+1)/nmid);
end

numDataPoints = 1000;
qs = linspace(minQs, maxQs, numDataPoints);
oldus = linspace(minOldus, maxOldus, numDataPoints);
ons = ones(1,numDataPoints);

inputData = [ons; qs; oldus];
vads = zeros(1,numDataPoints);

for i = 1:length(inputData)
    xbar = inputData(:,i);
    [vad, sig, sigp] = feedForward(xbar, W, V, sig, sigp, a, nmid);
    vads(i) = vad;

end

plot(vads)
