function [filtRW, RW] = generateRandomWalk(iterations, boundaryWalkVals, smoothingFac)
    N = iterations;
    W = boundaryWalkVals;
    RW = zeros(N,1) ;
    RW(1) = W(randperm(2,1)) ;
    for i = 2:N
        RW(i) = RW(i-1)+W(randperm(2,1)) ;
    end
    smoothing = round(iterations/smoothingFac);
    coeff = ones(1, smoothing)/smoothing;
    filtRW = filter(coeff, 1, RW);
end