function [vad, sig, sigp] = feedForward(xbar, W, V, sig, sigp, a, nmid)
VX = V*xbar;

for j=1:nmid-1
    ez = exp(-a(j)*VX(j));
    sig(j) = 1/( 1 + ez );
    sigp(j,j) = a(j)*ez*sig(j)*sig(j);
end
sig(nmid) = 1;

vad = W'*sig;

end

