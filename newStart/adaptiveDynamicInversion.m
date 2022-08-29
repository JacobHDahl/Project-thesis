function [deltaE, Wdot,Vdot,rdot] = adaptiveDynamicInversion(V,W,NNStruct,ConstStruct, externalInput, ref,q, oldu,vs, a,i)
% reference model
K_diff = NNStruct.K_diff;
nmid = NNStruct.nmid;
nin = NNStruct.nin;
gammaw = NNStruct.gammaw;
gammav = NNStruct.gammav;
lambda = NNStruct.lambda;
rdot = -K_diff*(externalInput-ref);
% error signals
error = ref - q;

% NN inputs

xbar(1) = 1;
xbar(2) = q;
xbar(3) = oldu;
xbar = xbar';

sig = zeros(nmid,1);
sigp = zeros(nmid,nmid);
% get weights from state vector
for j=1:nmid
    V(j,:) = vs((j-1)*nin+1:j*nin,i);
end

% adaptive controller
VX = V*xbar;
for j=1:nmid-1
    ez = exp(-a(j)*VX(j));
    sig(j) = 1/( 1 + ez );
    sigp(j,j) = a(j)*ez*sig(j)*sig(j);
end
sig(nmid) = 1;
vad = W'*sig;

[~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, Mq, MdeltaE] = LinearParamCalculation(ConstStruct);
% learning law
Wdot = -gammaw*(error*( sig' - xbar'*V'*sigp) + lambda*norm(error)*W' );
Vdot = -gammav*(sigp*W*error*xbar' + lambda*norm(error)*V);
% put NN update in a vector
wdot = Wdot;
for j=1:nmid
    vdot((j-1)*nin+1:j*nin) = Vdot(j,:);
end

deltaE = (rdot(1) - K_diff*error - vad - Mq*q)/MdeltaE;
end

