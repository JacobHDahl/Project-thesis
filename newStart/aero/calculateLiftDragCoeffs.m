function [CL,CD] = calculateLiftDragCoeffs(alpha,ConstStruct)
M = ConstStruct.M;
alpha0 = ConstStruct.alpha_0;
b = ConstStruct.b;
S = ConstStruct.S;

CL0 = ConstStruct.CL0;
AR = b^2 /S;
CL_alpha = pi*AR/(1 + sqrt(1 + (AR/2)^2));

CD_p = ConstStruct.CD_p;

CL_flatplate = 2*sign(alpha)*sin(alpha)*sin(alpha)*cos(alpha);
sigma = sigmaFunc(alpha,M,alpha0);
CL = (1-sigma)*(CL0+CL_alpha*alpha) + sigma*CL_flatplate;


CD = CD_p + ((CL0 + CL_alpha*alpha)^2 / (pi*exp(1)*AR));

end

