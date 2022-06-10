function [deltaT,deltaE,u,w,theta] = computeU(alpha,Va,gamma, ConstStruct)
m = ConstStruct.m;
% Aero parameters
rho = ConstStruct.rho;
CL0 = ConstStruct.CL0;
CL_alpha = ConstStruct.CL_alpha;
CL_q = ConstStruct.CL_q;
CL_deltaE = ConstStruct.CL_deltaE;
CD0 = ConstStruct.CD0;
CD_alpha = ConstStruct.CD_alpha;
CD_q = ConstStruct.CD_q;
CD_deltaE = ConstStruct.CD_deltaE;
CM0 = ConstStruct.CM0;
CM_alpha = ConstStruct.CM_alpha;
CM_q = ConstStruct.CM_q;
CM_deltaE = ConstStruct.CM_deltaE;
S = ConstStruct.S;
c = ConstStruct.c;
S_prop = ConstStruct.S_prop;
C_prop = ConstStruct.C_prop;
k_motor = ConstStruct.k_motor;

% paramsEnd
theta = alpha + gamma;
u = Va*cos(alpha);
w = Va * sin(alpha);
q = 0;

deltaE = (-CM0 -CM_alpha*alpha - 0.5*CM_q*c*q/Va)/CM_deltaE;

%deltaT calculation; eq F.2 in Beard&McLain
CD_ofAlpha = CD0 + CD_alpha*alpha;
CL_ofAlpha = CL0 + CL_alpha*alpha;
CX_ofAlpha = -CD_ofAlpha*cos(alpha) + CL_ofAlpha*sin(alpha);
CX_q_ofAlpha = -CD_q*cos(alpha) + CL_q*sin(alpha);
CX_deltaE_ofAlpha = -CD_deltaE*cos(alpha) + CL_deltaE*sin(alpha);


nominator = 2*m*(q*w + 9.81*sin(theta))- rho*(Va^2)*S*(CX_ofAlpha + 0.5*CX_q_ofAlpha*c*q/Va + CX_deltaE_ofAlpha *deltaE);
denominator = rho*S_prop*C_prop*k_motor^2;

deltaT = sqrt((nominator/denominator) + (Va^2/k_motor^2));
end

