function x_dot = f(x,u)
%This function is the nonlinear model of the airplane, but defined by Va,
%alpha and gamma.
%Params

ConstStruct = load("ConstFile.mat");

% ship parameters
m = ConstStruct.m;
Jy = ConstStruct.Jy;
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
g = 9.81;
S_prop = ConstStruct.S_prop;
C_prop = ConstStruct.C_prop;
k_motor = ConstStruct.k_motor;
% paramsEnd

alpha = x;

Va = u(1);
gamma_des = u(2);
theta = alpha + gamma_des;

u = Va * cos(alpha);
w = Va * sin(alpha);
q = 0;

nu = [u;w;q];

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

FTHRUST_X = 0.5*S_prop*C_prop*((k_motor*deltaT)^2-Va^2);

FG_X = -m*g*sin(theta);
FG_Z = m*g*cos(theta);

linear = 1; %toggle to select linear or non-linear aero-model
[FAERO_X,FAERO_Z, M_aero] = calculateAeroForces(nu,ConstStruct,deltaE,linear);

fx = FG_X + FTHRUST_X + FAERO_X ;
fz = FG_Z + FAERO_Z;

u_dot = -q*w  + (1/m)*fx;
w_dot = q*u + (1/m)*fz;
q_dot = M_aero/Jy;

x_dot = [u_dot, w_dot, q_dot]';
end

