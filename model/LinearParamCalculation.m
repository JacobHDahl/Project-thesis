function [Xu, Xw, Xq, XdeltaE, XdeltaT, Zu, Zw, Zq, ZdeltaE, Mu, Mw, Mq, MdeltaE] = LinearParamCalculation(ConstStruct)
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
S_prop = ConstStruct.S_prop;
C_prop = ConstStruct.C_prop;
k_motor = ConstStruct.k_motor;

deltaE_trim = ConstStruct.deltaE_trim;
deltaT_trim = ConstStruct.deltaT_trim;
u_trim = ConstStruct.u_trim;
w_trim = ConstStruct.w_trim;
Va_trim = ConstStruct.Va_trim;
alpha_trim = ConstStruct.alpha_trim;
q_trim = 0;

CX0 = -CD0*cos(alpha_trim) + CL0*sin(alpha_trim);
CX_alpha = -CD_alpha*cos(alpha_trim) + CL_alpha*sin(alpha_trim); %Check this to see if multiplication with alpha_trim is correct
CX_deltaE = -CD_deltaE*cos(alpha_trim) + CL_deltaE*sin(alpha_trim);
CX_q = -CD_q*cos(alpha_trim) + CL_q*sin(alpha_trim);


Xu = (1/m)*u_trim*rho*S*(CX0 + CX_alpha*alpha_trim + CX_deltaE*deltaE_trim)...
    - (1/m)*0.5*rho*S*w_trim*CX_alpha + (1/(4*m*Va_trim))*rho*S*c*CX_q*u_trim*q_trim ...
    - (1/m)*rho*S_prop*C_prop*u_trim;

Xw = -q_trim + (1/m)*w_trim*rho*S*(CX0 + CX_alpha*alpha_trim + CX_deltaE*deltaE_trim)...
    + (1/(4*m*Va_trim))*rho*S*c*CX_q*w_trim*q_trim + (1/m)*0.5*rho*S*CX_alpha*u_trim ...
    - (1/m)*rho*S_prop*C_prop*w_trim;

Xq = -w_trim + (1/(4*m))*rho*Va_trim*S*CX_q*c;

XdeltaE = (1/m)*0.5*rho*Va_trim*Va_trim*S*CX_deltaE;

XdeltaT = (1/m)*rho*S_prop*C_prop*k_motor*k_motor*deltaT_trim;

CZ0 = -CD0*sin(alpha_trim) - CL0*cos(alpha_trim);
CZ_alpha = -CD_alpha*sin(alpha_trim) - CL_alpha*cos(alpha_trim);
CZ_deltaE = -CD_deltaE*sin(alpha_trim)- CL_deltaE*cos(alpha_trim);
CZ_q = -CD_q*sin(alpha_trim) - CL_q*cos(alpha_trim);

Zu = q_trim + (1/m)*u_trim*rho*S*(CZ0 + CZ_alpha*alpha_trim + CZ_deltaE*deltaE_trim)...
    - (1/m)*0.5*rho*S*CZ_alpha*w_trim + (1/(4*m*Va_trim))*u_trim*rho*S*CZ_q*c*q_trim;

Zw = (1/m)*w_trim*rho*S*(CZ0 + CZ_alpha*alpha_trim + CZ_deltaE*deltaE_trim)...
    + (1/m)*0.5*rho*S*CZ_alpha*u_trim + (1/(4*m*Va_trim))*rho*w_trim*S*c*CZ_q*q_trim;

Zq = u_trim + (1/(4*m))*rho*Va_trim*S*CZ_q*c;

ZdeltaE = (1/m)*0.5*rho*Va_trim*Va_trim*S*CZ_deltaE;

Mu = (1/Jy)*u_trim*rho*S*c*(CM0 + CM_alpha*alpha_trim + CM_deltaE*deltaE_trim)...
    - (1/Jy)*0.5*rho*S*c*CM_alpha*w_trim + (1/(4*Jy*Va_trim))*rho*S*c*c*CM_q*u_trim*q_trim;

Mw = (1/Jy)*w_trim*rho*S*c*(CM0 + CM_alpha*alpha_trim + CM_deltaE*deltaE_trim)...
    + (1/Jy)*0.5*rho*S*c*CM_alpha*u_trim + (1/(4*Jy*Va_trim))*rho*S*c*c*CM_q*w_trim*q_trim;

Mq = (1/(4*Jy))*rho*Va_trim*Va_trim*S*c*c*CM_q;

MdeltaE = (1/Jy)*0.5*rho*Va_trim*Va_trim*S*c*CM_deltaE;
end

