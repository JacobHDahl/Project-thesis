function x_dot_lin = LinearizedModel(x_lin, u_lin, ConstStruct)
%LINEARIZEDMODEL is a linearized model of the NonLinear model
% linearized around trim
% ship parameters 
m = ConstStruct.m;
Jy = ConstStruct.Jy;
xg = ConstStruct.xg;
% Aero parameters
rho = ConstStruct.rho;
alpha_0 = ConstStruct.alpha_0;
M = ConstStruct.M;
CL0 = ConstStruct.CL0;
CL_alpha = ConstStruct.CL_alpha;
CL_q = ConstStruct.CL_q;
CL_deltaE = ConstStruct.CL_deltaE;
CD0 = ConstStruct.CD0;
CD_alpha = ConstStruct.CD_alpha;
CD_q = ConstStruct.CD_q;
CD_deltaE = ConstStruct.CD_deltaE;
CD_p = ConstStruct.CD_p;
CM0 = ConstStruct.CM0;
CM_alpha = ConstStruct.CM_alpha;
CM_q = ConstStruct.CM_q;
CM_deltaE = ConstStruct.CM_deltaE;
S = ConstStruct.S;
c = ConstStruct.c;
b = ConstStruct.b;
e = ConstStruct.e;
g = 9.81;
S_prop = ConstStruct.S_prop;
C_prop = ConstStruct.C_prop;
k_motor = ConstStruct.k_motor;

deltaE_trim = ConstStruct.deltaE_trim;
deltaT_trim = ConstStruct.deltaT_trim;
u_trim = ConstStruct.u_trim;
w_trim = ConstStruct.w_trim;
theta_trim = ConstStruct.theta_trim;
Va_trim = ConstStruct.Va_trim;
gamma_trim = ConstStruct.gamma_trim;
alpha_trim = ConstStruct.alpha_trim;
q_trim = 0;


[Xu, Xw, Xq, XdeltaE, XdeltaT, Zu, Zw, Zq, ZdeltaE, Mu, Mw, Mq, MdeltaE] = LinearParamCalculation(ConstStruct);

A_lin = [Xu, Xw, Xq, -g*cos(theta_trim), 0;
        Zu, Zw, Zq, -g*sin(theta_trim), 0;
        Mu, Mw, Mq,      0,             0;
        0,  0,  1,       0              0;
        sin(theta_trim), -cos(theta_trim), 0, u_trim*cos(theta_trim) + w_trim*sin(theta_trim), 0];
B_lin = [XdeltaE, XdeltaT;
        ZdeltaE,    0    ;
        MdeltaE,    0    ;
           0   ,    0    ;
           0   ,    0    ];

x_dot_lin = A_lin*x_lin + B_lin*u_lin;
end

