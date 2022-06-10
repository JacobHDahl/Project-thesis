function x_dot = f(x,u)
%This function is the nonlinear model of the airplane, but defined by Va,
%alpha and gamma.
%Params
%%
ConstStruct = load("ConstFile.mat");

h = 0.0001; %timestep
iterations = 1000000;

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


nominator = 2*m*(q*w + 9.81*sin(-theta))- rho*(Va^2)*S*(CX_ofAlpha + 0.5*CX_q_ofAlpha*c*q/Va + CX_deltaE_ofAlpha *deltaE);
denominator = rho*S_prop*C_prop*k_motor^2;

deltaT = sqrt((nominator/denominator) + (Va^2/k_motor^2));

R_stab_to_body = [cos(alpha),-sin(alpha);
    sin(alpha), cos(alpha)]; %rotation from stability to body

%Compute gravity

Fx_G = m*g*sin(-theta);
Fz_G = m*g*cos(-theta);

%Compute propeller thrust
Fx_thrust=0.5*rho*S_prop*C_prop*((k_motor*deltaT)*(k_motor*deltaT)-Va*Va);
if Fx_thrust < 0
    Fx_thrust = 0;
end
fancyAero = 1;
if fancyAero

    [CL_ofAlpha, CD_ofAlpha] = computeAeroCoeffs(alpha, alpha_0, M, S, b, e, CL0, CD_p);


    CL = CL_ofAlpha + CL_q * 0.5*c*q/Va + CL_deltaE*deltaE;
    CD = CD_ofAlpha + CD_q * 0.5*c*q/Va + CD_deltaE*deltaE;

    F_lift = 0.5*rho*Va*Va*S*CL;
    F_drag = 0.5*rho*Va*Va*S*CD;

    CM = CM0 + CM_alpha * alpha + CM_deltaE*deltaE + CM_q*0.5*c*q/Va;
    M_aero = 0.5*rho*Va*Va*S*c*CM;
    %M_aero = 30*sin(called/10);%0*0.5*rho*Va*Va*S*c*CM;

    %     if mod(called,100)==0
    %         disp("q")
    %         disp(q)
    %         disp("alph")
    %         disp(alpha)
    %     end

else
    %Linear model
    %Compute areodynamic forces in stability frame
    CL = CL0 + CL_alpha*alpha + CL_q * 0.5*c*q/Va + CL_deltaE*deltaE;
    CD = CD0 + CD_alpha*alpha + CD_q * 0.5*c*q/Va + CD_deltaE*deltaE;
    CM = CM0 + CM_alpha*alpha + CM_q * 0.5*c*q/Va + CM_deltaE*deltaE;

    F_lift = 0.5 * rho * Va*Va * S * CL;
    F_drag = 0.5 * rho * Va*Va * S * CD;
    M_aero = (0.5 * rho * Va*Va * S * c * CM);
end

F_aero = R_stab_to_body * [-F_drag;-F_lift]; %Transform to body frame

Fx_aero = F_aero(1);
Fz_aero = F_aero(2);

fx = Fx_G - Fx_aero + Fx_thrust;
fz = -Fz_G - Fz_aero;


CRB = nu(3)*[0, -1, 0;
    1, 0, 0;
    0, 0, 0];

F = [fx/m; fz/m; M_aero/Jy];

x_dot = CRB * nu + F;
end

