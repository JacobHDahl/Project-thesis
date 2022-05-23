function nu_dot = NonLinFunc(eta,nu, ConstStruct)
h = 0.0001; %timestep
iterations = 100000;

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


% CONTROL
[deltaT, deltaE] = PIDControl(eta,nu,ConstStruct);

Va = sqrt(nu(1)*nu(1)+ nu(2)*nu(2));
theta = eta(3);
q = nu(3);
alpha = atan2(nu(2),nu(1));%angle of attack




R_body_to_inertial = [cos(theta), sin(theta);
    -sin(theta), cos(theta)]; %Rotation matrix from body to inertial

R_inertial_to_body = R_body_to_inertial';


R_stab_to_body = [cos(alpha),-sin(alpha);
    sin(alpha), cos(alpha)]; %rotation from stability to body

%Compute gravity
FG = R_inertial_to_body*[0; m*9.81];

Fx_G = FG(1);
Fz_G = FG(2);

%Compute propeller thrust
Fx_thrust=0.5*rho*S_prop*C_prop*((k_motor*deltaT)*(k_motor*deltaT)-Va*Va);
fancyAero = 1;
if fancyAero

    [CL_ofAlpha, CD_ofAlpha] = computeAeroCoeffs(alpha, alpha_0, M, S, b, e, CL0, CD_p);


    CL = CL_ofAlpha + CL_q * 0.5*c*q/Va + CL_deltaE*deltaE;
    CD = CD_ofAlpha + CD_q * 0.5*c*q/Va + CD_deltaE*deltaE;

    F_lift = 0.5*rho*Va*Va*S*CL;
    F_drag = 0.5*rho*Va*Va*S*CD;

    CM = CM0 + CM_alpha * alpha + CM_q*0.5*c*q/Va + CM_deltaE*deltaE;
    M_aero = 0.5*rho*Va*Va*S*c*CM;
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


nu_dot = CRB * nu + F;
end

