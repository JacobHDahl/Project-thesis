function x_dot = f(x,u)
%This function is the nonlinear model of the airplane, but defined by Va,
%alpha and gamma. 
%Params
%%
m = 25;          % mass (kg)
Jy = 2.135;         % pitch moment of inertia (kg m^3)
xg = 0;              % CG x-ccordinate (m)

% Aero parameters
rho = 1.2682; %air density
alpha_0 = 0.4712; %rad
M = 50; %blending factor in aero coefficient calculation taken from beard&McLain appendix E

CL0 = 0.28; % Coefficient of lift at 0 pitch taken from beard&McLain appendix E
CL_alpha = 3.45; %Derivative of CL wrt. alpha taken from beard&McLain appendix E
CL_q = 0; %Derivative of CL wrt. q taken from beard&McLain appendix E
CL_deltaE = -0.36; %Derivative of CL wrt. deltaE taken from beard&McLain appendix E

CD0 = 0.03; % Coefficient of drag
CD_alpha = 0.3; %Derivative of CD wrt. alpha taken from beard&McLain appendix E
CD_q = 0; %Derivative of CD wrt. q taken from beard&McLain appendix E
CD_deltaE = 0; %Derivative of CD wrt. deltaE taken from beard&McLain appendix E
CD_p = 0.0437;

CM0 = -0.02338; % Aero moment coefficient
CM_alpha = -0.38; %Derivative of CM wrt. alpha taken from beard&McLain appendix E
CM_q = -3.6; %Derivative of CM wrt. q taken from beard&McLain appendix E
CM_deltaE = -0.5; %Derivative of CM wrt. deltaE taken from beard&McLain appendix E

S = 0.55;  %Wing area taken from beard&McLain appendix E
c = 0.18994;  %moment arm for wing taken from beard&McLain appendix E
b = 2.8956;%wingspan taken from beard&McLain appendix E
e = 0.9;

S_prop = 0.2027;%taken from beard&McLain appendix E
C_prop = 1; %just tuning here?

%delta_t = 0.5; %thrust variable
k_motor = 80; %taken from beard&McLain appendix E

%%
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
    M_aero = 0.5*rho*Va*Va*S*c*CM/Jy;
else
    %Linear model
    %Compute areodynamic forces in stability frame
    CL = CL0 + CL_alpha*alpha + CL_q * 0.5*c*q/Va + CL_deltaE*deltaE;
    CD = CD0 + CD_alpha*alpha + CD_q * 0.5*c*q/Va + CD_deltaE*deltaE;
    CM = CM0 + CM_alpha*alpha + CM_q * 0.5*c*q/Va + CM_deltaE*deltaE;

    F_lift = 0.5 * rho * Va*Va * S * CL;
    F_drag = 0.5 * rho * Va*Va * S * CD;
    M_aero = (0.5 * rho * Va*Va * S * c * CM)/Jy;
end

F_aero = R_stab_to_body * [-F_drag;-F_lift]; %Transform to body frame

Fx_aero = F_aero(1);
Fz_aero = F_aero(2);

fx = Fx_G - Fx_aero + Fx_thrust;
fz = -Fz_G - Fz_aero;

CRB = nu(3)*[0, -1, 0;
    1, 0, 0;
    0, 0, 0];


F = [fx/m; fz/m; M_aero];


nu_dot = CRB * nu + F;

x_dot = nu_dot;
end

