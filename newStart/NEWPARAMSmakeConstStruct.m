clear all;clc
h = 0.01; %timestep
t_final = 50;
iterations = t_final/h;

% ship parameters 
m = 25;          % mass (kg)
Jy = 2.135;         % pitch moment of inertia (kg m^3)
xg = 0;              % CG x-ccordinate (m)

% Aero parameters
rho = 1.2682; %air density 
alpha_0 = 0.4712; %rad
M = 50; %blending factor in aero coefficient calculation taken from beard&McLain appendix E

CL0 = 0.23; % Coefficient of lift at 0 pitch taken from beard&McLain appendix E
CL_alpha = -0.98;%5.61; %Derivative of CL wrt. alpha taken from beard&McLain appendix E
CL_q = 0;%7.95; %Derivative of CL wrt. q taken from beard&McLain appendix E
CL_deltaE = -0.36;%0.13; %Derivative of CL wrt. deltaE taken from beard&McLain appendix E

CD0 = 0.043; % Coefficient of drag
CD_alpha = 0.03; %Derivative of CD wrt. alpha taken from beard&McLain appendix E
CD_q = 0; %Derivative of CD wrt. q taken from beard&McLain appendix E
CD_deltaE = 0;%0.0135; %Derivative of CD wrt. deltaE taken from beard&McLain appendix E
CD_p = 0;

CM0 = 0.0135; % Aero moment coefficient
CM_alpha = -2.74; %0.38 %Derivative of CM wrt. alpha taken from beard&McLain appendix E
CM_q = -38.21; %Derivative of CM wrt. q taken from beard&McLain appendix E
CM_deltaE = -0.99; %Derivative of CM wrt. deltaE taken from beard&McLain appendix E

S = 0.55;  %Wing area taken from beard&McLain appendix E
c = 0.18994;  %moment arm for wing taken from beard&McLain appendix E
b = 2.8956;%wingspan taken from beard&McLain appendix E
e = 0.9;
g = 9.81;

S_prop = 0.2027;%taken from beard&McLain appendix E
C_prop = 1; %just tuning here?
k_motor = 80; %taken from beard&McLain appendix E

deltaE_trim = -0.1336;
deltaT_trim = 0.4734;
u_trim = 34.9505; 
w_trim = 1.8607;
theta_trim = 0.0532;
Va_trim = 35;
gamma_trim = 0;
alpha_trim = 0.0532;
q_trim = 0;


save ConstFile *
clear
