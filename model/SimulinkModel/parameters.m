%Simulation parameters
sample_time = 0.01;
nu_init = [35,0,0];
eta_init = [0,0,deg2rad(15)];

% ship parameters 
m = 13.5;          % mass (kg)
Jy = 1.135;         % pitch moment of inertia (kg m^3)
xg = 0;              % CG x-ccordinate (m)


%TODO: Add all params to a struct which is easier to pass to functions
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
delta_t = 0.3901; %thrust variable
k_motor = 80; %taken from beard&McLain appendix E