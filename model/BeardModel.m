% eta = [pn, pd, theta]'; % pos north(x), pos down(z), pitch angle in inertial frame
% eta = R * nu; %R being the rotation matrix from body to inertial
% nu = [u, w, q]'; % velocity in x, velocity in z and pitchrate in bodyframe

% [udot, wdot] = [-q*w; q*u] + 1/m * [fx; fz];
% qdot = m/Jy + M_aero; 
% 
% fx = Fx_G + Fx_aero + Fx_thrust;
% fz = Fz_G + Fz_aero
%Fx_g = sin(theta)*m*g; 
%Fz_g = cos(theta)*m*g;

%Fx_thrust=0.5*rho*S_prop*C_prop*((k_motor*delta_t)*(k_motor*delta_t)-V_a*V_a);

%M_aero = 0.5 * rho * V_a*V_a * S * c * C_M
% F_lift = 0.5 * rho * V_a*V_a * S * C_L;
% F_drag = 0.5 * rho * V_a*V_a * S * C_D;
% 
% F_aero = R_stab_to_body * [F_drag;F_lift];
% Fx_aero = F_aero(1);
% Fz_aero = F_aero(2);


h = 0.01; %timestep
iterations = 2000;

% ship parameters 
m = 13.5;          % mass (kg)
Jy = 1.135;         % pitch moment of inertia (kg m^3)
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
delta_t = 0.5; %thrust variable
k_motor = 80; %taken from beard&McLain appendix E

nus = zeros(3,iterations);
etas = zeros(3,iterations);

us = zeros(3,iterations);

%us(1,:) = 1000*ones(1,iterations);
%us(2,:) = 100*ones(1,iterations);
%us(3,:) = deg2rad(10)*ones(1,iterations);

eta_init = [0, 0, deg2rad(10)]'; % x, z, theta in inertial frame
nu_init = [100, 0, 0]'; % v(velocity_x), w(velocity_z), omega(rot_velocity) in body frame

nus(:,1) = nu_init;
etas(:,1) = eta_init;

anim = animation;
t = zeros(1,iterations);

for i = 1:iterations-1
    t(i) = (i-1)*h;
    nu = nus(:,i);
    eta = etas(:,i);
    u = us(:,i);

    %Input
    deltaE = 0;
    
    %Extract values
    theta = eta(3);
    q = nu(3);
    Va = sqrt(nu(1)*nu(1)+ nu(2)*nu(2));
    alpha = atan2(nu(2),nu(1));%angle of attack

    R_stab_to_body = [cos(-alpha),-sin(-alpha);
    sin(-alpha), cos(-alpha)]; %rotation from stability to body

    %Compute gravity
    Fx_G = sin(theta)*m*9.81;
    Fz_G = cos(theta)*m*9.81;
    
    %Compute propeller thrust
    Fx_thrust=0.5*rho*S_prop*C_prop*((k_motor*delta_t)*(k_motor*delta_t)-Va*Va);
    fancyAero = 1;
    if fancyAero

        [CL_ofAlpha, CD_ofAlpha] = computeAeroCoeffs(alpha, alpha_0, M, S, b, e, CL0, CD_p);
        

        CL = CL_ofAlpha + CL_q * 0.5*c*Va*q + CL_deltaE*deltaE;
        CD = CD_ofAlpha + CD_q * 0.5*c*Va*q + CL_deltaE*deltaE;
        
        F_lift = 0.5*rho*Va*Va*S*CL;
        F_drag = 0.5*rho*Va*Va*S*CD;

        CM = CM0 + CM_alpha * alpha;
        M_aero = 0.5*rho*Va*Va*S*c*CM/Jy;

    else
        %Compute areodynamic forces in stability frame
        M_aero = 0;%(0.5 * rho * Va*Va * S * c * C_M)/Jy;
        F_lift = 0.5 * rho * Va*Va * S * C_L;
        F_drag = 0.5 * rho * Va*Va * S * C_D;
    end
    
    

    


    F_aero = R_stab_to_body * [-F_drag;-F_lift]; %Transform to body frame

    Fx_aero = F_aero(1);
    Fz_aero = F_aero(2);

    fx = Fx_G + Fx_aero - Fx_thrust;
    fz = -Fz_G - Fz_aero;
    
    CRB = nu(3)*[0, -1, 0;
                1, 0, 0;
                0, 0, 0];


    F = [fx/m; fz/m; M_aero];
    R = Rzyx(0,0,theta); %Rotation matrix from body to inertial

    nu_dot = CRB * nu + F;

    eta_dot = R * nu;

    nu = nu + h * nu_dot;
    eta = eta + h * eta_dot;

    
    anim=anim.update(eta);
    nus(:,i+1) = nu;
    etas(:,i+1) = eta;

    pause(h); %This relates to the perceived time spent in the simulation. Relate to time t. TODO

end


