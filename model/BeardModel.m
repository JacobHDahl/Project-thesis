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

%%%%POSITIVE ROTATION IS ANTI-CLOCKWISE


h = 0.0001; %timestep
iterations = 100000;

% ship parameters 
m = 25;          % mass (kg)
Jy = 2.135;         % pitch moment of inertia (kg m^3)
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
CM_alpha = -0.38; %0.38 %Derivative of CM wrt. alpha taken from beard&McLain appendix E
CM_q = -3.6; %Derivative of CM wrt. q taken from beard&McLain appendix E
CM_deltaE = -0.5; %Derivative of CM wrt. deltaE taken from beard&McLain appendix E

S = 0.55;  %Wing area taken from beard&McLain appendix E
c = 0.18994;  %moment arm for wing taken from beard&McLain appendix E
b = 2.8956;%wingspan taken from beard&McLain appendix E
e = 0.9;

g = 9.81;

S_prop = 0.2027;%taken from beard&McLain appendix E
C_prop = 1; %just tuning here?
deltaT_init = 0.65; %thrust variable
%deltaE = -0.0498;
k_motor = 80; %taken from beard&McLain appendix E

deltaE_trim = -0.1037;%-0.1190;
deltaT_trim = 0.4564;%0.3231;
u_trim = 34.9020;%23.8915;
w_trim = 2.6176;%2.2791;
theta_trim = 0.0749;%0.0951;
Va_trim = 35;
gamma_trim = 0;
alpha_trim = 0.0749;%0.0951;

nus = zeros(3,iterations);
etas = zeros(3,iterations);

eta_init = [0, 0, theta_trim]'; % x, z, theta in inertial frame
nu_init = [u_trim, w_trim, 0]'; % v(velocity_x), w(velocity_z), omega(rot_velocity) in body frame

nus(:,1) = nu_init;
etas(:,1) = eta_init;

anim = animation;
t = zeros(1,iterations);


for i = 1:iterations-1
    t(i) = (i-1)*h;
    nu = nus(:,i);
    eta = etas(:,i);
    u = us(:,i);
    Va = sqrt(nu(1)*nu(1)+ nu(2)*nu(2));

    %Control

    if(i==1)
        theta_error_integrated = 0;
        height_error_integrated = 0;
        Va_error_integrated = 0;
    end
    zeta_v1 = 1; %Damping. tunable
    omega_n_v1 = 100; %Bandwidth. tunable.


    a_v1 = (1/m)*rho*Va_trim*S*(CD0 + CD_alpha*alpha_trim + CD_deltaE*deltaE_trim)...
            + (1/m)*rho*S_prop*C_prop*Va_trim;
    a_v2 = (1/m)*rho*S_prop*C_prop*k_motor^2 * deltaT_trim;

    Kpv1 = (2*zeta_v1*omega_n_v1 - a_v1) / a_v2;
    Kiv1 = omega_n_v1^2 / a_v2;

    Va_target = Va_trim+1;
    
    Va_error = Va_target - Va;
    Va_error_integrated = Va_error_integrated + h*Va_error;
    disp(Va_error)
    
    % PITCH ATTITUDE HOLD
    

    a_theta_1 = -0.25*rho*Va*c*S*CM_q*c/Jy;
    a_theta_2 = -0.5*rho*(Va^2)*c*S*CM_alpha/Jy;
    a_theta_3 = 0.5*rho*(Va^2)*c*S*CM_deltaE/Jy;

    deltaE_min = -deg2rad(45);
    deltaE_max = deg2rad(45);
    theta_error_max = deg2rad(20);

    Kp_theta = -deltaE_max/theta_error_max;
    
    omega_n_theta = sqrt(a_theta_2 + Kp_theta); %Bandwidth
    zeta_theta = 0.3; %Damping. Tunable

    Kd_theta = (2*zeta_theta*omega_n_theta - a_theta_1)/a_theta_3;
    Ki_theta = 0;


    % AIR SPEED HOLD USING PITCH

    zeta_v2 = 1; %Damping factor. tunable
    W_v2 = 10; %Bandwidth factor. Higher -> lower bandwidth. Tunable.

    omega_n_v2 = (1/W_v2)*omega_n_theta;

    Kiv2 = -omega_n_v2/g;
    Kpv2 = (a_v1 - 2*zeta_v2*omega_n_v2)/g;

    Va_target = Va_trim;
    Va_error = Va_target - Va;
    Va_error_integrated = Va_error_integrated + h*Va_error;

    % TARGET AND DELTA CALCULATION
    
    theta_target = Kpv2*Va_error + Kiv2 * Va_error_integrated;
    theta_error = theta_target - eta(3);
    theta_error_integrated = theta_error_integrated + h*theta_error;


    deltaE = Kp_theta*theta_error + Ki_theta*theta_error_integrated - Kd_theta*nu(3);
    deltaT = deltaT_trim + Kpv1*Va_error + Kiv1*Va_error_integrated;
%     deltaE = deltaE_trim;
%     deltaT = deltaT_trim;
    if deltaT < 0
        deltaT = 0;
    end

    if deltaE < deltaE_min
        deltaE = deltaE_min;
    elseif deltaE > deltaE_max
        deltaE = deltaE_max;

    end
    

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
    fancyAero = 0;
    if fancyAero

        [CL_ofAlpha, CD_ofAlpha] = computeAeroCoeffs(alpha, alpha_0, M, S, b, e, CL0, CD_p);
        

        CL = CL_ofAlpha + CL_q * 0.5*c*q/Va + CL_deltaE*deltaE;
        CD = CD_ofAlpha + CD_q * 0.5*c*q/Va + CD_deltaE*deltaE;
        
        F_lift = 0.5*rho*Va*Va*S*CL;
        F_drag = 0.5*rho*Va*Va*S*CD;

        CM = CM0 + CM_alpha * alpha+ CM_q*0.5*c*q/Va + CM_deltaE*deltaE;
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
    
    nu_mediate = nu(1:2);
    eta_dot = R_body_to_inertial * nu_mediate;
    eta_dot(end+1) = nu(3);

    nu = nu + h * nu_dot;
    eta = eta + h * eta_dot;

    
    if mod(i,100)==0
        anim=anim.update(eta);
    end
    
    nus(:,i+1) = nu;
    etas(:,i+1) = eta;

    pause(h); %This relates to the perceived time spent in the simulation. Relate to time t. TODO

end


