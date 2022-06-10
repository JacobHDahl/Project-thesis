% eta = [pn, pd, theta]'; % pos north(x), pos down(z), pitch angle in inertial frame
% eta = R * nu; %R being the rotation matrix from body to inertial
% nu = [u, w, q]'; % velocity in x, velocity in z and pitchrate in bodyframe

%%%%POSITIVE ROTATION IS ANTI-CLOCKWISE
clear all;
addpath("Control\");
addpath(genpath("hebi/"))

ConstStruct = load("ConstFile.mat");

h = ConstStruct.h;
iterations = ConstStruct.iterations;

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
q_trim = ConstStruct.q_trim;


nus = zeros(3,iterations);
etas = zeros(3,iterations);
eta_init = [0, 0, theta_trim]'; % x, z, theta in inertial frame
nu_init = [u_trim,w_trim, q_trim]'; % v(velocity_x), w(velocity_z), omega(rot_velocity) in body frame

% eta_lin_init = [0,0,theta_trim]';
% x_lin_init = [u_trim,w_trim, q_trim, theta_trim, 0]';
% 
% eta_lins = zeros(3,iterations);
% x_lins = zeros(5,iterations);
% eta_lins(:,1) = eta_lin_init;
% x_lins(:,1) = x_lin_init;

nus(:,1) = nu_init;
etas(:,1) = eta_init;

anim = animation(1);
%anim_lin = animation(2);
t = zeros(1,iterations);

kb = HebiKeyboard();
deltaE = deltaE_trim;
deltaT = deltaT_trim;

for i = 1:iterations-1
    t(i) = (i-1)*h;
    nu = nus(:,i);
    eta = etas(:,i);
    theta = eta(3);
    Va = sqrt(nu(1)*nu(1) + nu(2)*nu(2));
    alpha = atan2(nu(2),nu(1));

    
    u = nu(1);
    w = nu(2);
    q = nu(3);

%     eta_lin = eta_lins(:,i);
%     x_lin = x_lins(:,i);
    R_body_to_inertial = [cos(theta), -sin(theta);
                        sin(theta), cos(theta)]; %Rotation matrix from body to inertial
    
    %control
    Va_target = Va_trim;
    height_target = 0;
%     [deltaT, deltaE] = PIDControl(eta,nu,ConstStruct,Va_target, height_target);
%     %[deltaT, deltaE] = ezPID(eta,nu,ConstStruct,Va_target,height_target);
%     [deltaT, deltaE] = newPID(eta,nu,ConstStruct,Va_target,height_target);


%     deltaE = (2*q_dot_des*Jy/(CM_deltaE*rho*Va*Va*S*c)) -(CM0 + CM_alpha*alpha + 0.5*CM_q*c*q/(Va))/CM_deltaE;
    %Dynamic inversion ^
    %[Xu, Xw, Xq, XdeltaE, XdeltaT, Zu, Zw, Zq, ZdeltaE, Mu, Mw, Mq, MdeltaE] = LinearParamCalculation(ConstStruct);
    
    %Linear dynamic inversion using linearized model
    u_dash = u - u_trim;
    w_dash = w - w_trim;
    q_dash = q - q_trim;
    theta_dash = theta - theta_trim;
    

    %[deltaT, deltaE] = DynamicInv(eta, nu, ConstStruct);
%     deltaT = 0;
%     deltaE = 0;


    state = read(kb);
    if all(state.keys('w'))
        deltaE = deltaE - 0.001;
    elseif all(state.keys('s'))
        deltaE = deltaE+0.001;

    elseif all(state.keys('d'))
        deltaT = deltaT + 0.00001;
    elseif all(state.keys('a'))
        deltaT = deltaT - 0.00001;
    end
    
    u_action = [deltaT,deltaE]';
    

    % Model here
%     x_dash = [u_dash, w_dash, q_dash, theta_dash,0]';
%     u_lin = [deltaT-deltaT_trim,deltaE-deltaE_trim]';
% 
%     x_dash_dot = LinearizedModel(x_dash,u_action,ConstStruct);
%     x_lin_dot = x_dash_dot;
%     nu_lin_dot = [x_lin_dot(1), x_lin_dot(2), x_lin_dot(3)]';
% 
%     nu_lin = nu + h*nu_lin_dot;
%     nu_lin_mediate = nu_lin(1:2);
%     eta_lin_dot = R_body_to_inertial*nu_lin_mediate;
%     eta_lin_dot(end+1) = nu_lin(3);
%     eta_lin = eta + h*eta_lin_dot;


    nu_dot = NonLinFunc(eta,nu,u_action,ConstStruct);
    nu = nu + h * nu_dot;

    nu_mediate = nu(1:2);
    eta_dot = R_body_to_inertial * nu_mediate;
    eta_dot(end+1) = nu(3);

    eta = eta + h * eta_dot;

    
    if mod(i,100)==0
        anim=anim.update(eta);
        %anim_lin = anim_lin.update(eta_lin);
    end
    
    nus(:,i+1) = nu;
    etas(:,i+1) = eta;

    pause(h); %This relates to the perceived time spent in the simulation. Relate to time t. TODO

end


