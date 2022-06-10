clear all;

addpath(genpath("hebi"));
addpath("aero\");
addpath("helping_functions\");
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
nu_init = [u_trim,w_trim, 0]'; % v(velocity_x), w(velocity_z), omega(rot_velocity) in body frame


nus(:,1) = nu_init;
etas(:,1) = eta_init;

%anim_lin = animation(2);
t = zeros(1,iterations);

anim = animation(1);

kb = HebiKeyboard();
deltaE = deltaE_trim;
deltaT = deltaT_trim;
height_target = 0;

for i = 1:iterations
    t(i) = (i-1)*h;
    nu = nus(:,i);
    eta = etas(:,i);

    theta = eta(3);
    

    state = read(kb);
    if all(state.keys('w'))
        deltaE = deltaE - 0.001;
    elseif all(state.keys('s'))
        deltaE = deltaE+0.001;

    elseif all(state.keys('d'))
        deltaT = deltaT + 0.001;
    elseif all(state.keys('a'))
        deltaT = deltaT - 0.001;
    elseif all(state.keys('k'))
        height_target = 15;
    end
    
    Va_target = Va_trim;
    [deltaT, deltaE] = newPID(eta, nu, ConstStruct, Va_target, height_target);
    
    u_action = [deltaT,deltaE]';
    
    nu_dot = dynamicModel(nu,eta,u_action,ConstStruct);
    
    

    eta_dot = nu2eta_dot(nu,theta);

    eta = eta + eta_dot*h;
    nu = nu + nu_dot*h;


    if mod(i,300)==0
        anim=anim.update(eta);
        %anim_lin = anim_lin.update(eta_lin);
    end

    nus(:,i+1) = nu;
    etas(:,i+1) = eta;

    pause(h); %This relates to the perceived time spent in the simulation. Relate to time t. TODO

end