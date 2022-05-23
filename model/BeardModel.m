% eta = [pn, pd, theta]'; % pos north(x), pos down(z), pitch angle in inertial frame
% eta = R * nu; %R being the rotation matrix from body to inertial
% nu = [u, w, q]'; % velocity in x, velocity in z and pitchrate in bodyframe

%%%%POSITIVE ROTATION IS ANTI-CLOCKWISE
clear
addpath("Control\");

ConstStruct = load("ConstFile.mat");

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

nus = zeros(3,iterations);
etas = zeros(3,iterations);

eta_init = [0, 10, theta_trim]'; % x, z, theta in inertial frame
nu_init = [u_trim,w_trim, 0]'; % v(velocity_x), w(velocity_z), omega(rot_velocity) in body frame

nus(:,1) = nu_init;
etas(:,1) = eta_init;

anim = animation;
t = zeros(1,iterations);


for i = 1:iterations-1
    t(i) = (i-1)*h;
    nu = nus(:,i);
    eta = etas(:,i);
    theta = eta(3);
    R_body_to_inertial = [cos(theta), sin(theta);
                        -sin(theta), cos(theta)]; %Rotation matrix from body to inertial


    % Model here
    nu_dot = NonLinFunc(eta,nu,ConstStruct);
    
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

    pause(h*5); %This relates to the perceived time spent in the simulation. Relate to time t. TODO

end


