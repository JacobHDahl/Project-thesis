clear all;

addpath(genpath("hebi"));
addpath("aero\");
addpath("LinearModel\")
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


%Preallocation
nus = zeros(3,iterations);
etas = zeros(3,iterations);
eta_init = [0, 0, theta_trim]'; % x, z, theta in inertial frame
nu_init = [u_trim,w_trim, 0]'; % v(velocity_x), w(velocity_z), omega(rot_velocity) in body frame

nus(:,1) = nu_init;
etas(:,1) = eta_init;
t = zeros(1,iterations);



tswitch = 5;
K_diff = -10;
gammaw = 10;
gammav = 50;
lambda = 0.01;
nmid = 5;
nin = 3;
amin = 0.01;
amax = 10;
% precompute activation potentials
a = zeros(nmid,1);
for i=1:nmid-1
    a(i) = tan(atan(amin) + (atan(amax) - atan(amin))*(i+1)/nmid);
end
% preallocate arrays
ref = zeros(1,iterations);
us = zeros(1,iterations);
weights = zeros(nmid,iterations);
wdot = zeros(1,nmid);
vs = zeros(nmid*nin,iterations);
vdot = zeros(1,nmid*nin);
rdot = zeros(1,1);
xdot = zeros(1,1);
Wdot = zeros(1,nmid);
W = zeros(1,nmid); % output weights
Vdot = zeros(nmid,nin);
V = zeros(nmid,nin); %Input weights
xbar = zeros(nin,1); %Input to NN. state, plant input u, and bias.
sig = zeros(nmid,1);
sigp = zeros(nmid,nmid);



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
    q = nu(3);

    % external input

    if mod(t(i),2*tswitch)<tswitch
        externalInput = 0.5;
    else
        externalInput = -0.5;
    end
    %externalInput = 0.01;
    % reference model
    rdot = -K_diff*(externalInput-ref(1,i) );
    % error signals
    e = ref(1,i) - q;
    % NN inputs
    if i>1
        oldu = us(i-1);
    else
        oldu = 0;
    end
    xbar(1) = 1;
    xbar(2) = q;
    xbar(3) = oldu;

    % get weights from state vector
    W = weights(:,i);
    for j=1:nmid
        V(j,:) = vs((j-1)*nin+1:j*nin,i);
    end

    % adaptive controller
    VX = V*xbar;
    for j=1:nmid-1
        ez = exp(-a(j)*VX(j));
        sig(j) = 1/( 1 + ez );
        sigp(j,j) = a(j)*ez*sig(j)*sig(j);
    end
    sig(nmid) = 1;
    vad = W'*sig;

    [Xu, Xw, Xq, XdeltaE, XdeltaT, Zu, Zw, Zq, ZdeltaE, Mu, Mw, Mq, MdeltaE] = LinearParamCalculation(ConstStruct);

    deltaE_max = deg2rad(40);
    deltaE = (rdot(1) - K_diff*e - vad - Mq*q)/MdeltaE;
    if deltaE > deltaE_max

        deltaE = deltaE_max;

    elseif deltaE < -deltaE_max
        deltaE = -deltaE_max;

    end

    if mod(i,100)==0
        disp(deltaE)
    end
    us(i) = deltaE;

    % learning law
    Wdot = -gammaw*(e*( sig' - xbar'*V'*sigp) + lambda*norm(e)*W' );
    Vdot = -gammav*(sigp*W*e*xbar' + lambda*norm(e)*V);
    % put NN update in a vector
    wdot = Wdot;
    for j=1:nmid
        vdot((j-1)*nin+1:j*nin) = Vdot(j,:);
    end

    %Get keyboard input
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
        height_target =  55;
    end

    Va_target = Va_trim;
    %[deltaT, deltaE] = newPID(eta, nu, ConstStruct, Va_target, height_target);
    deltaT = deltaT_trim;
    u_action = [deltaT,deltaE]';

    nu_dot = dynamicModel(nu,eta,u_action,ConstStruct);
    % plant model


    eta_dot = nu2eta_dot(nu,theta);

    eta = eta + eta_dot*h;
    nu = nu + nu_dot*h;

    if mod(i,1000)==0
        anim=anim.update(eta);
        %anim_lin = anim_lin.update(eta_lin);
    end

    % numerically integrate
    ref(:,i+1) = ref(:,i) + rdot*h;
    weights(:,i+1) = weights(:,i) + wdot'*h;
    vs(:,i+1) = vs(:,i) + vdot'*h;

    nus(:,i+1) = nu;
    etas(:,i+1) = eta;

    %pause(h); %This relates to the perceived time spent in the simulation. Relate to time t. TODO

end
figure(2)
plot(ref)
hold on
plot(nus(3,:))