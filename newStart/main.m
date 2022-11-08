clear all;

addpath(genpath("hebi"));
addpath("aero\");
addpath("LinearModel\")
addpath("helping_functions\");
ConstStruct = load("ConstFile.mat");
LUT = readtable("airfoildata\xf-aquilasm-il-50000.csv");
% Set seed
rng(999);

h = ConstStruct.h;
iterations = ConstStruct.iterations*4;
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
nu_init = [u_trim,w_trim, 0]';
nus(:,1) = nu_init;
etas(:,1) = eta_init;
t = zeros(1,iterations);

tswitch = 5;
K_diff = -10;
gammaw = 50;
gammav = 1;
lambda = 0.05;
nmid = 10;

nin = 3;
amin = 0.01;
amax = 10;


NNStruct.K_diff = K_diff;
NNStruct.gammaw = gammaw;
NNStruct.gammav = gammav;
NNStruct.lambda = lambda;
NNStruct.nmid = nmid;
NNStruct.nin = nin;

% precompute activation potentials
a = zeros(nmid,1);
for i=1:nmid-1
    a(i) = tan(atan(amin) + (atan(amax) - atan(amin))*(i+1)/nmid);
end
% preallocate arrays
vads = zeros(1,iterations);
errors = zeros(1,iterations);
ref = zeros(1,iterations);
us = zeros(1,iterations);
weights = zeros(nmid,iterations);
recurrentWs = zeros(nmid*nmid,iterations);
vs = zeros(nmid*nin,iterations);
vdot = zeros(1,nmid*nin);
recdot = zeros(1,nmid*nmid);
rdot = zeros(1,1);
xdot = zeros(1,1);
Wdot = zeros(1,nmid);
W = zeros(1,nmid); % output weights
Vdot = zeros(nmid,nin);
V = zeros(nmid,nin); %Input weights
xbar = zeros(nin,1); %Input to NN. state, plant input u, and bias.
sig = zeros(nmid,1);
sigp = zeros(nmid,nmid);
sigs = zeros(nmid,iterations);
stallIdxs = [];

height_targets = zeros(1,iterations);
smoothingFac = 10; %lower is more smoothing
disp("Generating random walk setpoints")
[height_setpoints, unfilt] = generateRandomWalk(iterations, [-0.5 0.5], smoothingFac);
disp("Finished random walk generation")
disp("Starting simulation")
plot(height_setpoints);
hold on
plot(unfilt)
legend("Filtered", "UnFiltered")
hold off
anim = animation(1);

kb = HebiKeyboard();
deltaE = deltaE_trim;
deltaT = deltaT_trim;
deltaE_old = 0;
height_error_integrated = 0;
externalInput = 0;
theta_error_integrated = 0;

activation_prime = @(x) 1-tanh(x).*tanh(x);
activation = @tanh;

mse = @(y_true, y_pred) mean((y_true - y_pred).^2);
mse_prime = @(y_true, y_pred) 2*(y_pred-y_true)/length(y_true);


%Create new, randomly initiated net
% net = Network();
% FC1 = FCLayer(5,10);
% net = net.add(FC1);
% AL1 = ActivationLayer(activation, activation_prime);
% net = net.add(AL1);
% FC2 = FCLayer(10,10);
% net = net.add(FC2);
% AL2 = ActivationLayer(activation, activation_prime);
% net = net.add(AL2);
% FC2 = FCLayer(10,1);
% net = net.add(FC2);
%net = net.use(mse, mse_prime);

% Load previously trained neural net
netStrkt = load("trained_net.mat");
net = netStrkt.net;

for i = 1:iterations
    t(i) = (i-1)*h;
    nu = nus(:,i);
    nu(1:2) = nu(1:2);
    eta = etas(:,i);
    theta = eta(3);
    q = nu(3);
    alpha = atan2(nu(2),nu(1));
    
    Va = sqrt(nu(1)^2 + nu(2)^2);
  
    height_setpoint = height_setpoints(i);
    timeComp = mod(t(i), round(iterations/5)*h);
    if timeComp<=50
        height_target = mean(height_targets(1:i))+10;
    elseif timeComp > 50 && timeComp < 100
        height_target = mean(height_targets(1:i))+10;
    else
        height_target = height_setpoint;
    end

    height_targets(i) = height_target;

    Kp_theta = 1;
    Ki_theta = 0;
    theta_target = generateThetaTarget(ConstStruct,nu,eta,height_target);
    theta_error = theta_target - eta(3);
    theta_error_integrated = theta_error_integrated + h*theta_error;
    externalInput = Kp_theta*theta_error + Ki_theta*theta_error_integrated;

    %Get keyboard input
    state = read(kb);
    if all(state.keys('w')) && externalInput ~= 0.3
        externalInput = 0.3;
    elseif all(state.keys('s'))  && externalInput ~=-0.3
        externalInput = -0.3;
    elseif all(state.keys('d'))
        externalInput = 0;
    end
    
    rdot = -K_diff*(externalInput-ref(1,i) );
    % error signals
    error = ref(1,i) - q;
    errors(i) = error;
    W = weights(:,i);
    for j=1:nmid
        V(j,:) = vs((j-1)*nin+1:j*nin,i);
    end

    % NN inputs
    if i>2
        oldu = us(i-1);
        oldVad = vads(i-1);
        oldoldVad = vads(i-2);
    else
        oldu = 0;
        oldVad = 0;
        oldoldVad = 0;
    end
    
    xbar(1) = nu(1);
    xbar(2) = nu(2);
    xbar(3) = q;
    xbar(4) = oldu;
    xbar(5) = oldVad;
    
    [net, vad] = net.feedForward(xbar);
    learning_rate = 0.01;
    if i < iterations
        net = net.adapt(error, learning_rate);
    end

    CM = CM0 + CM_alpha*alpha + CM_q*0.5*c*q/Va + CM_deltaE*deltaE;
    M_aero = 0.5*rho*Va*Va*S*c*CM;
    const = 0.5*rho*Va*Va*S*c/Jy;
    [Xu, Xw, Xq, XdeltaE, XdeltaT, Zu, Zw, Zq, ZdeltaE, Mu, Mw, Mq, MdeltaE] = LinearParamCalculation(ConstStruct);
    deltaE = (rdot(1) - K_diff*error - vad - Mq*q)/MdeltaE;

    % Perfect dynamic inv:
    %deltaE = (rdot(1)-K_diff*error-vad)/(const*CM_deltaE) -(CM0 + CM_alpha*alpha + 0.5*CM_q*c*q/Va)/CM_deltaE;
    deltaE_max = deg2rad(40);
    if deltaE > deltaE_max
        deltaE = deltaE_max;
    elseif deltaE < -deltaE_max
        deltaE = -deltaE_max;
    end

    sigs(:,i) = sig;
    vads(i) = vad;
    
    us(i) = deltaE;
    Va_target = Va_trim;
    
    %[deltaT, deltaE] = newPID(eta, nu, ConstStruct, Va_target, height_target);
    deltaT = deltaT_trim;
    u_action = [deltaT,deltaE]';

    modelNoise = 0.1*sin(t(i));
    nu_dot = dynamicModel(nu,eta,u_action,ConstStruct, LUT, modelNoise);
    
    nu(1:2) = nu(1:2);
    eta_dot = nu2eta_dot(nu,theta);

    eta = eta + eta_dot*h;
    nu = nu + nu_dot*h;
    
    % Comment out to remove live animation
%     if mod(i,2)==0
%         anim=anim.update(eta);
%         %anim_lin = anim_lin.update(eta_lin);
%     end

    if mod(i,1000)==0
        disp(['Iteration: ' num2str(i) ' of ' num2str(iterations)])
    end

    vs(:,i+1) = vs(:,i) + vdot'*h;
    ref(:,i+1) = ref(:,i) + rdot*h;
    nus(:,i+1) = nu;
    etas(:,i+1) = eta;
end
nonAdaptMeanError = 0.5323;
fprintf(['[\b' 'Mean error: ' num2str(mean(errors)) ']\b\n'])

qs = nus(3,1:end-1);
ons = ones(1,length(qs));
oldus = us(1:length(qs));
inputs = [ons; qs; oldus];

%save("symbolicRegData.mat", "inputs", "vads");

fig = figure(2);

plot(t,ref(1:end-1),'Linewidth',1)
hold on
plot(t,nus(3,1:end-1),'Linewidth',1)
xlabel('Time[s]')
legend('Reference target','Pitchrate')
hold off

figure(3)
plot(t,height_targets(1:end),'Linewidth',1);
hold on
plot(t,etas(2,1:end-1),'Linewidth',1)
xlabel('Time[s]')
ylabel('Height[m]')
legend('Height target','UAV height')
hold off

% weight_norms = vecnorm(weights,2);
% figure(4)
% plot(weight_norms)
% legend("Weight norm")
% hold off

% refData = ref;
% qData = nus(3,:);
% heightTargetData = height_targets;
% heightData = etas(2,:);
% save('qDataFlying.mat','qData');
% save('refDataFlying.mat','refData');
% save('heightTargetDataFlying.mat','heightTargetData');
% save('heightDataFlying.mat','heightData');
% save('t.mat',"t");

% heightDataPID = etas(2,:);
% save('heightDataFlyingPID.mat','heightDataPID');

% exportgraphics(fig, "NNoutputOverTime.eps")

% hold on
% plot(t,nus(3,1:end-1))
% xlabel('time[s]')
% ylabel('Pitch rate[deg/s]')
% data = nus(3,:);
% save('randomData2.mat','data');
% dataref = ref;
% save('randomRef2.mat','dataref');
%save('t_filtered.mat','t');
