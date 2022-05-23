function [deltaT, deltaE] = PIDControl(eta, nu, ConstStruct)
%PIDCONTROL Summary of this function goes here
%   Detailed explanation goes here
persistent initiated
persistent theta_error_integrated
persistent height_error_integrated
persistent Va_error_integrated
persistent called


h = ConstStruct.h;

m = ConstStruct.m;
Jy = ConstStruct.Jy;
rho = ConstStruct.rho;
CD0 = ConstStruct.CD0;
CD_alpha = ConstStruct.CD_alpha;
CD_deltaE = ConstStruct.CD_deltaE;
CM_alpha = ConstStruct.CM_alpha;
CM_q = ConstStruct.CM_q;
CM_deltaE = ConstStruct.CM_deltaE;
S = ConstStruct.S;
c = ConstStruct.c;
g = 9.81;
S_prop = ConstStruct.S_prop;
C_prop = ConstStruct.C_prop;
k_motor = ConstStruct.k_motor;

deltaE_trim = ConstStruct.deltaE_trim;
deltaT_trim = ConstStruct.deltaT_trim;
Va_trim = ConstStruct.Va_trim;
alpha_trim = ConstStruct.alpha_trim;


if (isempty(initiated))
    initiated = 1;
    theta_error_integrated = 0;
    height_error_integrated = 0;
    Va_error_integrated = 0;
    called = 1;

end

% AIR SPEED HOLD USING THROTTLE

Va = sqrt(nu(1)^2 + nu(2)^2);
zeta_v1 = 1; %Damping. tunable
omega_n_v1 = 100; %Bandwidth. tunable.


a_v1 = (1/m)*rho*Va_trim*S*(CD0 + CD_alpha*alpha_trim + CD_deltaE*deltaE_trim)...
    + (1/m)*rho*S_prop*C_prop*Va_trim;
a_v2 = (1/m)*rho*S_prop*C_prop*k_motor^2 * deltaT_trim;

Kpv1 = (2*zeta_v1*omega_n_v1 - a_v1) / a_v2;
Kiv1 = omega_n_v1^2 / a_v2;

Va_target = Va_trim+5;

Va_error = Va_target - Va;
Va_error_integrated = Va_error_integrated + h*Va_error;

% PITCH ATTITUDE HOLD


a_theta_1 = -0.25*rho*Va*c*S*CM_q*c/Jy;
a_theta_2 = -0.5*rho*(Va^2)*c*S*CM_alpha/Jy;
a_theta_3 = 0.5*rho*(Va^2)*c*S*CM_deltaE/Jy;

deltaE_min = -deg2rad(45);
deltaE_max = deg2rad(45);
theta_error_max = deg2rad(20);

Kp_theta = -deltaE_max/theta_error_max;

KDC_theta = (Kp_theta*a_theta_3)/(a_theta_2+Kp_theta*a_theta_3);

omega_n_theta = sqrt(a_theta_2 + Kp_theta); %Bandwidth
zeta_theta = 0.3; %Damping. Tunable

Kd_theta = (2*zeta_theta*omega_n_theta - a_theta_1)/a_theta_3;
Ki_theta = 0;

% ALTITUDE HOLD  USING PITCH
W_h = 5; %Bandwidth multiplier. Tunable.
zeta_h = 1; %Damping. Tunable
omega_n_h = (1/W_h)*omega_n_theta;

Ki_h = 2*(omega_n_h^2)/(KDC_theta*Va);
Kp_h = (2*zeta_h*omega_n_h)/(KDC_theta*Va);

height_target = 0;
height_error = height_target - eta(2);
height_error_integrated = height_error_integrated + h*height_error;


% AIR SPEED HOLD USING PITCH

zeta_v2 = 1; %Damping factor. tunable
W_v2 = 10; %Bandwidth factor. Higher -> lower bandwidth. Tunable.

omega_n_v2 = (1/W_v2)*omega_n_theta;

Kiv2 = -omega_n_v2/g;
Kpv2 = (a_v1 - 2*zeta_v2*omega_n_v2)/g;

Va_target = Va_trim+5;
Va_error = Va_target - Va;
Va_error_integrated = Va_error_integrated + h*Va_error;
% TARGET AND DELTA CALCULATION

theta_target_max = deg2rad(30);
theta_target_min = deg2rad(-30);

%theta_target = 5*Kpv2*Va_error + Kiv2 * Va_error_integrated;
theta_target = Kp_h * height_error + Ki_h*height_error_integrated;
if theta_target > theta_target_max
    theta_target = theta_target_max;
elseif theta_target < theta_target_min
    theta_target = theta_target_min;
end
disp("theta_target")
disp(rad2deg(theta_target))
theta_error = theta_target - eta(3);
theta_error_integrated = theta_error_integrated + h*theta_error;


deltaE = 3*Kp_theta*theta_error + Ki_theta*theta_error_integrated - 3*Kd_theta*nu(3);
deltaT = deltaT_trim + Kpv1*Va_error + Kiv1*Va_error_integrated;
if called < 100
    deltaE = deltaE_trim;
    deltaT = deltaT_trim;
end

if deltaT < 0
    deltaT = 0;
end

called = called +1;

% if deltaE < deltaE_min
%     deltaE = deltaE_min;
% elseif deltaE > deltaE_max
%     deltaE = deltaE_max;
% 
% end
end

