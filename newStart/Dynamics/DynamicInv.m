function [deltaT, deltaE] = DynamicInv(eta, nu, ConstStruct)
%PIDCONTROL Summary of this function goes here
%   Detailed explanation goes here
persistent initiated
persistent theta_error_integrated
persistent height_error_integrated
persistent Va_error_integrated
persistent called
persistent theta_target_prev


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
q_trim = ConstStruct.q_trim;
u_trim = ConstStruct.u_trim;
w_trim = ConstStruct.w_trim;


if (isempty(initiated))
    initiated = 1;
    theta_error_integrated = 0;
    height_error_integrated = 0;
    Va_error_integrated = 0;
    called = 1;
    theta_target_prev = 0;

end
theta = eta(3);
R_body_to_inertial = [cos(theta), sin(theta);
                        -sin(theta), cos(theta)];

height_target = 0;
% if called > 5000
%     height_target = 40;
% end
height_error = height_target - eta(2);

% AIR SPEED HOLD USING THROTTLE

Va = sqrt(nu(1)^2 + nu(2)^2);
zeta_v1 = 1; %Damping. tunable
omega_n_v1 = 100; %Bandwidth. tunable.


a_v1 = (1/m)*rho*Va_trim*S*(CD0 + CD_alpha*alpha_trim + CD_deltaE*deltaE_trim)...
    + (1/m)*rho*S_prop*C_prop*Va_trim;
a_v2 = (1/m)*rho*S_prop*C_prop*k_motor^2 * deltaT_trim;

Kpv1 = (2*zeta_v1*omega_n_v1 - a_v1) / a_v2;
Kiv1 = omega_n_v1^2 / a_v2;

Va_target = Va_trim + 5;%(height_error/2);


Va_error = Va_target - Va;
Va_error_integrated = Va_error_integrated + h*Va_error;





height_target = 0;
% if called > 5000
%     height_target = 40;
% end
height_error = height_target - eta(2);
height_error_integrated = height_error_integrated + h*height_error;

Va_target = Va_trim+5;
Va_error = Va_target - Va;
Va_error_integrated = Va_error_integrated + h*Va_error;
% TARGET AND DELTA CALCULATION

theta_target_max = deg2rad(50);
theta_target_min = deg2rad(-50);

nu_body = nu(1:2);
nu_inertial = R_body_to_inertial * nu_body;

a_theta_1 = -rho*Va*Va*c*S*CM_alpha*c/(2*Jy*2*Va);
a_theta_2 = -rho*Va*Va*c*S*CM_alpha/(2*Jy);
a_theta_3 = rho*Va*Va*c*S*CM_deltaE/(2*Jy);

deltaE_max = deg2rad(50);
e_theta_max = deg2rad(10);

Kp_theta = -deltaE_max/e_theta_max;

omega_n_theta = sqrt(a_theta_2 + abs(a_theta_3)*deltaE_max/e_theta_max); %bandwidth


K_theta_DC = Kp_theta*a_theta_3/(a_theta_2 + Kp_theta*a_theta_3);

%ALTITUDE HOLD using commanded pitch

Wh = 5; %bandwidth scaler tunable

omega_n_h = omega_n_theta/Wh;
zeta_h = 0.9; %damping tunable

Ki_h = omega_n_h^2 / (K_theta_DC*Va);

Kp_h = (2*zeta_h*omega_n_h)/(K_theta_DC*Va);

Kd_h = 0;

height_error = height_target - eta(2);
height_error_integrated = height_error_integrated + h*height_error;

theta_target = Kp_h*height_error + Ki_h*height_error_integrated;


if theta_target > theta_target_max
    theta_target = theta_target_max;

elseif theta_target < theta_target_min
    theta_target = theta_target_min;

end


[Xu, Xw, Xq, XdeltaE, XdeltaT, Zu, Zw, Zq, ZdeltaE, Mu, Mw, Mq, MdeltaE] = LinearParamCalculation(ConstStruct);

%Linear dynamic inversion using linearized model

q_dot_des = 0;
q_des = -1;%theta_target-theta_target_prev;
if mod(called,100)==0
    disp("q_des")
    disp(q_des)
end

called = called +1;
theta_target_prev = theta_target;

u = nu(1);
w = nu(2);
q = nu(3);
q_dot_dash_des = q_dot_des - q_trim;
u_dash = u - u_trim;
w_dash = w - w_trim;
q_dash = q - q_trim;

K = 200;

deltaE_dash = (1/MdeltaE)*(q_dot_dash_des - Mu*u_dash - Mw*w_dash - Mq*q_dash + K*(q_des-q));

deltaE = deltaE_dash + deltaE_trim;
deltaT = deltaT_trim + Kpv1*Va_error + Kiv1*Va_error_integrated;

if deltaT < 0
    deltaT = 0;
end



% if deltaE < deltaE_min
%     deltaE = deltaE_min;
% elseif deltaE > deltaE_max
%     deltaE = deltaE_max;
%
% end
end

