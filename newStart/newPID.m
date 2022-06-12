function [deltaT, deltaE] = newPID(eta, nu, ConstStruct, Va_target, height_target)

h = ConstStruct.h;
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

persistent height_error_integrated;
persistent called;

if isempty(height_error_integrated)
    height_error_integrated = 0;
    called = 1;
end

Va = sqrt(nu(1)^2 + nu(2)^2);
alpha = atan2(nu(2),nu(1));


a_theta_1 = -rho*Va*Va*c*S*CM_alpha*c/(2*Jy*2*Va);
a_theta_2 = -rho*Va*Va*c*S*CM_alpha/(2*Jy);
a_theta_3 = rho*Va*Va*c*S*CM_deltaE/(2*Jy);

deltaE_max = deg2rad(40);
e_theta_max = deg2rad(10);

Kp_theta = -deltaE_max/e_theta_max;

omega_n_theta = sqrt(a_theta_2 + abs(a_theta_3)*deltaE_max/e_theta_max); %bandwidth
zeta_theta = 0.1; %tunable damping


Kd_theta = (2*zeta_theta*omega_n_theta - a_theta_1)/a_theta_3;

K_theta_DC = Kp_theta*a_theta_3/(a_theta_2 + Kp_theta*a_theta_3);

%ALTITUDE HOLD using commanded pitch

Wh = 15; %bandwidth scaler tunable

omega_n_h = omega_n_theta/Wh;
zeta_h = 1; %damping tunable

Ki_h = omega_n_h^2 / (K_theta_DC*Va);

Kp_h = (2*zeta_h*omega_n_h)/(K_theta_DC*Va);

height_error = height_target - eta(2);
height_error_integrated = height_error_integrated + h*height_error;

if height_error < 1
    height_error_integrated = 0;
end

theta_target = Kp_h*height_error + Ki_h*height_error_integrated;

theta_target_max = deg2rad(50);
if theta_target > theta_target_max
    theta_target = theta_target_max;

elseif theta_target < -theta_target_max
    theta_target = -theta_target_max;
end


%Calculate gains

theta_error = theta_target - eta(3);

deltaE = Kp_theta*theta_error - Kd_theta*nu(3);

deltaT = deltaT_trim;

if deltaE > deltaE_max

    deltaE = deltaE_max;

elseif deltaE < -deltaE_max
    deltaE = -deltaE_max;

end
if mod(called,500)==0
    disp("theta_target")
    disp(rad2deg(theta_target))
    disp("theta_error")
    disp(rad2deg(theta_error))

end
called = called + 1;

end

