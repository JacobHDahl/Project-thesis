function theta_target = generateThetaTarget(ConstStruct,nu,eta,height_target)
h = ConstStruct.h;
% ship parameters

Jy = ConstStruct.Jy;
% Aero parameters
rho = ConstStruct.rho;
CM_alpha = ConstStruct.CM_alpha;
CM_deltaE = ConstStruct.CM_deltaE;
S = ConstStruct.S;
c = ConstStruct.c;


persistent height_error_integrated;

if isempty(height_error_integrated)
    height_error_integrated = 0;
end
Va = sqrt(nu(1)^2 + nu(2)^2);

a_theta_2 = -rho*Va*Va*c*S*CM_alpha/(2*Jy);
a_theta_3 = rho*Va*Va*c*S*CM_deltaE/(2*Jy);

deltaE_max = deg2rad(40);
e_theta_max = deg2rad(10);

Kp_theta = -3*deltaE_max/e_theta_max;

omega_n_theta = sqrt(a_theta_2 + abs(a_theta_3)*deltaE_max/e_theta_max); %bandwidth

K_theta_DC = Kp_theta*a_theta_3/(a_theta_2 + Kp_theta*a_theta_3);

%ALTITUDE HOLD using commanded pitch

Wh = 4; %bandwidth scaler tunable

omega_n_h = omega_n_theta/Wh;
zeta_h = 1; %damping tunable

Ki_h =0.00020;% omega_n_h^2 / (K_theta_DC*Va);

Kp_h =0.008;% (2*zeta_h*omega_n_h)/(K_theta_DC*Va);

height_error = height_target - eta(2);
height_error_integrated = height_error_integrated + h*height_error;

% if height_error < 1
%     height_error_integrated = height_error_integrated*0.6;
% end

theta_target = Kp_h*height_error + Ki_h*height_error_integrated;

theta_target_max = deg2rad(50);
if theta_target > theta_target_max
    theta_target = theta_target_max;

elseif theta_target < -theta_target_max
    theta_target = -theta_target_max;
end

end

