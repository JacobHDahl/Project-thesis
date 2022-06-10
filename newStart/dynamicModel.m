function nu_dot = dynamicModel(nu,eta,u_action,ConstStruct)
m = ConstStruct.m;
Jy = ConstStruct.Jy;
g = 9.81;
rho = ConstStruct.rho;
S_prop = ConstStruct.S_prop;
C_prop = ConstStruct.C_prop;
k_motor = ConstStruct.k_motor;



u = nu(1);
w = nu(2);
q = nu(3);
theta = eta(3);
Va = sqrt(u^2 + w^2);
alpha = atan2(w,u);

deltaT = u_action(1);
deltaE = u_action(2);


FTHRUST_X = 0.5*S_prop*C_prop*((k_motor*deltaT)^2-Va^2);

FG_X = -m*g*sin(theta);
FG_Z = m*g*cos(theta);


linear = 0; %toggle to select linear or non-linear aero-model
[FAERO_X,FAERO_Z, M_aero] = calculateAeroForces(nu,ConstStruct,deltaE,linear);




fx = FG_X + FTHRUST_X + FAERO_X ;
fz = FG_Z + FAERO_Z;


u_dot = -q*w  + (1/m)*fx;
w_dot = q*u + (1/m)*fz;
q_dot = M_aero/Jy;


nu_dot = [u_dot, w_dot, q_dot]';

end

