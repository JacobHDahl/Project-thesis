function [res, deltaT,deltaE,u_init,w_init,theta_init] =  trim(Va,gamma, ConstStruct)
%Trim takes in a desired airspeed(Va) and climb angle(gamma) and computes
%the trim conditions of the airplane using the angle of attack(alpha)

%Set up the function to optimize. (x_dot-f(x,u))^2
func = @(x)(([0,0,0]-f(x,[Va,gamma])')*([0,0,0]-f(x,[Va,gamma])')');

%Initial guess
x0 = deg2rad(0);

[alpha,res] = fminsearch(func,x0);

[deltaT, deltaE, u_init, w_init,theta_init] = computeU(alpha,Va,gamma, ConstStruct);


end