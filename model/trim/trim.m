function [res, deltaT,deltaE,u_init,w_init,theta_init] =  trim(Va,gamma)
%Trim takes in a desired airspeed(Va) and climb angle(gamma) and computes
%the trim conditions of the airplane using the angle of attack(alpha)

%Va = 24 gives approx the best results for level flight(gamma=0)


%Set up the function to optimize. (x_dot-f(x,u))^2
func = @(x)(([0,0,0]-f(x,[Va,gamma])')*([0,0,0]-f(x,[Va,gamma])')');

%Initial guess
x0 = deg2rad(5);

[alpha,res] = fminsearch(func,x0);

[deltaT, deltaE, u_init, w_init,theta_init] = computeU(alpha,Va,gamma);


end