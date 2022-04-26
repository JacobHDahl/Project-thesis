function [CL, CD] = computeAeroCoeffs(alpha, alpha_0, M, S, b, e, CL0, CD_p)
%computeAeroForces computes aerodynamical forces of the airplane
% using a more complex model for the coefficients for drag, lift and pitch
% moment


AR = b*b/S;

CL_alpha = pi*AR / (1 + sqrt(1 + (AR/2)*(AR/2)));

CL = (1-sigmoid_alpha(alpha,alpha_0,M))*(CL0 + CL_alpha*alpha) + sigmoid_alpha(alpha,alpha_0,M)*(sign(alpha)*sin(alpha)*sin(alpha)*cos(alpha));

CD = CD_p + (CL0 + CL_alpha * alpha)^2 / pi*e*AR;

end

