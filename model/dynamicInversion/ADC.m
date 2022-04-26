function [MACH,QBAR] =  ADC(VT,H)
% air data computer
R0 = 2.377*10^-3; % sea-level density
TFAC = 1.0 - 0.703E-5 * H;
T = 519.0 * TFAC; % temperature
RHO = R0 * (TFAC^4.14); % density
MACH= VT/(sqrt((1.4*1716.3*T))); % Mach number
QBAR = 0.5*RHO*VT*VT; % dynamic pressure
end