% Inverse Dynamics Controller for F16 Linear Dynamics
function xdot=dynamicInv(t,x)
global y
% VT= x(1); ! True airspeed
% ALPH= x(2); ! Angle of Attack in rads.
% THTA= x(3); ! Pitch attitude in rads.
% Q = x(4); ! Pitch rate rad/s
% elev= x(5); ! elevator actuator

% Inverse Dynamics Controller
% Model of aircraft
a = [ -0.1270 -235.0000 -32.2000 -9.5100     -0.2440;
    0       -0.9690     0       0.9080      -0.0020;
    0       0           0       1.0             0;
    0       -4.5600     0       -1.5800     -0.2000;
    0       0           0       0           -20.2];
b = [0, 0, 0, 0, 20.2]';
c = [-0.01, 16.262, 0, 13.378, -0.0485];
%y=cstar’ modifed
% command input
r = 1; % check step response
rdot= 0;
% controller parameters
K= 10;
% plant outputs, tracking errors, and control inputs
y = c*x; % y= cstar  modified
e = r-y;
v = K*e;
w = rdot - c*a*x + v;
u = inv(c*b) * w;
tht1 = 0;
uelev= u;
% Aircraft State Equations
xdot(1)= -0.1270*x(1) -235.0*x(2) -32.3*x(3) -9.51*x(4) -0.244*x(5) +62.8*tht1;
xdot(2)= -0.9690*x(2) +0.908*x(4) -0.002*x(5) -0.04*tht1;
xdot(3)= x(4);
xdot(4)= -4.56*x(2) -1.58*x(4) -0.2*x(5);
xdot(5)= -020.2*x(5) +20.2*uelev;
xdot=xdot';