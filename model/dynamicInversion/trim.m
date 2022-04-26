% Cost Function for 3-DOF Aircraft
function f = trim(s)
global x u gamma
u(1)= s(1);
u(2)= s(2);
x(2)= s(3);
x(3)= x(2)+ gamma;
time= 0.0;
xd=transp(time,x,u);
f = xd(1) ̂ 2 + 100*xd(2) ̂ 2 + 10*xd(4) ̂ 2;