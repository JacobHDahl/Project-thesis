% parameter choices
ConstStruct = load("ConstFile.mat");
h = ConstStruct.h;
dt = 0.05; 
tfinal = 50; 
tswitch = 5;
Mdelta = -10; 
Mq = -1;
K = -1;
gammaw = 1; 
gammav = 10;
lambda = 0.01;
nmid = 5; 
nin = 3;
amin = 0.01; 
amax = 10;
% precompute activation potentials
a = zeros(nmid,1);
for i=1:nmid-1
    a(i) = tan(atan(amin) + (atan(amax) - atan(amin))*(i+1)/nmid);
end
% preallocate arrays
points = tfinal/dt + 1;
t = zeros(points,1); 
r = zeros(points,1);
x = zeros(points,1); 
u = zeros(points,1);
w = zeros(points,nmid); 
wdot = zeros(1,nmid);
v = zeros(points,nmid*nin); 
vdot = zeros(1,nmid*nin);
rdot = zeros(1,1); 
xdot = zeros(1,1);
Wdot = zeros(1,nmid); 
W = zeros(1,nmid);
Vdot = zeros(nmid,nin); 
V = zeros(nmid,nin);
xbar = zeros(nin,1);
sig = zeros(nmid,1); 
sigp = zeros(nmid,nmid);


for i=1:points
    t(i)=(i-1)*dt;
    % external input
    if mod(t(i),2*tswitch)<tswitch
        externalInput = 1;
    else 
        externalInput = -1;
    end
    % reference model
    rdot = -K*( externalInput - r(i,1) );
    % error signals
    e = r(i,1) - x(i,1);
    % NN inputs
    if i>1 
        oldu = u(i-1);
    else 
        oldu = 0;
    end
    xbar(1) = 1;
    xbar(2) = x(i,1);
    xbar(3) = oldu;
    % get weights from state vector
    W = w(i,:);
    for j=1:nmid
        V(j,:) = v(i,(j-1)*nin+1:j*nin);
    end
    % adaptive controller
    vx = V*xbar;
    for j=1:nmid-1
        ez = exp(-a(j)*vx(j));
        sig(j) = 1/( 1 + ez );
        sigp(j,j) = a(j)*ez*sig(j)*sig(j);
    end
    sig(nmid) = 1;
    vad = W*sig;
    u(i) = (rdot(1) - K*e - vad - Mq*x(i,1))/Mdelta;
    % plant model
    xdot = sin(x(i,1)) + Mdelta*u(i) + Mq*x(i,1);
    % learning law
    Wdot = -gammaw*(e*( sig' - xbar'*V'*sigp) + lambda*norm(e)*W );
    Vdot = -gammav*(sigp*W'*e*xbar' + lambda*norm(e)*V);
    % put NN update in a vector
    wdot = Wdot;
    for j=1:nmid
        vdot((j-1)*nin+1:j*nin) = Vdot(j,:);
    end

    % numerically integrate
    if i==1
        x(i+1,:) = x(i,:) + xdot*dt;
        r(i+1,:) = r(i,:) + rdot*dt;
        w(i+1,:) = w(i,:) + wdot*dt;
        v(i+1,:) = v(i,:) + vdot*dt;
    elseif i<points
        x(i+1,:) = x(i,:) + ( 1.5*xdot - 0.5*oldxdot )*dt;
        r(i+1,:) = r(i,:) + ( 1.5*rdot - 0.5*oldrdot )*dt;
        w(i+1,:) = w(i,:) + ( 1.5*wdot - 0.5*oldwdot )*dt;
        v(i+1,:) = v(i,:) + ( 1.5*vdot - 0.5*oldvdot )*dt;
    end
    oldxdot = xdot; 
    oldrdot = rdot;
    oldwdot = wdot; 
    oldvdot = vdot;
end