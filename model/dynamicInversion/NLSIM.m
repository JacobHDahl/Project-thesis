% NLSIM.M Nonlinear Simulation
clear all
% global % add variables as needed
name= input('Enter Name of State Equations m-file : ','s');
icfile= input('Enter Name of i.c. File : ','s');
tmp= dlmread(icfile,',');
n=tmp(1); m=tmp(2);
x=tmp(3:n+2); u=tmp(n+3:n+m+2);
stat=fclose('all');
runtime= input('Enter Run-Time : ');
dt = input('Enter Integration Time-step : ');
N=runtime/dt; k=0; NP= fix(max(1,N/500)); time=0.;
xd= feval(name,time,x,u); % Set variables in state equations
%save=u(2); % For Example 3.6-3 only
for i=0:N
time=i*dt;
if rem(i,NP)==0
k=k+1;
y(k,1)= x(1); % record data as needed
y(k,2)= x(2);
%y(k,3)=
end
%if time>=2 % For Example 3.6-3
% u(2)=save;
%elseif time>=1.5
% u(2)=save-2;
%elseif time>=1.0
% u(2)=save+2;
%else
% u(2)=save;
%end
[x]= RK4(name,time,dt,x,u);
end
t= NP*dt*[0:k-1];
figure(1)
plot(y(:,1), y(:,2)) % For Van der Pol
grid on
axis([-3,3,-4,5])
xlabel('X(1)')
ylabel('X(2)')
text(-1.8,3.2,'(-2,3)')