clear all
clc
addpath("..\");
ConstStruct = load("ConstFile.mat");
%Va = 38.2, gamma = 0 gives approx zero res.
Va = 35;
gamma = 0;

[res, deltaT,deltaE,u_init,w_init,theta_init] =  trim(Va,gamma, ConstStruct);
res