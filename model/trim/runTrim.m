clear all
clc
addpath("C:\Users\jacob\Documents\MATLAB\Project-thesis\model");
%Va = 38.2, gamma = 0 gives approx zero res.
Va = 38.2; 
gamma = 0;

[res, deltaT,deltaE,u_init,w_init,theta_init] =  trim(Va,gamma);
res