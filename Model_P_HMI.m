
% calculate P_HMI at fault free situation

clc; clear; close all;

% inputs
sigma = 0.1;
l = 0.5;

% P_HMI model
P_HMI = 2*normcdf(-l/sigma);

% display the result
disp(P_HMI)
