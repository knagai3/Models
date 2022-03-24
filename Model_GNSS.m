% GNSS EKF measurement
% Written by: Kana Nagai 2022/3/20

%% Setup
clc;clear;beep off; close all
addpath('../Models/function');
addpath('../Models/constell');
set(0, 'DefaultLineLineWidth', 2);
set(0,'defaultAxesFontSize',15);

%% Input
getinput
IMUType = 'STIM300';

%%

% get GNSS almanac data
[vis_alm, time_index] = getALM(1);

% get initialization
[Pbar, save0] = getGNSS_Ini(vis_alm,time_index,IMUType);

% get alignmnet
[Pbar,save1] = getGNSS_Ali(Pbar,vis_alm,time_index,IMUType);

%% Plot *******************************************************************

figure
plot(1:length(save0),save0);
ylim([0,0.1])

