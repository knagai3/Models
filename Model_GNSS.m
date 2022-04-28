%% Setup
clc;clear;beep off;close all
addpath('../Models/constell');
addpath('../Models/function');
set(0, 'DefaultLineLineWidth', 2);
set(0,'defaultAxesFontSize',15);


%% Input
getinput
IMUType = 'STIM300';


%%  GNSS Initialization with the EKF

% get GNSS almanac data
[vis_alm, time_index] = getALM(1);

% get initialization
[Pbar, save0] = getGNSS_Ini(IMUType,vis_alm,time_index);

% get alignmnet
[Pbar, save1] = getGNSS_Ali(Pbar,IMUType,vis_alm,time_index);


%% Plot *******************************************************************

figure()
save_all = [save0(:,1:2);save1(:,1:2)];
plot(1:length(save_all),save_all);
ylim([0,0.1])

figure()
save_all = [save0(:,3);save1(:,3)];
plot(1:length(save_all),save_all);

