% INS EKF dynamic model
% Written by: Kana Nagai 2022/3/20

%% Setup
clc;clear;beep off; close all
addpath('../Models/function');
set(0, 'DefaultLineLineWidth', 2);
set(0,'defaultAxesFontSize',15);

%% Input
getinput
IMUType = 'STIM300';

%% IMU Error Drift with EKF

% mesurement steps (INS 20Hz, 1 hr)
m = 20*60*60;

% perfect initial Pbar
Pbar = zeros(15);

% INS specification
[tau_a, tau_g, sqrtQa, sqrtQg, sig_na, sig_ng, sig_ba0, sig_bg0] = inputIMUType(IMUType);
 
% Dynamic model (constant velocity)
[phi,gamaWgamaT] = getGNSS_Dynamic(tau_a,tau_g,sig_na,sig_ng,sqrtQa,sqrtQg);

% the EKF
    for i = 1:m
    
        % save position data
        save_r(i,1) = Pbar(1,1)^0.5;        
        
        Phat = Pbar;
        Pbar = phi*Phat*phi'+ gamaWgamaT;
    
    end

%% Plot *******************************************************************
%% 
% position error comparison *approx*. 20sec

figure
plot(1:20*20,save_r(1:20*20,1))
hold on
grid on
xlim([0,20*20])
xlabel('time (s)')
ylabel('position error change (m, 1\sigma)')
legend({'STIM300 EKF'},...
    'Location','northwest','NumColumns',3)
xticks([0,100,200,300,400])
xticklabels({'0','5','10','15','20'})

