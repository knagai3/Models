% ZUPT EKF Measurment Model
% Written by: Kana Nagai 2022/3/18

%% Setup
clc;clear;beep off;close all
addpath('../Models/function');
set(0, 'DefaultLineLineWidth', 2);
set(0,'defaultAxesFontSize',15);

%% Input
getinput
IMUType = 'STIM300';

%% INS/ZUPT Error Drift with EKF ******************************************

% mesurement time = frequency*time steps
m = 20*20;

% perfect initial Pbar
Pbar = zeros(15);

% INS setting
[tau_a, tau_g, sqrtQa, sqrtQg, sig_na, sig_ng, sig_ba0, sig_bg0] = inputIMUType(IMUType);

% Dynamic model (constant velocity)
[phi,gamaWgamaT] = getGNSS_Dynamic(tau_a,tau_g,sig_na,sig_ng,sqrtQa,sqrtQg);

% ZUPD measurment model        
bRn = eye(3);
v_b = [0;0;0];
H(1:3,:) = [zeros(3,3) bRn skew(v_b) zeros(3,6)];
% violation error
V = 1e-3^2*eye(3);

% update time
t_up = 15;

%% the EKF

for i = 1:m

save_Pbar(i,1) = sqrt(Pbar(1,1));

    if i < 20*t_up % without update
    Phat = Pbar;

    else % ZUPD
    L = Pbar*H'/(V + H*Pbar*H');
    Phat = (eye(size(L,1))-L*H)*Pbar;
    end

Pbar = phi*Phat*phi'+ gamaWgamaT;

end

%% plot *******************************************************************
%%

figure
plot(1:m,save_Pbar)
hold on
xlabel('time steps (20 Hz)')
ylabel('error change (m)')

