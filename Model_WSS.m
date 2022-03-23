% INS/WSS/VDC EKF Measurment Model
% Written by: Kana Nagai 2022/3/18

%% Setup
clc;clear;beep off; close all
addpath('../Models/function');
set(0, 'DefaultLineLineWidth', 2);
set(0,'defaultAxesFontSize',15);

%% Input
getinput
IMUType = 'STIM300';

%% INS/WSS/VDC Error Drift with EKF ***************************************

% perfect initial Pbar
Pbar = blkdiag(zeros(15), 1e+6*eye(2));

% mesurement time = (frequency)(sec)
m = 20*120;

% Dynamic model (constant velocity)
[phi,GamaW,Qo,~,~,gamaWgamaT] = getDynamicSS(IMUType);

% Speed Sensor Measurement Model
[H1,GammaV,Ro] = getSS(IMUType,5);

% Solve Cross Correlation Issue
Uo = zeros(length(Qo),length(Ro));
[~, ~, ~, sqrtQg, ~, ~, ~, ~] = inputIMUType(IMUType);
Uo(4:6,1:3) = sqrtQg^2;

Q = gamaWgamaT;
R = GammaV*Ro*GammaV';
U = GamaW*Uo*GammaV';

phic = phi-U/R*H1;
Qc = Q-U/R*U';

% the EKF

GPS_update = dt_gps/dt_ins;

for i = 1:m
    
% save position
save_Pbar(i,1) = sqrt(Pbar(1,1));
save_Pbar(i,2) = sqrt(Pbar(2,2));

if mod(i+GPS_update-1,GPS_update) == 0
H = H1;
V = R;
L = Pbar*H'/(V + H*Pbar*H');
Phat = (eye(size(L,1))-L*H)*Pbar*(eye(size(L,1))-L*H)'+L*V*L';
else
Phat = Pbar;
end

Pbar = phic*Phat*phic'+ Qc;

end


%% plot *******************************************************************
%%
% plot everything

figure
plot(1:m,save_Pbar)
hold on; grid on
ylim([0 0.12])
xlabel('time (s)')
ylabel('position error change (m, 1\sigma)')
legend({'STIM300 + WSS','STIM300 + NHL'},...
    'Location','southeast','NumColumns',3)
set(gcf,'position',[0,0,700,300])
set(gca,...
   'XTickLabel',{'0','10','20','30','40','50','60','70','80','90','100'},...
    'XTick',[0 10*20 20*20 30*20 40*20 50*20 60*20 70*20 80*20 90*20 100*20])

