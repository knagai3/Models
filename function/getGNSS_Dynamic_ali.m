function [phi,gamaWgamaT] = getGNSS_Dynamic_ali(vis_temp,n,IMUType)

getinput

prn_temp = vis_temp(:,4);
prn_temp1 = prn_temp(prn_temp > 0 & prn_temp < 25,:);
prn_temp2 = prn_temp(prn_temp > 37 & prn_temp < 62,:);
prn_temp3 = prn_temp(prn_temp > 74 & prn_temp < 99,:);
prn_temp4 = prn_temp(prn_temp > 169 & prn_temp < 201,:);

if length(prn_temp1)<2
prn_temp1 = [];
end

if length(prn_temp2)<2
prn_temp2 = [];
end

if length(prn_temp3)<2
prn_temp3 = [];
end

if length(prn_temp4)<2
prn_temp4 = [];
end

nSVs = length(prn_temp1)+length(prn_temp2)+length(prn_temp3)+length(prn_temp4);

A = [isempty(prn_temp1) isempty(prn_temp2) isempty(prn_temp3) isempty(prn_temp4)];
idx = A==0;
num_con=sum(idx(:));

% Get IMU
[Fn, Gu] = getIMUModel(n);

% Get Multipath Noise
[Fm, V_m] = getGPSnoise(nSVs);

% Get IMU Noise
[Fb, V_n, V_v, ~, ~] = getIMUNoise(IMUType);

% Plant matrix
F = [Fn, -Gu; zeros(6,9), Fb];
F = blkdiag(F, Fm, zeros(nSVs-num_con));

% Compose an Augmented process noise covariance matrix (W)
W = blkdiag(V_v, V_n, V_m);

% Noise coefficient matrix
Gw = [blkdiag(-Gu, eye(6) ,eye(2*nSVs)); zeros(nSVs-num_con, length(W))];

% Get discrete process parameters
phi = expm(F*dt_ins);
gamaWgamaT = getgamaWpgamaT(F, W, Gw, dt_ins);

