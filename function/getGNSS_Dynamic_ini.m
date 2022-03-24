function [phi,gamaWgamaT,sig_bg0,sig_ba0] = getGNSS_Dynamic_ini(nSVs, IMUType, num_con)

getinput

% Get IMU
[Fn, Gu] = getIMUModel0;

% Get Multipath Noise
[Fm, V_m] = getGPSnoise(nSVs);

% Get IMU Noise
[Fb, V_n, V_v, sig_bg0, sig_ba0] = getIMUNoise(IMUType);

% Plant matrix
F = [Fn, -Gu; zeros(6,9), Fb];
F = blkdiag(F, Fm, zeros(nSVs-num_con));

% Compose an Augmented process noise covariance matrix
W = blkdiag(V_v, V_n, V_m);

% Noise coefficient matrix
Gw = [blkdiag(-Gu, eye(6) ,eye(2*nSVs)); zeros(nSVs-num_con, length(W))];

% Get discrete process parameters
phi = expm(F*dt_ins);
gamaWgamaT = getgamaWpgamaT(F, W, Gw, dt_ins);

