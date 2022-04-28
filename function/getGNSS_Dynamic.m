function [phi, gamaWgamaT, GamaW, Wc] = getGNSS_Dynamic(input, IMUType, nSVs, num_con)

% -------------------------------------------------------------------------
% generates linearized Dynamic model of INS/GNSS/WSS integration
% input (1x9) = [v_b(xyz) a_b(xyz) E_n(xyz) Edotn(xyz)]
% -------------------------------------------------------------------------

getinput

% Get IMU
[Fn, Gu] = getIMUModel(input);

% Get Multipath Noise
[Fm, V_m] = getGPSnoise(nSVs);

% Get IMU Noise
[Fb, V_n, V_v, ~, ~] = getIMUNoise(IMUType);

% Plant matrix
% INS plant matrix
F = [Fn, -Gu; zeros(6,9), Fb];

% combine INS, WSS, GNSS plant matrix
% 9 PVA + 6 INS bias + 2 WSS + rest GNSS MP and N
F = blkdiag(F, zeros(2), Fm, zeros(nSVs-num_con));

% combine process noise covariance matrix (accel., gyro, and MP noise)
W = blkdiag(V_v, V_n, V_m);

% Noise coefficient matrix
Gw = blkdiag(-Gu, eye(6) ,eye(2*nSVs));
Gw = [Gw(1:15,:); zeros(2,length(W)); Gw(16:end,:); zeros(nSVs-num_con,length(W))];

% Get discrete process parameters
phi = expm(F*dt_ins);

gamaWgamaT = getgamaWpgamaT(F, W, Gw, dt_ins);

% some error parts for the correlation issue of WSS
GamaW = Gw*dt_ins;

Wc = W/dt_ins;


