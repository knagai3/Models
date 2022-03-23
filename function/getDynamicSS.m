function [phi,GamaW,W,sig_bg0,sig_ba0,gamaWgamaT] = getDynamicSS(IMUType)

% *************************************************************************
% This is the dynamic model for speed sensor integration
% *************************************************************************

getinput

% Get IMU
[Fn, Gu] = getIMUModel0;

% Get IMU Noise
[Fb, V_n, V_v, sig_bg0, sig_ba0] = getIMUNoise(IMUType);

% Plant matrix
F = [Fn, -Gu; zeros(6,9), Fb];

% tire radius
F = blkdiag(F, zeros(2));

% Compose an Augmented process noise covariance matrix
Qw = blkdiag(V_v, V_n);

% Noise coefficient matrix
Gw = [blkdiag(-Gu, eye(6)); zeros(2, length(Qw))];

% Get discrete process parameters
phi = expm(F*dt_ins);

gamaWgamaT = getgamaWpgamaT(F, Qw, Gw, dt_ins);

GamaW = Gw*dt_ins;

W = Qw/dt_ins;


