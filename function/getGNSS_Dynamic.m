function [phi,gamaWgamaT] = getGNSS_Dynamic(tau_a,tau_g,sig_na,sig_ng,sqrtQa,sqrtQg)

%% GETGNSS_DYNAMIC This function for daynamic model. 
% 
% Input:
% * tau_a - Time constant of accelerometer bias [s]
% * tau_g - Time constant of gyroscope bias [s]
% * sqrtQa - accelerometer power spectral densities [m/s^2/sqrt(Hz)]
% * sqrtQg - gyro power spectral densities [rad/s/sqrt(Hz)]
% * sig_na - 1-sigma accelerometer bias white noise [m/s^3/sqrt(Hz)]
% * sig_ng - 1-sigma gyro bias white noise [rad/s^2/sqrt(Hz)]
% 
% Output:
% * phi - state transit matrix in discrete time
% * gamaWgamaT - in discrete time
%
% Written by: Kana Nagai 1/11/22
% 

%%
getinput

% Get IMU
[Fn, Gu] = getIMUModel0;

% Get IMU Noise
% constructs IMU bias plant matrix and apply scaling
Fb = blkdiag((-1/tau_a)*eye(3),(-1/tau_g)*eye(3));
% Construct IMU msmt white noise covariance matrix 
V_v = blkdiag(sqrtQa^2*eye(3),sqrtQg^2*eye(3));  
% Construct IMU bias noise covariance matrix 
V_n = blkdiag(sig_na^2*eye(3),sig_ng^2*eye(3));

% Plant matrix
F = [Fn, -Gu; zeros(6,9), Fb];

% Compose an Augmented process noise covariance matrix
W = blkdiag(V_v, V_n);

% Noise coefficient matrix
Gw = blkdiag(-Gu, eye(6));

% Get discrete process parameters
phi = expm(F*dt_ins);
gamaWgamaT = getgamaWpgamaT(F, W, Gw, dt_ins);