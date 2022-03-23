function [Fb, V_n, V_v, sig_bg0, sig_ba0] = getIMUNoise(IMUType)

[tau_a, tau_g, sqrtQa, sqrtQg, sig_na, sig_ng, sig_ba0, sig_bg0]...
                                                   = inputIMUType(IMUType);
       
% constructs IMU bias plant matrix and apply scaling
Fb = blkdiag((-1/tau_a)*eye(3),(-1/tau_g)*eye(3));

% Construct IMU bias noise covariance matrix 
V_n = blkdiag(sig_na^2*eye(3),sig_ng^2*eye(3));

% Construct IMU msmt white noise covariance matrix 
V_v = blkdiag(sqrtQa^2*eye(3),sqrtQg^2*eye(3));            

         



