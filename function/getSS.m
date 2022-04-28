function [H,V,GammaV,Ro] = getSS(IMUType,nSVs,num_con,v)

% [H,GammaV,Ro] = getSS(IMUType,nSVs,num_con,v)

% -------------------------------------------------------------------------
% construct WSS measurment model
%
% Input:
% IMUType - IMU type
% nSVs - number of satellites
% num_con - number of constallation
% v - velocity [m/s]
% 
% Output: 
% H - msmt matrix
% GammaV - noise coeeficient
% Ro - measurement noise vector
%
% Written by: Kana Nagai 2022/3/25
% -------------------------------------------------------------------------

getinput

% Speed Sensor Parameters
% GPS reciever position (m)
l = [-1.5;0;1.5];

bRn = eye(3);
% radius of tire (m)
r0 = 0.325;
% gyro measurement PSD (rad/sqrt(s))
[~, ~, ~, sqrtQg, ~, ~, ~, ~] = inputIMUType(IMUType);
% speed sensor white noise (m/s)
sig_z = 0.05;
% compression white noise (m/s) 
sig_h = 0.05; % Kana applies arbitary number

% speed sensor measurement (rad/s)
omega_R = v/r0;
omega_L = v/r0;

% H matrix
L = skew(l);
H2 = [-omega_R/2 -omega_L/2; 0 0; 0 0];
H3 = zeros(3,3*nSVs-num_con);
H = [zeros(3) bRn skew([v 0 0])+L*bRn*skew(w_ie_e) zeros(3) L H2 H3];

%H = H(2:3,:);

% *************************************************************************
% error part
% *************************************************************************

GammaV = [L eye(3)];

% gyro measurement continuous PSD^2 (rad/sqrt(s))^2 to discrete (rad/s)^2 
v_g = sqrtQg^2/dt_ins*eye(3);

% speed sensor white noise (m/s)^2
v_z = sig_z^2;

% NHL constraint violation white noise (m/s)^2
zeta = sig_v^2;

% compression white noise (m/s)^2
v_h = sig_h^2;

% measurement noise
Ro = blkdiag(v_g,v_z,zeta,v_h);

% temporarily set noise matrix
V = GammaV*Ro*GammaV';

%V = V(2:3,2:3);