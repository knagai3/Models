function [H,V] = getNHL(nSVs,num_con,v,E_n)

% -------------------------------------------------------------------------
% construct NHL measurment model
%
% Input:
% nSVs - number of satellites
% num_con - number of constallation
% v - velocity [m/s]
% 
% Output: 
% H - msmt matrix
% V - msmt noise
%
% Written by: Kana Nagai 2022/4/5
% -------------------------------------------------------------------------

getinput

% ZUPD measurment model  
[~, bRn, ~, ~] = getTransfermatrix(E_n, R_llh);
% bRn = eye(3);
v_b = [v;0;0];
H(1:3,:) = [zeros(3,3) bRn skew(v_b) zeros(3,6) zeros(3,2) zeros(3,3*nSVs-num_con)];
H = H(2,:);

% violation error
V = 1e-3^2;





