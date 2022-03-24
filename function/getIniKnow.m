function Pbar_0 = getIniKnow(nSVs,IMUType,num_con)
%% 
% consist initial knowledge matrix

getinput

[~,~,~,~,~,~,sig_ba0,sig_bg0] = inputIMUType(IMUType);

big = 1e+6;
rbar_0 = big*eye(3);
vbar_0 = big*eye(3);
Ebar_0 = deg2rad(6.5)^2*eye(3);
babar_0 = sig_ba0^2*eye(3);
bgbar_0 = sig_bg0^2*eye(3);
mbar_code_0 = sig_code_mp^2*eye(nSVs);
mbar_carr_0 = sig_carr_mp^2*eye(nSVs);
Nbar_0 = big*eye(nSVs-num_con);
Pbar_0 = blkdiag(rbar_0, vbar_0, Ebar_0, babar_0, bgbar_0, mbar_carr_0, mbar_code_0, Nbar_0);