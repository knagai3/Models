function [HH, V_rhophi] = getGNSS_Msmt(nSVs,num_con,A,vis_temp)

getinput

Gs = [];
m = [];
F = [];

for i = 1:4

if i==1 && A(1)==0 % GPS
gps_temp = vis_temp(vis_temp(:,4) > 0 & vis_temp(:,4) < 25,:);
los_temp = gps_temp(:,1:3);
[nSVs_temp,~] = size(gps_temp);
lam = 299792458/1575.42e6;

for j = 1:nSVs_temp-1 
G(j,:) = -(los_temp(1+j,:)-los_temp(1,:));
end

Gs = [Gs; G];

sd2dd = [-ones(nSVs_temp-1,1), eye(nSVs_temp-1)];
m = blkdiag(m,sd2dd);

f = lam*eye(nSVs_temp-1);
F = blkdiag(F,f);

elseif i==2 && A(2)==0 % GLO   
glo_temp = vis_temp(vis_temp(:,4) > 37 & vis_temp(:,4) < 62,:);
los_temp = glo_temp(:,1:3);
[nSVs_temp,~] = size(glo_temp);
lam = 299792458/1602e6;

for j = 1:nSVs_temp-1 
G(j,:) = -(los_temp(1+j,:)-los_temp(1,:));
end

Gs = [Gs; G];

sd2dd = [-ones(nSVs_temp-1,1), eye(nSVs_temp-1)];
m = blkdiag(m,sd2dd);

f = lam*eye(nSVs_temp-1);
F = blkdiag(F,f);

elseif i==3 && A(3)==0 % Gal
gal_temp = vis_temp(vis_temp(:,4) > 74 & vis_temp(:,4) < 99,:);
los_temp = gal_temp(:,1:3);
[nSVs_temp,~] = size(gal_temp);
lam = 299792458/1575.42e6;

for j = 1:nSVs_temp-1 
G(j,:) = -(los_temp(1+j,:)-los_temp(1,:));
end

Gs = [Gs; G];

sd2dd = [-ones(nSVs_temp-1,1), eye(nSVs_temp-1)];
m = blkdiag(m,sd2dd);

f = lam*eye(nSVs_temp-1);
F = blkdiag(F,f);

elseif i== 4 && A(4)==0 % i = 4 % Bei
bei_temp = vis_temp(vis_temp(:,4) > 169 & vis_temp(:,4) < 201,:);
los_temp = bei_temp(:,1:3);
[nSVs_temp,~] = size(bei_temp);
lam = 299792458/1575.42e6;

for j = 1:nSVs_temp-1 
G(j,:) = -(los_temp(1+j,:)-los_temp(1,:));
end

Gs = [Gs; G];

sd2dd = [-ones(nSVs_temp-1,1), eye(nSVs_temp-1)];
m = blkdiag(m,sd2dd);

f = lam*eye(nSVs_temp-1);
F = blkdiag(F,f);
end


clear G sd2dd f


end

Gs = [Gs;Gs];
M = blkdiag(m,m);
F = [F; zeros(nSVs-num_con)];

HH  = [Gs, zeros(size(Gs,1),12), M, F];

% initialize thermal noise covariace matrix
V_sd_carr_th = eye(nSVs)*sig_carr_th^2;
V_sd_code_th = eye(nSVs)*sig_code_th^2;

% obtains double-difference code and carrier msmt error covariances
% for diagonal terms -in the batch- only 
V_dd_carr_th = m*V_sd_carr_th*m';
V_dd_code_th = m*V_sd_code_th*m';

% Combine code&carrier diagonal terms of covariances diagonally 
V_rhophi = blkdiag(V_dd_carr_th,V_dd_code_th);