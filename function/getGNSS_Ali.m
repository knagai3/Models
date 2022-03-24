function [Pbar,save1] = getGNSS_Ali(Pbar,vis_alm,time_index,IMUType)
%% 
% 

getinput
prn_old = [];
n = tI/dt_gps;

for i = 1:n

% vis_temp = [los_n(nx3), prn(nx1), el(nx1), az(nx1)]
vis_temp = vis_alm(time_index(tG/dt_gps+i):time_index(tG/dt_gps+i+1)-1,:);

sub_sort_constellation

if i > 1
    
    if length(prn_both1)~=length(prn_old1) || length(prn_both1)~=length(prn_temp1)
    sub_GPS_change
    end

    if length(prn_both2)~=length(prn_old2) || length(prn_both2)~=length(prn_temp2)
    sub_GLO_change
    end

    if length(prn_both3)~=length(prn_old3) || length(prn_both3)~=length(prn_temp3)
    sub_GAL_change
    end

    if length(prn_both4)~=length(prn_old4) || length(prn_both4)~=length(prn_temp4)
    sub_BEI_change
    end

end
%% 
% count nSVs

sub_count_nSVs
%% 
% EKF model

% Dynamic model
[phi,gamaWgamaT] = getGNSS_Dynamic_ali(vis_temp,i,IMUType);

% Msmt model    
[HH, V_rhophi] = getGNSS_Msmt(nSVs,num_con,A,vis_temp);
%% 
% EKF

    for k = 1:dt_gps/dt_ins
         
        save1(i,1) = Pbar(1,1);
        
        if mod(k,dt_gps/dt_ins) == 0 && nSVs~=0
        H = HH;
        V = V_rhophi;
        L = Pbar*H'/(V + H*Pbar*H');
        Phat = (eye(size(L,1))-L*H)*Pbar*(eye(size(L,1))-L*H)'+L*V*L';
        else
        Phat = Pbar;
        end
    
    Pbar = phi*Phat*phi'+ gamaWgamaT;
    
    end
%% 
% 

prn_old = prn_temp;
    
end