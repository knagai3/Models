function [Pbar,save1] = getGNSS_Ali(Pbar,vis_alm,time_index,IMUType)

getinput
prn_old = [];
n = tI/dt_gps;

for i = 1:n

% fix GNSS satellite geometry at the end time
% vis_temp = [los_n(nx3), prn(nx1), el(nx1), az(nx1)]
vis_temp = vis_alm(time_index(tG/dt_gps):time_index(tG/dt_gps+1)-1,:);

sub_sort_constellation

% Dynamic model
[phi,gamaWgamaT] = getGNSS_Dynamic_ali(vis_temp,i,IMUType);

% Msmt model    
[HH, V_rhophi] = getGNSS_Msmt(nSVs,num_con,A,vis_temp);

%% the EKF

    for k = 1:dt_gps/dt_ins
         
        save1(i,1) = sqrt(Pbar(1,1));
        save1(i,2) = sqrt(Pbar(2,2)); 
        
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
prn_old = prn_temp;
    
end