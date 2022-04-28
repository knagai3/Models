function [Pbar, save0] = getGNSS_Ini(IMUType,vis_alm,time_index)

getinput
prn_old = [];
sensor = 2;

n = tG/dt_gps;

for i = 1:n

% vis_temp = [los_n(nx3), prn(nx1), el(nx1), az(nx1)]
vis_temp = vis_alm(time_index(i):time_index(i+1)-1,:);

sub_sort_constellation

    if i == 1
        
    Pbar = getIniKnow(IMUType,nSVs,num_con);
       
    else
    
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
% Consist EKF model

% Dynamic model (constant velocity)
[phi, gamaWgamaT, GamaW, Qo] = getGNSS_Dynamic(zeros(1,12), IMUType, nSVs, num_con);

% GNSS Measurement model
[H_GNSS, V_GNSS] = getGNSS_Msmt(nSVs,num_con,A,vis_temp);

% Speed Sensor Measurement Model
%[H_NHL, V_NHL] = getNHL(nSVs,num_con,0,[0;0;0]);
[H_SS,V_SS,GammaV,Ro] = getSS(IMUType,nSVs,num_con,0);

%% 
% EKF calculation

    for j = 1:dt_gps/dt_ins  

        save0(i,1) = sqrt(Pbar(1,1));
        save0(i,2) = sqrt(Pbar(2,2));
        save0(i,3) = rad2deg(sqrt(Pbar(9,9)));        
 
            if mod(j,dt_gps/dt_ins) == 0 

                for k = 1:sensor 

                    if k == 1 && nSVs~=0
                    % GNSS update    
                    H = H_GNSS;
                    V = V_GNSS;

                    else
                    % NHL update    
                    H = H_SS;
                    V = V_SS;
                    Pbar = Phat;

                    end

                L = Pbar*H'/(V + H*Pbar*H');
                Phat = (eye(size(L,1))-L*H)*Pbar*(eye(size(L,1))-L*H)'+ L*V*L';
                
                end

            else
                
            Phat = Pbar;

            end
        
        if sensor ==1    
        % GNSS update    
        Pbar = phi*Phat*phi'+ gamaWgamaT;
        
        else
        % WSS update
        % Solve Cross Correlation Issue
        Uo = zeros(length(Qo),length(Ro));
        [~, ~, ~, sqrtQg, ~, ~, ~, ~] = inputIMUType(IMUType);
        Uo(4:6,1:3) = sqrtQg^2;

        Q = gamaWgamaT;
        R = V_SS;
        U = GamaW*Uo*GammaV';

        phic = phi-U/R*H_SS;
        Qc = Q-U/R*U';
        
        Pbar = phic*Phat*phic'+ Qc;
        end
           
    end
  
%% 
% set for the next step
prn_old = prn_temp_bar;

end 

