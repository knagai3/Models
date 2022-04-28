function [Pbar,save1] = getGNSS_Ali(Pbar,IMUType,vis_alm,time_index)

getinput
prn_old = [];
sensor = 2;

n = tI/dt_gps;

for i = 1:n

% fix GNSS satellite geometry at the end time
% vis_temp = [los_n(nx3), prn(nx1), el(nx1), az(nx1)]
vis_temp = vis_alm(time_index(tG/dt_gps):time_index(tG/dt_gps+1)-1,:);

% sort SVs
sub_sort_constellation

%% 
% Consist EKF model
load('motion_z.mat')

% input = [v_b(xyz) a_b(xyz) E_n(xyz) Edotn(xyz)]
input = [v_b(:,i)' a_b(:,i)' E_n(:,i)' Edot_n(:,i)'];

% Dynamic model (changing velocity)
[phi, gamaWgamaT, GamaW, Qo] = getGNSS_Dynamic(input, IMUType, nSVs, num_con);

% GNSS Measurement model
[H_GNSS, V_GNSS] = getGNSS_Msmt(nSVs,num_con,A,vis_temp);

% Speed Sensor Measurement Model
%[H_NHL, V_NHL] = getNHL(nSVs,num_con,v_b(1,i),E_n(:,i));
[H_SS,V_SS,GammaV,Ro] = getSS(IMUType,nSVs,num_con,v_b(1,i));

%% 
% EKF calculation

    for j = 1:dt_gps/dt_ins  

        save1(i,1) = sqrt(Pbar(1,1));
        save1(i,2) = sqrt(Pbar(2,2));
        save1(i,3) = rad2deg(sqrt(Pbar(9,9)));        
 
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
