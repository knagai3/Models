function [vis_alm, time_index] = getALM(GNSS)

% Input:
% GNSS - GNSS = 0: evaluate GPS only, GNSS = 1: evaluate GNSS
% 
% Output: 
% * vis_alm - alm data
% * time_index - the time row for the alm data
%
% Written by: Kana Nagai 2022/1/30

%% Input values
getinput

% set start and end at UTC time
[t_start, t_end] = getsimtime(UTC_time, tG);

% change from the UTC time to GPS time
start_gps = utc2gps(t_start);
stop_gps = utc2gps(t_end);

%% Read Almanac data
% read .alm data and return only the visible satellites info.  
% vis_alm = [los_n (nx3), prn (nx1), el (nx1), az (nx1)] 
% GPS alm file

alm_file = 'almGPS.alm';
alm_2_use = readyuma(alm_file, 0);
alm_2_use = alm_2_use(alm_2_use(:,2) == 0,:);
ephem = alm2geph(alm_2_use);
gps_ephem = ephem;

%%  
if GNSS == 1
    
% glonass alm file
    clear alm_file alm_2_use ephem
    alm_file = 'almglonass.alm';
    alm_2_use = readyuma(alm_file, 0);
    alm_2_use = alm_2_use(alm_2_use(:,2) == 0,:);
    ephem = alm2geph(alm_2_use);
    gps_ephem = [gps_ephem; ephem];
% galileo alm file
    clear alm_file alm_2_use ephem
    alm_file = 'almgalileo24.alm';
    alm_2_use = readyuma(alm_file, 0);
    alm_2_use = alm_2_use(alm_2_use(:,2) == 0,:);
    ephem = alm2geph(alm_2_use);
    gps_ephem = [gps_ephem; ephem];
% Bei-do alm file
    clear alm_file alm_2_use ephem
    alm_file = 'almbeidou.alm';
    alm_2_use = readyuma(alm_file, 0);
    alm_2_use = alm_2_use(alm_2_use(:,2) == 0,:);
    ephem = alm2geph(alm_2_use);
    gps_ephem = [gps_ephem; ephem];   

end

%%
% Compute satellite positions in E frame for given times
[t_gps , prn_gps , x_gps] = propgeph(gps_ephem ,start_gps, stop_gps ,dt_gps);

% reference station position vector in E frame
R_e = lla2ecef(R_llh');

% LOS vectors in ECEF
[t_los_gps, gps_los, los_ind] = los(t_gps(1,:), R_e, t_gps, [prn_gps x_gps]);
                                  
% Convert LOS from E to N frame                            
gps_los_n = ecef2ned(gps_los,R_llh');

% find azimuth and elevation from los vectors
[az_los, el_los] = ned2azel(gps_los_n);

% Elevation mask[deg]
mask = deg2rad(5);

% eliminate SVs which are out of horizon limit (mask)
[~, I_pass] = vis_data(mask,[az_los el_los]);
prn_gps_los = prn_gps(los_ind(:, 2));

% Reset the data arrays to contain only visible data                                               
t = t_los_gps(I_pass,:);
los_n = normvect(gps_los_n(I_pass,:));
prn = prn_gps_los(I_pass,:);
el = el_los(I_pass,:);
az = az_los(I_pass,:);
vis_alm = [los_n, prn, el, az];

% find time index
GPS_time_restarts = find(diff(t(:,2))~=0);
GPS_time_restarts = GPS_time_restarts + ones(length(GPS_time_restarts),1);
time_index = [1; GPS_time_restarts];

