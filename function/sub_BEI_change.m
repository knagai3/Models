
% -------------------------------------------------------------------------
% - handle satellite change for each constellation
% - judge the satellite is whetehr in or out
% -------------------------------------------------------------------------

%% count number of satellites

[temp_prn_old,temp_prn_temp,temp_prn_both,nSV_GPS,nSV_GLO,nSV_GAL,nSV_BEI] = ...
           count_satellites(prn_old4, prn_temp4, prn_temp1, prn_temp2, prn_temp3, prn_old4);

%% satellite out

if length(temp_prn_both)<length(temp_prn_old)

    if length(temp_prn_both)<2
    sub_BEI_constellation_out
    else
    sub_BEI_satellite_out
    end

end

%% satellite in

if length(temp_prn_both)<length(temp_prn_temp)
    
    if length(temp_prn_both)<2
    sub_BEI_constellation_in
    else
    sub_BEI_satellite_in
    end 
    
end

clear temp_prn_old temp_prn_temp temp_prn_both nSV_GPS nSV_GLO nSV_GAL nSV_BEI nSV_GNSS

%% function

function [temp_prn_old,temp_prn_temp,temp_prn_both,nSV_GPS,nSV_GLO,nSV_GAL,nSV_BEI] = ...
                count_satellites(prn_old, prn_temp, GPS, GLO, GAL, BEI)

temp_prn_old = prn_old;
temp_prn_temp = prn_temp;
temp_prn_both = intersect(temp_prn_old, temp_prn_temp);

nSV_GPS = length(GPS);
nSV_GLO = length(GLO);
nSV_GAL = length(GAL);
nSV_BEI = length(BEI);

end