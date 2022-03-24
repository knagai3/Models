% *************************************************************************
% handle satellite change for GPS
% *************************************************************************

%% count number of satellites
temp_prn_old = prn_old1;
temp_prn_temp = prn_temp1;
prn_both1 = intersect(prn_old1, prn_temp1);
temp_prn_both = prn_both1;

nSV_GLO = length(prn_old2);
nSV_GAL = length(prn_old3);
nSV_BEI = length(prn_old4);

if length(prn_old2)<2
nSV_GLO = 0;
end

if length(prn_old3)<2
nSV_GAL = 0;
end

if length(prn_old4)<2
nSV_BEI = 0;
end

%% satellite out

if length(prn_both1)<length(prn_old1)
sub_GPS_out
clear nSV_GPS nSV_GNSS
end

%% satellite in

if length(prn_both1)<length(prn_temp1)
sub_GPS_in
end
clear temp_prn_old temp_prn_temp temp_prn_both nSV_GLO nSV_GAL nSV_BEI nSV_GNSS
