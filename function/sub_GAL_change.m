% *************************************************************************
% handle satellite chagne for GAL
% *************************************************************************

%% count number of satellites

temp_prn_old = prn_old3;
temp_prn_temp = prn_temp3;
prn_both3 = intersect(prn_old3, prn_temp3);
temp_prn_both = prn_both3;

nSV_GPS = length(prn_temp1);
nSV_GLO = length(prn_temp2);
nSV_BEI = length(prn_old4);

if length(prn_temp1)<2
nSV_GPS = 0;
end

if length(prn_temp2)<2
nSV_GLO = 0;
end

if length(prn_old4)<2
nSV_BEI = 0;
end

%% satellite out

if length(prn_both3)<length(prn_old3)
sub_GAL_out
clear nSV_GAL nSV_GNSS    
end

%% satellite in

if length(prn_both3)<length(prn_temp3)
sub_GAL_in
clear temp_prn_old temp_prn_temp temp_prn_both nSV_BEI nSV_GNSS
end
