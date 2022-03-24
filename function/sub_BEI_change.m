% *************************************************************************
% handle satellite chagne for BEI
% *************************************************************************

%% count number of satellites

temp_prn_old = prn_old4;
temp_prn_temp = prn_temp4;
prn_both4 = intersect(prn_old4, prn_temp4);
temp_prn_both = prn_both4;

nSV_GPS = length(prn_temp1);
nSV_GLO = length(prn_temp2);
nSV_GAL = length(prn_temp3);

if length(prn_temp1)<2
nSV_GPS = 0;
end

if length(prn_temp2)<2
nSV_GLO = 0;
end

if length(prn_temp3)<2
nSV_GAL = 0;
end

%% satellite out

if length(prn_both4)<length(prn_old4)
sub_BEI_out
clear nSV_BEI nSV_GNSS
end

%% satellite in

if length(prn_both4)<length(prn_temp4)
sub_BEI_in
clear temp_prn_old temp_prn_temp temp_prn_both nSV_GNSS    
end

