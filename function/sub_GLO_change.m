% *************************************************************************
% handle satellite chagne for GLO
% *************************************************************************

%% count number of satellites
temp_prn_old = prn_old2;
temp_prn_temp = prn_temp2;
prn_both2 = intersect(prn_old2, prn_temp2);
temp_prn_both = prn_both2;

nSV_GPS = length(prn_temp1);
nSV_GAL = length(prn_old3);
nSV_BEI = length(prn_old4);

if length(prn_temp1)<2
nSV_GPS = 0;
end

if length(prn_old3)<2
nSV_GAL = 0;
end

if length(prn_old4)<2
nSV_BEI = 0;
end

%% satellite out

if length(prn_both2)<length(prn_old2)
sub_GLO_out
clear nSV_GLO nSV_GNSS
end

%% satellite in

if length(prn_both2)<length(prn_temp2)
sub_GLO_in
clear temp_prn_old temp_prn_temp temp_prn_both nSV_GLO nSV_GAL nSV_BEI nSV_GNSS
end
