
prn_temp = vis_temp(:,4);

% sort satellites old
[prn_old1, prn_old2, prn_old3, prn_old4] = sort_GNSS(prn_old);

% sort satellites into each constellation
[prn_temp1, prn_temp2, prn_temp3, prn_temp4] = sort_GNSS(prn_temp);

% erase the satellite number is one
if length(prn_temp1)<2
prn_temp1 = [];
end

if length(prn_temp2)<2
prn_temp2 = [];
end

if length(prn_temp3)<2
prn_temp3 = [];
end

if length(prn_temp4)<2
prn_temp4 = [];
end

% find number of satellites
prn_temp_bar = [prn_temp1; prn_temp2; prn_temp3; prn_temp4];
nSVs = length(prn_temp_bar);

% count the number of constellations
A = [isempty(prn_temp1) isempty(prn_temp2) isempty(prn_temp3) isempty(prn_temp4)];
idx = A==0;
num_con = sum(idx(:));

% find satellites do not change
prn_both = intersect(prn_old, prn_temp_bar);
[prn_both1, prn_both2, prn_both3, prn_both4] = sort_GNSS(prn_both);


%% function to sort constellation by prn
function [b1, b2, b3, b4] = sort_GNSS(a)

b1 = a(a > 0 & a < 25,:);
b2 = a(a > 37 & a < 62,:);
b3 = a(a > 74 & a < 99,:);
b4 = a(a > 169 & a < 201,:);

end
