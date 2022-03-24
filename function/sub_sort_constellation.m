% find number of satellites
[nSVs,~] = size(vis_temp);
prn_temp = vis_temp(:,4);
prn_both = intersect(prn_old, prn_temp);

% sort satellites into each constellation
[prn_temp1, prn_temp2, prn_temp3, prn_temp4] = sort_GNSS(prn_temp);

% sort satellites change
[prn_old1, prn_old2, prn_old3, prn_old4] = sort_GNSS(prn_old);

% sort satellites do not change
[prn_both1, prn_both2, prn_both3, prn_both4] = sort_GNSS(prn_both);

% count the number of constellations
A = [isempty(prn_temp1) isempty(prn_temp2) isempty(prn_temp3) isempty(prn_temp4)];
idx = A==0;
num_con = sum(idx(:));
%%
function [b1, b2, b3, b4] = sort_GNSS(a)

b1 = a(a > 0 & a < 25,:);
b2 = a(a > 37 & a < 62,:);
b3 = a(a > 74 & a < 99,:);
b4 = a(a > 169 & a < 201,:);

end