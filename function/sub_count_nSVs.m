% find number of satellites
if isempty(vis_temp) == 1
prn_temp = [];
else
prn_temp = vis_temp(:,4);
end

% sort satellites into each constellation
[prn_temp1, prn_temp2, prn_temp3, prn_temp4] = sort_GNSS(prn_temp);

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

nSVs = length(prn_temp1)+length(prn_temp2)+length(prn_temp3)+length(prn_temp4);

A = [isempty(prn_temp1) isempty(prn_temp2) isempty(prn_temp3) isempty(prn_temp4)];
idx = A==0;
num_con=sum(idx(:));

%%
function [b1, b2, b3, b4] = sort_GNSS(a)

b1 = a(a > 0 & a < 25,:);
b2 = a(a > 37 & a < 62,:);
b3 = a(a > 74 & a < 99,:);
b4 = a(a > 169 & a < 201,:);

end