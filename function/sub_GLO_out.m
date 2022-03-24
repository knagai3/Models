
% *************************************************************************
% some satellites are out
% *************************************************************************

if length(prn_temp2)>=2 && length(prn_both2)>=2
n = length(prn_old2)-length(prn_both2);

for j = 1:n
nSV_GLO = length(prn_old2)+1-j;
nSV_GNSS = nSV_GPS+nSV_GLO+nSV_GAL+nSV_BEI;
[~, ia, ib] = intersect(temp_prn_both, temp_prn_old);
prn_index = find(ia-ib, 1);

if ia==ib
prn_index = length(temp_prn_both)+1;
end

if prn_index == 1
D = eye(length(Pbar));
D1 = -1*ones(nSV_GLO-1,1);
if nSV_GPS == 0
a = 15+2*(nSV_GNSS);
else    
a = 15+2*(nSV_GNSS)+(nSV_GPS-1);
end
D(a+1:a+nSV_GLO-1,a+1) = D1;
Pbar = D*Pbar*D';
end
Pbar1 = Pbar;
J1 = eye(15);
J2 = eye(nSV_GNSS);
prn_index1 = nSV_GPS+prn_index;
J2(prn_index1,:) = [];
J3 = eye(length(Pbar)-(15+2*(nSV_GNSS)));
if nSV_GPS == 0
prn_index2 = prn_index;
else    
prn_index2 = (nSV_GPS-1)+prn_index;
end
if prn_index == 1
J3(prn_index2,:) = [];
else
J3(prn_index2-1,:) = [];
end
J = blkdiag(J1,J2,J2,J3);
Pbar = J*Pbar*J';

temp_prn_old(prn_index) = [];  

end

% *************************************************************************
% when all satellites are out
% *************************************************************************
else
n = length(prn_old2)-1;

for j = 1:n

% number of satellites before one of them is out
nSV_GLO = length(prn_old2)+1-j;
nSV_GNSS = nSV_GPS+nSV_GLO+nSV_GAL+nSV_BEI;

% the smallest prn index that will be out
temp_prn_both = temp_prn_old(1);
[~, ia, ib] = intersect(temp_prn_both, temp_prn_old);
prn_index = find(ia-ib, 1);
if ia==ib
prn_index = length(temp_prn_both)+1;
end

% Change Pbar size   
J1 = eye(15);                                   
J2 = eye(nSV_GNSS);
prn_index1 = nSV_GPS+prn_index;
J2(prn_index1,:) = [];    
J3 = eye(length(Pbar)-(15+2*(nSV_GNSS)));
if nSV_GPS == 0
prn_index2 = prn_index;
else    
prn_index2 = (nSV_GPS-1)+prn_index;
end
if prn_index == 1
J3(prn_index2,:) = [];
else
J3(prn_index2-1,:) = [];
end    
J = blkdiag(J1,J2,J2,J3);
Pbar = J*Pbar*J';

% update temp_prn_old    
temp_prn_old(prn_index) = [];

end

nSV_GLO = 1;
nSV_GNSS = nSV_GPS+nSV_GLO+nSV_GAL+nSV_BEI;
prn_index = 1; 
J1 = eye(15);                                 
J2 = eye(nSV_GNSS);
prn_index1 = nSV_GPS+prn_index;
J2(prn_index1,:) = [];
J3 = eye(length(Pbar)-(15+2*(nSV_GNSS))); 
J = blkdiag(J1,J2,J2,J3);
Pbar = J*Pbar*J';
temp_prn_old(prn_index) = [];

end