

% *************************************************************************
% some satellites are in
% *************************************************************************
if length(prn_old1)>=2 && length(prn_both1)>=2
n = length(prn_temp1)-length(prn_both1);
contains = any(prn_old1(:) == prn_temp1(1));

for j = 1:n

% *************************************************************************
% number of satellites after one of them is in
% *************************************************************************        
nSV_GPS = length(temp_prn_old)+j;
nSV_GNSS = nSV_GPS+nSV_GLO+nSV_GAL+nSV_BEI;

% *************************************************************************
% the smallest prn index that will be in
% *************************************************************************   
[~, ia, ib] = intersect(temp_prn_both, temp_prn_temp);
prn_index = find(ia-ib, 1);
if ia==ib
prn_index = nSV_GPS;
end

% if isempty(prn_index)== 1
% prn_index = nSV_GPS;
% end 

% *************************************************************************
% step(1) constatnt part
% ************************************************************************* 
J1 = eye(15);

% *************************************************************************
% step(2) mp part
% *************************************************************************     
J2 = eye(nSV_GNSS);    
J2(:,prn_index) = [];

% *************************************************************************
% step(3) N part
% *************************************************************************
J3 = eye((length(Pbar)+3)-(15+2*(nSV_GNSS)));
if prn_index == 1
J3(:,prn_index) = [];
else
J3(:,prn_index-1) = [];
end

% *************************************************************************
% step(4) Pbar change
% *************************************************************************
J = blkdiag(J1,J2,J2,J3);
Pbar = J*Pbar*J';

% *************************************************************************
% step(5) replace
% *************************************************************************

getinput
Pbar(15+prn_index,15+prn_index) = sig_carr_mp^2;
Pbar(15+(nSV_GNSS)+prn_index,15+(nSV_GNSS)+prn_index) = sig_code_mp^2;
if prn_index == 1                                  
Pbar(15+2*(nSV_GNSS)+prn_index,15+2*(nSV_GNSS)+prn_index) = 1E6;
else
Pbar(14+2*(nSV_GNSS)+prn_index,14+2*(nSV_GNSS)+prn_index) = 1E6;
end

% *************************************************************************
% step(6) MS changes
% *************************************************************************
if prn_index == 1 && contains == 1
D = eye(length(Pbar));
D1 = -1*ones(nSV_GPS-1,1);
a = 15+2*(nSV_GNSS);
D(a+1:a+nSV_GPS-1,a+2) = D1;
Pbar = D*Pbar*D';
end

% update temp_prn_both
if prn_index ==1
temp_prn_both = [temp_prn_temp(prn_index); temp_prn_both];
else
temp_prn_both = [temp_prn_both(1:prn_index-1); temp_prn_temp(prn_index); temp_prn_both(prn_index:end)];    
end

end


if contains == 0
a1 = 15+2*(nSV_GNSS)+1;
a2 = 15+2*(nSV_GNSS)+length(prn_temp1)-1;
Pbar(a1:a2,:) = 0;
Pbar(:,a1:a2) = 0;
Pbar(a1:a2,a1:a2) = 1E6*eye(length(prn_temp1)-1);

end


% *************************************************************************
% when satellites return from no satellites
% *************************************************************************
else

nSV_GPS = 1;
nSV_GNSS = nSV_GPS+nSV_GLO+nSV_GAL+nSV_BEI;
prn_index = 1;
J1 = eye(15);
J2 = eye(nSV_GNSS);
J2(:,prn_index) = [];
J3 = eye((length(Pbar)+2)-(15+2*(nSV_GNSS)));
J = blkdiag(J1,J2,J2,J3);
Pbar = J*Pbar*J';                                              
getinput
Pbar(15+prn_index,15+prn_index) = sig_carr_mp^2;
Pbar(15+(nSV_GNSS)+prn_index,15+(nSV_GNSS)+prn_index) = sig_code_mp^2;
temp_prn_both = [temp_prn_temp(prn_index); temp_prn_both];

% nSV_GPS > 1
n = length(prn_temp1)-1;

for j = 1:n

% number of satellites after one of them is in
nSV_GPS = 1+j;
nSV_GNSS = nSV_GPS+nSV_GLO+nSV_GAL+nSV_BEI;

% the smallest prn index that will be in
[~, ia, ib] = intersect(temp_prn_both, temp_prn_temp); 
prn_index = find(ia-ib, 1);
if ia==ib
prn_index = nSV_GPS;
end

% Change Pbar size 
J1 = eye(15); 
J2 = eye(nSV_GNSS);    
J2(:,prn_index) = [];
J3 = eye((length(Pbar)+3)-(15+2*(nSV_GNSS)));
if prn_index == 1
J3(:,prn_index) = [];
else
J3(:,prn_index-1) = [];
end
J = blkdiag(J1,J2,J2,J3);
Pbar = J*Pbar*J';                                              

% getinput
Pbar(15+prn_index,15+prn_index) = sig_carr_mp^2;
Pbar(15+(nSV_GNSS)+prn_index,15+(nSV_GNSS)+prn_index) = sig_code_mp^2;
if prn_index == 1                                  
Pbar(15+2*(nSV_GNSS)+prn_index,15+2*(nSV_GNSS)+prn_index) = 1E6;
else
Pbar(14+2*(nSV_GNSS)+prn_index,14+2*(nSV_GNSS)+prn_index) = 1E6;
end

% MS changes
if prn_index == 1
D = eye(length(Pbar));
D1 = -1*ones(nSV_GPS-1,1);
a = 15+2*(nSV_GNSS);
D(a+1:a+nSV_GPS-1,a+2) = D1;
Pbar = D*Pbar*D';
end

if prn_index ==1
temp_prn_both = [temp_prn_temp(prn_index); temp_prn_both];
else
temp_prn_both = [temp_prn_both(1:prn_index-1); temp_prn_temp(prn_index); temp_prn_both(prn_index:end)];    
end
    
end



end % if length(prn_temp1)>1 && length(prn_old1)<2