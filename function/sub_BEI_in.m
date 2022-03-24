% *************************************************************************
% some satellites are in
% *************************************************************************
if length(prn_old4)>=2 && length(prn_both4)>=2
n = length(prn_temp4)-length(prn_both4);
contains = any(prn_old4(:) == prn_temp4(1));

for j = 1:n

% *************************************************************************
% number of satellites after one of them is in
% *************************************************************************        
nSV_BEI = length(temp_prn_old)+j;    
nSV_GNSS = nSV_GPS+nSV_GLO+nSV_GAL+nSV_BEI;

% *************************************************************************
% the smallest prn index that will be in
% *************************************************************************    
[~, ia, ib] = intersect(temp_prn_both, temp_prn_temp); 
prn_index = find(ia-ib, 1);
if ia==ib
prn_index = nSV_BEI;
end

if prn_index ==1
temp_prn_both = [temp_prn_temp(prn_index); temp_prn_both];
else
temp_prn_both = [temp_prn_both(1:prn_index-1); temp_prn_temp(prn_index); temp_prn_both(prn_index:end)];    
end

% *************************************************************************
% step(1) constatnt part
% *************************************************************************   
J1 = eye(15); 
% *************************************************************************
% step(2) mp part
% *************************************************************************                                  
J2 = eye(nSV_GNSS);   
prn_index1 = nSV_GPS+nSV_GLO+nSV_GAL+prn_index;
J2(:,prn_index1) = [];
% *************************************************************************
% step(3) N part
% ***********************************************************************
J3 = eye((length(Pbar)+3)-(15+2*(nSV_GNSS)));
if nSV_GPS == 0 && nSV_GLO == 0 && nSV_GAL == 0
prn_index2 = prn_index;
elseif nSV_GPS == 0 && nSV_GLO == 0 && nSV_GAL ~= 0
prn_index2 = (nSV_GAL-1)+prn_index;
elseif nSV_GPS ~= 0 && nSV_GLO ~= 0 && nSV_GAL == 0
prn_index2 = (nSV_GPS-1+nSV_GLO-1)+prn_index;
elseif nSV_GPS == 0 && nSV_GLO ~= 0 && nSV_GAL == 0
prn_index2 = (nSV_GLO-1)+prn_index;
elseif nSV_GPS ~= 0 && nSV_GLO == 0 && nSV_GAL ~= 0
prn_index2 = (nSV_GPS-1+nSV_GAL-1)+prn_index;
elseif nSV_GPS ~= 0 && nSV_GLO == 0 && nSV_GAL == 0
prn_index2 = (nSV_GPS-1)+prn_index;
elseif nSV_GPS == 0 && nSV_GLO ~= 0 && nSV_GAL ~= 0
prn_index2 = (nSV_GLO-1+nSV_GAL-1)+prn_index;
else    
prn_index2 = (nSV_GPS-1+nSV_GLO-1+nSV_GAL-1)+prn_index;
end
if prn_index == 1
J3(:,prn_index2) = [];
else
J3(:,prn_index2-1) = [];
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
Pbar(15+prn_index1,15+prn_index1) = sig_carr_mp^2;
Pbar(15+(nSV_GNSS)+prn_index1,15+(nSV_GNSS)+prn_index1) = sig_code_mp^2;
if prn_index == 1                                  
Pbar(15+2*(nSV_GNSS)+prn_index2,15+2*(nSV_GNSS)+prn_index2) = 1E6;
else
Pbar(14+2*(nSV_GNSS)+prn_index2,14+2*(nSV_GNSS)+prn_index2) = 1E6;
end

% *************************************************************************
% step(6) MS changes
% *************************************************************************
if prn_index == 1 && contains == 1
D = eye(length(Pbar));
D1 = -1*ones(nSV_BEI-1,1);
if nSV_GPS == 0 && nSV_GLO == 0 && nSV_GAL == 0
a = 15+2*(nSV_GNSS);
elseif nSV_GPS == 0 && nSV_GLO == 0 && nSV_GAL ~= 0
a = 15+2*(nSV_GNSS)+(nSV_GAL-1);
elseif nSV_GPS ~= 0 && nSV_GLO ~= 0 && nSV_GAL == 0
a = 15+2*(nSV_GNSS)+(nSV_GPS-1+nSV_GLO-1);
elseif nSV_GPS == 0 && nSV_GLO ~= 0 && nSV_GAL == 0
a = 15+2*(nSV_GNSS)+(nSV_GLO-1);
elseif nSV_GPS ~= 0 && nSV_GLO == 0 && nSV_GAL ~= 0
a = 15+2*(nSV_GNSS)+(nSV_GPS-1+nSV_GAL-1);
elseif nSV_GPS ~= 0 && nSV_GLO == 0 && nSV_GAL == 0
a = 15+2*(nSV_GNSS)+(nSV_GPS-1);
elseif nSV_GPS == 0 && nSV_GLO ~= 0 && nSV_GAL ~= 0
a = 15+2*(nSV_GNSS)+(nSV_GLO-1+nSV_GAL-1);
else    
a = 15+2*(nSV_GNSS)+(nSV_GPS-1+nSV_GLO-1+nSV_GAL-1);
end
D(a+1:a+nSV_BEI-1,a+2) = D1;
Pbar = D*Pbar*D';
end

end

if contains == 0

if nSV_GPS == 0 && nSV_GLO == 0 && nSV_GAL == 0
a1 = 15+2*(nSV_GNSS)+1;
a2 = 15+2*(nSV_GNSS)+length(prn_temp4)-1;    

elseif nSV_GPS == 0 && nSV_GLO == 0 && nSV_GAL ~= 0
a1 = 15+2*(nSV_GNSS)+1+length(prn_temp3)-1;
a2 = 15+2*(nSV_GNSS)+length(prn_temp3)-1+length(prn_temp4)-1;   

elseif nSV_GPS ~= 0 && nSV_GLO ~= 0 && nSV_GAL == 0
a1 = 15+2*(nSV_GNSS)+1+length(prn_temp1)-1+length(prn_temp2)-1;
a2 = 15+2*(nSV_GNSS)+length(prn_temp1)-1+length(prn_temp2)-1+length(prn_temp4)-1;   

elseif nSV_GPS == 0 && nSV_GLO ~= 0 && nSV_GAL == 0
a1 = 15+2*(nSV_GNSS)+1+length(prn_temp2)-1;
a2 = 15+2*(nSV_GNSS)+length(prn_temp2)-1+length(prn_temp4)-1;   

elseif nSV_GPS ~= 0 && nSV_GLO == 0 && nSV_GAL ~= 0
a1 = 15+2*(nSV_GNSS)+1+length(prn_temp1)-1+length(prn_temp3)-1;
a2 = 15+2*(nSV_GNSS)+length(prn_temp1)-1+length(prn_temp3)-1+length(prn_temp4)-1;   

elseif nSV_GPS ~= 0 && nSV_GLO == 0 && nSV_GAL == 0
a1 = 15+2*(nSV_GNSS)+1+length(prn_temp1)-1;
a2 = 15+2*(nSV_GNSS)+length(prn_temp1)-1+length(prn_temp4)-1;   

elseif nSV_GPS == 0 && nSV_GLO ~= 0 && nSV_GAL ~= 0
a1 = 15+2*(nSV_GNSS)+1+length(prn_temp2)-1+length(prn_temp3)-1;
a2 = 15+2*(nSV_GNSS)+length(prn_temp2)-1+length(prn_temp3)-1+length(prn_temp4)-1;   

else    
a1 = 15+2*(nSV_GNSS)+1+length(prn_temp1)-1+length(prn_temp2)-1+length(prn_temp3)-1;
a2 = 15+2*(nSV_GNSS)+length(prn_temp1)-1+length(prn_temp2)-1+length(prn_temp3)-1+length(prn_temp4)-1; 
end    
    
Pbar(a1:a2,:) = 0;
Pbar(:,a1:a2) = 0;
Pbar(a1:a2,a1:a2) = 1E6*eye(length(prn_temp4)-1);
end 



% *************************************************************************
% when satellites return from no satellites
% *************************************************************************

else
nSV_BEI = 1;
nSV_GNSS = nSV_GPS+nSV_GLO+nSV_GAL+nSV_BEI;
prn_index = 1;
prn_index1 = nSV_GPS+nSV_GLO+nSV_GAL+prn_index;
J1 = eye(15); 
J2 = eye(nSV_GNSS);
J2(:,prn_index1) = [];
J3 = eye((length(Pbar)+2)-(15+2*(nSV_GNSS)));
J = blkdiag(J1,J2,J2,J3);
Pbar = J*Pbar*J';                                            

Pbar(15+prn_index1,15+prn_index1) = sig_carr_mp^2;
Pbar(15+(nSV_GNSS)+prn_index1,15+(nSV_GNSS)+prn_index1) = sig_code_mp^2;
temp_prn_both = [temp_prn_temp(prn_index); temp_prn_both];

% nSV_BEI > 1
n = length(prn_temp4)-1;

for j = 1:n
% *************************************************************************
% nSV_GPS: number of satellites after one of them is in
% *************************************************************************        
nSV_BEI = 1+j;
nSV_GNSS = nSV_GPS+nSV_GLO+nSV_GAL+nSV_BEI;
% *************************************************************************
% the smallest prn index that will be in
% *************************************************************************    
[~, ia, ib] = intersect(temp_prn_both, temp_prn_temp); 
prn_index = find(ia-ib, 1);
    if ia==ib
    prn_index = nSV_BEI;
    end
% *************************************************************************
% step(1) constatnt part
% *************************************************************************   
J1 = eye(15); 
% *************************************************************************
% step(2) mp part
% *************************************************************************                                  
J2 = eye(nSV_GNSS); 
prn_index1 = nSV_GPS+nSV_GLO+nSV_GAL+prn_index;
J2(:,prn_index1) = [];
% *************************************************************************
% step(3) N part
% ***********************************************************************
J3 = eye((length(Pbar)+3)-(15+2*(nSV_GNSS)));
    if nSV_GPS == 0 && nSV_GLO == 0 && nSV_GAL == 0
    prn_index2 = prn_index;
    elseif nSV_GPS == 0 && nSV_GLO == 0 && nSV_GAL ~= 0
    prn_index2 = (nSV_GAL-1)+prn_index;
    elseif nSV_GPS ~= 0 && nSV_GLO ~= 0 && nSV_GAL == 0
    prn_index2 = (nSV_GPS-1+nSV_GLO-1)+prn_index;
    elseif nSV_GPS == 0 && nSV_GLO ~= 0 && nSV_GAL == 0
    prn_index2 = (nSV_GLO-1)+prn_index;
    elseif nSV_GPS ~= 0 && nSV_GLO == 0 && nSV_GAL ~= 0
    prn_index2 = (nSV_GPS-1+nSV_GAL-1)+prn_index;
    elseif nSV_GPS ~= 0 && nSV_GLO == 0 && nSV_GAL == 0
    prn_index2 = (nSV_GPS-1)+prn_index;
    elseif nSV_GPS == 0 && nSV_GLO ~= 0 && nSV_GAL ~= 0
    prn_index2 = (nSV_GLO-1+nSV_GAL-1)+prn_index;
    else    
    prn_index2 = (nSV_GPS-1+nSV_GLO-1+nSV_GAL-1)+prn_index;
    end
if prn_index == 1
J3(:,prn_index2) = [];
else
J3(:,prn_index2-1) = [];
end
% *************************************************************************
% step(4) Pbar change
% *************************************************************************
J = blkdiag(J1,J2,J2,J3);
Pbar = J*Pbar*J';                                              
% *************************************************************************
% step(5) replace
% *************************************************************************
Pbar(15+prn_index1,15+prn_index1) = sig_carr_mp^2;
Pbar(15+(nSV_GNSS)+prn_index1,15+(nSV_GNSS)+prn_index1) = sig_code_mp^2;
if prn_index == 1                                  
Pbar(15+2*(nSV_GNSS)+prn_index2,15+2*(nSV_GNSS)+prn_index2) = 1E6;
else
Pbar(14+2*(nSV_GNSS)+prn_index2,14+2*(nSV_GNSS)+prn_index2) = 1E6;
end
% *************************************************************************
% step(6) MS changes
% *************************************************************************
if prn_index == 1
D = eye(length(Pbar));
D1 = -1*ones(nSV_GPS-1,1);
a = 15+2*(nSV_GNSS);
D(a+1:a+nSV_GPS-1,a+2) = D1;
Pbar = D*Pbar*D';
end
% *************************************************************************
% step(7) fix temp_prn_both
% *************************************************************************
if prn_index ==1
temp_prn_both = [temp_prn_temp(prn_index); temp_prn_both];
else
temp_prn_both = [temp_prn_both(1:prn_index-1); temp_prn_temp(prn_index); temp_prn_both(prn_index:end)];    
end

end

end