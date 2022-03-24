
% *************************************************************************
% some satellites are out
% *************************************************************************
if length(prn_temp3)>=2 && length(prn_both3)>=2

n = length(prn_old3)-length(prn_both3);

for j = 1:n
% *************************************************************************
% nSV_GAL: number of satellites before one of them is out
% *************************************************************************
nSV_GAL = length(prn_old3)+1-j;
nSV_GNSS = nSV_GPS+nSV_GLO+nSV_GAL+nSV_BEI;
% *************************************************************************
% the smallest prn index that will be out
% *************************************************************************
[~, ia, ib] = intersect(temp_prn_both, temp_prn_old);
prn_index = find(ia-ib, 1);
if ia==ib
prn_index = length(temp_prn_both)+1;
end
% *************************************************************************
% step(1) % if index is 1, the MS changes form 1 to 2
% Pbar's size does not change in the moment
% *************************************************************************   
if prn_index == 1
D = eye(length(Pbar));
D1 = -1*ones(nSV_GAL-1,1);
if nSV_GPS == 0 && nSV_GLO == 0
a = 15+2*(nSV_GNSS);
elseif nSV_GPS == 0 && nSV_GLO ~= 0
a = 15+2*(nSV_GNSS)+(nSV_GLO-1);        
elseif nSV_GPS ~= 0 && nSV_GLO == 0 
a = 15+2*(nSV_GNSS)+(nSV_GPS-1);    
else    
a = 15+2*(nSV_GNSS)+(nSV_GPS-1+nSV_GLO-1);
end
D(a+1:a+nSV_GAL-1,a+1) = D1;
Pbar = D*Pbar*D';
end
% *************************************************************************
% step(2) constatnt part
% *************************************************************************   
J1 = eye(15); 
% *************************************************************************
% step(3) mp part
% *************************************************************************                                  
J2 = eye(nSV_GNSS);
prn_index1 = nSV_GPS+nSV_GLO+prn_index;
J2(prn_index1,:) = [];
% *************************************************************************
% step(3) N part
% *************************************************************************    
J3 = eye(length(Pbar)-(15+2*(nSV_GNSS)));
if nSV_GPS == 0 && nSV_GLO == 0
prn_index2 = prn_index; 
elseif nSV_GPS == 0 && nSV_GLO ~= 0
prn_index2 = (nSV_GLO-1)+prn_index;        
elseif nSV_GPS ~= 0 && nSV_GLO == 0 
prn_index2 = (nSV_GPS-1)+prn_index; 
else
prn_index2 = (nSV_GPS-1+nSV_GLO-1)+prn_index; 
end
if prn_index == 1
J3(prn_index2,:) = [];
else
J3(prn_index2-1,:) = [];
end
% *************************************************************************
% step(4) Pbar change
% *************************************************************************    
J = blkdiag(J1,J2,J2,J3);
Pbar = J*Pbar*J';
% *************************************************************************
% fix prn_old due to satellite out
% *************************************************************************    
temp_prn_old(prn_index) = [];  
end

else
    
n = length(prn_old3)-1;

for j = 1:n
% *************************************************************************
% nSV_GLO: number of satellites before one of them is out
% *************************************************************************
nSV_GAL = length(prn_old3)+1-j;
nSV_GNSS = nSV_GPS+nSV_GLO+nSV_GAL+nSV_BEI;
% *************************************************************************
% step(1) the smallest prn index that will be out
% *************************************************************************
temp_prn_both = temp_prn_old(1);
[~, ia, ib] = intersect(temp_prn_both, temp_prn_old);
prn_index = find(ia-ib, 1);
if ia==ib
prn_index = length(temp_prn_both)+1;
end
% *************************************************************************
% step(2) constatnt part
% *************************************************************************   
J1 = eye(15); 
% *************************************************************************
% step(3) mp part
% *************************************************************************                                  
J2 = eye(nSV_GNSS);
prn_index1 = nSV_GPS+nSV_GLO+prn_index;
J2(prn_index1,:) = [];
% *************************************************************************
% step(3) N part
% *************************************************************************    
J3 = eye(length(Pbar)-(15+2*(nSV_GNSS)));
    if nSV_GPS == 0 && nSV_GLO == 0
    prn_index2 = prn_index; 
    elseif nSV_GPS == 0 && nSV_GLO ~= 0
    prn_index2 = (nSV_GLO-1)+prn_index;        
    elseif nSV_GPS ~= 0 && nSV_GLO == 0 
    prn_index2 = (nSV_GPS-1)+prn_index; 
    else
    prn_index2 = (nSV_GPS-1+nSV_GLO-1)+prn_index; 
    end
if prn_index == 1
J3(prn_index2,:) = [];
else
J3(prn_index2-1,:) = [];
end
% *************************************************************************
% step(4) Pbar change
% *************************************************************************    
J = blkdiag(J1,J2,J2,J3);
Pbar = J*Pbar*J';
% *************************************************************************
% fix prn_old due to satellite out
% *************************************************************************    
temp_prn_old(prn_index) = [];
end

% *************************************************************************
% erase the smallest prn related colum and row
% *************************************************************************
nSV_GAL = 1;
nSV_GNSS = nSV_GPS+nSV_GLO+nSV_GAL+nSV_BEI;
% *************************************************************************
% the smallest prn index that will be out
% *************************************************************************
prn_index = 1;
% *************************************************************************
% step(2) constatnt part
% *************************************************************************   
J1 = eye(15); 
% *************************************************************************
% step(3) mp part
% *************************************************************************                                  
J2 = eye(nSV_GNSS);
prn_index1 = nSV_GPS+nSV_GLO+prn_index;
J2(prn_index1,:) = [];
% *************************************************************************
% step(4) Pbar change
% *************************************************************************    
J3 = eye(length(Pbar)-(15+2*(nSV_GNSS))); 
J = blkdiag(J1,J2,J2,J3);
Pbar = J*Pbar*J';
% *************************************************************************
% fix prn_old due to satellite out
% *************************************************************************    
temp_prn_old(prn_index) = [];



end