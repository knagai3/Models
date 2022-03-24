
% *************************************************************************
% some satellites are out
% *************************************************************************
if length(prn_temp1)>=2 && length(prn_both1)>=2
n = length(prn_old1)-length(prn_both1);

for j = 1:n
% number of satellites before one of them is out
nSV_GPS = length(prn_old1)+1-j;
nSV_GNSS = nSV_GPS+nSV_GLO+nSV_GAL+nSV_BEI;

% the smallest prn index that will be out
[~, ia, ib] = intersect(temp_prn_both, temp_prn_old);
prn_index = find(ia-ib, 1);
if ia==ib
prn_index = length(temp_prn_both)+1;
end
% *************************************************************************
% step(1) if index is 1, the master satellite changes form 1 to 2
% Pbar's size does not change in the step
% *************************************************************************   
if prn_index == 1
D = eye(length(Pbar));
D1 = -1*ones(nSV_GPS-1,1);
a = 15+2*(nSV_GNSS);
D(a+1:a+nSV_GPS-1,a+1) = D1;
Pbar = D*Pbar*D';
end
% *************************************************************************
% step(2) Change Pbar size
% *************************************************************************    
% (i) constatnt states   
J1 = eye(15); 
% (ii) multi path states                                  
J2 = eye(nSV_GNSS);
J2(prn_index,:) = [];
% (iii) cycle ambiguity states    
J3 = eye(length(Pbar)-(15+2*(nSV_GNSS)));
if prn_index == 1
J3(prn_index,:) = [];
else
J3(prn_index-1,:) = [];
end
% (iv) erase the states    
J = blkdiag(J1,J2,J2,J3);
Pbar = J*Pbar*J';
% *************************************************************************
% step(3) update temp_prn_old
% *************************************************************************    
temp_prn_old(prn_index) = [];
end

% *************************************************************************
% when all satellites are out
% *************************************************************************
else
n = length(prn_old1)-1;

for j = 1:n

% number of satellites before one of them is out
nSV_GPS = length(prn_old1)+1-j;
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
J2(prn_index,:) = [];
J3 = eye(length(Pbar)-(15+2*(nSV_GNSS)));
if prn_index == 1
J3(prn_index,:) = [];
else
J3(prn_index-1,:) = [];
end
J = blkdiag(J1,J2,J2,J3);
Pbar = J*Pbar*J';

% update temp_prn_old    
temp_prn_old(prn_index) = [];

end

% erase the last sattelite
nSV_GPS = 1;
nSV_GNSS = nSV_GPS+nSV_GLO+nSV_GAL+nSV_BEI;
prn_index = 1;    
J2 = eye(nSV_GNSS);
J2(prn_index,:) = [];
J3 = eye(length(Pbar)-(15+2*(nSV_GNSS))); 
J = blkdiag(J1,J2,J2,J3);
Pbar = J*Pbar*J';    
temp_prn_old(prn_index) = [];

end
