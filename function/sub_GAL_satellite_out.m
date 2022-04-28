
% count number of out satellites
n = length(temp_prn_old)-length(temp_prn_both);

% erase the satellites step by step    
for j = 1:n

% count number of GPS satellites before one of them is out
% *************************************************************************
nSV_GAL = length(temp_prn_old);
% *************************************************************************

% count number of GNSS satellites before one of them is out    
nSV_GNSS = nSV_GPS+nSV_GLO+nSV_GAL+nSV_BEI;

% find the smallest prn index that will be out
[~, ia, ib] = intersect(temp_prn_both, temp_prn_old);
prn_index = find(ia-ib, 1);

% fix code bug when the smallest prn is the last one
if ia==ib
prn_index = length(temp_prn_both)+1;
end

% -------------------------------------------------------------------------
% 1-1. the master satellite is changed when index is 1, 
% -------------------------------------------------------------------------

if prn_index == 1
D = eye(length(Pbar));
D1 = -1*ones(nSV_GAL-1,1);
% *************************************************************************
if nSV_GPS == 0 && nSV_GLO == 0
a = 17+2*(nSV_GNSS);
elseif nSV_GPS ~= 0 && nSV_GLO == 0 
a = 17+2*(nSV_GNSS)+(nSV_GPS-1);    
elseif nSV_GPS == 0 && nSV_GLO ~= 0
a = 17+2*(nSV_GNSS)+(nSV_GLO-1);        
else    
a = 17+2*(nSV_GNSS)+(nSV_GPS-1+nSV_GLO-1);
end
% *************************************************************************
D(a+1:a+nSV_GAL-1,a+1) = D1;
Pbar = D*Pbar*D';
end

% -------------------------------------------------------------------------
% 1-2. Change Pbar size
% -------------------------------------------------------------------------    

% (i) constatnt states   
J1 = eye(17); 

% (ii) MP states                                  
J2 = eye(nSV_GNSS);
% *************************************************************************
prn_index1 = nSV_GPS+nSV_GLO+prn_index;
% *************************************************************************
J2(prn_index1,:) = [];

% (iii) cycle ambiguity states    
J3 = eye(length(Pbar)-(17+2*(nSV_GNSS)));
% *************************************************************************
if nSV_GPS == 0 && nSV_GLO == 0
prn_index2 = prn_index; 
elseif nSV_GPS == 0 && nSV_GLO ~= 0
prn_index2 = (nSV_GLO-1)+prn_index;        
elseif nSV_GPS ~= 0 && nSV_GLO == 0 
prn_index2 = (nSV_GPS-1)+prn_index; 
else
prn_index2 = (nSV_GPS-1+nSV_GLO-1)+prn_index; 
end
% *************************************************************************
if prn_index == 1
J3(prn_index2,:) = [];
else
J3(prn_index2-1,:) = [];
end

% (iv) erase the out satellite states 
J = blkdiag(J1,J2,J2,J3);
Pbar = J*Pbar*J';

% -------------------------------------------------------------------------
% 1-3. update temp_prn_old
% -------------------------------------------------------------------------

temp_prn_old(prn_index) = [];

end



