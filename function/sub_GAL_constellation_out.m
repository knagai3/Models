
clear n

% count number of out satellites
n = length(temp_prn_old);

% count number of GPS satellites before one of them is out
% *************************************************************************
nSV_GAL = length(temp_prn_old);
% *************************************************************************

% count number of GNSS satellites before one of them is out    
nSV_GNSS = nSV_GPS+nSV_GLO+nSV_GAL+nSV_BEI;

prn_index = 1;

% (i) constatnt states
J1 = eye(17);

% (ii) MP states                                  
J2 = eye(nSV_GNSS);
% *************************************************************************
prn_index1 = nSV_GPS+nSV_GLO+prn_index;
% *************************************************************************
J2(prn_index1:prn_index1+n-1,:) = [];

% (iii) cycle ambiguity states    
J3 = eye(length(Pbar)-(17+2*(nSV_GNSS)));
% *************************************************************************
if nSV_GPS == 0 && nSV_GLO == 0
prn_index2 = prn_index;
elseif nSV_GPS ~= 0 && nSV_GLO == 0 
prn_index2 = (nSV_GPS-1)+prn_index;
elseif nSV_GPS == 0 && nSV_GLO ~= 0
prn_index2 = (nSV_GLO-1)+prn_index;         
else
prn_index2 = (nSV_GPS-1+nSV_GLO-1)+prn_index; 
end
% *************************************************************************
J3(prn_index2:prn_index2+n-2,:) = [];

% (iv) erase the out satellite states 
J = blkdiag(J1,J2,J2,J3);
Pbar = J*Pbar*J';

% (v) update temp_prn_old
temp_prn_old(prn_index) = [];

