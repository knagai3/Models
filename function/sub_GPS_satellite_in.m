   
n = length(temp_prn_temp)-length(temp_prn_both);

% % check master satellite status
% contains = any(temp_prn_old(:) == temp_prn_temp(1));

for j = 1:n

% count number of GPS satellites after one of them is in
% *************************************************************************
nSV_GPS = length(temp_prn_old)+j;
% *************************************************************************

% count number of GNSS satellites after one of them is in
nSV_GNSS = nSV_GPS+nSV_GLO+nSV_GAL+nSV_BEI;

% the smallest prn index that will be in
[~, ia, ib] = intersect(temp_prn_both, temp_prn_temp);
prn_index = find(ia-ib, 1);

% fix code bug when the smallest prn is the last one
    if ia==ib
% *************************************************************************
    prn_index = nSV_GPS;
% *************************************************************************
    end

% -------------------------------------------------------------------------
% 1. Change Pbar size
% -------------------------------------------------------------------------    

% (i) constatnt states
J1 = eye(17);

% (ii) MP states 
J2 = eye(nSV_GNSS);
% *************************************************************************
prn_index1 = 0 + prn_index;
% *************************************************************************    
J2(:,prn_index1) = [];

% (iii) cycle ambiguity states
J3 = eye((length(Pbar)+3)-(17+2*(nSV_GNSS)));

% *************************************************************************
prn_index2 = prn_index;
% *************************************************************************

    if prn_index == 1
    J3(:,prn_index2) = [];
    else
    J3(:,prn_index2-1) = [];
    end

% (iv) add the in satellite states
J = blkdiag(J1,J2,J2,J3);
Pbar = J*Pbar*J';

% (v) replace the initial knowledge
Pbar(17+prn_index1,17+prn_index1) = sig_carr_mp^2;
Pbar(17+(nSV_GNSS)+prn_index1,17+(nSV_GNSS)+prn_index1) = sig_code_mp^2;

    if prn_index == 1                                  
    Pbar(17+2*(nSV_GNSS)+prn_index2,17+2*(nSV_GNSS)+prn_index2) = 1E6;
    else
    Pbar(16+2*(nSV_GNSS)+prn_index2,16+2*(nSV_GNSS)+prn_index2) = 1E6;
    end

% -------------------------------------------------------------------------
% 2. master satellite change
% -------------------------------------------------------------------------

    if prn_index == 1 % && contains == 1
    D = eye(length(Pbar));
    D1 = -1*ones(nSV_GPS-1,1);

% *************************************************************************
    a = 17+2*(nSV_GNSS);
% *************************************************************************

    D(a+1:a+nSV_GPS-1,a+1) = D1;
    Pbar = D*Pbar*D';
    end

% -------------------------------------------------------------------------
% 3. update temp_prn_both
% -------------------------------------------------------------------------

    if prn_index ==1
    temp_prn_both = [temp_prn_temp(prn_index); temp_prn_both];
    else
    temp_prn_both = [temp_prn_both(1:prn_index-1); temp_prn_temp(prn_index); temp_prn_both(prn_index:end)];    
    end

end


% -------------------------------------------------------------------------
% 4. when no master satellites exsist
% -------------------------------------------------------------------------
% 
% if contains == 0
% 
% a1 = 17+2*(nSV_GNSS)+1;
% a2 = 17+2*(nSV_GNSS)+length(prn_temp1)-1;
% Pbar(a1:a2,:) = 0;
% Pbar(:,a1:a2) = 0;
% Pbar(a1:a2,a1:a2) = 1E6*eye(length(prn_temp1)-1);
% 
% end
    
