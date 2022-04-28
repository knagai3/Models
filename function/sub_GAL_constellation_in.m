
clear n

n = length(temp_prn_temp);

for j = 1:n

% count number of GPS satellites after one of them is in
% *************************************************************************
nSV_GAL = j;
% *************************************************************************

% count number of GNSS satellites after one of them is in
nSV_GNSS = nSV_GPS+nSV_GLO+nSV_GAL+nSV_BEI;

prn_index = 1;
% *************************************************************************
prn_index1 = nSV_GPS+nSV_GLO+prn_index;
% *************************************************************************

    if j == 1
    
    J1 = eye(17);
    J2 = eye(nSV_GNSS);
    J2(:,prn_index1) = [];
    J3 = eye((length(Pbar)+2)-(17+2*(nSV_GNSS)));
    J = blkdiag(J1,J2,J2,J3);
    Pbar = J*Pbar*J';                                              
    Pbar(17+prn_index1,17+prn_index1) = sig_carr_mp^2;
    Pbar(17+(nSV_GNSS)+prn_index1,17+(nSV_GNSS)+prn_index1) = sig_code_mp^2;    

    else
    
    % -------------------------------------------------------------------------
    % 1. Change Pbar size
    % -------------------------------------------------------------------------

    % (i) constatnt states
    J1 = eye(17);

    % (ii) MP states 
    J2 = eye(nSV_GNSS);
    J2(:,prn_index1) = [];

    % (iii) N states
    J3 = eye((length(Pbar)+3)-(17+2*(nSV_GNSS)));

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
    J3(:,prn_index2) = [];

    % (iv) add the in satellite states
    J = blkdiag(J1,J2,J2,J3);
    Pbar = J*Pbar*J'; 

    % (v) replace the initial knowledge
    Pbar(17+prn_index1,17+prn_index1) = sig_carr_mp^2;
    Pbar(17+(nSV_GNSS)+prn_index1,17+(nSV_GNSS)+prn_index1) = sig_code_mp^2;                           
    Pbar(17+2*(nSV_GNSS)+prn_index2,17+2*(nSV_GNSS)+prn_index2) = 1E6;

    end

% -------------------------------------------------------------------------
% 3. update temp_prn_both
% -------------------------------------------------------------------------

temp_prn_both = [temp_prn_temp(prn_index); temp_prn_both];

end

size(Pbar)