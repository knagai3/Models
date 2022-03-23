 function [Fn, Gu] = getIMUModel0

% *************************************************************************
% Generates IMU data including specific force
% fs_b and body angular velocity w_ib_b
% when vehicle is stopping.  
% *************************************************************************

getinput

% velocity [m/s] in Body frame        
v_b = [10; 0; 0];
% accele [m/s^2] in Body frame
a_b = [0; 0; 0];
% atitude [deg] in Body frame        
E_n = [0; 0; 0]; 
% angler velocity [rad/s] in Body frame        
Edot_n = [0; 0; 0];

[nRb, bRn, nRe, Q_be] = getTransfermatrix(E_n, R_llh);

% Earth rotation rate [rad/s] in NED frame
w_ie_n = nRe*w_ie_e;

% *************************************************************************
% INS input values
% *************************************************************************

% specific force input in Body frame
fs_b = a_b + bRn*cross(2*w_ie_n, nRb*v_b)-bRn*g_n;
% angler velocity input in Body frame
w_ib_b = Q_be*Edot_n + bRn*w_ie_n;

% *************************************************************************
% Fn and Gu parameters
% *************************************************************************

ss = w_ib_b - bRn*w_ie_n;

[dR_nb_dE, dQ_be_inv_dE] = getDerivativeMatrices(E_n);

Ks = blkdiag(ss', ss', ss')*dQ_be_inv_dE + ...
     blkdiag(w_ie_n', w_ie_n', w_ie_n')*dR_nb_dE;

% *************************************************************************
% Consists Fn and Gu
% ************************************************************************* 

Fn = [zeros(3) eye(3) zeros(3);...
      zeros(3) -2*skew(w_ie_n) nRb*skew(fs_b);...
      zeros(3) zeros(3) Ks];

Gu = [zeros(3) zeros(3);...
      nRb zeros(3);...
      zeros(3) inv(Q_be)];
    
 end
 
% *************************************************************************
% Derivative Matrices
% ************************************************************************* 

 function [dR_nb_dE, dQ_be_inv_dE] = getDerivativeMatrices(E0_n)
%---------------------------------------------------------------------
%   Calculates 
%   dR_nb_dE (9x3) which is derivative of rotation matrix R_nb (3x3) and 
%   dQ_be_inv_dE which is the derivative of inverse of Q_be matrix w.r.t.
%   attitude vector E.
%   (Ref. p29, Progress Report 3)
%---------------------------------------------------------------------
phi = E0_n(1);
theta = E0_n(2);
psi = E0_n(3);

dR_nb_dE = zeros(9,3);
dQ_be_inv_dE = zeros(9,3);

% For dR_nb_dE
% find derivatives with respect to phi
dR_nb_dE(1,1) = 0;
dR_nb_dE(1,2) = sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta);
dR_nb_dE(1,3) = cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta);

dR_nb_dE(2,1) = 0;
dR_nb_dE(2,2) = cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi);
dR_nb_dE(2,3) = - cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta);

dR_nb_dE(3,1) = 0;
dR_nb_dE(3,2) = cos(phi)*cos(theta);
dR_nb_dE(3,3) = -cos(theta)*sin(phi);

% find derivatives with respect to theta
dR_nb_dE(4,1) = -cos(psi)*sin(theta);
dR_nb_dE(4,2) = cos(psi)*cos(theta)*sin(phi);
dR_nb_dE(4,3) = cos(phi)*cos(psi)*cos(theta);

dR_nb_dE(5,1) = -sin(psi)*sin(theta);
dR_nb_dE(5,2) = cos(theta)*sin(phi)*sin(psi);
dR_nb_dE(5,3) = cos(phi)*cos(theta)*sin(psi);

dR_nb_dE(6,1) = -cos(theta);
dR_nb_dE(6,2) = -sin(phi)*sin(theta);
dR_nb_dE(6,3) = -cos(phi)*sin(theta);

% find derivatives with respect to psi
dR_nb_dE(7,1) = -cos(theta)*sin(psi);
dR_nb_dE(7,2) = - cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta);
dR_nb_dE(7,3) = cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta);

dR_nb_dE(8,1) = cos(psi)*cos(theta);
dR_nb_dE(8,2) = cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi);
dR_nb_dE(8,3) = sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta);

dR_nb_dE(9,1) = 0;
dR_nb_dE(9,2) = 0;
dR_nb_dE(9,3) = 0;


% For dQ_be_inv_dE
% find derivatives with respect to phi
dQ_be_inv_dE(1,1) = 0;
dQ_be_inv_dE(1,2) = (cos(phi)*sin(theta))/cos(theta);
dQ_be_inv_dE(1,3) = -(sin(phi)*sin(theta))/cos(theta);

dQ_be_inv_dE(2,1) = 0;
dQ_be_inv_dE(2,2) = -sin(phi);
dQ_be_inv_dE(2,3) = -cos(phi);

dQ_be_inv_dE(3,1) = 0;
dQ_be_inv_dE(3,2) = cos(phi)/cos(theta);
dQ_be_inv_dE(3,3) = -sin(phi)/cos(theta);
                                                                                       
% find derivatives with respect to theta
dQ_be_inv_dE(4,1) = 0;
dQ_be_inv_dE(4,2) = sin(phi)/cos(theta)^2;
dQ_be_inv_dE(4,3) = cos(phi)/cos(theta)^2;

dQ_be_inv_dE(5,1) = 0;
dQ_be_inv_dE(5,2) = 0;
dQ_be_inv_dE(5,3) = 0;

dQ_be_inv_dE(6,1) = 0;
dQ_be_inv_dE(6,2) = -(sin(phi)*sin(theta))/(sin(theta)^2 - 1);
dQ_be_inv_dE(6,3) = (cos(phi)*sin(theta))/cos(theta)^2;

% find derivatives with respect to psi
dQ_be_inv_dE(7,1) = 0;
dQ_be_inv_dE(7,2) = 0;
dQ_be_inv_dE(7,3) = 0;

dQ_be_inv_dE(8,1) = 0;
dQ_be_inv_dE(8,2) = 0;
dQ_be_inv_dE(8,3) = 0;

dQ_be_inv_dE(9,1) = 0;
dQ_be_inv_dE(9,2) = 0;
dQ_be_inv_dE(9,3) = 0;
end