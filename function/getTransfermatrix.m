function [nRb, bRn, nRe, Q_be] = getTransfermatrix(E_n, R_llh)
% *************************************************************************
% n = ENU (East North Up) frame
% b = Body frame E = x, N = y, D = z
% e = ECEF (Earth Center Earch Fixed) frame 
% nRb = from b to n, nRe = from e to n
% *************************************************************************

% from Body to ENU frame 
nRb = get_nRb(E_n);
% from ENU to Body frame 
bRn = nRb';
% from ECEF to ENU frame
nRe = get_nRe(R_llh);
% Euler angle rates to body angular rates
Q_be = get_Qbe(E_n);

end

function nRb = get_nRb(E0_n)
% *************************************************************************
% Get rotation matrix (nRb) to change body to ENU frame by 
% phi, theta, psi [rad] (roll-pitch-yaw)
% *************************************************************************

phi = E0_n(1); % Roll
theta = E0_n(2); % Pitch
psi = E0_n(3); % Yaw

nRb = zeros(3);

nRb(1,1) = cos(psi)*cos(theta);
nRb(1,2) = -sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi);
nRb(1,3) = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);

nRb(2,1) = sin(psi)*cos(theta);
nRb(2,2) = cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi);
nRb(2,3) = -cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);

nRb(3,1) = -sin(theta);
nRb(3,2) = cos(theta)*sin(phi);
nRb(3,3) = cos(theta)*cos(phi);

end

function nRe = get_nRe(R_llh)
% *************************************************************************
%  Get rotation matrix (nRe) to change ECEF to ENU frame by 
%  lattitude [rad] and longitude [rad] of the reference station.
%  ref. APPENDIX C COORDINATE TRANSFORMATIONS
%  nRe = R_ne
% *************************************************************************

lat = R_llh(1);
lon = R_llh(2);

% ENU    
nRe = [-sin(lon)            cos(lon)            0        ;...
       -sin(lat)*cos(lon)  -sin(lat)*sin(lon)   cos(lat) ;...
       cos(lat)*cos(lon)    cos(lat)*sin(lon)   sin(lat)];

% NED
% nRe = [-sin(lat)*cos(lon) -sin(lon) -cos(lat)*cos(lon);...
%        -sin(lat)*sin(lon)  cos(lon) -cos(lat)*sin(lon);...
%         cos(lat)           0        -sin(lat)];

end

function Q_be = get_Qbe(E0_b)
% *************************************************************************
% Calculates Q_be (3x3) angler velocity transformation matrix form body
% frame to ENU frame.
% (Ref. p20, Progress Report 3)
% *************************************************************************

phi = E0_b(1);
theta = E0_b(2);

Q_be = [1   0         -sin(theta)         ;...
        0   cos(phi)   cos(theta)*sin(phi);...
        0  -sin(phi)   cos(theta)*cos(phi)];
    
end



