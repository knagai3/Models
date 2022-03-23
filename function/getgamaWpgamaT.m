function gamaWpgamaT = getgamaWpgamaT(F, W, Gw, dt_ins)

%---------------------------------------------------------------------
%   Obtains overall process noise covariance matrix using "Van Loan"
%   algorithm under short time interval assumption   
%
%   dx_k = F*x_k + Gw*w           (Continuous Model)
%   x_k+1 = Phi*x_k + Gama_w*w_k  (Discrete Model)  
%
%   Inputs:
%          F    = Plant matrix of linear continuous model 
%          Gw   = Noise coeffs matrix of continious model
%          W    = Covariance matrix of continious process noise
%
%   Outputs:
%          gamaWpgamaT = Covariance matrix of the noise term Gama_w*w
%                        i.e. Gama_w*w ~ N(0,gamaWpgamaT)
%---------------------------------------------------------------------

% get size of dynamic matrix
nF = size(F,1);

% get C
C = [-F Gw*W*Gw'; zeros(nF) F']; 

% get CC
CC = expm(C*dt_ins);
nCC = size(CC,1);

% define F3 and G2 as
G2 = CC(1:nF,nF+1:nCC);
F3 = CC(nF+1:2*nF,nF+1:nCC);

% overall process noise covariance matrix 
gamaWpgamaT = F3'*G2;



