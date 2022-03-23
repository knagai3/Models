function [Fm, V_m] = getGPSnoise(nSVs)

getinput

% get multipath error model (1st order Gauss-Markov)
Fm = -1/tau_mp*eye(2*nSVs);

% 1-sigma noise on multipath (continious) [m/(s^0.5)]
sig_carr_mp_cont = sig_carr_mp*sqrt(2/tau_mp);
sig_code_mp_cont = sig_code_mp*sqrt(2/tau_mp);

% single-difference code & carrier multipath noise [m^2/s]
V_sd_carr_mp = eye(nSVs)*sig_carr_mp_cont^2;
V_sd_code_mp = eye(nSVs)*sig_code_mp_cont^2;

% Combine code&carrier and obtain multipath process noise [m^2/s]
V_m = blkdiag(V_sd_carr_mp,V_sd_code_mp);