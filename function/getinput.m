
% *************************************************************************
% Earth rotation / Gravity / Earth size / Location
% *************************************************************************

% Earth rotation rate in ECEF [rad/s]
w_ie_e = [0; 0; 7.2921158553e-5];

% Gravity in ENU [m/s^2]
g_n = [0; 0; -9.780326];
g = g_n(3,1);

% Earth radius in ENU [m]
R_n = [0; 0; -6371e3]; 

% reference location in ECEF [degree]
R_llh = [45.344354010691177; -87.904841399975837; 0];
R_llh = deg2rad(R_llh);

% *************************************************************************
% INS initial value
% *************************************************************************

% INS measurement sampling rate [s]
dt_ins = 1/20;

% *************************************************************************
% GPS initial value
% *************************************************************************

% GNSS measurement sampling rate [s]
dt_gps = 1/2;
% GPS start time
UTC_time = [2020 7 1 0 00 00];

% time for initalization [s]
tG = 60*15;

% time for alighnment [s]
tI = 30;

% time for simulation [s]
tS = 200;

% total evaluation time for ins [s]
tof = tG+tI+tS;

% *************************************************************************
% GPS noise
% *************************************************************************

% Multipath time constant [s]  
tau_mp = 60;

% One sigma carrier phase thermal noise [m]
sig_carr_th = 0.001;
% One sigma code phase thermal noise [m]
sig_code_th = 0.25;

% One sigma carrier phase multipath
sig_carr_mp = 0.005;
% One sigma code phase multipath 
sig_code_mp = 0.5;

% Multipath time constant [s]  
tau_carr_mp = 150;
tau_code_mp = 80;

% Multipath error at elevation [m]
% One sigma carrier phase multipath > 33    
sig_carr_mp_abv = 0.00766;
% One sigma carrier phase multipath < 33
sig_carr_mp_blw = 0.027;

% One sigma code phase multipath > 33
sig_code_mp_abv = 0.62;
% One sigma code phase multipath < 33
sig_code_mp_blw = 1.38;

% *************************************************************************
% velocity violation
% *************************************************************************

% One sigma of velocity violation [m/s]
sig_v = 0.001;


