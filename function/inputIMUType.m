function [tau_a, tau_g, sqrtQa, sqrtQg, sig_na, sig_ng, sig_ba0, sig_bg0]...
           = inputIMUType(IMUType)
%% This function converts from IMU Type to PSD.       
%% 
% Input:
%
% * g_n - gravity in x,y,z directions
% * IMUType - chose INS type 
% 
% Output:
% 
% * tau_a - Time constant of accelerometer bias [s]
% * tau_g - Time constant of gyroscope bias [s]
% * sqrtQa - accelerometer power spectral densities [m/s^2/sqrt(Hz)]
% * sqrtQg - gyro power spectral densities [rad/s/sqrt(Hz)]
% * sig_na - 1-sigma accelerometer bias white noise [m/s^3/sqrt(Hz)]
% * sig_ng - 1-sigma gyro bias white noise [rad/s^2/sqrt(Hz)]
% * sig_ba0 - 1-sigma accelerometer initial bias [m/s^2]
% * sig_bg0 - 1-sigma gyro initial bias [rad/s]
% 
% 
%% Accelerometer
% 
% * Velocity Random Walk (VRW) [m/s/sqrt(hr)]
% * Bias Stability (ABS) [milli-g] * milli-g = 1e-3*9.8 m/s^2
% * Bias Time Constant [hr]
% * Bias Repeatibility [milli-g]
%% Gyro
%
% * Angler Random Walk (ARW) [deg/sqrt(hr)]
% * Gyro Bias Stability [deg/hr]
% * Bias Time Constant [hr]
% * Bias Repeatibility [deg/hr]

% The list of IMU types

switch IMUType
      
    case 'STIM300'      % Tactical Grade (Sensonor)
        VRW = 0.07;     % Velocity random walk, m/s/sqrt(hr)
        ABS = 0.05;     % Accelerometer bias stability, milli-g
        ATC = 1;        % Accelerometer bias time constant, hr
        ABR = 0.75;     % Accelerometer bias repeatibility, milli-g
        ARW = 0.15;     % Angular random walk, deg/sqrt(hr)
        GBS = 0.5;      % Gyro bias stability, deg/hr
        GTC = 1;        % Gyro bias time constant, hr
        GBR = 4.0;      % Gyro bias repeatibility, deg/hr
        
    case 'HGi200'       % Industrial Grade
                        % (https://aerospace.honeywell.com/content/dam/aerobt/en/documents/learn/products/sensors/brochures/N61-2534-000-000-HGi200-mems.pdf)
        VRW = 0.04;     % Velocity random walk, m/s/sqrt(hr)
        ABS = 0.03;     % Accelerometer bias stability, milli-g
        ATC = 1;        % Accelerometer bias time constant, hr        
        ABR = 5;        % Accelerometer bias repeatibility, milli-g
        % ARW = 0.16;      % Angular random walk, deg/s sqrt(Hz)
        ARW = 0.3;      % Angular random walk, deg/s sqrt(Hz)
        GBS = 10;       % Gyro bias stability, deg/hr
        GTC = 1;        % Gyro bias time constant, hr         
        GBR = 260;      % Gyro bias repeatibility, deg/hr
        
    case 'HG9900'       % Navigation Grade (Honeywell)
        VRW = 0.0143;   % Velocity random walk, m/s/sqrt(hr)
        ABS = 0.010;    % Accelerometer bias stability, milli-g
        ATC = 2;        % Accelerometer bias time constant, hr 
        ABR = 0.025;    % Accelerometer bias repeatibility, milli-g  
        ARW = 0.002;    % Angular random walk, deg/sqrt(hr)
        GBS = 0.0006;   % Gyro bias stability, deg/hr
        GTC = 2;        % Gyro bias time constant, hr 
        GBR = 0.003;    % Gyro bias repeatibility, deg/hr
        
    case 'HG1900'       % Tactical Grade (Honeywell)
        ARW = 0.06;     % Angular random walk, deg/sqrt(hr)
        GBS = 1.0;      % Gyro bias stability, deg/hr
        GTC = 2;        % Gyro bias time constant, hr 
        GBR = 10.0;     % Gyro bias repeatibility, deg/hr
        VRW = 0.019;    % Velocity random walk, m/s/sqrt(hr)
        ABS = 0.05;     % Accelerometer bias stability, milli-g
        ATC = 2;        % Accelerometer bias time constant, hr 
        ABR = 1.0;      % Accelerometer bias repeatibility, milli-g
        
    case 'NOVATEL'      % Typical Industrial/Automative Grade (Robert, Groover, Brown)
        ARW = 0.97;     % Angular random walk, deg/sqrt(hr), converted as 5*10^(-2)*60
        GBS = 17.5;     % Gyro bias stability, deg/hr
        GTC = 0.5;      % Gyro bias time constant, hr 
        GBR = GBS*10;   % Gyro bias repeatibility, deg/hr
        VRW = 1.04;     % Velocity random walk, m/s/sqrt(hr), converted as 60*(10^(-3)*g*0.2)
        ABS = 0.31;     % Accelerometer bias stability, milli-g
        ATC = 0.5;      % Accelerometer bias time constant, hr 
        ABR = ABS*10;   % Accelerometer bias repeatibility, milli-g 
        
    case 'HG1120'       % Industrial/Automative Grade (Honeywell, was replaced by HGUIDE i200)
        VRW = 0.076;    % Velocity random walk, m/s/sqrt(hr)
        ABS = 0.11;     % Accelerometer bias stability, milli-g
        ATC = 2;        % Accelerometer bias time constant, hr 
        ABR = 8.00;     % Accelerometer bias repeatibility, milli-g       
        ARW = 0.6;      % Angular random walk, deg/sqrt(hr)
        GBS = 38;       % Gyro bias stability, deg/hr
        GTC = 2;        % Gyro bias time constant, hr 
        GBR = 500;      % Gyro bias repeatibility, deg/hr
        
    case 'CRISTA'       % Consumer Grade (Cloud Cap Tech)
        VRW = 0.261;    % Velocity random walk, m/s/sqrt(hr)
        ABS = 19.97;    % Accelerometer bias stability, milli-g
        ATC = 2;        % Accelerometer bias time constant, hr 
        ABR = 30;       % Accelerometer bias repeatibility, milli-g  
        ARW = 2.23;     % Angular random walk, deg/sqrt(hr)
        GBS = 1800;     % Gyro bias stability, deg/hr
        GTC = 2;        % Gyro bias time constant, hr 
        GBR = GBS*5;    % Gyro bias repeatibility, deg/hr
        
% Form text book (Random Signals and Applied Kalman Filtering, Robert Groover Brown)        
        
    case 'NAV'          % Typical Navigation Grade
        VRW = get_VRW1(0.003); % Velocity random walk, m/s/sqrt(hr)
        ABS = 0.01;     % Accelerometer bias stability, milli-g
        ATC = 1;        % Accelerometer bias time constant, hr 
        ABR = ABS*10;   % Accelerometer bias repeatibility, milli-g
        ARW = get_ARW1(3*1e-5); % Angular random walk, deg/sqrt(hr)
        GBS = 0.01;     % Gyro bias stability, deg/hr
        GTC = 1;        % Gyro bias time constant, hr 
        GBR = GBS*10;   % Gyro bias repeatibility, deg/hr
        
    case 'HTAC'         % Typical High Tactical Grade
        VRW = get_VRW1(0.05); % Velocity random walk, m/s/sqrt(hr)
        ABS = 0.2;      % Accelerometer bias stability, milli-g
        ATC = 1;        % Accelerometer bias time constant, hr 
        ABR = ABS*10;   % Accelerometer bias repeatibility, milli-g  
        ARW = get_ARW1(1e-3); % Angular random walk, deg/sqrt(hr)
        GBS = 0.1;      % Gyro bias stability, deg/hr
        GTC = 1;        % Gyro bias time constant, hr 
        GBR = GBS*10;   % Gyro bias repeatibility, deg/hr
        
    case 'LTAC'        % Typical Low Tactical Grade
        VRW = get_VRW1(0.1); % Velocity random walk, m/s/sqrt(hr)
        ABS = 1;        % Accelerometer bias stability, milli-g
        GTC = 1;        % Gyro bias time constant, hr 
        ABR = ABS*10;   % Accelerometer bias repeatibility, milli-g
        ARW = get_ARW1(1e-2); % Angular random walk, deg/sqrt(hr), converted as 1*10^(-2)*60
        GBS = 10;       % Gyro bias stability, deg/hr
        ATC = 1;        % Accelerometer bias time constant, hr 
        GBR = GBS*10;   % Gyro bias repeatibility, deg/hr
        
    case 'Auto'         % Typical Automative Grade
        VRW = get_VRW1(0.1); % Velocity random walk, m/s/sqrt(hr)
        ABS = 10;       % Accelerometer bias stability, milli-g
        ATC = 1;        % Accelerometer bias time constant, hr        
        ABR = ABS*10;   % Accelerometer bias repeatibility, milli-g
        ARW = get_ARW1(5*1e-2); % Angular random walk, deg/sqrt(hr)
        GBS = 100;      % Gyro bias stability, deg/hr
        GTC = 1;        % Gyro bias time constant, hr         
        GBR = GBS*10;   % Gyro bias repeatibility, deg/hr   
              
    case 'Perfect'      % Perfect Inertial for analysis purpose
        VRW = 1e-9;     % Velocity random walk, m/s/sqrt(hr)
        ABS = 1e-9;     % Accelerometer bias stability, milli-g
        ATC = 1e9;      % Accelerometer bias time constant, hr
        ABR = 1e-9;     % Accelerometer bias repeatibility, milli-g
        ARW = 0.15;     % Angular random walk, deg/sqrt(hr)
        GBS = 1e-9;     % Gyro bias stability, deg/hr
        GTC = 1e9;      % Gyro bias time constant, hr
        GBR = 1e-9;     % Gyro bias repeatibility, deg/hr

%      case 'STIM300'      % Tactical Grade (Sensonor)
%         VRW = 0.07;     % Velocity random walk, m/s/sqrt(hr)
%         ABS = 0.05;     % Accelerometer bias stability, milli-g
%         ATC = 1;        % Accelerometer bias time constant, hr
%         ABR = 0.75;     % Accelerometer bias repeatibility, milli-g
%         ARW = 0.15;     % Angular random walk, deg/sqrt(hr)
%         GBS = 0.5;      % Gyro bias stability, deg/hr
%         GTC = 1;        % Gyro bias time constant, hr
%         GBR = 4.0;      % Gyro bias repeatibility, deg/hr
        
end

%%
% PSD conversion
g = -9.780326;
% 1-sigma accelerometer initial bias [m/s^2]
sig_ba0 = ABR*1e-3*g;
% Time constant of accelerometer bias [s]
tau_a = ATC*3600;
% 1-sigma accelerometer bias white noise [m/s^2/sqrt(s)]
sig_na = (ABS*1e-3*g)*sqrt(2/tau_a);
% accelerometer power spectral densities [m/s^2/sqrt(Hz)]=[m/s/sqrt(s)]
sqrtQa = VRW/60;
% 1-sigma gyro initial bias [rad/s]
sig_bg0 = deg2rad(GBR)/3600;
% Time constant of gyroscope bias [s]
tau_g = GTC*3600;
% 1-sigma gyro bias white noise [rad/s/sqrt(s)] 
sig_ng = (deg2rad(GBS)/3600)*sqrt(2/tau_g);
% gyro power spectral densities [rad/s/sqrt(Hz)]=[rad/sqrt(s)]
sqrtQg = deg2rad(ARW)/60;  
end

%% Function
%% Velocity Random Walk
% milli-g/sqrt(Hz) => m/s/sqrt(hr)
function VRW = get_VRW1(vrw)
    VRW = vrw*9.780326*1e-3*60;
end

%% Angler Random Walk
% deg/s/sqrt(Hz) => deg/sqrt(hr)
function ARW = get_ARW1(arw)
    ARW = arw*60;
end