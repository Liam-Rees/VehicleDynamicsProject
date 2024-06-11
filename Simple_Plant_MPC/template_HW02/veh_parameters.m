%% Non-tunable parameters
% Tire
par.Reff     = 0.3035;              % wheel effective radius, m
par.RL       = 0.294;               % wheel loaded radius, m
par.g = 9.81;
par.Vinit   = 50 /3.6;              % initialization velocity, Don't TUNE
% Vehicle/Body (Camry)
par.mass     = 1380;                % vehicle mass, kg      
par.Izz      = 2634.5;              % body inertia around z-axis, kgm^2
par.L        = 2.79;                % wheelbase, m
par.l_f      = 1.384;               % distance from front axle to CoG, m
par.l_r      = par.L - par.l_f;     % distance from rear axle to CoG, m
par.hcg      = 0.609;               % height of vehicle CoG above road, m
par.hBf      = 0.8;               % half of front track width, m
par.hBr      = 0.8;               % half of rear track width, m

% Steering
par.i_steer  = 15.4;                % steering ratio, -
% Additional
par.m_f      = par.mass * par.l_r / par.L;      % front sprung mass, kg
par.m_r      = par.mass * par.l_f / par.L;      % rear sprung mass, kg
par.mu       = 1;                   % friction coefficient, -
par.Caf  = 120000;                  % Front axle cornering stiffness
par.Car = 190000;                   % Rear axle cornering stiffness
% Reference Generator
par.Calpha_front = 120000;          % front axle cornering stiffness
par.Calpha_rear  = 190000;          % rear axle cornering stiffness
par.Kus = par.m_f/par.Calpha_front - par.m_r/par.Calpha_rear; % understeer gradient

% second order TF identified from Sine Swept Test
par.wn      = 11;                   % yaw rate frequency
par.kseta   = 0.7;                  % yaw rate damping
par.tau     = 0.09;                 % yaw rate time constant

V_ref = 60 /3.6;                    % pre-maneuver speed, km/h

par.FzFL = (((par.l_r*par.mass*par.g)/(par.l_f+par.l_r)))/2;
par.FzFR = (((par.l_r*par.mass*par.g)/(par.l_f+par.l_r)))/2;
par.FzRL = (par.mass*par.g - ((par.l_r*par.mass*par.g)/(par.l_f+par.l_r)))/2;
par.FzRR = (par.mass*par.g - ((par.l_r*par.mass*par.g)/(par.l_f+par.l_r)))/2;