% Dr. Barys Shyrokau
% Template for homework assignment #2
% RO47017 Vehicle Dynamics & Control
% Intelligent Vehicles & Cognitive Robotics
% Department of Cognitive Robotics
% Faculty of Mechanical Engineering
% Delft University of Technology, The Netherlands

% Use and distribution of this material outside the RO47017 course 
% only with the permission of the course coordinator

%% Cleaning
clc; clear all; 
figHandles = findall(0, 'Type', 'figure');  % Find all figure handles
for i = 1:length(figHandles)
    clf(figHandles(i));  % Clear each figure
end

%% Non-tunable parameters
par.g = 9.81;
par.Vinit   = 50 /3.6;              % initialization velocity, Don't TUNE
% Vehicle/Body (Camry)
par.mass     = 1380;                % vehicle mass, kg      
par.Izz      = 2634.5;              % body inertia around z-axis, kgm^2
par.L        = 2.79;                % wheelbase, m
par.l_f      = 1.384;               % distance from front axle to CoG, m
par.l_r      = par.L - par.l_f;     % distance from rear axle to CoG, m
% Steering
par.i_steer  = 15.4;                % steering ratio, -
% Additional
par.m_f      = par.mass * par.l_r / par.L;      % front sprung mass, kg
par.m_r      = par.mass * par.l_f / par.L;      % rear sprung mass, kg
par.mu       = 1;                   % friction coefficient, -
%% Tunable parameters
% Reference Generator
par.Calpha_front = 120000;          % front axle cornering stiffness
par.Calpha_rear  = 190000;          % rear axle cornering stiffness
par.Kus = par.m_f/par.Calpha_front - par.m_r/par.Calpha_rear; % understeer gradient
% second order TF identified from Sine Swept Test
par.wn      = 11;                   % yaw rate frequency
par.kseta   = 0.7;                  % yaw rate damping
par.tau     = 0.09;                 % yaw rate time constant

P1 = 35000;
D1 = 50;
P2 = 40000;
D2 = 100; 
N = 50;
%% Add/ Change after this line
% Maneuver settings
% V_ref = 60 /3.6;                % pre-maneuver speed, km/h
V_ref = 100/3.6;                % pre-maneuver speed, km/h
par.noise = 1;                  % 1 = noise, 0 = noiseless         
par.controller = 0;             % 1 = PD,    0 = LQR

%% LQR gain
% Prepare state space model 
Ct  = par.Calpha_front + par.Calpha_rear;
Cs  = par.l_f * par.Calpha_front - par.l_r *par.Calpha_rear;
par.Cq2 = par.l_f^2 * par.Calpha_front + par.l_r^2 * par.Calpha_rear;
m = par.mass;
Izz = par.Izz;

n_points = 20; % number of data points
LQR_Vx_ref = linspace(60,100,n_points)/3.6; % longitudinal speed, km/h

% zero initialization
A_vx = zeros(n_points,1);
B_vx = zeros(n_points,1);
Ts = 0.1;

% LQR gain calculation
for i = 1:length(LQR_Vx_ref)
    Vx = LQR_Vx_ref(i);
    A_vx = -par.Cq2/(par.Izz * Vx);
    B_vx = 1/par.Izz;
    sysc = ss(A_vx,B_vx,1,[]);
    sysd = c2d(sysc,Ts)
    A_vx(i) = sysd.A
    B_vx(i) = sysd.B;
end
%% Model run
sim('Torque_vectoring.slx')

%% Yaw rate metric
zero_crossing_index = find(yawr_ref < 0, 1, 'first');
time_cross = time(zero_crossing_index)

num = trapz(abs(yawr_err(zero_crossing_index:end)));
den = trapz(abs(yawr_ref(zero_crossing_index:end)));
yaw_metric = num/den

%% Postprocessing
% This code is an example and can be extended for other relevant plots
% print('-sHW02_template', '-djpeg', 'model.jpg')

if par.controller == 1
    FontSize = 14;
    start_SwD = 6;
    figure(1);
    subplot(1,2,1)
    set(gcf,'Color','white');
    hold all
    plot(time(time>start_SwD), long_velocity(time>start_SwD),'-black')
    grid on
    xlabel('Time [s]')
    ylabel('Long velocity [m/s]')
    set(gca,'FontSize',FontSize)

    subplot(1,2,2)
    set(gcf,'Color','white');
    hold all
    plot(time(time>start_SwD), yaw_rate(time>start_SwD),'-black')
    plot(time(time>start_SwD), yawr_ref(time>start_SwD),'-blue')
    grid on
    xlabel('Time [s]')
    ylabel('Yaw rate [rad/s]')
    sgtitle('Longtitudinal velocity and yaw rate using a PD controller')
    legend('Yaw rate', 'Yaw reference')
    set(gca,'FontSize',FontSize)

    figure(2);
    set(gcf,'Color','white');
    hold all
    plot(time(time>start_SwD),pre_sat_yaw(time>start_SwD), '-blue')
    plot(time(time>start_SwD),added_yaw(time>start_SwD), '-black')
    grid on
    xlabel('Time [s]')
    ylabel('Added yaw moment [Nm]')
    title('Added yaw moment computed by a PD controller')
    legend('Controller output', 'Saturated')
    set(gca,'FontSize',FontSize)
end

if par.controller ~= 1
    FontSize = 14;
    start_SwD = 6;
    figure(1);
    subplot(1,2,1)
    set(gcf,'Color','white');
    hold all
    plot(time(time>start_SwD), long_velocity(time>start_SwD),'-black')
    grid on
    xlabel('Time [s]')
    ylabel('Long velocity [m/s]')
    set(gca,'FontSize',FontSize)
    
    subplot(1,2,2)
    set(gcf,'Color','white');
    hold all
    plot(time(time>start_SwD), yaw_rate(time>start_SwD),'-black')
    plot(time(time>start_SwD), yawr_ref(time>start_SwD),'-blue')
    grid on
    xlabel('Time [s]')
    ylabel('Yaw rate [rad/s]')
    sgtitle('Longtitudinal velocity and yaw rate using a LQR controller')
    legend('Yaw rate', 'Yaw reference')
    set(gca,'FontSize',FontSize)
    
    figure(2);
    set(gcf,'Color','white');
    hold all
    plot(time(time>start_SwD),pre_sat_yaw(time>start_SwD), '-blue')
    plot(time(time>start_SwD),added_yaw(time>start_SwD), '-black')
    grid on
    xlabel('Time [s]')
    ylabel('Added yaw moment [Nm]')
    title('Added yaw moment computed by a LQR controller')
    legend('Controller output', 'Saturated')
    set(gca,'FontSize',FontSize)
end

