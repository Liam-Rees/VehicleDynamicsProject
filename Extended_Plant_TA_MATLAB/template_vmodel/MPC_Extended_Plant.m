% Dr. Barys Shyrokau
% Template for homework assignment #3
% RO47017 Vehicle Dynamics & Control
% Use and distribution of this material outside the RO47017 course 
% only with the permission of the course coordinator

clc; clear all; close all; clear mex;

% controller settings
Ts = 0.01;	% controller frequency

% vehicle parameters (bicycle model)
veh_parameters;

% Prompt user for the desired MPC controller
controller_choice = input('Select MPC controller order (1, 2, or 3): ');

switch controller_choice
    case 1
        CTRL_choice('off','on','on');                   % Choosing which Case is selected in the Simulink
        %% ACADO set up for controller 1
        DifferentialState r; % definition of controller states
        Control Tau_FR Tau_FL Tau_RR Tau_RL; % definition of controller input
        OnlineData vx;
        % controller model of the plant
        % beta = atan(par.l_r * tan (delta) / par.L);

        f_ctrl = [dot(r) == -(par.l_f^2 * par.Calpha_front + par.l_r^2 * par.Calpha_rear)/(par.Izz*vx)*r-((Tau_FR-Tau_FL+Tau_RR-Tau_RL)*par.hBf)/(par.Reff*par.Izz)];

    case 2
        CTRL_choice('on','off','on');                   % Choosing which Case is selected in the Simulink
        %% ACADO set up for controller 2
        DifferentialState vy r; % definition of controller states
        Control Tau_FR Tau_FL Tau_RR Tau_RL; % definition of controller input
        OnlineData vx; % longitudinal velocity as online data

        f_ctrl = [
                dot(vy)    == -(par.Calpha_front+par.Calpha_rear)/(par.mass*vx)*vy+((par.l_r*par.Calpha_rear-par.l_f*par.Calpha_front)/(par.mass*vx)-vx)*r;...
                dot(r)     == (par.l_r*par.Calpha_rear-par.l_f*par.Calpha_front)/(par.Izz*vx)*vy-(par.l_r^2*par.Calpha_rear+par.l_f^2*par.Calpha_front)/(par.Izz*vx)*r-((Tau_FR-Tau_FL+Tau_RR-Tau_RL)*par.hBf)/(par.Reff*par.Izz)];

    case 3
        CTRL_choice('on','on','off');                   % Choosing which Case is selected in the Simulink
        %% ACADO set up for controller 3
        DifferentialState vx vy r; % definition of controller states
        Control Tau_FR Tau_FL Tau_RR Tau_RL; % definition of controller input

        f_ctrl = [dot(vx)    == vy*r;...
                dot(vy)    == -(par.Calpha_front+par.Calpha_rear)/(par.mass*vx)*vy+((par.l_r*par.Calpha_rear-par.l_f*par.Calpha_front)/(par.mass*vx)-vx)*r;...
                dot(r)     == (par.l_r*par.Calpha_rear-par.l_f*par.Calpha_front)/(par.Izz*vx)*vy-(par.l_r^2*par.Calpha_rear+par.l_f^2*par.Calpha_front)/(par.Izz*vx)*r-((Tau_FR-Tau_FL+Tau_RR-Tau_RL)*par.hBf)/(par.Reff*par.Izz)];

    otherwise
        error('Invalid selection. Please choose 1, 2, or 3.');
end

%% ACADO: controller formulation
acadoSet('problemname', 'PF_problem');
Np = 5;                                  % prediction horizon
ocp  = acado.OCP( 0.0, Np*Ts, Np);        % ACADO ocp

% Residual function definition based on ACADO
h = [diffStates ; controls];
hN = [diffStates];                       % terminal

% Initialization weights
W = acado.BMatrix(eye(length(h)));
WN = acado.BMatrix(eye(length(hN)));     % terminal

% Cost definition
ocp.minimizeLSQ(W,h);
ocp.minimizeLSQEndTerm(WN,hN);           % terminal

% Constraints definition
% beta_thd       = 10 / 180*pi;            % absolute sideslip 
vx_thd = 170/3.6;
MZ_thd = 10000;
% vy_vx_thd = 5*pi/180;
% vdy_vx_thd = 25*pi/180;
% lat_acc_thd = 0.85*mu*par.g;
% delta_thd = (2.76*360*pi)/(180*par.i_steer);
% delta_d_thd = (800*pi)/(180*par.i_steer);

% constraints in ACADO 
% ocp.subjectTo( -beta_thd   <= beta    <= beta_thd);
ocp.subjectTo( 0   <= vx  <= vx_thd);
% ocp.subjectTo( -MZ_thd   <= Mz <= MZ_thd);
% ocp.subjectTo( -vy_vx_thd   <= vy/vx    <= vy_vx_thd);
% ocp.subjectTo( -vdy_vx_thd   <= dot(vy)/vx    <= vdy_vx_thd);
% ocp.subjectTo( -lat_acc_thd   <= (dot(vy)+vx*r)    <= lat_acc_thd);
% ocp.subjectTo( -delta_thd   <= delta  <= delta_thd);
% ocp.subjectTo( -delta_d_thd   <= dot(delta)   <= delta_d_thd);

% define ACADO prediction model
ocp.setModel(f_ctrl);
    
% ACADO settings [Don't change these settings in your HMA]
mpc = acado.OCPexport( ocp );
mpc.set('HESSIAN_APPROXIMATION', 'GAUSS_NEWTON');       % solving algorithm
mpc.set('DISCRETIZATION_TYPE', 'MULTIPLE_SHOOTING');    % discretization algorithm
mpc.set('INTEGRATOR_TYPE', 'INT_IRK_GL2');              % intergation algorithm
mpc.set('NUM_INTEGRATOR_STEPS', 3*Np);                  % number of integration steps
mpc.set('LEVENBERG_MARQUARDT', 1e-4);                   % value for Levenberg-Marquardt regularization -> affects performance
mpc.set('SPARSE_QP_SOLUTION', 'FULL_CONDENSING_N2');
mpc.set('QP_SOLVER', 'QP_QPOASES3');
mpc.set('MAX_NUM_QP_ITERATIONS', 20) ;
mpc.set('HOTSTART_QP','YES');
mpc.set('GENERATE_SIMULINK_INTERFACE', 'YES');

% Set silent mode for ACADO solver
mpc.set('PRINTLEVEL', 'NONE');                          % Suppress solver output

%% Export and Compile flags
EXPORT  = 1;
COMPILE = 1;

% export code to the defined folder
if EXPORT
    mpc.exportCode('export_MPC');
end

% compilation of the S-function using autogenerated make script
if COMPILE
    global ACADO_;
    copyfile([ACADO_.pwd '/../../external_packages/qpoases3'], 'export_MPC/qpoases3')
    cd export_MPC
    make_acado_solver_sfunction
    copyfile('acado_solver_sfun.mex*', '../')
    cd ..
end

%% initial MPC settings
disp('Initialization')

switch controller_choice
    case 1
        X0 = [0];  % initial state conditions for controller 1 [vx yaw Xp Yp]
        input.x  = repmat(X0, Np + 1, 1).';  % size Np + 1
        input.od = repmat(V_ref, Np + 1, 1); % size Np + 1
        Uref     = zeros(Np, 4);
        input.u  = Uref.';
        input.y  = [repmat(X0, Np, 1) Uref].';  % reference trajectory, size Np + 1
        input.yN = X0.';  % terminal reference, size Np + 1
        input.W  = diag([1e6 1 1 1 1]);  % weight tuning
        input.WN = diag([0]);  % terminal weight tuning
        input.x0 = X0.';
        % controller bus initialization
        init.x   = input.x(:).';  % state trajectory
        init.u   = input.u(:).';  % control trajectory
        init.y   = input.y(:).';  % reference trajectory (up to Np - 1)
        init.yN  = input.yN(:).';  % terminal reference value (only for Np)
        init.W   = input.W(:).';  % stage cost matrix (up to Np - 1)
        init.WN  = input.WN(:).';  % terminal cost matrix (only for Np)
        init.x0  = input.x0(:).';  % initial state value
        init.od  = input.od(:).';  % Online data

    case 2
        X0 = [0 0];  % initial state conditions for controller 2 [vy r]
        input.x  = repmat(X0, Np + 1, 1).';  % size Np + 1
        input.od = repmat(V_ref, Np + 1, 1); % size Np + 1
        Uref     = zeros(Np, 4);
        input.u  = Uref.';
        input.y  = [repmat(X0, Np, 1) Uref].';  % reference trajectory, size Np + 1
        input.yN = X0.';  % terminal reference, size Np + 1
        input.W  = diag([0 1e6 1 1 1 1]);  % weight tuning
        input.WN = diag([0 0]);  % terminal weight tuning
        input.x0 = X0.';
        % controller bus initialization
        init.x   = input.x(:).';  % state trajectory
        init.u   = input.u(:).';  % control trajectory
        init.y   = input.y(:).';  % reference trajectory (up to Np - 1)
        init.yN  = input.yN(:).';  % terminal reference value (only for Np)
        init.W   = input.W(:).';  % stage cost matrix (up to Np - 1)
        init.WN  = input.WN(:).';  % terminal cost matrix (only for Np)
        init.x0  = input.x0(:).';  % initial state value
        init.od  = input.od(:).';  % Online data

    case 3
        X0 = [V_ref 0 0];  % initial state conditions for controller 3 [vx vy r]
        input.x  = repmat(X0, Np + 1, 1).';  % size Np + 1
        Uref     = zeros(Np, 4);
        input.u  = Uref.';
        input.y  = [repmat(X0, Np, 1) Uref].';  % reference trajectory, size Np + 1
        input.yN = X0.';  % terminal reference, size Np + 1
        input.W  = diag([0 0 1e6 1 1 1 1]);  % weight tuning
        input.WN = diag([0 0 0]);  % terminal weight tuning
        input.x0 = X0.';
        % controller bus initialization
        init.x   = input.x(:).';  % state trajectory
        init.u   = input.u(:).';  % control trajectory
        init.y   = input.y(:).';  % reference trajectory (up to Np - 1)
        init.yN  = input.yN(:).';  % terminal reference value (only for Np)
        init.W   = input.W(:).';  % stage cost matrix (up to Np - 1)
        init.WN  = input.WN(:).';  % terminal cost matrix (only for Np)
        init.x0  = input.x0(:).';  % initial state value

end


%% sim + process
sim("MPC_Extended_Plant_Simulink.slx")
yawr_ref = yaw_results.Data(:,1);
yawr_err = yaw_results.Data(:,1)-yaw_results.Data(:,2);
zero_crossing_index = find(yawr_ref < 0, 1, 'first');
time_cross = yaw_results.Time(zero_crossing_index);

num = trapz(abs(yawr_err(zero_crossing_index:end)));
den = trapz(abs(yawr_ref(zero_crossing_index:end)));
yaw_metric = num/den;

% Save important results
if controller_choice == 1
    P1.yaw.metric = yaw_metric;
    P1.yaw.results.Time= yaw_results.Time;
    P1.yaw.results.Data= yaw_results.Data;
    P1.Wheel.FL = WheelFL;
    P1.Wheel.FR = WheelFR;
    P1.Wheel.RL = WheelRL;
    P1.Wheel.RR = WheelRR;
    save("P1")
elseif controller_choice == 2
    P2.yaw.metric = yaw_metric;
    P2.yaw.results.Time= yaw_results.Time;
    P2.yaw.results.Data= yaw_results.Data;
    P2.Wheel.FL = WheelFL;
    P2.Wheel.FR = WheelFR;
    P2.Wheel.RL = WheelRL;
    P2.Wheel.RR = WheelRR;
    save("P2")
elseif controller_choice == 3
    P3.yaw.metric = yaw_metric;
    P3.yaw.results.Time= yaw_results.Time;
    P3.yaw.results.Data= yaw_results.Data;
    P3.Wheel.FL = WheelFL;
    P3.Wheel.FR = WheelFR;
    P3.Wheel.RL = WheelRL;
    P3.Wheel.RR = WheelRR;
    save("P3")
end

%% Plots for report
load("P1.mat")
load("P2.mat")
load("P3.mat")

% Yaw rate plot

figure("Name","Yaw Error1")
hold on
grid on
plot(P1.yaw.results.Time(),    P1.yaw.results.Data( :, 1 ))
plot(P1.yaw.results.Time(),    P1.yaw.results.Data( :, 2 ))
subtitle(["Yaw Velocity Metric:",P1.yaw.metric])
ylabel("Yaw Rate (rad/s)")
xlabel("Time (s)")
xlim([9.5 12.5])
legend("reference", "actual")

exportgraphics(gcf,"yaw_error_case1.png");
exportgraphics(gcf,"yaw_error_case1.eps");
hold off

figure("Name","Yaw Error2")
hold on
grid on
plot(P2.yaw.results.Time(),    P2.yaw.results.Data( :, 1 ))
plot(P2.yaw.results.Time(),    P2.yaw.results.Data( :, 2 ))
subtitle(["Yaw Velocity Metric:",P2.yaw.metric])
ylabel("Yaw Rate (rad/s)")
xlabel("Time (s)")
xlim([9.5 12.5])
legend("reference", "actual")

exportgraphics(gcf,"yaw_error_case2.png");
exportgraphics(gcf,"yaw_error_case2.eps");
hold off

figure("Name","Yaw Error3")
hold on
grid on
plot(P3.yaw.results.Time(),    P3.yaw.results.Data( :, 1 ))
plot(P3.yaw.results.Time(),    P3.yaw.results.Data( :, 2 ))
subtitle(["Yaw Velocity Metric:",P3.yaw.metric])
ylabel("Yaw Rate (rad/s)")
xlabel("Time (s)")
xlim([9.5 12.5])
legend("reference", "actual")

exportgraphics(gcf,"yaw_error_case3.png");
exportgraphics(gcf,"yaw_error_case3.eps");
hold off

%% Control Input plots
figure("Name","Control Input plot")
subplot(3,2,1)
plot(P1.Wheel.FL)
hold on
plot(P1.Wheel.RL)
xlim([9.5 14])
% ylim([-750 750])
grid on
ylabel("") % ylabel("Applied Torque (Nm)")
xlabel("") % xlabel("Time(s)")
title("Left-One state")
legend("Front", "Rear", location="northeast")

subplot(3,2,2)
plot(P1.Wheel.FR)
hold on
plot(P1.Wheel.RR)
xlim([9.5 14])
% ylim([-750 750])
grid on
ylabel("") % ylabel("Applied Torque (Nm)")
xlabel("") % xlabel("Time(s)")
title("Right-One state")

subplot(3,2,3)
plot(P2.Wheel.FL)
hold on
plot(P2.Wheel.RL)
xlim([9.5 14])
% ylim([-750 750])
grid on
ylabel("Applied Torque (Nm)")
xlabel("") % xlabel("Time(s)")
title("Left-Two states")

subplot(3,2,4)
plot(P2.Wheel.FR)
hold on
plot(P2.Wheel.RR)
xlim([9.5 14])
% ylim([-750 750])
grid on
ylabel("") % ylabel("Applied Torque (Nm)")
xlabel("") % xlabel("Time(s)")
title("Right-Two states")

subplot(3,2,5)
plot(P3.Wheel.FL)
hold on
plot(P3.Wheel.RL)
xlim([9.5 14])
% ylim([-750 750])
grid on
ylabel("") % ylabel("Applied Torque (Nm)")
xlabel("Time(s)")
title("Left-Three states")

subplot(3,2,6)
plot(P3.Wheel.FR)
hold on
plot(P3.Wheel.RR)
xlim([9.5 14])
% ylim([-750 750])
grid on
ylabel("") % ylabel("Applied Torque (Nm)")
xlabel("Time(s)")
title("Right-Three states")

exportgraphics(gcf,"intTA_CI.png");
exportgraphics(gcf,"intTA_CI.eps");

% %%
% 
% figure("Name","Yaw Error")
% hold on
% grid on
% plot(yaw_results.Time(),    yaw_results.Data( :, 1 ))
% plot(yaw_results.Time(),    yaw_results.Data( :, 2 ))
% % subtitle([["RMSE:" RMS_error]])
% subtitle( ["Yaw Velocity Metric:",yaw_metric])
% ylabel("Yaw Rate (rad/s)")
% xlabel("Time (s)")
% legend("reference", "actual")
% 
% exportgraphics(gcf,"yaw_error_case"+num2str(controller_choice+".png");
% exportgraphics(gcf,"yaw_error_case"+num2str(controller_choice+".eps");
% hold off


% legend("yaw rate","reference yaw rate")
% title(["yaw rate vs refrence yaw rate."])
% subtitle( ["Yaw Velocity Metric:",yaw_metric])
% f = gcf
% exportgraphics(f,['YVM PID60.png'])
% hold off


% display(RMS_error)

% %% Control Inputs
% figure("Name","Controller Moment to the Wheels")
% subplot(2,2,1)
% plot(WheelFL)
% xlim([9 14])
% ylim([-750 750])
% grid on
% ylabel("Applied Torque (Nm)")
% xlabel("Time(s)")
% title("Front Left")
% 
% subplot(2,2,2)
% plot(WheelFR)
% xlim([9 14])
% ylim([-750 750])
% grid on
% ylabel("Applied Torque (Nm)")
% xlabel("Time(s)")
% title("Front Right")
% 
% subplot(2,2,3)
% plot(WheelRL)
% xlim([9 14])
% ylim([-750 750])
% grid on
% ylabel("Applied Torque (Nm)")
% xlabel("Time(s)")
% title("Rear Left")
% 
% subplot(2,2,4)
% plot(WheelRR)
% xlim([9 14])
% ylim([-750 750])
% grid on
% ylabel("Applied Torque (Nm)")
% xlabel("Time(s)")
% title("Rear Rigth")
% 
% exportgraphics(gcf,"App_Torque_case"+num2str(controller_choice)+".png");
% exportgraphics(gcf,"App_Torque_case"+num2str(controller_choice)+".eps");


%% Functions
function CTRL_choice(case_1, case_2, case_3)
% CTRL_choice: Set parameters and simulate the system
% WARNING, the selection is counterintuitive: to enable a system, select
% the 'off'-value. Likewise, to disable a system, select the 'on' value.
% It is essential that the multiple systems are not enabled at the same
% time.

open_system("MPC_Extended_Plant_Simulink");
set_param('MPC_Extended_Plant_Simulink/Case_1', 'commented', case_1);  % En-/Disable Case 1
set_param('MPC_Extended_Plant_Simulink/Case_2', 'commented', case_2);  % En-/Disable Case 2
set_param('MPC_Extended_Plant_Simulink/Case_3', 'commented', case_3);  % En-/Disable Case 3
end