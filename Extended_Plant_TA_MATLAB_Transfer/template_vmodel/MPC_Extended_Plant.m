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
        CTRL_choice('off','on','on');               % Selecting the controller in the simulink
        %% ACADO set up for controller 1
        DifferentialState r; % definition of controller states
        Control Tau_FL Tau_FR Tau_RL Tau_RR; % definition of controller input
        OnlineData vx ax ay;

        FzF = (((par.l_r*par.mass*par.g)/(par.l_f+par.l_r)))/2;
        FzR = (par.mass*par.g - ((par.l_r*par.mass*par.g)/(par.l_f+par.l_r)))/2;

        LongLT = (par.mass*par.hcg/(2*par.L))*ax;
        LatLT = (par.mass*par.hcg/(4*par.hBf))*ay;

        FL = FzF - LongLT - LatLT;
        FR = FzF - LongLT + LatLT;
        RL = FzR + LongLT - LatLT;
        RR = FzR + LongLT + LatLT;

        F_total = FL+FR+RL+RR;
        F_FL = FL/F_total;
        F_FR = FR/F_total;
        F_RL = RL/F_total;
        F_RR = RR/F_total;

        % controller model of the plant
        % f_ctrl = [dot(r) == -(par.l_f^2 * par.Calpha_front + par.l_r^2 * par.Calpha_rear)/(par.Izz*vx)*r-((Tau_FR*F_FR-Tau_FL*F_FL+Tau_RR*F_RR-Tau_RL*F_RL)*par.hBf)/(par.Reff*par.Izz)];
        f_ctrl = [dot(r) == -(par.l_f^2 * par.Calpha_front + par.l_r^2 * par.Calpha_rear)/(par.Izz*vx)*r+((Tau_FR-Tau_FL+Tau_RR-Tau_RL)*par.hBf)/(par.Reff*par.Izz)];

    case 2
        CTRL_choice('on','off','on');               % Selecting the controller in the simulink
        %% ACADO set up for controller 2
        DifferentialState vy r; % definition of controller states
        Control Tau_FL Tau_FR Tau_RL Tau_RR; % definition of controller input
        OnlineData vx ax ay;

        FzF = (((par.l_r*par.mass*par.g)/(par.l_f+par.l_r)))/2;
        FzR = (par.mass*par.g - ((par.l_r*par.mass*par.g)/(par.l_f+par.l_r)))/2;

        LongLT = (par.mass*par.hcg/(2*par.L))*ax;
        LatLT = (par.mass*par.hcg/(4*par.hBf))*ay;

        FL = FzF - LongLT - LatLT;
        FR = FzF - LongLT + LatLT;
        RL = FzR + LongLT - LatLT;
        RR = FzR + LongLT + LatLT;

        F_total = FL+FR+RL+RR;
        F_FL = FL/F_total;
        F_FR = FR/F_total;
        F_RL = RL/F_total;
        F_RR = RR/F_total;

        f_ctrl = [
                dot(vy)    == -(par.Calpha_front+par.Calpha_rear)/(par.mass*vx)*vy+((par.l_r*par.Calpha_rear-par.l_f*par.Calpha_front)/(par.mass*vx)-vx)*r;...
                dot(r)     == (par.l_r*par.Calpha_rear-par.l_f*par.Calpha_front)/(par.Izz*vx)*vy-(par.l_r^2*par.Calpha_rear+par.l_f^2*par.Calpha_front)/(par.Izz*vx)*r+((Tau_FR-Tau_FL+Tau_RR-Tau_RL)*par.hBf)/(par.Reff*par.Izz)];

    case 3
        CTRL_choice('on','on','off');               % Selecting the controller in the simulink
        %% ACADO set up for controller 3
        DifferentialState vx vy r; % definition of controller states
        Control Tau_FL Tau_FR Tau_RL Tau_RR; % definition of controller input
        OnlineData ax ay;

        FzF = (((par.l_r*par.mass*par.g)/(par.l_f+par.l_r)))/2;
        FzR = (par.mass*par.g - ((par.l_r*par.mass*par.g)/(par.l_f+par.l_r)))/2;

        LongLT = (par.mass*par.hcg/(2*par.L))*ax;
        LatLT = (par.mass*par.hcg/(4*par.hBf))*ay;

        FL = FzF - LongLT - LatLT;
        FR = FzF - LongLT + LatLT;
        RL = FzR + LongLT - LatLT;
        RR = FzR + LongLT + LatLT;

        F_total = FL+FR+RL+RR;
        F_FL = FL/F_total;
        F_FR = FR/F_total;
        F_RL = RL/F_total;
        F_RR = RR/F_total;

        f_ctrl = [dot(vx)    == vy*r;...
                dot(vy)    == -(par.Calpha_front+par.Calpha_rear)/(par.mass*vx)*vy+((par.l_r*par.Calpha_rear-par.l_f*par.Calpha_front)/(par.mass*vx)-vx)*r;...
                dot(r)     == (par.l_r*par.Calpha_rear-par.l_f*par.Calpha_front)/(par.Izz*vx)*vy-(par.l_r^2*par.Calpha_rear+par.l_f^2*par.Calpha_front)/(par.Izz*vx)*r+((Tau_FR-Tau_FL+Tau_RR-Tau_RL)*par.hBf)/(par.Reff*par.Izz)];

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
Tau_max = 2500;
% vy_vx_thd = 5*pi/180;
% vdy_vx_thd = 25*pi/180;
% lat_acc_thd = 0.85*mu*par.g;
% delta_thd = (2.76*360*pi)/(180*par.i_steer);
% delta_d_thd = (800*pi)/(180*par.i_steer);

% constraints in ACADO 
ocp.subjectTo( 0   <= vx  <= vx_thd);
ocp.subjectTo(-Tau_max <= Tau_FL/F_FL <= Tau_max);
ocp.subjectTo(-Tau_max <= Tau_RL/F_RL <= Tau_max);
ocp.subjectTo(-Tau_max <= Tau_FR/F_FR <= Tau_max);
ocp.subjectTo(-Tau_max <= Tau_RR/F_RR <= Tau_max);

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
        input.od = repmat([V_ref;0;0], Np + 1, 1); % size Np + 1
        Uref     = zeros(Np, length(controls));
        input.u  = Uref.';
        input.y  = [repmat(X0, Np, 1) Uref].';  % reference trajectory, size Np + 1
        input.yN = X0.';  % terminal reference, size Np + 1
        input.W  = diag([1e6 1 1 1 1]);  % weight tuning
        input.WN = diag([0]);  % terminal weight tuning
        input.x0 = X0.';

    case 2
        X0 = [0 0];  % initial state conditions for controller 2 [vy r]
        input.x  = repmat(X0, Np + 1, 1).';  % size Np + 1
        input.od = repmat([V_ref;0;0], Np + 1, 1); % size Np + 1
        Uref     = zeros(Np, length(controls));
        input.u  = Uref.';
        input.y  = [repmat(X0, Np, 1) Uref].';  % reference trajectory, size Np + 1
        input.yN = X0.';  % terminal reference, size Np + 1
        input.W  = diag([0 1e6 1 1 1 1]);  % weight tuning
        input.WN = diag([0 0]);  % terminal weight tuning
        input.x0 = X0.';

    case 3
        X0 = [V_ref 0 0];  % initial state conditions for controller 3 [vx vy r]
        input.x  = repmat(X0, Np + 1, 1).';  % size Np + 1
        input.od = repmat([0;0], Np + 1, 1); % size Np + 1
        Uref     = zeros(Np, length(controls));
        input.u  = Uref.';
        input.y  = [repmat(X0, Np, 1) Uref].';  % reference trajectory, size Np + 1
        input.yN = X0.';  % terminal reference, size Np + 1
        input.W  = diag([0 0 1e6 1 1 1 1]);  % weight tuning
        input.WN = diag([0 0 0]);  % terminal weight tuning
        input.x0 = X0.';
end

% controller bus initialization
init.x   = input.x(:).';  % state trajectory
init.u   = input.u(:).';  % control trajectory
init.y   = input.y(:).';  % reference trajectory (up to Np - 1)
init.yN  = input.yN(:).';  % terminal reference value (only for Np)
init.W   = input.W(:).';  % stage cost matrix (up to Np - 1)
init.WN  = input.WN(:).';  % terminal cost matrix (only for Np)
init.x0  = input.x0(:).';  % initial state value
init.od  = input.od(:).';  % Online data





%% sim + process
sim("MPC_Extended_Plant_Simulink.slx")
yawr_ref = yaw_results.Data(:,1);
yawr_err = yaw_results.Data(:,1)-yaw_results.Data(:,2);
zero_crossing_index = find(yawr_ref < 0, 1, 'first');
time_cross = yaw_results.Time(zero_crossing_index);

num = trapz(abs(yawr_err(zero_crossing_index:end)));
den = trapz(abs(yawr_ref(zero_crossing_index:end)));
yaw_metric = num/den;

figure("Name","Yaw Error")
hold on
grid on
plot(yaw_results.Time(),    yaw_results.Data( :, 1 ))
plot(yaw_results.Time(),    yaw_results.Data( :, 2 ))
% subtitle([["RMSE:" RMS_error]])
subtitle( ["Yaw Velocity Metric:",yaw_metric])
ylabel("Yaw Rate (rad/s)")
xlabel("Time (s)")
legend("reference", "actual")

exportgraphics(gcf,"yaw_error_case"+num2str(controller_choice)+".png");
exportgraphics(gcf,"yaw_error_case"+num2str(controller_choice)+".eps");
hold off

% legend("yaw rate","reference yaw rate")
% title(["yaw rate vs refrence yaw rate."])
% subtitle( ["Yaw Velocity Metric:",yaw_metric])
% f = gcf
% exportgraphics(f,['YVM PID60.png'])
% hold off

%% Control Inputs
figure("Name","Controller Moment to the Wheels")
subplot(2,2,1)
plot(WheelFL)
xlim([9 14])
ylim([-750 750])
grid on
ylabel("Applied Torque (Nm)")
xlabel("Time(s)")
title("Front Left")

subplot(2,2,2)
plot(WheelFR)
xlim([9 14])
ylim([-750 750])
grid on
ylabel("Applied Torque (Nm)")
xlabel("Time(s)")
title("Front Right")

subplot(2,2,3)
plot(WheelRL)
xlim([9 14])
ylim([-750 750])
grid on
ylabel("Applied Torque (Nm)")
xlabel("Time(s)")
title("Rear Left")

subplot(2,2,4)
plot(WheelRR)
xlim([9 14])
ylim([-750 750])
grid on
ylabel("Applied Torque (Nm)")
xlabel("Time(s)")
title("Rear Rigth")

exportgraphics(gcf,"App_Torque_case"+num2str(controller_choice)+".png");
exportgraphics(gcf,"App_Torque_case"+num2str(controller_choice)+".eps");

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
