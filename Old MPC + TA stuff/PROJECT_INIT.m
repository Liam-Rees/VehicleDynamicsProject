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
V_ref = 60 /3.6; 

%% ACADO set up 
DifferentialState r; % definition of controller states
Control Mz; % definition of controller input
OnlineData Vx; % longitudinal velocity as online data

% controller model of the plant
% beta = atan(par.l_r * tan (delta) / par.L);
Cq2 = par.l_f^2 * par.Caf + par.l_r^2 * par.Car;

% f_ctrl = [dot(r) == -Cq2/(par.Izz * 20)*r + Mz/par.Izz]; %no Vx
f_ctrl = [dot(r) == -Cq2/(par.Izz * Vx)*r + Mz/par.Izz];

%% ACADO: controller formulation
acadoSet('problemname', 'PF_problem');
Np = 40;                                  % prediction horizon
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
beta_thd       = 10 / 180*pi;            % absolute sideslip 
Mz_thd = 10000;

% constraints in ACADO 
ocp.subjectTo( -Mz_thd   <= Mz    <= Mz_thd);
ocp.subjectTo( -100000   <= r    <= 100000);

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
X0       = [0];             % initial state conditions [vx yaw Xp Yp]
% initialize controller bus
input.x  = repmat(X0, Np + 1, 1).';      % size Np + 1
input.od = repmat(Vx, Np + 1, 1);  % Initialize online data (Vx) for the prediction horizon
            % size Np + 1
Uref     = zeros(Np, 1);
input.u  = Uref.';
input.y  = [repmat(X0, Np, 1) Uref].';   % reference trajectory, size Np + 1
input.yN = X0.';                        % terminal reference, size Np + 1
% redefined in Simulink
input.W  = diag([2*1e4 1/(60/3.6)]);     % weight tuning !! Tune them in the Simulink model !!
input.WN = diag([0]);             % terminal weight tuning
input.x0 = X0.';
% controller bus initialization
init.x   = input.x(:).';                  % state trajectory
init.u   = input.u(:).';                  % control trajectory
init.y   = input.y(:).';                  % reference trajectory (up to Np - 1)
init.yN  = input.yN(:).';                % terminal reference value (only for Np)
init.W   = input.W(:).';                  % stage cost matrix (up to Np - 1)
init.WN  = input.WN(:).';                % terminal cost matrix (only for Np)
init.x0  = input.x0(:).';                % initial state value
init.od = input.od(:).';                 % Online data




%% Run

        % try
        %     % Run the simulation
        %     sim('Torque_Vectoring_Acado_noVxinput.slx');
        % catch
        %     "ERROR";
        % end




%% Postprocessing

% 
% start_SwD = 4.8
% 
% 
% figure
% plot(time(time>start_SwD), yaw_rate(time>start_SwD)); 
% hold on
% plot(time(time>start_SwD), ref_yaw_rate(time>start_SwD)); 
% grid on
% legend("yaw rate","reference yaw rate")
% title(["yaw rate vs refrence yaw rate."])
% subtitle( ["Yaw Velocity Metric:",yaw_velocity_metric])
% f = gcf
% exportgraphics(f,['YVM_.png'])
% hold off
