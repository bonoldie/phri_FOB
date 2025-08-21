
function franka_robot_s_function(block)

setup(block);


function setup(block)

% Register number of ports
block.NumInputPorts  = 2; %tau input, external wrench
block.NumOutputPorts = 2; %q (joint position) and dq (joint velocity)

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
%tau
block.InputPort(1).Dimensions        = 7;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = false;

%He wrench
block.InputPort(2).Dimensions        = 6;
block.InputPort(2).DatatypeID  = 0;  % double
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = false;

% Override output port properties
%q_out
block.OutputPort(1).Dimensions       = 7;
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';

%d_q_out
block.OutputPort(2).Dimensions       = 7;
block.OutputPort(2).DatatypeID  = 0; % double
block.OutputPort(2).Complexity  = 'Real';

% Register parameters
block.NumDialogPrms     = 2;

% Declare the number of continuous states (q and dq)
block.NumContStates = 14; % 7 for q and 7 for dq

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';


%block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
block.RegBlockMethod('InitializeConditions', @InitializeConditions);
block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('Outputs', @Outputs);     % Required
%block.RegBlockMethod('Update', @Update);
block.RegBlockMethod('Derivatives', @Derivatives);
block.RegBlockMethod('Terminate', @Terminate); % Required

%end setup

%%
%% PostPropagationSetup:
%%   Functionality    : Setup work areas and state variables. Can
%%                      also register run-time methods here
%%   Required         : No
%%   C MEX counterpart: mdlSetWorkWidths
%%
function DoPostPropSetup(block)
block.NumDworks = 1;

block.Dwork(1).Name            = 'F';
block.Dwork(1).Dimensions      = 3;
block.Dwork(1).DatatypeID      = 0;      % double
block.Dwork(1).Complexity      = 'Real'; % real
block.Dwork(1).UsedAsDiscState = true;


%%
%% InitializeConditions:
%%   Functionality    : Called at the start of simulation and if it is
%%                      present in an enabled subsystem configured to reset
%%                      states, it will be called when the enabled subsystem
%%                      restarts execution to reset the states.
%%   Required         : No
%%   C MEX counterpart: mdlInitializeConditions
%%
function InitializeConditions(block)

q_init = block.DialogPrm(1).Data';  % Initial joint positions
dq_init =block.DialogPrm(2).Data'; % Initial joint velocities

% Initialize the continuous states with q and dq
block.ContStates.Data = [q_init; dq_init];


%%
%% Start:
%%   Functionality    : Called once at start of model execution. If you
%%                      have states that should be initialized once, this
%%                      is the place to do it.
%%   Required         : No
%%   C MEX counterpart: mdlStart
%%
function Start(block)

%simrobot= block.DialogPrm(1).data;

% Store numeric data (like F_vector) in Dwork
%block.Dwork(1).Data = simrobot.dynamic.Fv; % Directly assign numeric vector
%block.Dwork(1).Data = 0;

%end Start

%%
%% Outputs:
%%   Functionality    : Called to generate block outputs in
%%                      simulation step
%%   Required         : Yes
%%   C MEX counterpart: mdlOutputs
%%
function Outputs(block)

% Extract states q and dq
states = block.ContStates.Data;
q = states(1:7);   % Joint positions
dq = states(8:14);  % Joint velocities

% Output the joint positions and velocities
block.OutputPort(1).Data = q;
block.OutputPort(2).Data = dq;

%end Outputs


%%
%% Derivatives:
%%   Functionality    : Called to update derivatives of
%%                      continuous states during simulation step
%%   Required         : No
%%   C MEX counterpart: mdlDerivatives
%%
function Derivatives(block)

states = block.ContStates.Data;

x1 = states(1:7);     % Current joint positions
x2 = states(8:14);    % Current joint velocities

tau= block.InputPort(1).Data;
He= block.InputPort(2).Data;

%simrobot= block.DialogPrm(1).data;

%F = block.Dwork(1).Data;
%B_func = simrobot.dynamic.B_matrix;
%C_func = simrobot.dynamic.C_matrix;
%g_func = simrobot.dynamic.grad_U;
%jacobian_func = simrobot.kinematic.jacobian_geometric;

% Evaluate matrices with the current states
%B = B_func(x1(1),x1(2),x1(3));
%C = C_func(x1(1),x1(2),x1(3),x2(1),x2(2),x2(3));
%g = g_func(x1(1),x1(2),x1(3));
%J = jacobian_func(x1(1),x1(2),x1(3));

B = get_MassMatrix(x1)
C = get_CoriolisMatrix(x1,x2);
g = get_GravityVector(x1)
F = get_FrictionTorque(x2)

dx1=x2;
%dx2= B\( (tau-J'*He) -C*x2-diag(F)*x2-g);
dx2= B\( (tau) -C*x2-diag(F)*x2-g);

% joint_limits = [ -2*pi, 1.9*pi;  % Joint 1 revolute
%     -0.2, 0.05;  % Joint 2 prismatic
%     -2*pi, 1.9*pi]; % Joint 3 revolute
% 
% % Damping parameters
% damping_threshold = 0.05;  % Distance to apply damping (adjust as needed)
% damping_coefficient = 5;  % Strength of damping (adjust as needed)
% 
% %Apply damping near joint limits
% for i = 2:2
%     % Near the lower limit
%     dist_to_lower = x1(i) - joint_limits(i, 1);
% 
%     if dist_to_lower < damping_threshold && dx1(i) < 0
% 
%         inertia=-B*dx2-C*x2;
% 
%         tau(2)=g(2)+inertia(2);
%         dx2= B\(tau-C*x2-g);
%         disp(['Applying tau ', num2str(dx2(2)), ' near lower limit.']);
%     end
% 
% end

block.Derivatives.Data = [dx1; dx2];
%end Derivatives

%%
%% Terminate:
%%   Functionality    : Called at the end of simulation for cleanup
%%   Required         : Yes
%%   C MEX counterpart: mdlTerminate
%%
function Terminate(block)

%end Terminate

