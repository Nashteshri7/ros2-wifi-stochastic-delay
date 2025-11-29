%% Networked Robot Control Simulation
close
clear
clc

%% Network Parameters
% Message transmission frequency and period
fs_net = 150;
Ts_net = 1/fs_net;

% Sender to Receiver (AB) parameters (computed by fitting the PDF)
s2r.offset = 1.35;
s2r.mu = -1.03;
s2r.sigma = 0.54;

% Receiver to Sender (BA) parameters (computed by fitting the PDF)
r2s.offset = 1.01;
r2s.mu = -1.51;
r2s.sigma = 0.68;

%% DAC Parameters
% Resolution
dac.b = 16;

% q1 DAC
% q1 max torque
dac.q1.tau_max = 35;

% q1 quantization step
dac.q1.q = dac.q1.tau_max*2^(-dac.b + 1);

% q2 DAC
% q2 max torque
dac.q2.tau_max = 35;

% q2 quantization step
dac.q2.q = dac.q2.tau_max*2^(-dac.b + 1);

% q3 DAC
% q3 max torque
dac.q3.tau_max = 15;

% q3 quantization step
dac.q3.q = dac.q3.tau_max*2^(-dac.b + 1);

% q4 DAC
% q4 max torque
dac.q4.tau_max = 2.5;

% q4 quantization step
dac.q4.q = dac.q4.tau_max*2^(-dac.b + 1);

% q5 DAC
% q5 max torque
dac.q5.tau_max = 2.5;

% q5 quantization step
dac.q5.q = dac.q5.tau_max*2^(-dac.b + 1);

% q6 DAC 
% q6 max torque
dac.q6.tau_max = 1;

% q6 quantization step
dac.q6.q = dac.q6.tau_max*2^(-dac.b + 1);

%% Encoder Parameters
% Resolution
enc.b = 16;

% Quantization step
enc.q = 2*pi*2^(-enc.b);

%% Controller Parameters
% The structure is that of a PD controller with FF compensation of the
% disturbance induced by the gravitational pull

% q1 controller
PD.q1.Kp = 800;
PD.q1.Kd = 50;

% q2 controller
PD.q2.Kp = 1000;
PD.q2.Kd = 60;

% q3 controller
PD.q3.Kp = 800;
PD.q3.Kd = 10;

% q4 controller
PD.q4.Kp = 20;
PD.q4.Kd = 0.2;

% q5 controller
PD.q5.Kp = 10;
PD.q5.Kd = 0.25;

% q6 controller
PD.q6.Kp = 1;
PD.q6.Kd = 0.1;

%% Simulation Parameters
% Simulation time
T_sim = 2;

% Simulation step (to approximate ct dynamics of the robot)
fs_sim = fs_net*10;
Ts_sim = 1/(fs_sim);

%% Trajectory Parameters
% Time vector (to parametrize the curve)
t = (0:Ts_net:T_sim)';

% Circular trajectory
% Radius
r = 0.1;

% Angular velocity
omega = 2*pi/2;

% Center
c = [0.3 0 0.2];

% End effector cartesian reference trajectory...
% assuming constant orientation towards the floor,
% i.e.: R = [ 0, 0, 1;
%             0, 1, 0;
%            -1, 0, 0]
ref.x = c(1) + r*cos(omega*(t));
ref.y = c(2) + r*sin(omega*(t));
ref.z = c(3)*ones(size(t));

% ...converting it into a 4x4 transformation matrix
pose = zeros(4, 4, length(t));
for i = 1:length(t)
    pose(:, :, i) = [ 0, 0, 1, ref.x(i);
                      0, 1, 0, ref.y(i);
                     -1, 0, 0, ref.z(i);
                      0, 0, 0,    1    ];
end

% Converting it into a timeseries to import it into Simulink
pose_r = timeseries(pose, t);

%% Robot Parameters
% Loading the robot model with Earth's gravity
g = -9.81;
robot = loadrobot('abbIrb120', 'DataFormat', 'row', Gravity=[0 0 g]);

% Default initial conditions of the robot model
q_home = homeConfiguration(robot);

% Setting up the IK solver
ik = inverseKinematics('RigidBodyTree', robot);

% Setting the weights vector for the IK solver
w = [1 1 1 1 1 1];

% Computing the initial angular positions of the joints on the first point 
% of the trajectory
q0 = ik('tool0', pose(:, :, 1), w, q_home);

% Jacobian Matrix for the end effector
J = geometricJacobian(robot, q0, 'tool0');

% Initial cartesian velocity of the end effector on the first point of the
% trajectory
x_dot0 = [0; r*omega; 0; 0; 0; 0];

% Computing the initial angular velocity of the joints
q_dot0 = pinv(J)*x_dot0;

% Opening the Simulink model
% open_system('model.slx')