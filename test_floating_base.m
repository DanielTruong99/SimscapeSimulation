addpath('utility');
addpath('resources/leg/urdf');
addpath('resources/leg/meshes');
addpath('gen_files');
addpath('resources/g1/urdf');
addpath('resources/g1/meshes');

%% Robotic framework for floating base
robot = importrobot('leg.urdf');
q = homeConfiguration(robot);
T = getTransform(robot, q, 'L_toe', 'base');
robot_with_floating_frame = floatingBaseHelper();
robot_with_floating_frame.Gravity = [0, 0, -9.81];

addSubtree(robot_with_floating_frame, "floating_base_RZ", robot, ReplaceBase=false);


%% Time calculation for inverse dynamic between framework and hand
q_dot = rand(16, 1);
q_ddot = rand(16, 1);
q = rand(16, 1);
t_1 = [];
t_2 = [];
for index = 1:2
    tic; tau_robot_framework = inverseDynamics(robot_with_floating_frame, q, q_dot, q_ddot); time_framework = toc; t_1 = [t_1; time_framework];
    tic; tau_hand = computeM(q) * q_ddot + computeH(q, q_dot); time_hand = toc; t_2 = [t_2; time_hand];
end

% plot(1:length(t_1), t_1', 1:length(t_2), t_2');
% legend({ 'robot framework' , 'hand' });

%%
q_init = [[0, 0, 0.862] [0, 0, 0] [0; 0; -pi/3; 0.0; 0; 0; 0; -pi/3; 0.0; 0]']';
show(robot_with_floating_frame, q_init, Collisions="off",Visuals="on");
axis equal;

%% Inverse Kinematics for closed-chain
q_init = [[0, 0, 0.862] [0, 0, 0] zeros(1, 10)]';
ik_solver = generalizedInverseKinematics('RigidBodyTree', robot_with_floating_frame, 'ConstraintInputs' ,{ 'pose' , 'pose', 'pose'});
left_toe_constraint = constraintPoseTarget("L_toe", 'ReferenceBody', 'world');
left_toe_constraint.TargetTransform = getTransform(robot_with_floating_frame, q_init, 'L_toe', 'world');
right_toe_constraint = constraintPoseTarget("R_toe", 'ReferenceBody', 'world');
right_toe_constraint.TargetTransform = getTransform(robot_with_floating_frame, q_init, 'R_toe', 'world');

base_constraint = constraintPoseTarget("base", 'ReferenceBody', 'world');
base_rot = [1, 0, 0, 0];
base_rot_matrix = quat2rotm(base_rot);
% base_rot_matrix(:, 1) = -base_rot_matrix(:,1);
% base_rot_matrix(:, 2) = cross(base_rot_matrix(:,3), base_rot_matrix(:, 1));
% base_constraint.TargetTransform = [
%     base_rot_matrix, [-0.0628,  0.5007,  0.8479]';
%     0, 0, 0, 1;
% ];
% 
% % optional
% left_toe_constraint.TargetTransform = [
%     quat2rotm([0.8348, 0.0302, 0.4886, 0.2518]), [-0.3364,  0.5875,  0.1833]';
%     0, 0, 0, 1;
% ];
% 
% right_toe_constraint.TargetTransform = [
%     quat2rotm([0.9928, 0.0725, 0.0750, 0.0593]), [-0.0616,  0.4316,  0.0500]';
%     0, 0, 0, 1;
% ];
% 
% tic;
% q_solution = ik_solver(q_init, base_constraint, left_toe_constraint, right_toe_constraint);
% toc;

target_orientation = quatmultiply([cos((0 * pi/ 180) / 2) sin((0 * pi/ 180) / 2) * [-1, 0, 0]], [cos((30 * pi/ 180) / 2) sin((30 * pi/ 180) / 2) * [0, 0, -1]]);
base_constraint.TargetTransform = [
    quat2rotm([1, 0, 0, 0]), [0.03; 0; 0.84];
    0, 0, 0, 1;
];
q_solution = ik_solver(q_init, base_constraint, left_toe_constraint, right_toe_constraint);

show(robot_with_floating_frame, q_solution, Visuals="on");
axis equal;


%% Test G1 humanoid robot
robot = importrobot('g1.urdf');
q = homeConfiguration(robot);
% T = getTransform(robot, q, 'L_toe', 'base');
robot_with_floating_frame = floatingBaseHelper();
robot_with_floating_frame.Gravity = [0, 0, -9.81];

addSubtree(robot_with_floating_frame, "floating_base_RZ", robot, ReplaceBase=false);

show(robot, q, Collisions="on",Visuals="off");
axis equal;








