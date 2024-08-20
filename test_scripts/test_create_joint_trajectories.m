addpath('urdf');
addpath('utility');
addpath('resources/leg/urdf');
addpath('resources/leg/meshes');
addpath('gen_files');
addpath('test_scripts');

%%
robot = importrobot('leg.urdf');
q = homeConfiguration(robot);
T = getTransform(robot, q, 'L_toe', 'base');
leg_robot = floatingBaseHelper();
leg_robot.Gravity = [0, 0, -9.81];

addSubtree(leg_robot, "floating_base_RZ", robot, ReplaceBase=false);

%%
load('my_robot_stepinfo.mat')

%% Consider 1 step
step_infor = stepinfos{2};
ik_solver = generalizedInverseKinematics('RigidBodyTree', leg_robot, 'ConstraintInputs' ,{ 'pose' , 'pose', 'pose'});
q_init = [
    0.0333;
   -0.0000;
    0.8000;
   -0.0000;
   -0.0000;
    0.0000;
   -0.0000;
    0.0000;
   -0.3688;
    0.8420;
   -0.4732;
   -0.0000;
    0.0000;
   -0.3688;
    0.8420;
   -0.4732;
];

orientation = [cos(pi/4) sin(pi/4)*[0 0 -1]];
offset_z = 0.035;
joint_trajectories.time = step_infor.timevec;
joint_trajectories.data = [];
for index = 1:length(step_infor.timevec)
    right_toe_r = quatmultiply(orientation, quatmultiply([0 step_infor.footplant'], quatconj(orientation))); 
    right_toe_r(1) = [];
    right_toe_r(end) = right_toe_r(end) + offset_z;
    right_toe_orientation = [1, 0, 0, 0];
    right_toe_pose = [right_toe_r, right_toe_orientation];

    left_toe_r = quatmultiply(orientation, quatmultiply([0 step_infor.swing(:, index)'], quatconj(orientation))); 
    left_toe_r(1) = [];
    left_toe_r(end) = left_toe_r(end) + offset_z;
    left_toe_orientation = [1, 0, 0, 0];
    left_toe_pose = [left_toe_r, left_toe_orientation];

    base_r = step_infor.state(:, index);
    base_r = [base_r(1) base_r(3) base_r(5)];
    base_r = quatmultiply(orientation, quatmultiply([0 base_r], quatconj(orientation))); 
    base_r(1) = [];
    base_r(end) = base_r(end) + offset_z;
    base_orientation = [1, 0, 0, 0];
    base_pose = [base_r, base_orientation];

    q_solution = solveIKforLeg(q_init, base_pose, left_toe_pose, right_toe_pose, ik_solver); 
    q_init = q_solution;
    joint_trajectories.data = [joint_trajectories.data, q_solution];
end





function q_solution = solveIKforLeg(q_init, base_pose, left_toe_pose, right_toe_pose, solver)
    % pose = [r quaternion]
    
    left_toe_constraint = constraintPoseTarget("L_toe", 'ReferenceBody', 'world');
    left_toe_constraint.TargetTransform = [
        quat2rotm(left_toe_pose(4:end)), left_toe_pose(1:3)';
        0, 0, 0, 1;
    ];

    right_toe_constraint = constraintPoseTarget("R_toe", 'ReferenceBody', 'world');
    right_toe_constraint.TargetTransform = [
        quat2rotm(right_toe_pose(4:end)), right_toe_pose(1:3)';
        0, 0, 0, 1;
    ];
    
    base_constraint = constraintPoseTarget("base", 'ReferenceBody', 'world');
    base_constraint.TargetTransform = [
        quat2rotm(base_pose(4:end)), base_pose(1:3)';
        0, 0, 0, 1;
    ];
    
    q_solution = solver(q_init, base_constraint, left_toe_constraint, right_toe_constraint);
end