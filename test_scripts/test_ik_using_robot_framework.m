addpath('urdf');
addpath('utility');
addpath('resources/leg/urdf');
addpath('resources/leg/meshes');
addpath('gen_files');
load('/home/daniel/Workspace/Master Project/Humanoid/Scripts/RobotFrameWork/test_scripts/my_robot_footinfo.mat')

%%
leg_robot = rigidBodyTree("DataFormat", "column");
leg_robot.BaseName = 'new_base';
robot = importrobot('leg.urdf');
addSubtree(leg_robot, "new_base", robot, ReplaceBase=false);

%%
ik_solver = generalizedInverseKinematics('RigidBodyTree', leg_robot, 'ConstraintInputs' ,{ 'pose' , 'pose'});
q_init = zeros(10, 1);


right_toe_r =  [0, -0.12, -0.7650];
right_toe_orientation = [1, 0, 0, 0];
right_toe_pose = [right_toe_r, right_toe_orientation];

left_toe_r =  [0, 0.12, -0.7650];
left_toe_orientation = [1, 0, 0, 0];
left_toe_pose = [left_toe_r, left_toe_orientation];

tic; q_solution = solveIKforLeg(q_init, left_toe_pose, right_toe_pose, ik_solver); toc;
% q_solution left; right
show(leg_robot, q_solution);
axis equal

%%
ik_solver = generalizedInverseKinematics('RigidBodyTree', leg_robot, 'ConstraintInputs' ,{ 'pose' , 'pose'});
q_init = [0.0000    0.0000   -0.4241    0.8481   -0.4241    0.0000    0.0000   -0.4241    0.8481   -0.4241]';
joint_trajector.time = [];
joint_trajector.signals.values = [];
joint_trajector.signals.dimensions = [];

for index = 1:length(footinfos)
    for time_index = 1: length(footinfos{index}.timevec)
        left_foot = footinfos{index}.footleft(:, time_index);
        right_foot = footinfos{index}.footright(:, time_index);

        left_toe_r = [left_foot(3), -left_foot(1), left_foot(5) + 0.035];
        right_toe_r = [right_foot(3), -right_foot(1), right_foot(5) + 0.035];


        left_toe_orientation = [1, 0, 0, 0];
        left_toe_pose = [left_toe_r, left_toe_orientation];

        right_toe_orientation = [1, 0, 0, 0];
        right_toe_pose = [right_toe_r, right_toe_orientation];

        % solve ik
        q_solution = solveIKforLeg(q_init, left_toe_pose, right_toe_pose, ik_solver);
        q_init = q_solution;

        % store solution
        joint_trajector.time = [joint_trajector.time, footinfos{index}.timevec(time_index)];
        joint_trajector.signals.values = [joint_trajector.signals.values, q_solution];
    end
end





function q_solution = solveIKforLeg(q_init, left_toe_pose, right_toe_pose, solver)
    % pose = [r quaternion]
    
    left_toe_constraint = constraintPoseTarget("L_toe", 'ReferenceBody', 'new_base');
    left_toe_constraint.TargetTransform = [
        quat2rotm(left_toe_pose(4:end)), left_toe_pose(1:3)';
        0, 0, 0, 1;
    ];

    right_toe_constraint = constraintPoseTarget("R_toe", 'ReferenceBody', 'new_base');
    right_toe_constraint.TargetTransform = [
        quat2rotm(right_toe_pose(4:end)), right_toe_pose(1:3)';
        0, 0, 0, 1;
    ];
    
    
    q_solution = solver(q_init, left_toe_constraint, right_toe_constraint);
end