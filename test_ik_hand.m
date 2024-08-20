addpath('urdf');
addpath('utility');
addpath('resources/leg/urdf');
addpath('resources/leg/meshes');
addpath('gen_files');
addpath('test_scripts')

%% Robotic framework for floating base
leg_robot = rigidBodyTree("DataFormat", "column");
leg_robot.BaseName = 'new_base';
robot = importrobot('leg.urdf');
addSubtree(leg_robot, "new_base", robot, ReplaceBase=false);

ax = show(leg_robot, zeros(10, 1), Visuals="off");


% Set transparency for all visual meshes
% The transparency level (FaceAlpha) can be set between 0 (fully transparent) and 1 (fully opaque)
transparencyLevel = 1.0;

% Find all patch objects (these represent the visual meshes)
patches = findall(ax, 'Type', 'Patch');

% Set the transparency for each patch object
for i = 1:numel(patches)
    patches(i).FaceAlpha = transparencyLevel;
end

axis equal
view([135, 30]);
camzoom(0.7);
%%
load('test_scripts/matrix_for_ik_toe.mat');
load('test_scripts/matrix_for_ik_toe_right.mat');
errors = [];
for index = 1:1000
    q = randomConfiguration(leg_robot);
    pose_l_toe = getTransform(leg_robot, q, "L_toe", "base");
    pose_r_toe = getTransform(leg_robot, q, "R_toe", "base");
    q_solution = solveIKLegRobot(pose_l_toe, pose_r_toe, A_left, b_left, A_right, b_right);

    error = norm(q - q_solution);
    errors = [errors, error];
end

plot(errors)
xlabel('Sample Index');
ylabel('|q_{random} - q_{solution}| (10^{-12} rad)')
title('Test Inverse Kinematic Solution')
grid on

function q_solution = solveIKLegRobot(l_toe_pose, r_toe_pose, A_left, b_left, A_right, b_right)
    % l_toe_pose = 4x4, in base frame.
    joint_index = containers.Map;
    joint_index('hip') = 1; joint_index('hip2') = 2;
    joint_index('thigh') = 3; joint_index('toe') = 5; joint_index('calf') = 4; 
    qL = zeros(5, 1);
    qR = zeros(5, 1);

    % solve calf joint
    r_left_thigh = [0, 0.12, -0.127]';
    r_left_toe = l_toe_pose(1:3, end);
    relative = r_left_toe - r_left_thigh;
    L_square = norm(relative)^2;
    cosine_q_calf = (L_square - 2*(0.35^2)) / (2*0.35^2);
    qL(joint_index('calf')) = acos(cosine_q_calf);

    r_right_thigh = [0, -0.12, -0.127]';
    r_right_toe = r_toe_pose(1:3, end);
    relative = r_right_toe - r_right_thigh;
    L_square = norm(relative)^2;
    cosine_q_calf = (L_square - 2*(0.35^2)) / (2*0.35^2);
    qR(joint_index('calf')) = acos(cosine_q_calf);    

    % solve toe joint
    temp = A_left(l_toe_pose(1:3, 1:end), qL(joint_index('calf'))) \ b_left(l_toe_pose(1:3, 1:end));
    qL(joint_index('toe')) = atan2(temp(2), temp(1));

    temp = A_right(r_toe_pose(1:3, 1:end), qR(joint_index('calf'))) \ b_right(r_toe_pose(1:3, 1:end));
    qR(joint_index('toe')) = atan2(temp(2), temp(1));

    %solve hip joint
    qL(joint_index('hip')) = atan2(-l_toe_pose(1, 2), l_toe_pose(2, 2));
    qR(joint_index('hip')) = atan2(-r_toe_pose(1, 2), r_toe_pose(2, 2));

    %solve hip2 joint
    A = [l_toe_pose(1, 3), l_toe_pose(1, 1); l_toe_pose(2, 3), l_toe_pose(2, 1)];
    b = [cos(qL(joint_index('hip'))); sin(qL(joint_index('hip')))];
    temp = A \ b;
    s2 = l_toe_pose(3, 2);
    c2 = l_toe_pose(3, 3)*temp(2) - l_toe_pose(3, 1)*temp(1);
    qL(joint_index('hip2')) = atan2(s2, c2);
    qL(joint_index('thigh')) = atan2(temp(1), temp(2)) - qL(joint_index('toe')) - qL(joint_index('calf'));

    A = [r_toe_pose(1, 3), r_toe_pose(1, 1); r_toe_pose(2, 3), r_toe_pose(2, 1)];
    b = [cos(qR(joint_index('hip'))); sin(qR(joint_index('hip')))];
    temp = A \ b;
    s2 = r_toe_pose(3, 2);
    c2 = r_toe_pose(3, 3)*temp(2) - r_toe_pose(3, 1)*temp(1);
    qR(joint_index('hip2')) = atan2(s2, c2);
    qR(joint_index('thigh')) = atan2(temp(1), temp(2)) - qR(joint_index('toe')) - qR(joint_index('calf'));


    q_solution = [qL; qR];
end

%%
syms theta_1 theta_2 theta_3 theta_4 theta_5 c5 s5
q = sym('theta_', [1 5], 'real');
p = sym('p_', [3, 1], 'real');
R = sym('r_', [3, 3], 'real');
given_T = [R, p; 0 0 0 1];

T0 = [eye(3, 3), [0, -0.12, -0.127]'; 0 0 0 1]; 
T1 = [cos(q(1)) -sin(q(1)), 0, 0; sin(q(1)) cos(q(1)) 0 0; 0 0 1 0; 0 0 0 1]; 
T2 = [1 0, 0, 0; 0 cos(q(2)) -sin(q(2)) 0; 0 sin(q(2)) cos(q(2)) 0; 0 0 0 1];
T3 = [cos(q(3)) 0 sin(q(3)), 0; 0 1 0 0;-sin(q(3)) 0 cos(q(3)) 0; 0 0 0 1]; 
T4 = [cos(q(4)) 0 sin(q(4)), 0; 0 1 0 0;-sin(q(4)) 0 cos(q(4)) -0.35; 0 0 0 1];
T5 = [cos(q(5)) 0 sin(q(5)), 0; 0 1 0 0;-sin(q(5)) 0 cos(q(5)) -0.35; 0 0 0 1];

T = T0 * T1 * T2 * T3 * T4 * T5; % T0 * T1 * T2 * T3 * T4 * T5 = given_T
T04 = T0 * T1 * T2 * T3 * T4;
T03 = T0 * T1 * T2 * T3;
T = simplify(T);
temp = given_T * inv(T5) * inv(T4);
temp = simplify(temp);
temp = expand(temp);
eqn = [temp(1,end); temp(2, end)]-T03(1:2, end);
eqn = subs(eqn, {cos(theta_5), sin(theta_5)}, {c5, s5});
[A_right, b_right] = equationsToMatrix(eqn, {c5 s5});
% A_right = matlabFunction(A_right, 'Vars', {[R, p], theta_4});
% b_right = matlabFunction(b_right, 'Vars', {[R, p]});












