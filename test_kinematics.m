addpath('urdf');
addpath('utility');
addpath('resources/leg/urdf');
addpath('resources/leg/meshes');
addpath('gen_files');

%% Robotic framework
robot = importrobot('leg.urdf');
q = homeConfiguration(robot);
T = getTransform(robot, q, 'L_toe', 'base');

robot_with_floating_frame = floatingBaseHelper();
robot_with_floating_frame.Gravity = [0, 0, -9.81];

addSubtree(robot_with_floating_frame, "floating_base_RZ", robot, ReplaceBase=false);

%% Dual quaternion
% syms theta_1 theta_2 theta_3 theta_4 theta_5 real
% qL = [theta_1, theta_2, theta_3, theta_4, theta_5];
qL = sym('theta_', [1 5], 'real');

LEFT_LEG_SCREW_PARAMETERS = [...
    struct('name', 'L_hip', 'a', [0, 0.12, -0.057], 'n', [0, 0, 1], 'd', 0, 'theta', qL(1)),...
    struct('name', 'L_hip2', 'a', [-0.0633, 0.12, -0.1270], 'n', [1, 0, 0], 'd', 0, 'theta', qL(2)),...
    struct('name', 'L_thigh', 'a', [0, 0.12, -0.127], 'n', [0, 1, 0], 'd', 0, 'theta', qL(3)),...
    struct('name', 'L_calf', 'a', [0, 0.12, -0.477], 'n', [0, 1, 0], 'd', 0, 'theta', qL(4)),...
    struct('name', 'L_toe', 'a', [0, 0.12,-0.827], 'n', [0, 1, 0], 'd', 0, 'theta', qL(5))
];

dual_q = DualQuaternion([1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
num_dof = length(LEFT_LEG_SCREW_PARAMETERS);
for index = 1:num_dof
    a = LEFT_LEG_SCREW_PARAMETERS(index).a;
    n = LEFT_LEG_SCREW_PARAMETERS(index).n;
    d = LEFT_LEG_SCREW_PARAMETERS(index).d;
    theta = LEFT_LEG_SCREW_PARAMETERS(index).theta;
    dual_q = dual_q * DualQuaternion(theta, n, a, d);
end

init_toe = DualQuaternion([1, 0, 0, 0], [0, 0, 0.12,-0.827]);
transformed_toe = dual_q * init_toe * (dual_q.')';
r = transformed_toe.m_DualPart; r = r(2:end); r = simplify(r);
r = matlabFunction(r, 'Vars',{qL});

orientation = dual_q.m_RealPart; orientation = simplify(orientation);
orientation = matlabFunction(orientation, 'Vars',{qL});

%% Test position computation
n = 1000;
test_result = zeros(1, n);
config = zeros(1, 5);
errors = [];
for test_index = 1:n
    q = randomConfiguration(robot);
    for index = 1:5
        config(index) = q(index).JointPosition;
    end

    r_matlab = getTransform(robot, q, 'L_toe', 'base'); r_matlab = r_matlab(1:3, end);
    r_hand = r(config);

    error = norm(r_hand - r_matlab.');
    test_result(test_index) = error;
    errors = [errors, error];
end

fprintf("The position error: %f\n", mean(test_result));
plot(errors, 'LineWidth', 1.5)
xlabel('Sample Index');
ylabel('|r_{hand} - r_{toolbox}| (10^{-16} m)')
title('Comparation between Forward KInematic Solution from My Appoach and Robotic Toolbox')
grid on

%% Test orientation computation
n = 1000;
test_result = zeros(1, n);
config = zeros(1, 5);
for test_index = 1:n
    q = randomConfiguration(robot);
    for index = 1:5
        config(index) = q(index).JointPosition;
    end

    orientation_matlab = getTransform(robot, q, 'L_toe', 'base'); orientation_matlab = orientation_matlab(1:3, 1:3);
    orientation_matlab = rotm2quat(orientation_matlab);
    orientation_hand = orientation(config);

    orientation_matlab = orientation_matlab * sign(orientation_matlab(1));
    orientation_hand = orientation_hand * sign(orientation_hand(1));
    orientation_error = quatmultiply(orientation_matlab, quatconj(orientation_hand));
    orientation_error = quatnormalize(orientation_error);
    test_result(test_index) = norm(orientation_error(2:end));
end

fprintf("The orientation error: %f\n", mean(test_result));
