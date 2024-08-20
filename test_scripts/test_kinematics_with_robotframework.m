addpath('urdf');
addpath('utility');
addpath('resources/leg/urdf');
addpath('resources/leg/meshes');
addpath('gen_files');
%%
r = [0.1; 0.5; 0.892];
phi = [pi/6; pi/6; pi/6];
qL = [0; pi/3; pi/6; -pi/4; 0];
qR = [0; 0; 0; 0; 0];

%% Robotic framework
robot = importrobot('leg.urdf');
% T = getTransform(robot, q, 'L_toe', 'base');
robot_with_floating_frame = floatingBaseHelper();
robot_with_floating_frame.Gravity = [0, 0, -9.81];
addSubtree(robot_with_floating_frame, "floating_base_RZ", robot, ReplaceBase=false);

q = [r; phi; qL; qR];
T = getTransform(robot_with_floating_frame, q, 'base', 'world');

%%
LEFT_LEG_SCREW_PARAMETERS = [...
    struct('name', 'L_hip', 'a', [0, 0.12, -0.057], 'n', [0, 0, 1], 'd', 0, 'theta', qL(1)),...
    struct('name', 'L_hip2', 'a', [-0.0633, 0.12, -0.1270], 'n', [1, 0, 0], 'd', 0, 'theta', qL(2)),...
    struct('name', 'L_thigh', 'a', [0, 0.12, -0.127], 'n', [0, 1, 0], 'd', 0, 'theta', qL(3)),...
    struct('name', 'L_calf', 'a', [0, 0.12, -0.477], 'n', [0, 1, 0], 'd', 0, 'theta', qL(4)),...
    struct('name', 'L_toe', 'a', [0, 0.12,-0.827], 'n', [0, 1, 0], 'd', 0, 'theta', qL(5))
];

RIGHT_LEG_SCREW_PARAMETERS = [...
    struct('name', 'R_hip', 'a', [0, -0.12, -0.057], 'n', [0, 0, 1], 'd', 0, 'theta', qR(1)),...
    struct('name', 'R_hip2', 'a', [-0.0633, -0.12, -0.1270], 'n', [1, 0, 0], 'd', 0, 'theta', qR(2)),...
    struct('name', 'R_thigh', 'a', [0, -0.12, -0.127], 'n', [0, 1, 0], 'd', 0, 'theta', qR(3)),...
    struct('name', 'R_calf', 'a', [0, -0.12, -0.477], 'n', [0, 1, 0], 'd', 0, 'theta', qR(4)),...
    struct('name', 'R_toe', 'a', [0, -0.12,-0.827], 'n', [0, 1, 0], 'd', 0, 'theta', qR(5))
];

BASE.orientation = pure_rot([1, 0, 0], phi(1)) * pure_rot([0, 1, 0], phi(2)) * pure_rot([0, 0, 1], phi(3));
BASE.orientation = BASE.orientation.m_RealPart;

LEFT_LEG = containers.Map;
dual_q = DualQuaternion([1, 0, 0, 0], [0, 0, 0]);
for index = 1:length(LEFT_LEG_SCREW_PARAMETERS)
    link_infor = struct();

    %%% Mass, Inertial matrix asignment
    m = robot.Bodies{index}.Mass;
    p = robot.Bodies{index}.CenterOfMass;
    I_rb = robot.Bodies{index}.Inertia;
    p_skew = [0, -p(3), p(2); p(3), 0, -p(1); -p(2), p(1), 0];
    I_rb = [I_rb(1), I_rb(6), I_rb(5); I_rb(6), I_rb(2), I_rb(4); I_rb(5), I_rb(4), I_rb(3)];

    %%% CoM position
    joint_name = LEFT_LEG_SCREW_PARAMETERS.name;
    dual_q = dual_q *  DualQuaternion(  LEFT_LEG_SCREW_PARAMETERS(index).theta,...
                                        LEFT_LEG_SCREW_PARAMETERS(index).n,...
                                        LEFT_LEG_SCREW_PARAMETERS(index).a,...
                                        LEFT_LEG_SCREW_PARAMETERS(index).d  );
    initial_rc_base = LEFT_LEG_SCREW_PARAMETERS(index).a + p;
    rc_base = dual_q * DualQuaternion([1, 0, 0, 0], [0, initial_rc_base]) * (dual_q.')'; rc_base = rc_base.m_DualPart(2:end);
    rc_base = quatmultiply(BASE.orientation, quatmultiply([0 rc_base], quatconj(BASE.orientation))); rc_base = rc_base(2:end);
    rc = r + rc_base.';

    r_joint = dual_q * DualQuaternion([1, 0, 0, 0], [0, 0, 0, 0]) * (dual_q.')'; r_joint = r_joint.m_DualPart(2:end);
    r_joint = quatmultiply(BASE.orientation, quatmultiply([0 r_joint], quatconj(BASE.orientation))); r_joint = r_joint(2:end);
    r_joint = r + r_joint.';

    %%% Orientation
    orientation = quatmultiply(BASE.orientation, dual_q.m_RealPart);

    %%% Save link information into map
    link_infor.r_joint = r_joint;
    link_infor.m = m;
    link_infor.rc_relative = p;
    link_infor.I = I_rb + m * p_skew^2;
    link_infor.screw_parameter = LEFT_LEG_SCREW_PARAMETERS(index);
    link_infor.rc = rc;
    link_infor.orientation = orientation;
    LEFT_LEG(LEFT_LEG_SCREW_PARAMETERS(index).name) = link_infor;
end

RIGHT_LEG = containers.Map;
dual_q = DualQuaternion([1, 0, 0, 0], [0, 0, 0]);
for index = 1:length(RIGHT_LEG_SCREW_PARAMETERS)
    link_infor = struct();

    %%% Mass, Inertial matrix asignment
    m = robot.Bodies{index + 5}.Mass;
    p = robot.Bodies{index + 5}.CenterOfMass;
    I_rb = robot.Bodies{index + 5}.Inertia;
    p_skew = [0, -p(3), p(2); p(3), 0, -p(1); -p(2), p(1), 0];
    I_rb = [I_rb(1), I_rb(6), I_rb(5); I_rb(6), I_rb(2), I_rb(4); I_rb(5), I_rb(4), I_rb(3)];
    
    %%% CoM position
    joint_name = RIGHT_LEG_SCREW_PARAMETERS.name;
    dual_q = dual_q *  DualQuaternion(  RIGHT_LEG_SCREW_PARAMETERS(index).theta,...
                                        RIGHT_LEG_SCREW_PARAMETERS(index).n,...
                                        RIGHT_LEG_SCREW_PARAMETERS(index).a,...
                                        RIGHT_LEG_SCREW_PARAMETERS(index).d  );
    initial_rc_base = RIGHT_LEG_SCREW_PARAMETERS(index).a + p;
    rc_base = dual_q * DualQuaternion([1, 0, 0, 0], [0, initial_rc_base]) * (dual_q.')'; rc_base = rc_base.m_DualPart(2:end);
    rc_base = quatmultiply(BASE.orientation, quatmultiply([0 rc_base], quatconj(BASE.orientation))); rc_base = rc_base(2:end);
    rc = r + rc_base.';

    %%% Orientation
    orientation = quatmultiply(BASE.orientation, dual_q.m_RealPart);

    %%% Save link information into map
    link_infor.m = m;
    link_infor.rc_relative = p;
    link_infor.I = I_rb + m * p_skew^2;
    link_infor.screw_parameter = RIGHT_LEG_SCREW_PARAMETERS(index);
    link_infor.rc = rc;
    link_infor.orientation = orientation;
    RIGHT_LEG(RIGHT_LEG_SCREW_PARAMETERS(index).name) = link_infor;
end

%% Test orientation
n = length(LEFT_LEG_SCREW_PARAMETERS);
test_result = zeros(1, n);
for index = 1:n
    orientation_matlab = getTransform(robot_with_floating_frame, q, LEFT_LEG_SCREW_PARAMETERS(index).name, 'world'); orientation_matlab = orientation_matlab(1:3, 1:3);
    orientation_matlab = rotm2quat(orientation_matlab);
    orientation_hand = LEFT_LEG(LEFT_LEG_SCREW_PARAMETERS(index).name).orientation;

    orientation_matlab = orientation_matlab * sign(orientation_matlab(1));
    orientation_hand = orientation_hand * sign(orientation_hand(1));
    orientation_error = quatmultiply(orientation_matlab, quatconj(orientation_hand));
    orientation_error = quatnormalize(orientation_error);
    test_result(index) = norm(orientation_error(2:end));
end