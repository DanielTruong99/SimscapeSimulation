addpath('urdf');
addpath('utility');
addpath('resources/leg/meshes');
addpath('resources/leg/urdf');

% Define symbolic variables
syms x y z real
syms x_dot y_dot z_dot real 
phi = sym('phi_', [3, 1], 'real');
phi_dot = sym('dot_phi_', [3, 1], 'real');
qL = sym('theta_L', [5, 1], 'real');
qL_dot = sym('dot_theta_L', [5, 1], 'real');
qR = sym('theta_R', [5, 1], 'real');
qR_dot = sym('dot_theta_R', [5, 1], 'real');
qB = [x; y; z; phi];
qB_dot = [x_dot; y_dot; z_dot; phi_dot];

qM = [qR; qL];
qM_dot = [qR_dot; qL_dot];
q = [qB; qM];
q_dot = [qB_dot; qM_dot];



% Read URDF file
robot = importrobot('leg.urdf');




% Define robot parameters
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


BASE.m = 16;
BASE.rc_reletive = [-7.0270e-14 -3.9404e-13 0.0587];
BASE.I = [...
    0.329343, 0.000022,         0;...
    0.000022, 0.156277,         0;...
           0,        0,  0.304508;
];

BASE.orientation = pure_rot([1, 0, 0], phi(1)) * pure_rot([0, 1, 0], phi(2)) * pure_rot([0, 0, 1], phi(3));
BASE.orientation = simplify(BASE.orientation.m_RealPart);
base_rc_relative_world = quatmultiply(BASE.orientation, quatmultiply([0, BASE.rc_reletive], quatconj(BASE.orientation)));
base_rc_relative_world = base_rc_relative_world(2:end);
BASE.rc = base_rc_relative_world.' + qB(1:3);



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
    rc = qB(1:3) + rc_base.';

    %%% Orientation
    orientation = quatmultiply(BASE.orientation, dual_q.m_RealPart);

    %%% Save link information into map
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
    rc = qB(1:3) + rc_base.';

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





%%% Dynamic Model 
J_cell = {};
D_cell = {};
g_vector = [0; 0; -9.81];
mr = zeros(3, 1);

% Base
rc = BASE.rc;
qc = BASE.orientation;

[J_cell{end + 1}, D_cell{end + 1}] = calcKinematicLink(rc, qc.', BASE.m, BASE.I, q, q_dot);
mr = mr + BASE.m * rc;

% Left legs
for index = 1:5
    joint_name = LEFT_LEG_SCREW_PARAMETERS.name;
    rc = LEFT_LEG(joint_name).rc;
    qc = LEFT_LEG(joint_name).orientation;
    m = LEFT_LEG(joint_name).m;
    I = LEFT_LEG(joint_name).I;
    [J_cell{end + 1}, D_cell{end + 1}] = calcKinematicLink(rc, qc.', m, I, q, q_dot);
    mr = mr + m * rc;
end

% Right legs
for index = 1:5
    joint_name = RIGHT_LEG_SCREW_PARAMETERS.name;
    rc = RIGHT_LEG(joint_name).rc;
    qc = RIGHT_LEG(joint_name).orientation;
    m = RIGHT_LEG(joint_name).m;
    I = RIGHT_LEG(joint_name).I;
    [J_cell{end + 1}, D_cell{end + 1}] = calcKinematicLink(rc, qc.', m, I, q, q_dot);
    mr = mr + m * rc;
end

% Dynamic Model Parameter
M = calcMassMatrix(J_cell, D_cell);
C = calcCoriolisMatrix(M, q, q_dot);
G = calcGravityMatrix(mr, g_vector, q);
H = C * q_dot + G;

M = matlabFunction(M, 'File','gen_files/computeM', 'Vars',{q},'Outputs',{'M'});
H = matlabFunction(H, 'File','gen_files/computeH', 'Vars',{q, q_dot},'Outputs',{'H'});