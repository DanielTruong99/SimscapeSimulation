addpath('urdf');
addpath('utility');
addpath('resources/leg/urdf');
addpath('resources/leg/meshes');
addpath('gen_files');

%%
robot = importrobot('leg.urdf');
q = homeConfiguration(robot);
T = getTransform(robot, q, 'L_toe', 'base');
robot_with_floating_frame = floatingBaseHelper();
robot_with_floating_frame.Gravity = [0, 0, -9.81];

addSubtree(robot_with_floating_frame, "floating_base_RZ", robot, ReplaceBase=false);

%%
q_init = [[0, 0, 0.862] [0, 0, 0] zeros(1, 10)]';
T_L_toe_cmd = getTransform(robot_with_floating_frame, q_init, 'L_toe', 'world');
T_R_toe_cmd = getTransform(robot_with_floating_frame, q_init, 'R_toe', 'world');

L_toe_cmd = [T_L_toe_cmd(1:3, end)];
R_toe_cmd = [0; 0; 0; T_R_toe_cmd(1:3, end)];
base_cmd = [ 0; 0; 0; 0.1/3.0; 0; 0.80];

q = q_init;

base = getTransform(robot_with_floating_frame, q, 'base', 'world');
base = [rotm2eul(base(1:3, 1:3),'XYZ')'; base(1:3, end)];

L_toe = getTransform(robot_with_floating_frame, q, 'L_toe', 'world');
L_toe = [rotm2eul(L_toe(1:3, 1:3),'XYZ')'; L_toe(1:3, end)];

R_toe = getTransform(robot_with_floating_frame, q, 'L_toe', 'world');
R_toe = [rotm2eul(R_toe(1:3, 1:3),'XYZ')'; R_toe(1:3, end)];

for index = 1 : 100
    J1 = geometricJacobian(robot_with_floating_frame, q, "L_toe");
    delta_q = pinv(J1) * (L_toe_cmd - L_toe);
    J1_pre = J1;
    N1_0 = eye(16, 16) - pinv(J1_pre) * J1_pre;
    
    J2 = geometricJacobian(robot_with_floating_frame, q, "R_toe");
    N1 = N1_0;
    J2_pre = J2 * N1;
    delta_q = delta_q + pinv(J2_pre) * (R_toe_cmd - R_toe - J2 * delta_q);
    
    J3 = geometricJacobian(robot_with_floating_frame, q, "base");
    N2_1 = eye(16, 16) - pinv(J2_pre) * J2_pre;
    N2 = N1 * N2_1;
    J3_pre = J3 * N2;
    delta_q = delta_q + pinv(J3_pre) * (base_cmd - base - J3 * delta_q);
    
    q = q + delta_q;

    base = getTransform(robot_with_floating_frame, q, 'base', 'world');
    base = [rotm2eul(base(1:3, 1:3),'XYZ')'; base(1:3, end)];
    
    L_toe = getTransform(robot_with_floating_frame, q, 'L_toe', 'world');
    L_toe = [rotm2eul(L_toe(1:3, 1:3),'XYZ')'; L_toe(1:3, end)];
    
    R_toe = getTransform(robot_with_floating_frame, q, 'L_toe', 'world');
    R_toe = [rotm2eul(R_toe(1:3, 1:3),'XYZ')'; R_toe(1:3, end)];
end

show(robot_with_floating_frame, q)
axis equal
