addpath('urdf');
addpath('utility');
addpath('resources/leg/urdf');
addpath('resources/leg/meshes');
addpath('gen_files');

%% Robotic framework for floating base
robot = importrobot('leg.urdf');
q = homeConfiguration(robot);
leg_robot = floatingBaseHelper();
leg_robot.Gravity = [0, 0, -9.81];

addSubtree(leg_robot, "floating_base_RZ", robot, ReplaceBase=false);


%%
T1 = DH([0, 0, 0, t1]);
T2 = DH([0, 1, 0, t2]);
T3 = DH([pi/4, 0, sqrt(2), t3]);
T4 = DH([0, sqrt(2), 0, t4]);

P = [x; y; z; 1];
T = T1 * T2 * T3 * T4;
P_FK = T * P; P_FK = P_FK(1:3, end);


function y = DH(link_parameters)
    alpha = link_parameters(1); a = link_parameters(2); 
    d = link_parameters(3); theta = link_parameters(4);
    alpha = alpha/pi; theta = theta/pi;
    y = [
        cospi(theta), -sinpi(theta), 0, a;
        sinpi(theta)*cospi(alpha), cospi(theta)*cospi(alpha), -sinpi(alpha), -sinpi(alpha)*d;
        sinpi(theta)*sinpi(alpha), cospi(theta)*sinpi(alpha), cospi(alpha), cospi(alpha)*d;
        0, 0, 0, 1;
    ];
end















