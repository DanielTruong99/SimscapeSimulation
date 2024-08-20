addpath('utility');
addpath('resources/mit_humanoid');
% addpath('resources/leg/urdf');
% addpath('resources/leg/meshes');
addpath('resources/leg10');
% addpath('resources/mit_humanoid/urdf');

%%
robot = importrobot('mit_humanoid_fixed_arms.urdf');
robot_with_floating_frame = floatingBaseHelper();
robot_with_floating_frame.Gravity = [0, 0, -9.81];

addSubtree(robot_with_floating_frame, "floating_base_RZ", robot, ReplaceBase=false);

q = [0; 0; 0.862; zeros(3, 1); 0; 0; 0.0; 0.0; 0; 0; 0; 0.0; 0.0; 0];
show(robot_with_floating_frame,q, Collisions="off",Visuals="on");
% show(robot_with_floating_frame, q);
axis equal

%%
robot = importrobot('leg10.urdf');
robot_with_floating_frame = floatingBaseHelper();
robot_with_floating_frame.Gravity = [0, 0, -9.81];

addSubtree(robot_with_floating_frame, "floating_base_RZ", robot, ReplaceBase=false);

q = [0; 0; 0.862; zeros(3, 1); 0; 0.1; 0; 0.0; 0; 0; -0.1; 0; 0.0; 0];
show(robot_with_floating_frame,q, Collisions="off",Visuals="on");
% show(robot_with_floating_frame, q);
axis equal

%%
robot = importrobot('leg.urdf');
show(robot,Collisions="on",Visuals="on");


function robot = floatingBaseHelper(df)
    arguments
        df = "column"
    end
    robot = rigidBodyTree(DataFormat=df);
    robot.BaseName = 'world';
    jointaxname = {'PX','PY','PZ','RX','RY','RZ'};
    jointaxval = [eye(3); eye(3)];
    parentname = robot.BaseName;
    for i = 1:numel(jointaxname)
        bname = ['floating_base_',jointaxname{i}];
        jname = ['floating_base_',jointaxname{i}];
        rb = rigidBody(bname);
        rb.Mass = 0;
        rb.Inertia = zeros(1,6);
        rbjnt = rigidBodyJoint(jname,jointaxname{i}(1));
        rbjnt.JointAxis = jointaxval(i,:);
        rbjnt.PositionLimits = [-inf inf];
        rb.Joint = rbjnt;
        robot.addBody(rb,parentname);
        parentname = rb.Name;
    end
end


