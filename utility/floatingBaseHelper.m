function robot = floatingBaseHelper(df)
    arguments
        df = "column"
    end
    robot = rigidBodyTree("DataFormat", df);
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