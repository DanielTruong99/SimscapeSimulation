function torque_ff = ID(input)
    persistent leg_robot
    if isempty(leg_robot)
        robot = importrobot('leg.urdf');
        leg_robot = floatingBaseHelper();
        leg_robot.Gravity = [0, 0, -9.81];
        addSubtree(leg_robot, "floating_base_RZ", robot, ReplaceBase=false);
    end

    q = input(1:16);
    q_dot = input(17:32);
    q_ddot_cmd = input(33:48);
    fr_cmd = input(49:end);
    fr_right_cmd = fr_cmd(1:3);
    fr_left_cmd = fr_cmd(4:end);
    
    % Calculate constriant jacobian
    J_right_toe = geometricJacobian(leg_robot, q, "R_toe"); J_right_toe(1:3, :) = [];
    J_left_toe = geometricJacobian(leg_robot, q, "L_toe");  J_left_toe(1:3, :) = [];

    % Calculate inverse dynamic
    % Torque ff from Robotic Toolbox
    torque_ff = inverseDynamics(leg_robot, q, q_dot, q_ddot_cmd) - J_right_toe.' * fr_right_cmd - J_left_toe.' * fr_left_cmd;
    
    % Torque ff from my approach
    % torque_ff = computeM(q) * q_ddot_cmd + computeH(q, q_dot) - J_right_toe.' * fr_right_cmd - J_left_toe.' * fr_left_cmd;
    % 
    torque_ff = torque_ff(7:end);
end
