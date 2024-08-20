function [J, D] = calcKinematicLink(r, quat, m, I, q, q_dot)
    w = jacobian(quat, q_dot) * q_dot; w = simplify(w);
    w = 2 * quatmultiply(w.', quat.');
    w = w(2:end).'; w = simplify(w);

    Jv = jacobian(r, q); Jv = simplify(Jv);
    Jw = jacobian(w, q_dot); Jw = simplify(Jw);
    J = [Jv; Jw];

    D = blkdiag(m*eye(3, 3), I);
end