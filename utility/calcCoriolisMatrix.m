function C = calcCoriolisMatrix(M, q, q_dot)
    n = length(q);  
    jacob_vec_M = jacobian(M(:), q); jacob_vec_M = simplify(jacob_vec_M);
    C = kron(q_dot.', eye(n, n)) * jacob_vec_M - 0.5 * (kron(eye(n, n), q_dot.') * jacob_vec_M).';
    % C = simplify(C);
end