function M = calcMassMatrix(J_cell, D_cell)
    J = cell2sym(J_cell.');
    D = blkdiag(D_cell{:});
    M = J.' * D * J;
    % M = simplify(M);
end