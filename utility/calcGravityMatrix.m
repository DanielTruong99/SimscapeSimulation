function G = calcGravityMatrix(mr, g, q)
    potential_energy = g.' * mr;
    G = gradient(potential_energy, q);
    % G = simplify(G);
    G = -G;
end