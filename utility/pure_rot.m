function q = pure_rot(axis, alpha)
    n = axis;
    a = [0, 0, 0];
    d = 0;

    q = DualQuaternion(alpha, n, a, d);
end