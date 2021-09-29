function [inertial_new, inertial_new_P] = propagate(inertial, inertial_P, NUM_AGENTS, q_perceived)

    F = [];
    Q = [];
    for a = 1:NUM_AGENTS
        F_local = eye(4);
        F_local(1,3) = 1;
        F_local(2,4) = 1;
        F = blkdiag(F, F_local);

        Q_local = eye(4);
        Q_local(3,3) = 0.1;
        Q_local(4,4) = 0.1;
        Q = blkdiag(Q, Q_local);
    end

    inertial_new = F * inertial;
    inertial_new_P = F * inertial_P * F' + q_perceived * Q;

end