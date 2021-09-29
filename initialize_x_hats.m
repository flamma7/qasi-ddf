function [x_hats, Ps] = initialize_x_hats(x_gt, P, NUM_AGENTS, STATES, BLUE_NUM)

    % Create block diagonal transformation matrix
    total_rot_mat = [];
    P_new = [];
    for i = 1:NUM_AGENTS
        theta = x_gt(STATES*(i-1)+3);
        rot_mat = [
            1, 0, 0, 0, 0, 0;
            0, 1, 0, 0, 0, 0;
            0, 0, 0, cos(theta), -sin(theta), 0;
            0, 0, 0, sin(theta), cos(theta), 0
        ];
        total_rot_mat = blkdiag(total_rot_mat, rot_mat);
        P_new = blkdiag(P_new, P);
    end
    x_hat = total_rot_mat * x_gt;
    x_hats = repmat(x_hat, 1, BLUE_NUM);
    P_new = total_rot_mat * P_new * total_rot_mat';
    Ps = repmat(P_new, 1, BLUE_NUM);
end