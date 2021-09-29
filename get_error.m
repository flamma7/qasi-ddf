function [error] = get_error(x_gt, x_hat, NUM_AGENTS, STATES)

    total_rot_mat = [];
    for i = 1:NUM_AGENTS
        theta = x_gt(STATES*(i-1)+3);
        rot_mat = [
            1, 0, 0, 0, 0, 0;
            0, 1, 0, 0, 0, 0;
            0, 0, 0, cos(theta), -sin(theta), 0;
            0, 0, 0, sin(theta), cos(theta), 0
        ];
        total_rot_mat = blkdiag(total_rot_mat, rot_mat);
    end

    x_gt_tracking = total_rot_mat * x_gt;
    error = x_gt_tracking - x_hat;
end