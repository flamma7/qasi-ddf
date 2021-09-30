function [x_nav, P_nav, x_hat, P] = intersect_estimates(x_nav, P_nav, x_hat, P, agent, STATES)
    % Intersect the two estimates
    % Perform Partial State CI on both estimates to update remaining states

    theta = x_nav(3,1);
    rot_mat_nav = [
        1, 0, 0, 0, 0, 0;
        0, 1, 0, 0, 0, 0;
        0, 0, 0, cos(theta), -sin(theta), 0;
        0, 0, 0, sin(theta), cos(theta), 0
    ];
    nav_est_x = rot_mat_nav * x_nav;
    nav_est_P = rot_mat_nav * P_nav * rot_mat_nav';

    rot_mat = zeros(4,size(x_hat,1));
    rot_mat(:,4*(agent-1)+1:4*agent) = eye(4);
    x_hat_agent = rot_mat * x_hat;
    P_agent = rot_mat * P * rot_mat';

    [mean_result, cov_result] = covariance_intersect(nav_est_x, nav_est_P, x_hat_agent, P_agent);

    % PSCI for tracking filter
    D_inv = inv(cov_result) - inv(P_agent);
    D_inv_d = inv(cov_result)*mean_result - inv(P_agent)*x_hat_agent;
    D_inv_zeros = rot_mat' * D_inv * rot_mat;
    D_inv_d_zeros = rot_mat' * D_inv_d;
    P_new = inv( inv(P) + D_inv_zeros);
    x_hat = P_new * (inv(P)*x_hat + D_inv_d_zeros);
    P = P_new;

    % PSCI for navigation Filter (just X,Y)
    rot_mat = zeros(2, STATES);
    rot_mat(1:2,1:2) = eye(2);
    mean_result = mean_result(1:2, 1);
    cov_result = cov_result(1:2, 1:2);
    x_nav_position = rot_mat * x_nav;
    P_nav_position = rot_mat * P_nav * rot_mat';
    
    D_inv = inv(cov_result) - inv(P_nav_position);
    D_inv_d = inv(cov_result)*mean_result - inv(P_nav_position)*x_nav_position;
    D_inv_zeros = rot_mat' * D_inv * rot_mat;
    D_inv_d_zeros = rot_mat' * D_inv_d;
    P_nav_new = inv( inv(P_nav) + D_inv_zeros);
    x_nav = P_nav_new * (inv(P_nav)*x_nav + D_inv_d_zeros);
    P_nav = P_nav_new;
end