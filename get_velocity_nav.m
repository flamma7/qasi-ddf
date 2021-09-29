function [accel] = get_velocity_nav(state, target_point)

    x = state(1,1);
    y = state(2,1);
    theta = state(3,1);
    v = state(4,1);
    theta_dot = state(6,1);
    
    target_x = target_point(1,1);
    target_y = target_point(2,1);

    delta_x = target_x - x;
    delta_y = target_y - y;

    target_angle = atan2(delta_y, delta_x);
    delta_angle = normalize_angle( target_angle - theta );

    TARGET_V = 0.3;
    DELTA_CHANGE = 0.1;
    MAX_ANGLE_CHANGE = 0.2;

    if abs(delta_angle) > MAX_ANGLE_CHANGE
        TARGET_THETA_DOT = MAX_ANGLE_CHANGE * sign(delta_angle);
    else
        TARGET_THETA_DOT = delta_angle;
    end
    delta_theta_dot = TARGET_THETA_DOT - theta_dot;

    % Simplest controller
    delta_v = TARGET_V - v;

    accel = DELTA_CHANGE * [delta_v; delta_theta_dot];

    
    % % Normalize the target velocities
    % if norm([delta_x, delta_y]) > 2
    %     target_vel_x = MAX_VEL * delta_x / norm([delta_x, delta_y]);
    %     target_vel_y = MAX_VEL * delta_y / norm([delta_x, delta_y]);
    % else
    %     target_vel_x = APPROACH_VEL * delta_x / norm([delta_x, delta_y]);
    %     target_vel_y = APPROACH_VEL * delta_y / norm([delta_x, delta_y]);
    % end
    
    % accel = DELTA_CHANGE * [target_vel_x - dx; target_vel_y - dy];
end