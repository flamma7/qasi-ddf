function [accel] = get_velocity(state, target_point)

    x = state(1,1);
    y = state(2,1);
    dx = state(3,1);
    dy = state(4,1);
    
    target_x = target_point(1,1);
    target_y = target_point(2,1);
    
    DELTA_CHANGE = 0.05;
    MAX_VEL = 0.3;
    APPROACH_VEL = 0.15;
    
    delta_x = target_x - x;
    delta_y = target_y - y;
    
    % Normalize the target velocities
    if norm([delta_x, delta_y]) > 2
        target_vel_x = MAX_VEL * delta_x / norm([delta_x, delta_y]);
        target_vel_y = MAX_VEL * delta_y / norm([delta_x, delta_y]);
    else
        target_vel_x = APPROACH_VEL * delta_x / norm([delta_x, delta_y]);
        target_vel_y = APPROACH_VEL * delta_y / norm([delta_x, delta_y]);
    end
    
    accel = DELTA_CHANGE * [target_vel_x - dx; target_vel_y - dy];
end