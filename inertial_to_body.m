function [body, body_Q] = inertial_to_body(inertial, inertial_Q)
    % Converts inertial frame to body frame
    yaw = inertial(3);
    vel = sqrt( inertial(4)^2 + inertial(5)^2 )
    body = [inertial(1:3), vel, 0, inertial(6)]';

    % Construct the rotation matrix
    
end
