function [body_new, body_new_Q] = propagate_nav(body, body_Q)

    x = body(1);
    y = body(2);
    theta = body(3);
    v = body(4);
    vp = body(5);
    theta_dot = body(6);

    x_new = x + v * cos(theta);
    y_new = y + v * sin(theta);
    theta_new = normalize_angle( theta + theta_dot );

    body_new = [x_new, y_new, theta_new, v, vp, theta_dot]';

    dxdtheta = -v*sin(theta);
    dxdv = cos(theta);
    dydtheta = v * cos(theta);
    dydv = sin(theta);

    F = eye(size(body_Q));
    F(1,3) = dxdtheta;
    F(1,4) = dxdv;
    F(2,3) = dydtheta;
    F(2,4) = dydv;
    
    body_new_Q = F*body_Q * F';
end