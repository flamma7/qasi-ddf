function [r_ellipse] = get_covariance(mean, cov)

    [V,D] = eig(cov);
    evals = diag(D);
    [argmaxvalue, argmax] = max(evals);
    [argminvalue, argmin] = min(evals);
    largest_eigenvector = V(:,argmax);
    angle = atan2( largest_eigenvector(2,1), largest_eigenvector(1,1));
    if(angle < 0)
        angle = angle + 2*pi;
    end

    chisquare_val = 2.4477;
    a=chisquare_val*sqrt(argmaxvalue);
    b=chisquare_val*sqrt(argminvalue);
    theta_grid = linspace(0,2*pi, 500);
    ellipse_x_r  = a*cos( theta_grid );
    ellipse_y_r  = b*sin( theta_grid );
    %Define a rotation matrix
    R = [ cos(angle) sin(angle); -sin(angle) cos(angle) ];

    %let's rotate the ellipse to some angle phi
    r_ellipse = [ellipse_x_r;ellipse_y_r]' * R;
    
    % Translate the ellipse to be centered on the mean
    r_ellipse(:,1) = r_ellipse(:,1) + mean(1,1);
    r_ellipse(:,2) = r_ellipse(:,2) + mean(2,1);
end