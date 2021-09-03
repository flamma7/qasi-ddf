function [x_hat, P] = covariance_intersect(a_hat, A, b_hat, B)
    A_inv = inv(A);
    B_inv = inv(B);
    
    cost = @(w) trace(inv( w*A_inv + (1-w)*B_inv ) );
    
    w_optimal = fminbnd(cost, 0, 1);
    
    P = inv( w_optimal * A_inv + (1-w_optimal) * B_inv );
    x_hat = P*(w_optimal * A_inv *a_hat + (1-w_optimal)*B_inv*b_hat);
end