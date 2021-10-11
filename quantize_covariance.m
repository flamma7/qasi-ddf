function [new_x_hat, new_P] = quantize_Covariance(x_hat, P, NUM_AGENTS)

    n = size(x_hat,1);
    new_x_hat = zeros(size(x_hat));
    new_P = P;

    pos_max = 11;
    vel_max = 1.0;
    P_max = 30;

    % POSITION STATES
    get_pos = zeros(2*NUM_AGENTS, 4*NUM_AGENTS);
    for i = 1:NUM_AGENTS
        get_pos(2*(i-1)+1:2*i, 4*(i-1)+1:4*(i-1)+2) = eye(2);
    end
    pos_states = get_pos * x_hat;

    b = 8;
    delta_s = pos_max / 2^(b-1);
    k = 1:2^b;
    codebook = flip(pos_max - k*delta_s);
    for s = 1:length(pos_states)
        state = pos_states(s);
        up_indices = find( codebook > state);

        % Check that at least one was greater
        if length(up_indices) < 1
            up_index = codebook(end);
        else
            up_index = up_indices(1);
        end

        % Check that 
        if up_index == 1
            x = codebook(1);
        else
            x_down_prob = ( codebook(up_index) - state) / delta_s;
            if binornd(1, x_down_prob)
                x = codebook(up_index - 1);
            else
                x = codebook(up_index);
            end
        end
        pos_states(s) = x;
    end

    new_x_hat = new_x_hat + get_pos' * pos_states;
    pos_states = sum(get_pos', 2);
    P = P + eye(n) .* pos_states * delta_s;

    % VELOCITY STATES
    get_vel = zeros(2*NUM_AGENTS, 4*NUM_AGENTS);
    for i = 1:NUM_AGENTS
        get_vel(2*(i-1)+1:2*i, 4*(i-1)+3:4*(i-1)+4) = eye(2);
    end
    vel_states = get_vel * x_hat;

    b = 8;
    delta_s = vel_max / 2^(b-1);
    k = 1:2^b;
    codebook = flip( vel_max - k*delta_s );
    for s = 1:length(vel_states)
        state = vel_states(s);
        up_indices = find( codebook > state);

        % Check that at least one was greater
        if length(up_indices) < 1
            disp("Velocity greater than compression bounds!");
            up_index = codebook(end);
        else
            up_index = up_indices(1);
        end

        % Check that 
        if up_index == 1
            state = codebook(1)
        else
            x_down_prob = ( codebook(up_index) - state) / delta_s;
            if binornd(1, x_down_prob)
                x = codebook(up_index - 1);
            else
                x = codebook(up_index);
            end
        end
        vel_states(s) = x;
    end

    new_x_hat = new_x_hat + get_vel' * vel_states
    vel_states = sum(get_vel', 2);
    P = P + eye(n) .* vel_states * delta_s

    % P QUANTIZATION

    b = 8;
    k = 1:2^b;
    delta_not = P_max / 2^(b-1);
    delta_d = (P_max + (n-1)*delta_not/2) / (2^b - 1);

    codebook_off = P_max - k*delta_not;
    codebook_diag = P_max + (n-1)*delta_not/2 - k*delta_d;
