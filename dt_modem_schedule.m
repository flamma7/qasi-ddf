function [x_hat, P, x_common, P_common, ledger] = dt_modem_schedule(BLUE_NUM, NUM_AGENTS, loop_num, x_hat, P, x_hats, Ps, x_gt, w, w_perceived_modem_range, w_perceived_modem_azimuth, STATES, TRACK_STATES, agent, x_common, P_common, ledger)

    ping_delay = 3;
    broadcast_delay = 4;

    ping_time = 3*BLUE_NUM + broadcast_delay;
    agent_share_times = [];
    for b = 1:BLUE_NUM
        agent_share_times = [agent_share_times, ping_time + broadcast_delay*b];
    end
    total_time = agent_share_times(end);
    iter = mod( loop_num, total_time );

    % Surface broadcasts modem measurements
    if iter == ping_time
        [x_hat, P, ledger] = dt_filter_modem(x_hat, P, x_gt, w, w_perceived_modem_range, w_perceived_modem_azimuth, BLUE_NUM, STATES, TRACK_STATES, ledger, agent, loop_num);
    else % Check if an agent is sharing
        for b = 1:BLUE_NUM
            % When agent shares fuse with the other agents, not this one
            if b == agent
                continue
            elseif agent_share_times(b) == iter % This agent is sharing
                [x_hat_b, P_b] = get_estimate(x_hats, Ps, 4, NUM_AGENTS, b);
                % [x_hat, P] = covariance_intersect(x_hat, P, x_hat_b, P_b); % Intersect estimates
                
            end
        end
    end


end