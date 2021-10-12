function [x_hat, P, x_common, P_common, ledger, x_hats, Ps, explicit_cnt, implicit_cnt, last_share_index] = dt_modem_schedule(...
                        BLUE_NUM, NUM_AGENTS, loop_num, x_hat, P, x_hats, Ps, x_gt, w, w_perceived_modem_range, ...
                        w_perceived_modem_azimuth, w_perceived_sonar_range, w_perceived_sonar_azimuth, ...
                        q_perceived_tracking, STATES, TRACK_STATES, agent, x_common, P_common, ledger, last_share_index,...
                        delta_range, delta_azimuth, max_num_meas, x_navs_history, P_navs_history, x_hat_history, P_history, ...
                        explicit_cnt, implicit_cnt, MODEM_LOCATION)
    
    % x_hat, P needed for modem measurement sharing
    % x_common, P_common, ledger, x_hats, Ps needed for sharing of measurements

    ping_delay = 3;
    broadcast_delay = 4;

    ping_time = ping_delay*BLUE_NUM + broadcast_delay;
    agent_share_times = [];
    for b = 1:BLUE_NUM
        agent_share_times = [agent_share_times, ping_time + broadcast_delay*b];
    end
    % agent_share_times = [17,11]; % TODO rem
    total_time = agent_share_times(end) + 1;
    iter = mod( loop_num, total_time );

    % Surface broadcasts modem measurements
    if iter == ping_time
        % TODO add
        [x_hat, P, ledger] = dt_filter_modem(x_hat, P, x_gt, w, w_perceived_modem_range, w_perceived_modem_azimuth, BLUE_NUM, STATES, TRACK_STATES, ledger, agent, loop_num, MODEM_LOCATION);
    else % Check if an agent is sharing
        for b = 1:BLUE_NUM
            if b == agent
                continue
            elseif agent_share_times(agent) == iter
                [x_hat_b, P_b] = get_estimate(x_hats, Ps, 4, NUM_AGENTS, b);
                [x_hat_quant, P_quant] = quantize_covariance(x_hat, P, NUM_AGENTS);
                [x_hat_b, P_b] = covariance_intersect(x_hat_quant, P_quant, x_hat_b, P_b); % Intersect estimates
                [x_hats, Ps] = set_estimate(x_hats, Ps, x_hat_b, P_b, b);
            end
            last_share_index = loop_num;
        end
    end % else
end % fxn