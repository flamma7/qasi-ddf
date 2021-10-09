function [x_hat, P, x_common, P_common, ledger, x_hats, Ps, explicit_cnt, implicit_cnt, last_share_index] = dt_modem_schedule(...
                        BLUE_NUM, NUM_AGENTS, loop_num, x_hat, P, x_hats, Ps, x_gt, w, w_perceived_modem_range, ...
                        w_perceived_modem_azimuth, w_perceived_sonar_range, w_perceived_sonar_azimuth, ...
                        q_perceived_tracking, STATES, TRACK_STATES, agent, x_common, P_common, ledger, last_share_index,...
                        delta_range, delta_azimuth, max_num_meas, x_navs_history, P_navs_history, x_hat_history, P_history, ...
                        explicit_cnt, implicit_cnt)
    
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
        [x_hat, P, ledger] = dt_filter_modem(x_hat, P, x_gt, w, w_perceived_modem_range, w_perceived_modem_azimuth, BLUE_NUM, STATES, TRACK_STATES, ledger, agent, loop_num);
    else % Check if an agent is sharing
        x_common_debug = x_common;
        P_common_debug = P_common;

        if agent_share_times(agent) == iter % Agent's turn to share

            [x_common_debug, P_common_debug, mult, share_buffer, new_explicit_cnt, new_implicit_cnt] = pull_buffer( ...
                            last_share_index, loop_num, x_common, P_common, ledger, delta_range, delta_azimuth, ...
                            max_num_meas, agent, q_perceived_tracking, w_perceived_modem_range, ...
                            w_perceived_modem_azimuth, w_perceived_sonar_range, w_perceived_sonar_azimuth, NUM_AGENTS);
            explicit_cnt = explicit_cnt + new_explicit_cnt;
            implicit_cnt = implicit_cnt + new_implicit_cnt;
            % print_buffer_stats(share_buffer)
            % if loop_num > 640
            %     original_buffer = share_buffer;
            %     share_buffer = quantize_buffer(share_buffer);
            %     original_buffer
            %     share_buffer
            %     diff = original_buffer - share_buffer
            % else
            %     
            % end
            share_buffer = quantize_buffer(share_buffer);

            for b = 1:BLUE_NUM % Loop through all agents and share measurements
                if b == agent % Don't share with ourselves...
                    continue
                end
                % disp("Agent " + int2str(agent) + " sharing with agent " + int2str(b));
                [last_x_hat, last_P] = get_estimate_index(x_hat_history, P_history, TRACK_STATES, last_share_index, b);

                % Get x_nav_history from x_navs_history
                [x_nav_history_b, P_nav_history_b] = get_estimate_nav_history(x_navs_history, P_navs_history, STATES, b);

                [x_hat_b1, P_b1] = get_estimate(x_hats, Ps, 4, NUM_AGENTS, b);

                [x_hat_b, P_b, x_common_debug2, P_common_debug2] = receive_buffer( ...
                            last_x_hat, last_P, last_share_index, loop_num-1, share_buffer, mult, ledger, ...
                            x_common, P_common, b, NUM_AGENTS, q_perceived_tracking, ...
                            delta_range, delta_azimuth, STATES, x_nav_history_b, P_nav_history_b, ...
                            w_perceived_modem_range, w_perceived_modem_azimuth, w_perceived_sonar_range, ...
                            w_perceived_sonar_azimuth);
                % assert(isequal(x_common_debug, x_common_debug2));
                % assert(isequal(P_common_debug, P_common_debug2));

                % Set the last_x_hats, last_Ps
                [x_hats, Ps] = set_estimate(x_hats, Ps, x_hat_b, P_b, b);
                
            end % for
            last_share_index = loop_num;
        end % if 
        x_common = x_common_debug;
        P_common = P_common_debug;
    end % else
end % fxn