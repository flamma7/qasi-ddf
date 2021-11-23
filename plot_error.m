%% Plots the error plots for each state

function [] = plot_error(error, P_history, NUM_LOOPS, TRACK_STATES, STATES, NUM_AGENTS, BLUE_NUM, agent, plot_title)
    figure('units','normalized','outerposition',[0 0 1 1]); % full screen

    ping_delay = 3;
    broadcast_delay = 4;
    ping_time = 3*BLUE_NUM + broadcast_delay;
    agent_share_times = [];
    for b = 1:BLUE_NUM
        agent_share_times = [agent_share_times, ping_time + broadcast_delay*b];
    end
    total_time = agent_share_times(end) + 1;
    modem_times = [];
    broadcast_times = [];
    for t = 1:NUM_LOOPS
        iter = mod( t, total_time );
        if iter == ping_time
            modem_times = [modem_times, t];
        else
            for at = agent_share_times
                if iter == at
                    broadcast_times = [broadcast_times, t];
                end
            end
        end
    end

    error = error(TRACK_STATES*(agent-1)+1: TRACK_STATES*agent, :);
    P_history = P_history(TRACK_STATES*(agent-1)+1: TRACK_STATES*agent, :);
    
    states_per_agent = TRACK_STATES / NUM_AGENTS;

    tiledlayout(NUM_AGENTS-1, states_per_agent);
    
    state_names = ["x","y","x vel", "y vel"];
    for a = 1:NUM_AGENTS
        if a == agent
            continue
        end
       for state = 1:states_per_agent
           nexttile;
           row = (a-1)*states_per_agent + state;
           plot(error(row, :),"red" );
           title(strcat("Agent ",int2str(a),"'s ", state_names(state)));
           ylabel("Error");
           xlabel("Time")
           ax = gca;
            ax.TitleFontSizeMultiplier = 2;
            ax.FontSize = 16
           hold on;
           bound = zeros(NUM_LOOPS,1);
            for i=1:NUM_LOOPS
                bound(i,1) = 2*sqrt( P_history(row, (i-1)*TRACK_STATES+row) );
            end
            plot(bound, "green");
            hold on;
            plot(-bound,"green");
            lgd = legend("Error", "2\sigma Bound");
            lgd.FontSize = 12;

            % For modem activity...
            % xline(modem_times, '--b');
            % xline(broadcast_times, "--c");
    end
    % sgtitle(plot_title);
    
end