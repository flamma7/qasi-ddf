%% Plots the error plots for each state

function [] = plot_error(error, P_history, NUM_LOOPS, TRACK_STATES, STATES, NUM_AGENTS, agent)
    figure('units','normalized','outerposition',[0 0 1 1]); % full screen

    error = error(TRACK_STATES*(agent-1)+1: TRACK_STATES*agent, :);
    P_history = P_history(TRACK_STATES*(agent-1)+1: TRACK_STATES*agent, :);
    
    states_per_agent = TRACK_STATES / NUM_AGENTS;

    tiledlayout(NUM_AGENTS, states_per_agent);
    
    state_names = ["x","y","x vel", "y vel"];
    for a = 1:NUM_AGENTS
       for state = 1:states_per_agent
           nexttile;
           row = (a-1)*states_per_agent + state;
           plot(error(row, :),"red" );
           title(strcat("Agent ",int2str(a),"'s ", state_names(state)));
           hold on;
           bound = zeros(NUM_LOOPS,1);
            for i=1:NUM_LOOPS
                bound(i,1) = 2*sqrt( P_history(row, (i-1)*TRACK_STATES+row) );
            end
            plot(bound, "green");
            hold on;
            plot(-bound,"green");
    end
    
end