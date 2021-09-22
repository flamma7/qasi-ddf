%% Plots the error plots for each state

function [] = plot_error_nav(error, P_history, NUM_LOOPS, TOTAL_STATES, NUM_AGENTS)
    figure('units','normalized','outerposition',[0 0 1 1]); % full screen
    
    STATES = TOTAL_STATES / NUM_AGENTS;
    tiledlayout(NUM_AGENTS, STATES);
    
    state_names = ["x","y","theta","speed","strafe speed","theta vel"];
    for agent = 1:NUM_AGENTS
       for state = 1:STATES
           nexttile;
           row = (agent-1)*STATES + state;
           plot(error(row, :),"red" );
           title(strcat("Agent ",int2str(agent),"'s ", state_names(state)));
           hold on;
           bound = zeros(NUM_LOOPS,1);
            for i=1:NUM_LOOPS
                bound(i,1) = 2*sqrt( P_history(row, (i-1)*TOTAL_STATES+row) );
            end
            plot(bound, "green");
            hold on;
            plot(-bound,"green");
    end
    
end