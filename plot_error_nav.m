%% Plots the error plots for each state

function [] = plot_error_nav(error, P_history, NUM_LOOPS, STATES, agent)
    figure('units','normalized','outerposition',[0 0 1 1]); % full screen
    
    tiledlayout(1, STATES);
    
    state_names = ["x","y","theta","speed","strafe speed","theta vel"];
    for state = 1:STATES
        nexttile;
        row = state;
        plot(error(row, :),"red" );
        title(strcat("Agent ",int2str(agent),"'s ", state_names(state)));
        hold on;
        bound = zeros(NUM_LOOPS,1);
        for i=1:NUM_LOOPS
            bound(i,1) = 2*sqrt( P_history(row, (i-1)*STATES+row) );
        end
        plot(bound, "green");
        hold on;
        plot(-bound,"green");
    end
end