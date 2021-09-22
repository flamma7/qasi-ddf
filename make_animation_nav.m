function [] = make_animation_nav(STATES, NUM_AGENTS, MAP_DIM, NUM_LOOPS, x_gt_history, x_hat_history, P_history)

    TOTAL_STATES = STATES * NUM_AGENTS;

    positions = zeros(NUM_AGENTS,2);
    for i = 1:NUM_AGENTS
        positions(i,:) = x_gt_history( STATES*(i-1)+1 : STATES*(i-1)+2, 1)';
    end
    sizes = [75 repelem(25, NUM_AGENTS-1)];
    colors = linspace(1,10,NUM_AGENTS);

    % Plot the covariance bounds
    for i = 1:NUM_AGENTS
        mean = x_hat_history(STATES*(i-1)+1:STATES*(i-1)+2, 1);
        cov = P_history(STATES*(i-1)+1:STATES*(i-1)+2, STATES*(i-1)+1:STATES*(i-1)+2);
        ellipse = get_covariance(mean, cov);
        positions = [positions; ellipse];
        sizes = [sizes, repelem(1, length(ellipse))];
        colors = [colors, repelem(colors(1,i), length(ellipse))];
    end
    
    figure
    plot1 = scatter(positions(:,1), positions(:,2), sizes, colors, "filled");
    xlim([-MAP_DIM*(3/2),MAP_DIM*(3/2)]);
    ylim([-MAP_DIM*(3/2),MAP_DIM*(3/2)]);
    for l = 1:NUM_LOOPS
        positions = zeros(NUM_AGENTS,2);
        for i = 1:NUM_AGENTS
            positions(i,:) = x_gt_history( STATES*(i-1)+1 : STATES*(i-1)+2, l)';
        end
        for i = 1:NUM_AGENTS
            mean = x_hat_history(STATES*(i-1)+1:STATES*(i-1)+2, l);
            cov = P_history(STATES*(i-1)+1:STATES*(i-1)+2, STATES*(i-1)+1 + TOTAL_STATES*(l-1):STATES*(i-1)+2 + TOTAL_STATES*(l-1));
            ellipse = get_covariance(mean, cov);
            positions = [positions; ellipse];
        end

        plot1.XData = positions(:,1);
        plot1.YData = positions(:,2);
        title(int2str(l));
        pause(0.1);
    end

end