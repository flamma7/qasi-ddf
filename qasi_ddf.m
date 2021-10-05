function [] = qasi_ddf(mc_run_num)

    % QASI-DDF
    % Steps
    % 1. Regular KF 2D world w/ linrel, gps
    % 2. ASD w/ delayed GPS measurements
    % 3. Exchange estimates w/ PSCI & CI
    % 4. Quantize Covariances
    % 5. ICI
    % 6. ICI using tracked common information
    % 7. Increase number of agents & duration
    % 8. Selective information sharing (compare w/ random sharing)

    if nargin < 1
        rng(0);
        close all; clear all; clc;
        SAVE_FILE = false; 
    else
        SAVE_FILE = true;
    end

    CONFIGURATION = "CI";
    

    % Initialize
    BLUE_NUM = 2; % Blue agents come first in the state vector
    RED_NUM = 1;
    NUM_AGENTS = BLUE_NUM + RED_NUM;
    STATES = 6; % Each agent has x,y,theta, x_vel,y_vel, theta_vel
    TRACK_STATES = 4 * NUM_AGENTS; % x,y,x_dot, y_dot for each agent
    TOTAL_STATES = STATES * NUM_AGENTS; 
    TOTAL_TRACK_STATES = TRACK_STATES * BLUE_NUM;
    NUM_LOOPS = 500;
    TIME_DELTA = 1;
    MAP_DIM = 20; % Square with side length
    PROB_DETECTION = 1.0;
    SONAR_RANGE = 20.0;

    DELTA_RANGE = 1.0;
    DELTA_AZIMUTH = 0.1;

    AGENT_TO_PLOT = 2;
    assert( AGENT_TO_PLOT < BLUE_NUM + 1 )

    schedule_count = 1;

    % Noise Params
    q = 0.05; % std
    w = 0.1; % std
    w_gps = 1.0; % std
    q_perceived = q*q;
    w_perceived = w*w;
    w_gps_perceived = w_gps*w_gps;

    %% I know q_perceived is good for navigation filter
    % My tuning parameters are process noise and measurement noise for the sonar and modem
    % Let's just tune with sonars for now
    q_perceived_tracking = 0.1;
    w_perceived_nonlinear = 0.2;
    w_perceived_modem_range = w_perceived_nonlinear;
    w_perceived_modem_azimuth = w_perceived_nonlinear;

    w_perceived_sonar_range = w_perceived_nonlinear;
    w_perceived_sonar_azimuth = w_perceived_nonlinear;

    Q = eye(TOTAL_STATES);
    for i =1:NUM_AGENTS % No process noise to the velocity
        Q(STATES*i-2,STATES*i-2) = 0.1;
        Q(STATES*i-1,STATES*i-1) = 0.1;
        Q(STATES*i,STATES*i) = 0.1;
    end

    % Truth Data and Process Matrices
    x_gt = rand(TOTAL_STATES,1) * MAP_DIM - (MAP_DIM) / 2;
    for i =1:NUM_AGENTS % Initialize all velocities to zero
        x_gt(STATES*i-2,1) = 0;
        x_gt(STATES*i-1,1) = 0;
        x_gt(STATES*i,1) = 0;
    end
    x_gt = normalize_state(x_gt, NUM_AGENTS, STATES);
    x_gt_history = zeros(TOTAL_STATES, NUM_LOOPS); % x,y,theta,fwd, strafe, theta_dot

    % Initialize Navigation Filter Estimate
    P = 0.1*eye(STATES);
    x_navs = reshape( x_gt, STATES, NUM_AGENTS);
    P_navs = repmat(P, 1, NUM_AGENTS);
    % x_nav = x_gt;
    % P_nav = P;

    % I should create a block diagonal matrix to get initial positions from x_gt
    [x_hats, Ps] = initialize_x_hats(x_gt, P, NUM_AGENTS, STATES, BLUE_NUM);
    [x_common, P_common] = get_estimate(x_hats, Ps, 4, NUM_AGENTS, 1); % copy the initial estimate

    x_nav_history = zeros(size(x_gt_history));
    P_nav_history = zeros(TOTAL_STATES, STATES*NUM_LOOPS);
    x_hat_error_history = zeros(TOTAL_TRACK_STATES, NUM_LOOPS);
    P_history = zeros(TOTAL_TRACK_STATES, TRACK_STATES*NUM_LOOPS);

    % Control Input
    accel = zeros(2*NUM_AGENTS,1); % FWD acceleration and theta acceleration
    U = zeros(TOTAL_STATES, 2*NUM_AGENTS);
    for i = 1:NUM_AGENTS
        U(STATES*(i-1)+4, 2*(i-1)+1) = 1; % FWD velocity
        U(STATES*(i-1)+6, 2*(i-1)+2) = 1; % theta dot
    end

    % Initialize waypoints
    waypoints = randi(2*MAP_DIM, 2*NUM_AGENTS,1) - MAP_DIM;

    loop_num = 1;
    while loop_num < NUM_LOOPS + 1
        
        % Calculate new waypoints
        for i = 1:NUM_AGENTS
            delta = waypoints(2*(i-1)+1 : 2*i) - x_gt(STATES*(i-1)+1:STATES*(i-1)+2,1);
            if norm(delta) < 1
                waypoints(2*(i-1)+1 : 2*i) = randi(2*MAP_DIM, 2,1) - MAP_DIM;
                % disp('reached!');
            end
            
        end
        
        % Get control input
        accel = zeros(2*NUM_AGENTS,1);
        for i= 1:NUM_AGENTS
            accel(2*(i-1)+1 : 2*i,1) = get_velocity_nav( x_gt(STATES*(i-1)+1 : STATES*i , 1), waypoints(2*(i-1)+1:2*i,1) );
        end
        
        % Update Truth
        for i =1:NUM_AGENTS
            x_gt_agent = x_gt(STATES*(i-1)+1:STATES*i, 1);
            [x_gt_agent, Z] = propagate_nav(x_gt_agent, zeros(STATES));
            x_gt(STATES*(i-1)+1:STATES*i,1) = x_gt_agent;
        end
        x_gt = x_gt + U*accel + Q*normrnd(0,q,TOTAL_STATES,1); % normal propagation and add scaled noise
        x_gt = normalize_state(x_gt, NUM_AGENTS, STATES);
        x_gt_history(:,loop_num) = x_gt;

        % Initialize data for implicit updates
        x_bars = zeros(size(x_hats));
        P_bars = zeros(size(Ps));
        for a = 1:BLUE_NUM
            [x_bar, P_bar] = get_estimate(x_hats, Ps, 4, NUM_AGENTS, a);
            [x_bar, P_bar] = propagate(x_bar, P_bar, NUM_AGENTS, q_perceived_tracking);
            [x_bars, P_bars] = set_estimate(x_bars, P_bars, 4, x_bar, P_bar, a);
        end
        [x_common_bar, P_common_bar] = propagate(x_common, P_common, NUM_AGENTS, q_perceived_tracking);
        x_common = x_common_bar;
        P_common = P_common_bar;

        for a = 1:BLUE_NUM

            %% NAVIGATION FILTER PREDICTION
            [x_nav, P_nav] = get_estimate_nav(x_navs, P_navs, STATES, a);
            [x_nav, P_nav] = propagate_nav( x_nav, P_nav ); % Predict
            ts2a = zeros(STATES, TOTAL_STATES); % total states to agent ownship states permutation matrix
            ts2a(:, STATES*(a-1)+1 : STATES*a) = eye(STATES);
            x_nav = x_nav + (ts2a*U)*accel;
            x_nav = normalize_state(x_nav, 1, STATES);
            P_nav = P_nav + q_perceived*(ts2a * Q * ts2a');
            % NAVIGATION FILTER CORRECTION
            [x_nav, P_nav] = filter_dvl(x_nav, P_nav, x_gt, w, w_perceived, NUM_AGENTS, TOTAL_STATES, a); % DVL
            [x_nav, P_nav] = filter_gyro(x_nav, P_nav, x_gt, w, w_perceived, NUM_AGENTS, TOTAL_STATES, a); % Gyro
            [x_nav, P_nav] = filter_compass(x_nav, P_nav, x_gt, w, w_perceived, NUM_AGENTS, TOTAL_STATES, a); % Compass

            %% TRACKING FILTER PREDICTION
            [x_hat, P] = get_estimate(x_hats, Ps, 4, NUM_AGENTS, a);
            [x_hat, P] = propagate(x_hat, P, NUM_AGENTS, q_perceived_tracking); % Scale the process noise to account for nonlinearities
            % TRACKING FILTER CORRECTION
            % [x_hat, P] = filter_sonar(x_hat, P, x_gt, w, w_perceived_sonar_range, w_perceived_sonar_azimuth, NUM_AGENTS, STATES, PROB_DETECTION, SONAR_RANGE, a, x_nav);
            [x_hat, P, x_common, P_common, x_hats_, Ps_] = filter_sonar_et( ...
                                                                        x_hat, P, x_gt, w, w_perceived_sonar_range, w_perceived_sonar_azimuth, ... 
                                                                        NUM_AGENTS, STATES, PROB_DETECTION, SONAR_RANGE, a, x_nav, x_common_bar, ...
                                                                        x_bars, P_bars, x_common, P_common, DELTA_RANGE, DELTA_AZIMUTH);
            [x_hat, P] = modem_schedule(BLUE_NUM, NUM_AGENTS, loop_num, x_hat, P, x_hats, Ps, x_gt, w, w_perceived_modem_range, w_perceived_modem_azimuth, STATES, TRACK_STATES, a);

            %% INTERSECT TRACK & NAV FILTER
            [x_nav, P_nav, x_hat, P] = intersect_estimates(x_nav, P_nav, x_hat, P, a, STATES);

            %% SAVE ESTIMATES
            [x_hats, Ps] = set_estimate(x_hats, Ps, 4, x_hat, P, a);
            [x_navs, P_navs] = set_estimate(x_navs, P_navs, STATES, x_nav, P_nav, a);

            % Record history for plotting
            x_nav_history(STATES*(a-1)+1:STATES*a, loop_num) = x_nav;
            P_nav_history(STATES*(a-1)+1:STATES*a, STATES*(loop_num-1)+1 : STATES*loop_num) = P_nav;
            % x_hat_history(TRACK_STATES*(a-1)+1:TRACK_STATES*a, loop_num) = x_hat;
            P_history(TRACK_STATES*(a-1)+1:TRACK_STATES*a, TRACK_STATES*(loop_num-1)+1 : TRACK_STATES*loop_num) = P;

            track_error = get_error(x_gt, x_hat, NUM_AGENTS, STATES);
            x_hat_error_history(TRACK_STATES*(a-1)+1:TRACK_STATES*a, loop_num) = track_error;
        end
        loop_num = loop_num + 1;
    end

    % Plot error
    ts2a = zeros(STATES, TOTAL_STATES);
    ts2a(:, STATES*(AGENT_TO_PLOT-1)+1 : STATES*AGENT_TO_PLOT) = eye(STATES);

    error = ts2a * (x_gt_history - x_nav_history);
    error = normalize_state(error, 1, STATES);
    P_nav_history_agent = ts2a * P_nav_history;
    % plot_error_nav(error, P_nav_history, NUM_LOOPS, STATES, AGENT_TO_PLOT);
    %plot_norm_error(error);

    plot_error(x_hat_error_history, P_history, NUM_LOOPS, TRACK_STATES, STATES, NUM_AGENTS, AGENT_TO_PLOT);

    % Make animation
    % make_animation_nav(STATES, NUM_AGENTS, MAP_DIM, NUM_LOOPS, x_gt_history, x_nav_history, P_nav_history);

    if SAVE_FILE
        filename = "monte_carlos/" + CONFIGURATION + "_" + mc_run_num + "_x";
        writematrix(x_hat_error_history, filename);

        filename = "monte_carlos/" + CONFIGURATION + "_" + mc_run_num + "_P";
        writematrix(P_history, filename);
    end

end