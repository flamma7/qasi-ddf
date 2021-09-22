close all; clear all; clc;
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

rng(0);

% Initialize
BLUE_NUM = 1; % Blue agents come first in the state vector
RED_NUM = 0;
NUM_AGENTS = BLUE_NUM + RED_NUM;
STATES = 6; % Each agent has x,y,theta, x_vel,y_vel, theta_vel
TOTAL_STATES = STATES * NUM_AGENTS; 
NUM_LOOPS = 500;
TIME_DELTA = 1;
MAP_DIM = 20;
PROB_DETECTION = 1.0;
SONAR_RANGE = 10.0;

AGENT_TO_PLOT = 1;

schedule_count = 1;

% Noise Params
q = 0.05; % std
w = 0.1; % std
w_gps = 1.0; % std
q_perceived = q*q;
w_perceived = w*w;
w_gps_perceived = w_gps*w_gps;

Q = eye(TOTAL_STATES);
for i =1:NUM_AGENTS % No process noise to the velocity
    Q(STATES*i-2,STATES*i-2) = 0.1;
    Q(STATES*i-1,STATES*i-1) = 0.1;
    Q(STATES*i,STATES*i) = 0.1;
end

% Truth Data and Process Matrices
x_gt = randn(TOTAL_STATES,1);
for i =1:NUM_AGENTS % Initialize all velocities to zero
    x_gt(STATES*i-2,1) = 0;
    x_gt(STATES*i-1,1) = 0;
    x_gt(STATES*i,1) = 0;
end
x_gt_history = zeros(TOTAL_STATES, NUM_LOOPS);
F = eye(TOTAL_STATES);
for i = 1:NUM_AGENTS
    F(STATES*i-5, STATES*i-2) = 1;
    F(STATES*i-4, STATES*i-1) = 1;
    F(STATES*i-3, STATES*i) = 1;
end

% Initialize Estimates
P = 0.1*eye(TOTAL_STATES);
x_hats = repmat(x_gt, 1, BLUE_NUM);
Ps = repmat(P, 1,BLUE_NUM);

x_hat_history = zeros(TOTAL_STATES, NUM_LOOPS);
P_history = zeros(TOTAL_STATES, TOTAL_STATES*NUM_LOOPS);

% Control Input
accel = zeros(2*NUM_AGENTS,1);
U = zeros(TOTAL_STATES, 2*NUM_AGENTS);
for i = 1:NUM_AGENTS
    U(4*i-1,2*i-1) = 1;
    U(4*i,2*i) = 1;
end

% Initialize waypoints
waypoints = randi(2*MAP_DIM, 2*NUM_AGENTS,1) - MAP_DIM;

loop_num = 1;
while loop_num < NUM_LOOPS
    
    % Calculate new waypoints
    for i = 1:NUM_AGENTS
        delta = waypoints(2*(i-1)+1 : 2*i) - x_gt(4*(i-1)+1:4*i-2,1);
        if norm(delta) < 1
            waypoints(2*(i-1)+1 : 2*i) = randi(2*MAP_DIM, 2,1) - MAP_DIM;
        end
    end
    
    % Get control input
    accel = zeros(2*NUM_AGENTS,1);
    for i= 1:NUM_AGENTS
        accel(2*(i-1)+1 : 2*i,1) = get_velocity( x_gt(4*(i-1)+1 : 4*i , 1), waypoints(2*(i-1)+1:2*i,1) );
    end
    
    % Update and store truth
    x_gt = F*x_gt + U*accel + Q*normrnd(0,q,TOTAL_STATES,1);
    x_gt_history(:,loop_num) = x_gt;

    % Run prediction step on each filter
    for a = 1:BLUE_NUM
        [x_hat, P] = get_estimate(x_hats, Ps, TOTAL_STATES, a);
        my_accel = zeros(size(accel));
        my_accel(2*a -1:2*a,1) = accel(2*a-1:2*a,1);
        
        % Propagate Filter
        x_hat = F*x_hat + U*my_accel;
        P = F*P*F' + q_perceived*Q;

        % Filter Measurements
        [x_hat, P] = filter_dvl(x_hat,P,x_gt, w, w_perceived, NUM_AGENTS, TOTAL_STATES, a);
        [x_hat, P] = filter_sonar(x_hat, P, x_gt, w, w_perceived, NUM_AGENTS, TOTAL_STATES, PROB_DETECTION, SONAR_RANGE);
        
        [x_hats, Ps] = set_estimate(x_hats, Ps, TOTAL_STATES, x_hat, P, a);
    end
    
    % Schedule
    % [3,3,4, 4, 4]
    if schedule_count== 9 % Global Measurements
        for a = 1:BLUE_NUM
            [x_hat, P] = get_estimate(x_hats, Ps, TOTAL_STATES, a);
            [x_hat, P] = filter_gps(x_hat,P,x_gt, w_gps, w_gps_perceived, NUM_AGENTS, TOTAL_STATES, 0);
            [x_hats, Ps] = set_estimate(x_hats, Ps, TOTAL_STATES, x_hat, P, a);
        end
    else % Broadcast states
       for a = 1:BLUE_NUM
           if schedule_count== 4*a + 9
               [x_hat1, P1] = get_estimate(x_hats, Ps, TOTAL_STATES, a);
               for a2 = 1:BLUE_NUM
                  if a == a2
                      continue
                  else
                      [x_hat2, P2] = get_estimate(x_hats, Ps, TOTAL_STATES, a2);
                      [c_hat, C] = covariance_intersect(x_hat1, P1, x_hat2, P2);
                      [x_hats, Ps] = set_estimate(x_hats,Ps, TOTAL_STATES, c_hat, C, a2);
                  end
              end
              break 
           end
       end
       
       if schedule_count== 4*BLUE_NUM + 9
           schedule_count= 0;
       end
    end
    schedule_count = schedule_count + 1;

    % Exchange estimates b/w agents

    % Store Estimates
    [x_hat, P] = get_estimate(x_hats, Ps, TOTAL_STATES, AGENT_TO_PLOT);
    x_hat_history(:, loop_num) = x_hat;
    P_history(:, TOTAL_STATES*(loop_num-1)+1 : TOTAL_STATES*loop_num) = P;

    loop_num = loop_num + 1;
end

% Find maximum of each element
% maxes = zeros(size(P));
% for i = 1:TOTAL_STATES
%     elem_history = [];
%     for k = 1:NUM_LOOPS
%         elem_history = [elem_history, P_history(i,1+(k-1)*TOTAL_STATES)];
%     end
%     maxes(i,1) = max(abs(elem_history));    
% end
% maxes(:,1)

% Plot error
error = x_gt_history - x_hat_history;
%plot_error(error, P_history, NUM_LOOPS, TOTAL_STATES, NUM_AGENTS);
%plot_norm_error(error);

% Make animation
%make_animation(TOTAL_STATES, NUM_AGENTS, MAP_DIM, NUM_LOOPS, x_gt_history, x_hat_history, P_history);