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
BLUE_NUM = 2;
RED_NUM = 0;
NUM_AGENTS = BLUE_NUM + RED_NUM;
NUM_STATES = 4 * NUM_AGENTS; % Each agent has x,y,x_vel,y_vel
NUM_LOOPS = 500;
TIME_DELTA = 1;
MAP_DIM = 10;
PROB_DETECTION = 1.0;
SONAR_RANGE = 5.0;

% Noise Params
q = 0.05; % std
w = 0.1; % std
Q = eye(NUM_STATES);
for i =1:NUM_AGENTS % No process noise to the velocity
    Q(4*i-1,4*i-1) = 0;
    Q(4*i,4*i) = 0;
end

% Truth Data and Process Matrices
x_gt = randn(NUM_STATES,1);
for i =1:NUM_AGENTS % Initialize all velocities to zero
    x_gt(4*i-1,1) = 0;
    x_gt(4*i,1) = 0;
end
x_gt_history = zeros(NUM_STATES, NUM_LOOPS);
F = eye(NUM_STATES);
for i = 1:NUM_AGENTS
    F(4*i-3, 4*i-1) = 1;
    F(4*i-2, 4*i) = 1;
end

% Initialize Estimates
P = 0.1*eye(NUM_STATES);
x_hat = x_gt;
q_perceived = 0.01;
w_perceived = 0.01;

x_hat_history = zeros(NUM_STATES, NUM_LOOPS);
P_history = zeros(NUM_STATES, NUM_STATES*NUM_LOOPS);

% Control Input
accel = zeros(2*NUM_AGENTS,1);
U = zeros(NUM_STATES, 2*NUM_AGENTS);
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
    
    % Update and Store Truth
    accel = zeros(2*NUM_AGENTS,1);
    for i= 1:NUM_AGENTS
        accel(2*(i-1)+1 : 2*i,1) = get_velocity( x_gt(4*(i-1)+1 : 4*i , 1), waypoints(2*(i-1)+1:2*i,1) );
    end
    
    x_gt = F*x_gt + U*accel + Q*normrnd(0,q,NUM_STATES,1);
    x_gt_history(:,loop_num) = x_gt;

    % Propagate Filter
    x_hat = F*x_hat + U*accel;
    P = F*P*F' + q_perceived*Q;

    % Generate Measurements
    % (may use past state values)
    
    % Filter Measurements
    [x_hat, P] = filter_gps(x_hat,P,x_gt, w, w_perceived, NUM_AGENTS, NUM_STATES, 1);
    [x_hat, P] = filter_sonar(x_hat, P, x_gt, w, w_perceived, NUM_AGENTS, NUM_STATES, PROB_DETECTION, SONAR_RANGE);
%     for a = 1:NUM_AGENTS
%         [x_hat, P] = filter_gps(x_hat,P,x_gt, w, w_perceived, NUM_AGENTS, NUM_STATES, a);
%     end

    % Exchange estimates b/w agents

    % Store Estimates
    x_hat_history(:, loop_num) = x_hat;
    P_history(:, NUM_STATES*(loop_num-1)+1 : NUM_STATES*loop_num) = P;

    loop_num = loop_num + 1;
end

% Plot error
error = x_gt_history - x_hat_history;
plot_error(error, P_history, NUM_LOOPS, NUM_STATES, NUM_AGENTS);

% Make animation
make_animation(NUM_STATES, NUM_AGENTS, MAP_DIM, NUM_LOOPS, x_gt_history, x_hat_history, P_history);