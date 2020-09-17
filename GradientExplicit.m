%% Robotarium SUSD Implamentation
% Robert Nelson 
% Georgia Tech Systems Research Lab
% rnelson71@gatech.edu

% Trying to get agents to move faster

%% Setup and Initialization
clc;
clear;
N = 4; % generate 4 agents, one for the source and 3 for the following 

% Matrix for the initial triangle agent positions
initial_poses = [0.2 0.3 0.5 0; 0.6 0.2 0.8 0; 1.0 1.0 1.0 0];

r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_poses);
si_to_uni = create_si_to_uni_dynamics(); % Reference to the function for unicycle dynamic creation
x = r.get_poses();
r.step();
%% SUSD Algorithm
q = [1; 0];
source = [0 0];
theta = pi/2;
counter = 0; % loop counter
graphed_infinity_points = 0;
while graphed_infinity_points < 20 % Stops once the full infinity has been created
%% Calculation of Covariance Matrix, n direction
    counter = counter + 1;
    agent_source_position = x(1:2, 4)'; % position of the malicious agent 

    if (mod(counter, 50) == 0) % Every 25 loop cycles
        graphed_infinity_points = graphed_infinity_points + 1;
        theta = theta + (1 / 5);
        source = [1.3 * cos(theta), 1.3 *cos(theta) * sin(theta)]; % Computes the location of points on the infinity based on angle
        if (mod(counter, 100) == 0)
            plot(agent_source_position(1), agent_source_position(2), 'r.','MarkerSize',5,'MarkerFaceColor','r');
        end
    end
    
    difference_vector = source - agent_source_position; % direction for the malicious agent to move in
    x = r.get_poses();
    
    % Virtual Field Calculation
    virtual_field_vals = zeros(1, N);
    k1= 1 / 1;
    virtual_field_vals_bounded = zeros(1, N);
    for i = 1:N % Calculating the virtual quadratic field observed at each agent
        virtual_field_vals(1, i) = ((2.5 * x(1, i) - 2.5 * agent_source_position(1))^2 + (2.5 * x(2, i) - 2.5 * agent_source_position(2))^2);
        virtual_field_vals_bounded(1, i) = k1 * virtual_field_vals(1, i) + .2; % Gives the value a base threshold for when tracking agents are close to malicious agent
    end
    rotation = [0, 1; -1, 0]; % 90 degree rotation matrix
    first = x(1:2, 1)';
    second = x(1:2, 2)';
    third = x(1:2, 3)';
    position = [first; second; third];
    center = mean(position); % Finds the average position of the agents for use in covariance calculation
    distance_from_center = position - center(ones(size(position,1),1),:);
    C = distance_from_center' * distance_from_center;
    tau=2; % Used for calculating covariance
    dtau=0.01;
    
    for t=0:dtau:tau
        q=q+(eye(2,2)-q*q')*C*q*dtau; % Algorithm for computing direction of maximum covariance
    end
    
    explicit_dir = agent_source_position - center;
    explicit_dir = explicit_dir / norm(explicit_dir)
    n_dir = rotation * q % Go in the direction perpendicular to the covariance
    difference_in_methods = abs(n_dir' - explicit_dir)
    n_dir = explicit_dir';
%% Formation Calculation
    weight_bounded = ones(3,3) * 1.2;
    rdot_formation = zeros(3,2); % Matrix to store agent's direction to maintain formation
    a = .4;
    b = .45;
    desired_distance=[0 b a;b 0 a;a a 0]; % Desired distances to the other agents
    for i=1:3
        for j=1:3 % Calculate the difference in desired position with respect to each other agent
            weight=norm(position(i,:)-position(j,:))-desired_distance(i,j);
            if weight>=0 % Distance to the other agent is larger than desired
                weight_bounded(i,j)=1-exp(-weight);
            else % Too close to the other agent
                weight_bounded(i,j)=-1+exp(weight);
            end
            % Adding the how much the current agent will have
            % to go to to each other agent. All directions will sum to
            % create a general direction to maintain formation
            rdot_formation(i,:)=rdot_formation(i,:)+weight_bounded(i,j)*(position(j,:)-position(i,:));
        end
    end
    
%% Calculations and Setting Velocities
    formation_gain = 8;
    virtual_field_gain = .2;
    ref_r1 = virtual_field_gain * 9 * n_dir + formation_gain * rdot_formation(1,:)';
    ref_r2 = virtual_field_gain * 9 * n_dir + formation_gain * rdot_formation(2,:)';
    ref_r3 = virtual_field_gain * 9 * n_dir + formation_gain * rdot_formation(3,:)';
    ref_r4 = 6 * difference_vector';
    ref_all = [ref_r1, ref_r2, ref_r3, ref_r4];
    ref_all = ref_all ./ 20;
    
    unicycle = si_to_uni(ref_all, x);
    r.set_velocities(1:N, unicycle);
    r.step();
end
%% End
r.debug();