%% Robotarium SUSD Implamentation
% Robert Nelson 
% Georgia Tech Systems Research Lab
% rnelson71@gatech.edu

%% Setup and Initialization
clc;
clear;
N = 4; % generate 4 agents, one for the source and 3 for the following 

% Matrix for the initial triangle agent positions
initial_poses = [0.6 0.7 0.9 0; 0.6 0.2 0.8 0; 1.0 1.0 1.0 0];

r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_poses);
si_to_uni = create_si_to_uni_dynamics();
x = r.get_poses();
r.step();
%% SUSD Algorithm
tic
q = [1; 0];
source = [0 0];
theta = pi/2;
counter = 0; % loop counter
main_counter = 0;
while main_counter < 200
%% Calculation of Covariance Matrix, n direction
    counter = counter + 1;
    agent_source_position = x(1:2, 4)';
    difference_vector = source - agent_source_position;

    if (mod(counter, 100) == 0)
        main_counter = main_counter + 1;
        theta = theta + (1 / 30);
        source = [cos(theta) cos(theta) * sin(theta)];
        if (mod(counter, 300) == 0)
            plot(agent_source_position(1), agent_source_position(2), 'r.','MarkerSize',5,'MarkerFaceColor','r');
        end
    end
    
    tau=2;dtau=0.01;
    x = r.get_poses();
    
    % Virtual Field Calculation
    virtual_field_vals = zeros(1, N);
    k1= 1 / 1;
    virtual_field_vals_bounded = zeros(1, N);
    for i = 1:N
        virtual_field_vals(1, i) = ((2.5 * x(1, i) - 2.5 * agent_source_position(1))^2 + (2.5 * x(2, i) - 2.5 * agent_source_position(2))^2);
        virtual_field_vals_bounded(1, i) = k1 * virtual_field_vals(1, i) + .2;
    end
    rotation = [0, 1; -1, 0];
    first = x(1:2, 1)';
    second = x(1:2, 2)';
    third = x(1:2, 3)';
    position = [first; second; third];
    center = mean(position);
    distance_from_center = position - center(ones(size(position,1),1),:);
    C = distance_from_center' * distance_from_center;
    
    for t=0:dtau:tau
        q=q+(eye(2,2)-q*q')*C*q*dtau; 
    end
    n_dir = rotation * q;

    
%% Formation Calculation
    weight_bounded = ones(3,3) * 1.2;
    rdot_formation = zeros(3,2);
    a = .5;
    b = .55;
    desired_distance=[0 b a;b 0 a;a a 0];
    for i=1:3
        for j=1:3
            weight=norm(position(i,:)-position(j,:))-desired_distance(i,j);
            if weight>=0
                weight_bounded(i,j)=1-exp(-weight);
            else
                weight_bounded(i,j)=-1+exp(weight);
            end
            rdot_formation(i,:)=rdot_formation(i,:)+weight_bounded(i,j)*(position(j,:)-position(i,:));
        end
    end
    
%% Calculations and Setting Velocities
    formation_gain = 4;
    ref_r1 = .1 * virtual_field_vals(1,1) * n_dir + formation_gain * rdot_formation(1,:)';
    ref_r2 = .1 * virtual_field_vals(1,2) * n_dir + formation_gain * rdot_formation(2,:)';
    ref_r3 = .1 * virtual_field_vals(1,3) * n_dir + formation_gain * rdot_formation(3,:)';
    ref_r4 = 2* difference_vector';
    ref_all = [ref_r1, ref_r2, ref_r3, ref_r4];
    ref_all = ref_all ./ 30;
    
    unicycle = si_to_uni(ref_all, x);
    r.set_velocities(1:N, unicycle);
    r.step();
end
%% End
r.debug();