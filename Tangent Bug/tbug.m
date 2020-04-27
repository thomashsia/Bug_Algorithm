%%%% Author: Boyang Xia %%%%

close all
clear all
clc

%% Configurations

J = complex(0, 1); % This will be used later.
pos_bug = [2, 2]; % Initialising the position of the bug, which will be updated later. [4, 5] [2, 1] [1, 8]
pos_goal = [10, 10];% % the Position of the Goal, which is fixed in this script. [8.5, 6]

step_size = 0.05; % the step size of the mobile robot is dependent on the map_size.
range = 1; % Range is the range of sensor around the robot.
sensor_res = 100; % Sensor Resolution on the sensor ray.
safety_factor = 9; % safety_factor affects the safety distance of mobile robot.
safety_dist = safety_factor * step_size;

obstacles = { 'rect', 'circle'; [4 12 4 4], [6 4 2] }; % Setting Obstacle Configuration
obs = obstacle(obstacles); % Function {get_obs}

%% Core Funtion of Tangent Bug Algorithm %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  d_f = inf; % Positive infinity. used to be d_f
d_xog = inf; % d_xog = d(x, o) + d(o, goal). used to be d_f_n

% k is for movie frame loop.
k = 1;

while true % if k = 1 is true, then start the motion.
    %% Firstly, we define and initialise some parameters.
    % dist_v is the distance vector between the goal and the bug.
    dist_v = pos_goal - pos_bug; 
    
    % dist is the distance (real number) between the goal and the bug.
    dist = sqrt(dist_v(1)^2 + dist_v(2)^2); 
    
    % bug_goal_angle is the angle between the goal and the bug.
    bug_goal_angle = angle_norm(angle(dist_v(1) + J*dist_v(2)) /pi*180); 
    
    %% Secondly
    if(dist <= step_size) % if d_reach is smaller than step size, then terminate (goal = bug)
        
        break
        
    end

    % Self-Defined Function {sense}, please check the .m if needed
    % t: theta; d: sensing distance; db: detection boundary.
    [t, d, db] = sense(obs, range, pos_bug, sensor_res);
    
    % Self-Defined Function {print_map}: illustrating obstacles, positions
    % of bug and goal, and detection boundary
    mapping(obs, pos_bug, pos_goal, db); 
    
    % Filming
    film(k) = getframe; 
    k = k + 1; % Frame index increases.
    
    % find the min of d and its index
    [d_min, dm_index] = min(d);
    
    %% Motion-to-Goal Behaviour
    
    if (d(floor(bug_goal_angle)) >= min(range, dist) && d(ceil(bug_goal_angle)) >= min(range, dist) && d_min > safety_dist)
        
        % the bug is moving directly to the goal point. 
        pos_bug = pos_bug + [cos(bug_goal_angle * pi/180), sin(bug_goal_angle * pi/180)] * step_size;
   
    else
    %% Boundary-Following Pre-Stage: detecting endpoints  
    
        % Self-Defined Function {endpoints}: This works not well, need
        % improvement.
        endpO = endpoints(d, range); % endpO is endpoints Oi.

        if (~isempty(endpO)) % if obstacle/endpoints are detected.
            
            for i = 1:length(endpO)
                
                plot(db(1, endpO(i)), db(2, endpO(i)), 'x', 'color', 'blue'); % Plot the det_boundary.
                
                % vector and distance between endpoints and goal.
                endp_goal_v = db(:, endpO(i)) - pos_goal';
                endp_goal_d = sqrt(sum(endp_goal_v.^2));
                
                % vector and distance between endpoints and bug.
                endp_bug_v = db(:, endpO(i)) - pos_bug';
                endp_bug_d = sqrt(sum(endp_bug_v.^2));
                
                if(d_xog > endp_goal_d + endp_bug_d) % minimisely update d_xog
                    
                    d_xog = endp_goal_d + endp_bug_d; 
                    endp_v = db(:, endpO(i))'; % endpoints vector
                    
                end
                
            end
            
            if (d_f > d_xog) % minimisely undate d_f and go_newendp
                
                d_f = d_xog;  
                go_endp = endp_v; 
                
                bug_endp = go_endp - pos_bug'; % distance vector
                bug_endp_angle = angle_norm(angle(bug_endp(1) + J * bug_endp(2)) * 180/pi); 
                
                
                if(d_min > safety_dist)
                    
                    % Move closer to the obstacle
                    pos_bug = pos_bug + [cos(bug_endp_angle * pi/180), sin(bug_endp_angle * pi/180)] * step_size;
                    
                else
                                        
                    % Back to the safe distance.
                    back = db(:, dm_index)' - pos_bug;
                    pos_bug = pos_bug - back/norm(back) * (safety_factor * step_size - d_min); % d(index_min)
                    
                    % Keep moving to the endpoints.
                    go_v = go_endp - db(:, dm_index)';
                    pos_bug = pos_bug + go_v/norm(go_v) * step_size;
                    
                end
                                
            else  % if d_f <= d_xog
                
                d_reach = d_f; % d_reach is the sum distance of min_dis and goal.
                
                if(((t(dm_index)) - bug_endp_angle) < 0 && d_min > safety_dist) % go upward
                    
                    bug_endp_angle = angle_norm(t(dm_index) - 90);
                    pos_bug = pos_bug + [cos(bug_endp_angle * pi/180), sin(bug_endp_angle * pi/180)] * step_size;
                
                elseif((t(dm_index)) - bug_endp_angle >= 0 && d_min > safety_factor * step_size) % go downward
                    
                    bug_endp_angle = angle_norm(t(dm_index) + 90);
                    pos_bug = pos_bug + [cos(bug_endp_angle * pi/180), sin(bug_endp_angle * pi/180)] * step_size;
                
                else % back to the safe distance.
                    
                    back = db(:,dm_index)' - pos_bug;
                    pos_bug = pos_bug - back / norm(back) * (safety_dist - d_min);
                    
                end

                
                while true %%%%%%%%%%%%%%%%%%%%%
                    
                    if(d_min > step_size) % n * step_size
                        
                        d_angle = angle_norm(t(dm_index) - 90); % d_angle
                        pos_bug = pos_bug + [cos(d_angle * pi/180), sin(d_angle * pi/180)] * step_size;
                        
                    else
                        
                        back = db(:,dm_index)' - pos_bug;
                        pos_bug = pos_bug - back/norm(back) * (step_size - d_min);
                        
                    end
                    
                    % update the relation of goal and bug
                    dist_v = pos_goal - pos_bug; % since the pos_bug is updated, so is diff.
                    dist = sqrt(dist_v(1)^2 + dist_v(2)^2);
                    bug_goal_angle = angle_norm(angle(dist_v(1) + J*dist_v(2)) /pi*180);
                    
                    if(dist < step_size || d_reach < d_f) % if (d_reach < step_size OR d_reach < d_f), terminate
                        
                        break
                        
                    end

                    % Self-Defined Function {sense}, please check the .m if needed
                    % t: theta; d: sensing distance; db: detection boundary.
                    [t, d, db] = sense(obs, range, pos_bug, sensor_res); % since the pos_bug is updated
                    [d_min, dm_index] = min(d);
                    
                    min_dis_goal = sqrt((db(1, dm_index) - pos_goal(1))^2 + (db(2, dm_index) - pos_goal(2))^2);
                    d_reach = d_min + min_dis_goal; % d_reach2 is updated.
                    
                    % Print updated map {print_map}
                    mapping(obs, pos_bug, pos_goal, db);
                    
                    % movie frame
                    film(k) = getframe;
                    k = k + 1;
                    
                end
                
            end
            
        else % (if (endp) isempty)
            
            disp('No Path Exist');
            break
            
        end
        
    end
    
end
