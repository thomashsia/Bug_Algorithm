%%%% Boyang Xia %%%%
%%% obstacles will be a cell that contains 
%%% polygos vx(first line), and vy(second line) s 
%%% by using size we will use it  thus (#of obstacles) will not be important

function [theta, distance, det_boundary] = sense(obstacle, range, pos, sensor_res)
%%%% obtacle is the output of Function {get_obs}: positions of each obstacle.
%%%% range is the angular range of the sensor.
%%%% pos = [pos_x, pos_y], is the position of the bug.
%%%% sensor_res is the sensor's resolution: range/sensor_res. i.e. sensor_res = 100 from the main.m.


size_obs = size(obstacle, 2); % size of the output of Function {get_obs}.
% size_obs = size_obs(2); % types of obstacles: i.e. circle, rect,
% boundary. in this case. size_obs = 3.

det_sample = 360; % sampling the detection.

%% Initialise the Output Arguments

% theta is linear segmentation of [0, 2*pi) with number of det_sample,
% length(theta) is value of res_sample.
theta = linspace(0, 2*pi * (1-1/det_sample), det_sample);

% distance is colunm vector(1, length(theta) all entries are value of
% range. distance is the detection range of the sensor.
distance = ones(1, length(theta)) * range;

% size(dist_pos) = 2, length(theta) = 2, 360 in this case
det_boundary = zeros(2, length(theta));

%% radiative detect

for i = 1 : det_sample % detect from 1 to 360
    
    % Segment the detect radius into unit given a fixed angle.
    unit_radius(1,:) = linspace(pos(1), pos(1) + range * cos(theta(i)), sensor_res);
    unit_radius(2,:) = linspace(pos(2), pos(2) + range * sin(theta(i)), sensor_res);
    
    % initialise the current_distance to sensor' s resolution
    current_distance = sensor_res; % ???
    
    for j = 1 : size_obs % detect each type of obstacle, in this case, size_obs = 3.
        
        obs_x = obstacle{1, j}; % the x position of obs j;
        obs_y = obstacle{2, j}; % the y position of obs j;
        
        % check whether the obstacle is detected.
        in = inpolygon(unit_radius(1,:), unit_radius(2,:), obs_x, obs_y); 
        
        in_status = find(in); % if in = 1 / is true, status is not empty
        
        if(~isempty(in_status)) % if in = 1 / status is not empty.
%             if(dummy(1) < curr_distance)
                
                current_distance = in_status(1); % current_distance = 1
                
%             end
        end
    end
    %% Output Arguments
    % distance of each angle: 
    % if no obstacle is detected, distance = range; 
    % if detected, distance = current
    distance(i) = (current_distance / sensor_res) * range ;
    
    % position of detection boundary of each angle:
    det_boundary(:,i) = [pos(1) + distance(i) * cos(theta(i)); pos(2) + distance(i) * sin(theta(i))];
end

theta = theta * 180/pi; 