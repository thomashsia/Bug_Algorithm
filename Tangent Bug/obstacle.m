%%%% Auther: Boyang Xia %%%%
%%%% This function is for generating obstacles. %%%%
%%%% The input arguments are types and property of the obstacles;
%%%% The output arguments are the locations of each obstacles.

function obstacles = obstacle(names)
%% Obtain the size of the inarg, initialise the ouarg
num_obs = size(names); % obtain the size of the input arguments
obstacles = cell(num_obs); % setting cell matrix placeholder with size of input argument

%% Check the type of obstacles and their properties (position)
for i = 1 : num_obs(2) % from 1 to the max colunm index of inarg
    
    obs_name = names{1, i}; % Names of Obstacles
    obs_pos = names{2, i}; % Positions of Obtacles
    
    switch obs_name
        
        case 'circle' % This case we use
            
            %for a circle x & y position of center and radius should be given
            res = 40;
            ang = linspace(0, 2*pi, res); % Linear Segmentation of 2*pi
            rad = ones(1,res) * obs_pos(3); % (1 by res I vector) * (radius)
            [X, Y] = pol2cart(ang, rad); % Function {pol2cart}
            %vertices of circle is obtained
            X = X(1, :) + obs_pos(1); % X position of each dice
            Y = Y(1, :) + obs_pos(2); % Y position of each dice
            
        case 'rect' % This case we use
            %for a square or rectangle x&y position of top-left corner
            %and length(parallel to x axis) and width(parallel to y axis)
            %vertices of rectangle is obtained
            X = [obs_pos(1); obs_pos(1)+obs_pos(3); obs_pos(1)+obs_pos(3);            obs_pos(1)];
            Y = [obs_pos(2);            obs_pos(2); obs_pos(2)-obs_pos(4); obs_pos(2)-obs_pos(4)];
            

%         case 'boundary'
%             % boundry object with mapsize
%             % vertices of object is obtained
%             X = [0; 1; 1; 0; -0.05; -0.05; 1.05; 1.05 ;-0.05;-0.05; 0] * obs_pos(1);
%             Y = [0; 0; 1; 1;     1;  1.05; 1.05;-0.05 ;-0.05;    1; 1] * obs_pos(1);
       
        
        otherwise % if not the above cases
            X = [1, 0, -1,  0];
            Y = [0, 1,  0, -1];
    end
    %% Outarg
    obstacles{1, i} = X; % the first row is the X position of the obstacle
    obstacles{2, i} = Y; % the second row is the Y position of the obstacle
end