%%%% Author: Boyang Xia %%%%
%%%% This script is designed for displaying the animation of movement.
%%%% Ploting the detection boundary, obstacles 
%%%% and positions of bug and goal.

function a = mapping(obs, pos_bug, pos_goal, dp)
    
    hold off
    %% Plot the detection/sensor boundary
    plot(dp(1,:), dp(2,:), '.'); %, 'color', 'blue') 
    hold on
    
    %% Plot the position of bug and goal point.
    plot(pos_bug(1), pos_bug(2), '.', 'color', 'black')
    plot(pos_goal(1), pos_goal(2), 'O', 'color', 'red')
    
    [a, b] = size(obs);
    
    %% fill the obstacles
    for i=1:b
        
        fill(obs{1,i}, obs{2,i}, 'w')
        grid on
        axis([0, 12, 0, 12])
    
    end
end
