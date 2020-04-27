%%%% Author: Boyang Xia %%%% 

function a = angle_norm(f) % This function is limitting the angle f within {0, 2pi}

a = f;

if(f >= 360)
    
    a = f - 360;
    
elseif(f < 0)
    
    a = f + 360;
    
end
    