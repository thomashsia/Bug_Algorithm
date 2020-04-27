function index = endpoints(d, range)

% ind = zeros(1, length(d));

for i = 1:length(d)
   
    if d(i) == range % eliminate all unused sensors
        
        d(i) = 0;
        
    else % set all used sensor to 1
        
        d(i) = 1;
        
    end
    
end

k = [d, d(1)];
f = zeros(1, length(d));

for i = 1: length(d)
    
    f(i) = k(i) - k(i+1);

end
index = find(f ~= 0);
end