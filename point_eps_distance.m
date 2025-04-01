function new_point = point_eps_distance(k, G, eps)
    % k and G are the 2D points (x, y), eps is the distance from k on the line to G.
    
    % Step 1: Calculate the direction vector from k to G
    direction = G - k;
    
    % Step 2: Normalize the direction vector
    direction_norm = direction / norm(direction);
    
    % Step 3: Scale the direction vector by eps
    offset = eps * direction_norm;
    
    % Step 4: Find the new point that is eps distance away from k
    new_point = k + offset;
end
