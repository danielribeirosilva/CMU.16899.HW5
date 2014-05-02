function [my_current_state] = get_current_state(state,observed_map,path,current_path_idx,distance_to_border,square_size,M,N)

    %square around car
    x_cur = round(state.x); 
    y_cur = round(state.y);
    observed_surrounding = zeros(square_size,square_size);
    x_min = max(1,x_cur - distance_to_border);
    x_max = min(M,x_cur + distance_to_border);
    y_min = max(1,y_cur - distance_to_border);
    y_max = min(N,y_cur + distance_to_border);
    center = distance_to_border + 1;
    pos_y_min = center-(y_cur-y_min);
    pos_y_max = center+(y_max-y_cur);
    pos_x_min = center-(x_cur-x_min);
    pos_x_max = center+(x_max-x_cur);
    observed_surrounding(pos_y_min:pos_y_max,pos_x_min:pos_x_max) = observed_map(y_min:y_max,x_min:x_max);
    row_observed_surrounding = reshape(observed_surrounding',1,numel(observed_surrounding));
    
    %orientation
    observed_theta = mod(state.theta,2*pi);
    if(observed_theta < 0)
        observed_theta = observed_theta + 2*pi;
    end
    
    %angle between orientation and path
    if(current_path_idx < size(path.x,1))
        path_x_A = path.x(current_path_idx);
        path_x_B = path.x(current_path_idx+1);
        path_y_A = path.y(current_path_idx);
        path_y_B = path.y(current_path_idx+1);
    else
        path_x_A = path.x(end-1);
        path_x_B = path.x(end);
        path_y_A = path.y(end-1);
        path_y_B = path.y(end);
    end
    
    current_path_vector = [path_x_B path_y_B]-[path_x_A path_y_A];
    
    reference_vector = [1 0];
    path_orientation = acos( dot(current_path_vector, reference_vector) / (norm(current_path_vector)*norm(reference_vector)) );
    angle_difference = path_orientation - observed_theta;
    
    %distance to path segment
    v = [path_y_B - path_y_A; path_x_A - path_x_B];
    r = [path_x_A - state.x; path_y_A - state.y];
    distance_to_path = abs(dot(v,r))/norm(v);

    my_current_state = [row_observed_surrounding,observed_theta,angle_difference,distance_to_path];

end