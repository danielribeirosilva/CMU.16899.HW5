% Run the path
% The variable 'path' must exist in the workspace

initialize_state;

for i=1:numel(path)
   
   action = path(i);
   [state, observed_map, flags] = motionModel(params, state, action, observed_map, map, goal);
   
%    plot(scale*state.x,scale*state.y,'b.', 'MarkerSize', 2*scale);
%    line(scale*[state.x,state.x+params.length/2*cos(state.theta)]',scale*[state.y,state.y+params.length/2*sin(state.theta)]','Color','Blue', 'LineSmoothing', 'on');

   display_environment;
   if (flags == 1 || flags ==2)
      break;
   end
end

hold off;