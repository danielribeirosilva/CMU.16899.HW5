function [path, flag] = aastar(s_start, s_goal, plan_map, params, n_restart)

if (nargin < 4)
   error('Needs at least 4 input parameters\n');
end

if (nargin < 5 )
   n_restart = 4;
end

[path, state_out, flag] = do_search(s_start, s_goal, plan_map, params);
while (~flag && n_restart > 1)
   [path2, state_out, flag]=do_search(state_out, s_goal, plan_map, params);
   path = [path; path2];
   n_restart = n_restart-1;
end

end