function keys_listener(src,event) 
    delta = 0.05;
    min_control = -1;
    max_control = 1;
    reverse = -2;
    switch event.Key
      %adjust selected control (increase control)
      case 'rightarrow'
        current_control = getappdata(0, 'control');
        if current_control + delta <= max_control
            setappdata(0, 'control', current_control + delta);
            fprintf('current control: %f',current_control + delta);
        else
            setappdata(0, 'control', max_control);
        end
      %adjust selected control (decrease control)
      case 'leftarrow'
        current_control = getappdata(0, 'control');
        if current_control - delta >= min_control
            setappdata(0, 'control', current_control - delta);
            fprintf('current control: %f',current_control - delta);
        else
            setappdata(0, 'control', min_control);
        end
      %set control to reverse
      case 'downarrow'
        setappdata(0, 'control', reverse);
        fprintf('current control: reverse');
      %confirm selection
      case 'return'
        setappdata(0, 'control_selected', 1);
      %stop simulation
      case 'escape'
        setappdata(0, 'stop_simulation', 1);
    end
    disp(event.Key);
  end