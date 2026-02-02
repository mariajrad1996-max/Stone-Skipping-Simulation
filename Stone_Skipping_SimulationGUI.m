function StoneSkippingGUI()
% Main function to create and run the Stone Skipping Simulation GUI
    
    % 1. Create Main Figure
    fig = figure('Name', 'Stone Skipping Simulation', ... % Sets the window title
                 'Position', [100, 100, 1000, 600], ... % Sets window position and size [left, bottom, width, height]
                 'NumberTitle', 'off', ... % Hides the figure number in the title
                 'MenuBar', 'none', ... % Removes the standard menu bar
                 'ToolBar', 'none', ... % Removes the standard toolbar
                 'Resize', 'on'); % Allows the user to resize the window

   % 2. Create Panels
    % Create left panel for inputs - this will contain all parameter controls
    inputPanel = uipanel('Parent', fig, ... % Attaches panel to main figure
                        'Title', 'Simulation Parameters', ... % Panel title
                        'Units', 'normalized', ... % Uses relative sizing (0-1) instead of pixels
                        'Position', [0.01, 0.01, 0.28, 0.98], ... % Position and size [left, bottom, width, height]
                        'FontSize', 12); % Sets the font size for the panel title

        % Create right panel for visualization - this will contain the plots
    visPanel = uipanel('Parent', fig, ... % Attaches panel to main figure
                      'Title', 'Visualization', ... % Panel title
                      'Units', 'normalized', ... % Uses relative sizing
                      'Position', [0.30, 0.01, 0.69, 0.98], ... % Position and size
                      'FontSize', 12); % Font size for panel title

    % Create advanced parameters panel (nested inside the input panel)
    advPanel = uipanel('Parent', inputPanel, ... % Attaches to input panel instead of main figure
                      'Title', 'Physical Parameters', ... % Panel title
                      'Units', 'normalized', ... % Uses relative sizing
                      'Position', [0.05, 0.35, 0.9, 0.25], ... % Position and size within parent panel
                      'FontSize', 10); % Slightly smaller font for this sub-panel

    % 3. Create Input Controls
    % Basic parameters - labels and edit fields
    
      % Initial Speed - Label
    uicontrol('Parent', inputPanel, ... % Attaches to input panel
              'Style', 'text', ... % Creates a text label (not editable)
              'String', 'Initial Speed (m/s):', ... % The text to display
              'Units', 'normalized', ... % Uses relative sizing
              'Position', [0.05, 0.90, 0.5, 0.04], ... % Position and size
              'HorizontalAlignment', 'left'); % Aligns text to the left
          
      % Initial Speed - Edit Field
    hSpeedEdit = uicontrol('Parent', inputPanel, ... % Attaches to input panel
                          'Style', 'edit', ... % Creates an editable text field
                          'String', '15', ... % Default value
                          'Units', 'normalized', ... % Uses relative sizing
                          'Position', [0.6, 0.90, 0.3, 0.04], ... % Position and size
                          'Tag', 'SpeedEdit'); % Tag for identifying this control later

      % Elevation Angle - Label
    uicontrol('Parent', inputPanel, ... % Attaches to input panel
              'Style', 'text', ... % Creates a text label
              'String', 'Elevation Angle (deg):', ... % The text to display
              'Units', 'normalized', ... % Uses relative sizing
              'Position', [0.05, 0.85, 0.5, 0.04], ... % Position and size
              'HorizontalAlignment', 'left'); % Aligns text to the left
          
     % Elevation Angle - Edit Field
    hElevationEdit = uicontrol('Parent', inputPanel, ... % Attaches to input panel
                              'Style', 'edit', ... % Creates an editable text field
                              'String', '15', ... % Default value
                              'Units', 'normalized', ... % Uses relative sizing
                              'Position', [0.6, 0.85, 0.3, 0.04], ... % Position and size
                              'Tag', 'ElevationEdit'); % Tag for identifying this control

       % Azimuth Angle - Label
    uicontrol('Parent', inputPanel, ... % Attaches to input panel
              'Style', 'text', ... % Creates a text label
              'String', 'Azimuth Angle (deg):', ... % The text to display
              'Units', 'normalized', ... % Uses relative sizing
              'Position', [0.05, 0.80, 0.5, 0.04], ... % Position and size
              'HorizontalAlignment', 'left'); % Aligns text to the left
          
     % Azimuth Angle - Edit Field
    hAzimuthEdit = uicontrol('Parent', inputPanel, ... % Attaches to input panel
                            'Style', 'edit', ... % Creates an editable text field
                            'String', '45', ... % Default value
                            'Units', 'normalized', ... % Uses relative sizing
                            'Position', [0.6, 0.80, 0.3, 0.04], ... % Position and size
                            'Tag', 'AzimuthEdit'); % Tag for identifying this control

    % Initial Height - Label
    uicontrol('Parent', inputPanel, ... % Attaches to input panel
              'Style', 'text', ... % Creates a text label
              'String', 'Initial Height (m):', ... % The text to display
              'Units', 'normalized', ... % Uses relative sizing
              'Position', [0.05, 0.75, 0.5, 0.04], ... % Position and size
              'HorizontalAlignment', 'left'); % Aligns text to the left
          
     % Initial Height - Edit Field
    hHeightEdit = uicontrol('Parent', inputPanel, ... % Attaches to input panel
                           'Style', 'edit', ... % Creates an editable text field
                           'String', '1', ... % Default value
                           'Units', 'normalized', ... % Uses relative sizing
                           'Position', [0.6, 0.75, 0.3, 0.04], ... % Position and size
                           'Tag', 'HeightEdit'); % Tag for identifying this control

    % 4. Create Advanced Parameters Controls
    
            % Stone Mass - Label
    uicontrol('Parent', advPanel, ... % Attaches to advanced parameters panel
              'Style', 'text', ... % Creates a text label
              'String', 'Stone Mass (kg):', ... % The text to display
              'Units', 'normalized', ... % Uses relative sizing
              'Position', [0.05, 0.80, 0.5, 0.15], ... % Position and size
              'HorizontalAlignment', 'left'); % Aligns text to the left
          
            % Stone Mass - Edit Field
    hMassEdit = uicontrol('Parent', advPanel, ... % Attaches to advanced parameters panel
                         'Style', 'edit', ... % Creates an editable text field
                         'String', '0.1', ... % Default value
                         'Units', 'normalized', ... % Uses relative sizing
                         'Position', [0.6, 0.80, 0.3, 0.15], ... % Position and size
                         'Tag', 'MassEdit'); % Tag for identifying this control

          % Air Drag Coefficient - Label
    uicontrol('Parent', advPanel, ... % Attaches to advanced parameters panel
              'Style', 'text', ... % Creates a text label
              'String', 'Air Drag Coefficient:', ... % The text to display
              'Units', 'normalized', ... % Uses relative sizing
              'Position', [0.05, 0.60, 0.5, 0.15], ... % Position and size
              'HorizontalAlignment', 'left'); % Aligns text to the left
    
          % Air Drag Coefficient - Edit Field
    hDragEdit = uicontrol('Parent', advPanel, ... % Attaches to advanced parameters panel
                         'Style', 'edit', ... % Creates an editable text field
                         'String', '0.01', ... % Default value
                         'Units', 'normalized', ... % Uses relative sizing
                         'Position', [0.6, 0.60, 0.3, 0.15], ... % Position and size
                         'Tag', 'DragEdit'); % Tag for identifying this control

          % Restitution Coefficient - Label
    uicontrol('Parent', advPanel, ... % Attaches to advanced parameters panel
              'Style', 'text', ... % Creates a text label
              'String', 'Restitution Coefficient:', ... % The text to display
              'Units', 'normalized', ... % Uses relative sizing
              'Position', [0.05, 0.40, 0.5, 0.15], ... % Position and size
              'HorizontalAlignment', 'left'); % Aligns text to the left
          
          % Restitution Coefficient - Edit Field
    hRestitutionEdit = uicontrol('Parent', advPanel, ... % Attaches to advanced parameters panel
                                'Style', 'edit', ... % Creates an editable text field
                                'String', '0.8', ... % Default value
                                'Units', 'normalized', ... % Uses relative sizing
                                'Position', [0.6, 0.40, 0.3, 0.15], ... % Position and size
                                'Tag', 'RestitutionEdit'); % Tag for identifying this control

    % Max Simulation Time - Label
    uicontrol('Parent', advPanel, ... % Attaches to advanced parameters panel
              'Style', 'text', ... % Creates a text label
              'String', 'Max Simulation Time (s):', ... % The text to display
              'Units', 'normalized', ... % Uses relative sizing
              'Position', [0.05, 0.20, 0.5, 0.15], ... % Position and size
              'HorizontalAlignment', 'left'); % Aligns text to the left
          
          % Max Simulation Time - Edit Field
    hSimTimeEdit = uicontrol('Parent', advPanel, ... % Attaches to advanced parameters panel
                            'Style', 'edit', ... % Creates an editable text field
                            'String', '8', ... % Default value
                            'Units', 'normalized', ... % Uses relative sizing
                            'Position', [0.6, 0.20, 0.3, 0.15], ... % Position and size
                            'Tag', 'SimTimeEdit'); % Tag for identifying this control

    % 5. Create Control Buttons
    
        % Run Simulation Button
    hRunButton = uicontrol('Parent', inputPanel, ... % Attaches to input panel
                          'Style', 'pushbutton', ... % Creates a clickable button
                          'String', 'Run Simulation', ... % Button text
                          'Units', 'normalized', ... % Uses relative sizing
                          'Position', [0.2, 0.25, 0.6, 0.06], ... % Position and size
                          'FontWeight', 'bold', ... % Makes the text bold
                          'Tag', 'RunButton'); % Tag for identifying this control

        % Reset Button
    hResetButton = uicontrol('Parent', inputPanel, ... % Attaches to input panel
                            'Style', 'pushbutton', ... % Creates a clickable button
                            'String', 'Reset', ... % Button text
                            'Units', 'normalized', ... % Uses relative sizing
                            'Position', [0.2, 0.17, 0.6, 0.06], ... % Position and size
                            'Tag', 'ResetButton'); % Tag for identifying this control

    % 6. Create Results Display
    
    % Skip Count Text - Shows number of skips
    hSkipCountText = uicontrol('Parent', inputPanel, ... % Attaches to input panel
                              'Style', 'text', ... % Creates a text label (not editable)
                              'String', 'Number of Skips: 0', ... % Initial text
                              'Units', 'normalized', ... % Uses relative sizing
                              'Position', [0.05, 0.10, 0.9, 0.04], ... % Position and size
                              'HorizontalAlignment', 'center', ... % Centers the text
                              'FontWeight', 'bold', ... % Makes the text bold
                              'FontSize', 12, ... % Sets font size
                              'Tag', 'SkipCountText'); % Tag for identifying this control

    % Distance Text - Shows total distance traveled
    hDistanceText = uicontrol('Parent', inputPanel, ... % Attaches to input panel
                             'Style', 'text', ... % Creates a text label (not editable)
                             'String', 'Total Distance: 0.00 m', ... % Initial text
                             'Units', 'normalized', ... % Uses relative sizing
                             'Position', [0.05, 0.05, 0.9, 0.04], ... % Position and size
                             'HorizontalAlignment', 'center', ... % Centers the text
                             'FontWeight', 'bold', ... % Makes the text bold
                             'FontSize', 12, ... % Sets font size
                             'Tag', 'DistanceText'); % Tag for identifying this control

    % 7. Create Visualization Components
    
            % 3D Axes for trajectory - Shows the 3D stone path
    hAxes3D = axes('Parent', visPanel, ... % Attaches to visualization panel
                  'Units', 'normalized', ... % Uses relative sizing
                  'Position', [0.05, 0.35, 0.9, 0.6], ... % Position and size
                  'Tag', 'Axes3D'); % Tag for identifying this control
    title(hAxes3D, 'Stone Skipping Trajectory'); % Sets the plot title
    xlabel(hAxes3D, 'x'); % Labels the x-axis
    ylabel(hAxes3D, 'y'); % Labels the y-axis
    zlabel(hAxes3D, 'z'); % Labels the z-axis
    grid(hAxes3D, 'on'); % Adds a grid to the plot
    view(hAxes3D, 45, 25); % Sets the initial 3D view angle
    axis(hAxes3D, 'equal'); % Makes the x, y, z scales equal
    hold(hAxes3D, 'on'); % Allows multiple plots on the same axes

            % 2D Axes for height vs. time plot - Shows the vertical trajectory
    hAxes2D = axes('Parent', visPanel, ... % Attaches to visualization panel
                  'Units', 'normalized', ... % Uses relative sizing
                  'Position', [0.05, 0.05, 0.9, 0.25], ... % Position and size
                  'Tag', 'Axes2D'); % Tag for identifying this control
    title(hAxes2D, 'Stone Vertical Trajectory Over Time'); % Sets the plot title
    xlabel(hAxes2D, 'Time (s)'); % Labels the x-axis
    ylabel(hAxes2D, 'Height (m)'); % Labels the y-axis
    grid(hAxes2D, 'on'); % Adds a grid to the plot
    hold(hAxes2D, 'on'); % Allows multiple plots on the same axes

    
        % Create a structure to store all UI handles - this makes them accessible in callback functions
    handles = struct('SpeedEdit', hSpeedEdit, ... % Stores the speed edit field handle
                    'ElevationEdit', hElevationEdit, ... % Stores the elevation angle edit field handle
                    'AzimuthEdit', hAzimuthEdit, ... % Stores the azimuth angle edit field handle
                    'HeightEdit', hHeightEdit, ... % Stores the height edit field handle
                    'MassEdit', hMassEdit, ... % Stores the mass edit field handle
                    'DragEdit', hDragEdit, ... % Stores the drag coefficient edit field handle
                    'RestitutionEdit', hRestitutionEdit, ... % Stores the restitution coefficient edit field handle
                    'SimTimeEdit', hSimTimeEdit, ... % Stores the simulation time edit field handle
                    'RunButton', hRunButton, ... % Stores the run button handle
                    'ResetButton', hResetButton, ... % Stores the reset button handle
                    'SkipCountText', hSkipCountText, ... % Stores the skip count text handle
                    'DistanceText', hDistanceText, ... % Stores the distance text handle
                    'Axes3D', hAxes3D, ... % Stores the 3D axes handle
                    'Axes2D', hAxes2D); % Stores the 2D axes handle

    % Store handles in the figure's UserData property - makes them accessible to all callbacks
    set(fig, 'UserData', handles);

    % 8. Initial Water Surface
    
    % Create initial water surface in 3D axes - blue plane at z=0
    water_range = 40; % Initial size of water surface (will expand as needed)
    [Xs, Ys] = meshgrid(-water_range:4:water_range); % Creates a grid of x,y points
    Zs = zeros(size(Xs)); % Sets all z values to 0 (water surface is at z=0)
    % Creates a surface plot for the water
    water_surface = surf(hAxes3D, Xs, Ys, Zs, 'FaceAlpha', 0.2, ... % Semi-transparent
                         'EdgeColor', 'none', ... % No grid lines
                         'FaceColor', 'cyan'); % Light blue color

    % Store water surface handle and range in figure's AppData - for later access
    setappdata(fig, 'water_surface', water_surface); % Stores the surface handle
    setappdata(fig, 'water_range', water_range); % Stores the current water range

    % 9. Set Callbacks
    
    % Set callback for Run button - function to call when button is clicked
    set(hRunButton, 'Callback', @runSimulation);
    
    % Set callback for Reset button - function to call when button is clicked
    set(hResetButton, 'Callback', @resetSimulation);
 end

% Run Button Callback Function
function runSimulation(hObject, ~)
    % This function is called when the Run button is clicked
    % hObject is the handle to the button that was clicked
    
    % Get the figure handle - finds the parent figure of the button
    fig = ancestor(hObject, 'figure');
    
    % Get all UI handles from figure's UserData - retrieves the stored handles
    handles = get(fig, 'UserData');
    
    % Disable Run button during simulation - prevents multiple clicks
    set(handles.RunButton, 'Enable', 'off');
    
    % Clear previous plots - prepares for new simulation
    cla(handles.Axes3D); % Clears the 3D plot
    hold(handles.Axes3D, 'on'); % Allows new plots to be added
    axis(handles.Axes3D, 'equal'); % Makes x, y, z scales equal
    grid(handles.Axes3D, 'on'); % Adds grid lines
    xlabel(handles.Axes3D, 'x'); % Labels x-axis
    ylabel(handles.Axes3D, 'y'); % Labels y-axis
    zlabel(handles.Axes3D, 'z'); % Labels z-axis
    title(handles.Axes3D, 'Stone Skipping Trajectory'); % Sets title
    view(handles.Axes3D, 45, 25); % Sets 3D view angle
    
    cla(handles.Axes2D); % Clears the 2D plot
    hold(handles.Axes2D, 'on'); % Allows new plots to be added
    grid(handles.Axes2D, 'on'); % Adds grid lines
    xlabel(handles.Axes2D, 'Time (s)'); % Labels x-axis
    ylabel(handles.Axes2D, 'Height (m)'); % Labels y-axis
    title(handles.Axes2D, 'Stone Vertical Trajectory Over Time'); % Sets title
    
    % Get input parameters and validate them - reads and checks all user inputs
    try
        % Basic parameters
        v0 = str2double(get(handles.SpeedEdit, 'String')); % Gets initial speed value
        if isnan(v0) || v0 <= 0 % Checks if value is valid
            error('Initial speed must be a positive number.'); % Shows error if invalid
        end
        
        elevation_deg = str2double(get(handles.ElevationEdit, 'String')); % Gets elevation angle
        if isnan(elevation_deg) % Checks if value is valid
            error('Elevation angle must be a number.'); % Shows error if invalid
        end
        
        azimuth_deg = str2double(get(handles.AzimuthEdit, 'String')); % Gets azimuth angle
        if isnan(azimuth_deg) % Checks if value is valid
            error('Azimuth angle must be a number.'); % Shows error if invalid
        end
        
        z0 = str2double(get(handles.HeightEdit, 'String')); % Gets initial height
        if isnan(z0) || z0 < 0 % Checks if value is valid
            error('Initial height must be a non-negative number.'); % Shows error if invalid
        end
        
        % Advanced parameters
        m = str2double(get(handles.MassEdit, 'String')); % Gets stone mass
        if isnan(m) || m <= 0 % Checks if value is valid
            error('Stone mass must be a positive number.'); % Shows error if invalid
        end
        
        cd = str2double(get(handles.DragEdit, 'String')); % Gets drag coefficient
        if isnan(cd) || cd < 0 % Checks if value is valid
            error('Air drag coefficient must be a non-negative number.'); % Shows error if invalid
        end
        
        r = str2double(get(handles.RestitutionEdit, 'String')); % Gets restitution coefficient
        if isnan(r) || r < 0 || r > 1 % Checks if value is valid
            error('Restitution coefficient must be between 0 and 1.'); % Shows error if invalid
        end
        
        T = str2double(get(handles.SimTimeEdit, 'String')); % Gets max simulation time
        if isnan(T) || T <= 0 % Checks if value is valid
            error('Maximum simulation time must be a positive number.'); % Shows error if invalid
        end
        
    catch ME
        % If validation fails, show error message and re-enable Run button
        errordlg(ME.message, 'Input Error'); % Shows error dialog with message
        set(handles.RunButton, 'Enable', 'on'); % Re-enables the Run button
        return; % Exits the function without running simulation
    end
    
    % Convert angles to radians - MATLAB trig functions use radians
    angle = deg2rad(elevation_deg); % Converts elevation from degrees to radians
    azimuth = deg2rad(azimuth_deg); % Converts azimuth from degrees to radians
    
    % Calculate initial velocity components - breaks velocity into x, y, z components
    vx = v0 * cos(angle) * cos(azimuth); % x-component of velocity
    vy = v0 * cos(angle) * sin(azimuth); % y-component of velocity
    vz = v0 * sin(angle); % z-component of velocity (vertical)
    
    % Set up initial state vector - position and velocity components
    u = [0; 0; z0; vx; vy; vz];  % [x; y; z; vx; vy; vz]
    
    % Set up simulation parameters
    g = 9.81;       % Gravity acceleration (m/s^2)
    dt = 0.01;      % Time step for simulation
    N = T / dt;     % Number of simulation steps
    
    % Initialize tracking variables - for storing simulation results
    impact_points = []; % Will store locations where stone hits water
    skip_count = 0; % Counter for number of skips
    trajectory_time = []; % Will store time points
    trajectory_z = []; % Will store height at each time point
    
    % Initialize water surface - creates the blue water plane
    water_range = 40; % Initial size of water surface
    setappdata(fig, 'water_range', water_range); % Stores current range
    [Xs, Ys] = meshgrid(-water_range:4:water_range); % Creates grid of points
    Zs = zeros(size(Xs)); % Sets all z values to 0
    % Creates water surface plot
    water_surface = surf(handles.Axes3D, Xs, Ys, Zs, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'FaceColor', 'cyan');
    setappdata(fig, 'water_surface', water_surface); % Stores surface handle
    
    % Initialize stone marker - red dot representing the stone
    disk = plot3(handles.Axes3D, u(1), u(2), u(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    
    % SIMULATION LOOP
    for i = 1:N % Loop through each time step
        % Store previous state for bounce detection
        u_prev = u; % Keeps track of previous position/velocity
        
        % RK4 integration - 4th order Runge-Kutta method for solving ODEs
        k1 = stoneODE(u, m, cd, g); 
        k2 = stoneODE(u + 0.5*dt*k1, m, cd, g); 
        k3 = stoneODE(u + 0.5*dt*k2, m, cd, g); 
        k4 = stoneODE(u + dt*k3, m, cd, g); 
        u_next = u + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
        
        % Check for bounce - detects when stone hits water surface
        if u(3) > 0 && u_next(3) <= 0 % If stone crosses water surface downward
            if abs(u(6)) > 0.1 % If it has enough vertical velocity to bounce
                u_next(3) = 0; % Reset height to water surface
                u_next(4:6) = r * [u(4); u(5); -u(6)]; %we multiply by r to decrease velocity due to energy loss and we flip vz since ti must go upward to skip again
                skip_count = skip_count + 1; % Count this as a skip
                impact_points = [impact_points; u_next(1:2).']; % saves impact point
                
                % Update skip count display in real-time
                set(handles.SkipCountText, 'String', ['Number of Skips: ', num2str(skip_count)]);
                drawnow limitrate; % Updates the display
            else
                % Stone doesn't have enough velocity to bounce
                u_next(3) = 0; % Keep at water surface
                u_next(4:6) = [0; 0; 0]; % Stop all motion
                break; % Exit the simulation loop
            end
        end
        
        % Update state - move to next position/velocity
        u = u_next; % Updates the current state
        
        % Check if water surface needs expanding - ensures water is always visible
        max_xy = max(abs(u(1:2))); % Gets maximum x or y coordinate
        if max_xy > water_range * 0.8 % If stone is near edge of current water
            water_range = ceil(max_xy + 20); % Expand water surface
            setappdata(fig, 'water_range', water_range); % Store new range
            [Xs, Ys] = meshgrid(-water_range:4:water_range); % Create new grid
            Zs = zeros(size(Xs)); % Set z values to 0
            set(water_surface, 'XData', Xs, 'YData', Ys, 'ZData', Zs); % Update plot
        end
        
        % Log trajectory data - for height vs. time plot
        trajectory_time(end+1) = i * dt; % Record current time
        trajectory_z(end+1) = u(3); % Record current height
        
        % Update stone position - moves the red dot
        set(disk, 'XData', u(1), 'YData', u(2), 'ZData', u(3));
        
        % Draw ripples at impact points - expanding circles where stone hit water
        for k = 1:size(impact_points, 1) % For each impact point
            x0 = impact_points(k,1); % x-coordinate of impact
            y0 = impact_points(k,2); % y-coordinate of impact
            ripple_radius = (i - k * 20) * dt * 2; % Calculate current radius
            
            if ripple_radius > 0 % If ripple should be visible
                theta = linspace(0, 2*pi, 100); % Points around a circle
                rippleX = x0 + ripple_radius * cos(theta); % x-coordinates of ripple
                rippleY = y0 + ripple_radius * sin(theta); % y-coordinates of ripple
                rippleZ = zeros(size(rippleX)); % z-coordinates (at water surface)
                plot3(handles.Axes3D, rippleX, rippleY, rippleZ, 'c:'); % Plot ripple
            end
        end
        
        % Update the display - refreshes the visualization
        drawnow limitrate; % Updates the plot with limited frame rate
        
        % Check if stone has stopped - end condition
        if u(3) <= 0 && norm(u(4:6)) < 0.1 % If at surface and moving very slowly
            break; % Exit the simulation loop
        end
    end
    % END SIMULATION LOOP 
    % Calculate final results
    total_distance = norm(u(1:2)); % Horizontal distance traveled
    
    % Update results display - show final values
    set(handles.SkipCountText, 'String', ['Number of Skips: ', num2str(skip_count)]);
    set(handles.DistanceText, 'String', ['Total Distance: ', num2str(total_distance, '%.2f'), ' m']);
    
    % Highlight results briefly - visual feedback that simulation is complete
    highlightResults(handles);
    
    % Plot height vs. time trajectory - shows vertical motion over time
    plot(handles.Axes2D, trajectory_time, trajectory_z, 'b', 'LineWidth', 1.5);
    
    % Add water surface line to height plot - horizontal line at z=0
    if ~isempty(trajectory_time) % If we have trajectory data
        hold(handles.Axes2D, 'on'); % Allow adding to plot
        plot(handles.Axes2D, [min(trajectory_time), max(trajectory_time)], [0, 0], 'k--'); % Dashed line
        legend(handles.Axes2D, 'Trajectory', 'Water Surface', 'Location', 'best'); % Add legend
        hold(handles.Axes2D, 'off'); % Stop adding to plot
    end
    
    % Re-enable the Run button - allows running another simulation
    set(handles.RunButton, 'Enable', 'on');
end

% Reset Button Callback Function
function resetSimulation(hObject, ~)
    % This function is called when the Reset button is clicked
    
    % Get the figure handle - finds the parent figure
    fig = ancestor(hObject, 'figure');
    
    % Get all UI handles from figure's UserData - retrieves stored handles
    handles = get(fig, 'UserData');
    
    % Clear plots - removes all visualizations
    cla(handles.Axes3D); % Clears 3D plot
    hold(handles.Axes3D, 'on'); % Allows new plots
    axis(handles.Axes3D, 'equal'); % Equal axis scaling
    grid(handles.Axes3D, 'on'); % Shows grid
    
    cla(handles.Axes2D); % Clears 2D plot
    hold(handles.Axes2D, 'on'); % Allows new plots
    grid(handles.Axes2D, 'on'); % Shows grid
    
    % Reset water surface - recreates initial water surface
    water_range = 40; % Initial size
    setappdata(fig, 'water_range', water_range); % Stores range
    [Xs, Ys] = meshgrid(-water_range:4:water_range); % Creates grid
    Zs = zeros(size(Xs)); % Sets z values to 0
    % Creates water surface plot
    water_surface = surf(handles.Axes3D, Xs, Ys, Zs, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'FaceColor', 'cyan');
    setappdata(fig, 'water_surface', water_surface); % Stores handle
    
    % Reset result labels - clears previous results
    set(handles.SkipCountText, 'String', 'Number of Skips: 0');
    set(handles.DistanceText, 'String', 'Total Distance: 0.00 m');
end

% Helper Functions

% Function to highlight results - visual feedback when simulation completes
function highlightResults(handles)
    % Briefly change text color to highlight updated results
    originalColor = get(handles.SkipCountText, 'ForegroundColor'); % Gets current color
    set(handles.SkipCountText, 'ForegroundColor', [0 0.6 0]); % Changes to green
    set(handles.DistanceText, 'ForegroundColor', [0 0.6 0]); % Changes to green
    pause(0.3); % Waits briefly
    set(handles.SkipCountText, 'ForegroundColor', originalColor); % Restores original color
    set(handles.DistanceText, 'ForegroundColor', originalColor); % Restores original color
end

% ODE Function Definition - Defines the physics equations

function dudt = stoneODE(u, m, cd, g)
    % This function calculates the derivatives for position and velocity
    % u = [x; y; z; vx; vy; vz] is the state vector
    
    vx = u(4); vy = u(5); vz = u(6); % Extract velocity components
    
    dudt = zeros(6,1); % Initialize derivative vector
    
    % Position derivatives = velocity components
    dudt(1:3) = [vx; vy; vz]; 
    
    % Velocity derivatives = acceleration components
    % Includes gravity (in z direction) and air drag (proportional to velocity)
    dudt(4:6) = [-cd*vx/m; -cd*vy/m; -g - cd*vz/m];
end
