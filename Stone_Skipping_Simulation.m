clc;
clear;

% constants
g = 9.81;       % Gravity (m/s^2)
cd = 0.01;      % Air drag coefficient
r = 0.8;        % Restitution coefficient. A physical constant the decides how much the stonw will skip depending on energy loss
m = 0.1;        % Stone mass (kg)
dt = 0.01;      % Time step
T = 5;          % Max simulation time
N = T / dt;     % Number of steps

        % inputs(the user should enter valid values)
%initial velocity
v0 = input('Enter initial launch speed (m/s): ');
while isempty(v0) || ~isnumeric(v0) || v0 <= 0
    disp('Invalid input. Please enter a positive number for speed.');
    v0 = input('Enter initial launch speed (m/s): ');
end


%Elevation angle: the angle above the horizontal plane
elevation_deg = input('Enter launch elevation angle (degrees above horizontal): ');
while isempty(elevation_deg) || ~isnumeric(elevation_deg)
    disp('Invalid input. Please enter a number for the angle.');
    elevation_deg = input('Enter launch elevation angle (degrees above horizontal): ');
end
angle = deg2rad(elevation_deg);  % Convert to radians



%Azimuth angle: in which direction the stone will go on the horizontal plane
azimuth_deg = input('Enter launch azimuth angle (direction the stone) in degrees: ');
while isempty(azimuth_deg) || ~isnumeric(azimuth_deg)
    disp('Invalid input. Please enter a number for the angle.');
    azimuth_deg = input('Enter launch azimuth angle (direction the stone) in degrees:  ');
end
azimuth = deg2rad(azimuth_deg); 
% azimuth = deg2rad(45);  % fixed azimuth angle


 %initial height of skipping
z0 = input('Enter initial launch height (m): ');
while isempty(z0) || ~isnumeric(z0) || z0 < 0
    disp('Invalid input. Please enter a non-negative number for height.');
    z0 = input('Enter initial launch height (m): ');
end


 %Velocity projections
vx = v0 * cos(angle) * cos(azimuth);
vy = v0 * cos(angle) * sin(azimuth);
vz = v0 * sin(angle);
u = [0; 0; z0; vx; vy; vz];  % Initial state: [x; y; z; vx; vy; vz]


%setting thr axes
figure;
hold on; axis equal; grid on;
xlabel('x'); ylabel('y'); zlabel('z');
title('RK4 Stone Skipping Simulation');
view(45, 25);


% % % Drawing water surface
% % [Xs, Ys] = meshgrid(-200:4:200); %Creates a 2D grid of x and y points from -200 to 200 with step 4 
% % Zs = zeros(size(Xs));
% % surf(Xs, Ys, Zs, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'FaceColor', 'cyan'); %Draws a surface in 3D using Xs, Ys, and Zs. This is water plane
% %                                                                               %'FaceAlpha', 0.2 Makes the water surface semi-transparent (20% opaque)
% %                                                                               % 'EdgeColor', 'none'	Hides the grid lines between squares on the water surface
% %                                                                               % 'FaceColor', 'cyan'	Colors the surface light blue (like water)


% Initialize growing water surface
water_range = 40; % Initial half-width of water surface so the total width is 80 units
[Xs, Ys] = meshgrid(-water_range:4:water_range);%Creates a 2D grid of x and y points from -40 to 40 with step 4
Zs = zeros(size(Xs));
water_surface = surf(Xs, Ys, Zs, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'FaceColor', 'cyan');%Draws a surface in 3D using Xs, Ys, and Zs. This is water plane
                                                                                             %'FaceAlpha', 0.2 Makes the water surface semi-transparent (20% opaque)
                                                                                             % 'EdgeColor', 'none'	Hides the grid lines between squares on the water surface
                                                                                             % 'FaceColor', 'cyan'	Colors the surface light blue (like water)




%Tracking Variables
impact_points = [];% A matrix that stores the (x, y) positions of the stone when it hits the water
skip_count = 0;%couter of skips
disk = []; %stores the plot object of the stone so it can be updated
trajectory_time = [];%an array that stores the time of the simulation
trajectory_z = [];%an array that stores the height for plotting


%Simulation Loop
for i = 1:N %N = T / dt if we want to simulate 10s with step time of 0.01s wewill have 1000 steps
    % RK4 integration
    k1 = stoneODE(u, m, cd, g);
    k2 = stoneODE(u + 0.5*dt*k1, m, cd, g);
    k3 = stoneODE(u + 0.5*dt*k2, m, cd, g);
    k4 = stoneODE(u + dt*k3, m, cd, g);
    u_next = u + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

    % Check for bounce
    if u(3) > 0 && u_next(3) <= 0 % if the stone crosses the water surface in a decreasing way .it might skip or sink depending on the vertical velocity
        if abs(u(6)) > 0.1 %and if it has enough vertical velocity vz
            u_next(3) = 0; % Reset height
            u_next(4:6) = r * [u(4); u(5); -u(6)]; %r is that reduce the velocity in the next state due to energy loss and we flip vz to skip and go upward (r=1 in ideal case) 
            skip_count = skip_count + 1;% so add a skip
            impact_points = [impact_points; u_next(1:2).']; %save the (x,y) bounce position to draw ripples later
        else
            break;  % Stone sinks since it has no enough vz
        end
    end

    % Update state
    u = u_next;%updates Stone's State
    max_xy = max(abs(u(1:2))); %checks how far the stone has gone
    if max_xy > water_range * 0.8 %If the stone is more than 80% toward the current water edge,Then we decide to expand the water surface
        water_range = ceil(max_xy + 20); % increases the range slightly beyond the stone's position (adds 20 units of margin)
        [Xs, Ys] = meshgrid(-water_range:4:water_range);%recalculate the surface grid with the new size
        Zs = zeros(size(Xs));
        set(water_surface, 'XData', Xs, 'YData', Ys, 'ZData', Zs);%updates the existing surface instead of redrawing
    end

    % Log vertical position and time
    trajectory_time(end+1) = i * dt;%saves the current time (i * dt: the time that has passed)
    trajectory_z(end+1) = u(3);%saves the stone's height at that moment

    % Draw the stone
    if isgraphics(disk), delete(disk); end %removes the old red dot
    disk = plot3(u(1), u(2), u(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');%draws a red dot at the new 3D position

    % Draw expanding ripples at each impact point
    for k = 1:size(impact_points, 1) %Loops through all impact points recorded previously /size(impact_points, 1) = number of skips
        x0 = impact_points(k,1);
        y0 = impact_points(k,2);
        ripple_radius = (i - k * 20) * dt * 2;%k is the index of the impact point (the earlier the impact, the longer the ripple has been expanding)
                                              %k * 20 acts like a delay, so the ripple only starts growing some time after the impact
                                              %dt * 2 scales the ripple's growth speed
                                              %older impact points will have larger ripples, newer ones will start small


        if ripple_radius > 0 %ripple_radius becomes positive only after enough time has passed since the stone hit the water at that point
            theta = linspace(0, 2*pi, 100);%used to create a full circle using sin and cos
            rippleX = x0 + ripple_radius * cos(theta);%computes the x-coordinates of points on the circle
            rippleY = y0 + ripple_radius * sin(theta);%computes the y-coordinates of points on the circle/with rippleX, they define a circle centered at (x0, y0)
            rippleZ = zeros(size(rippleX));%all Z-values are set to 0 because the ripple lies on the water surface
            plot3(rippleX, rippleY, rippleZ, 'c:');%draws the circula ripple as a 3D dashed cyan line
        end
    end

    drawnow limitrate;%update the figure smoothly without redrawing too fast so we can see how the stone skips

    % Stop if stone arrives to surface or below it & total velocity is very low
    if u(3) <= 0 && norm(u(4:6)) < 0.1
        break;%the simulation ends early using break because the stone has stopped skipping
    end
end %of for i=1:N

% Final output
fprintf('\nSimulation finished.\n');
fprintf('Total skips: %d\n', skip_count);
fprintf('Horizontal distance: %.2f m\n', norm(u(1:2)));%which calculates the Euclidean distance from (0,0) to current posotion

%Plot Vertical Trajectory
figure;
plot(trajectory_time, trajectory_z, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Height (z)');
title('Stone Vertical Trajectory Over Time');
grid on;

%ODE function :defines the differential equations of the motion of the skipping stone
function dudt = stoneODE(u, m, cd, g) %dudt: the derivative of the state vector (how position and velocity are changing with time)
    vx = u(4); vy = u(5); vz = u(6);
    dudt = zeros(6,1);
    dudt(1:3) = [vx; vy; vz]; 
    dudt(4:6) = [-cd*vx/m; -cd*vy/m; -g - cd*vz/m]; %cd:Drag force (resists motion), proportional to velocity
                                                    %Gravity (only affects vertical/z direction)
end
