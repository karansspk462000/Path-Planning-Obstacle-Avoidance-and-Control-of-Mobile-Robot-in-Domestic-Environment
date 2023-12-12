clc
clear all
close all
% Define the robot's initial position and heading
x0 = 0;
y0 = 0;
theta0 = pi/2;
%calling function of Dijkstra algo to get the shortest path
[len_path,points] = Simulation_dijkstra();

% Define the array of points that the robot will move over

%Data preprocessing
points = reshape(points, [2,len_path])';

xpt = points(:,1);  % Extract x-coordinates
ypt = points(:,2);  % Extract y-coordinates

% Plot fig for shortest path
figure;
hold on
grid on
plot(xpt, ypt, 'o');
axis equal;
xlim([0,6]);
ylim([0,6]);
xlabel('X-Coordinate');
ylabel('Y-Coordinate');

% Add labels for each point
for i = 1:size(points,1)
%     text(xpt+0.2, ypt+0.3, string(1:length(xpt))); % Add text slightly lower
    text(xpt(i)-0.4, ypt(i)+0.3, sprintf('(%0.1f, %0.1f)', xpt(i), ypt(i)), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
end

% Set the simulation time step
dt = 0.1;

% Set the distance threshold for stopping the robot
dist_thresh = 0.2;

% Initialize the robot's position and heading variables
x = x0;
y = y0;
theta = theta0;

% Initialize the simulation time and control index variables
t = 0;
i = 1;

% Initialize the path array with the robot's starting position
path = [x, y];

% Loop through the array of points and simulate the robot's movement
while i <= size(points, 1)% It gives no. of rows in the shortest points array

    % Get the current point that the robot should move towards
    goal_point = points(i, :);% Move the robot to the first point in the map
    
    % Calculate the distance and angle to the goal point
    dx = goal_point(1) - x;% how far is the robot from previous position in X direction
    dy = goal_point(2) - y;% how far is the robot from previous position in Y direction

    distance = sqrt(dx^2 + dy^2);% Calculate the eucledian distance
    goal_angle = atan2(dy, dx); %calculate the change in orientation with respect to the previous point
    
    % Calculate the error in angle and use a proportional controller to
    % turn the robot towards the goal point
    error_angle = goal_angle - theta;
    k_p = 10;
    omega = k_p * error_angle;
    
    % Calculate the robot's linear velocity using a constant speed
    % controller
    v = 1;
    
    % Update the robot's position and heading using the kinematic model
    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    theta = theta + omega * dt;
    
    % Add the current position to the path array
    path = [path; x, y];
    
    % If the robot has reached the goal point, move to the next point
    if distance < dist_thresh
        i = i + 1;
    end
    
    % Update the simulation time
    t = t + dt;

    % Plot the robot's position and heading
    plot(path(:, 1), path(:, 2), '-b');
    plot(x, y, 'ro');
    quiver(x, y, cos(theta), sin(theta));
    title(sprintf('Time: %.1f s', t));
   
    
    % Pause for a short time to visualize the movement
    pause(0.2);
end

% Plot the final path of the robot
plot(path(:, 1), path(:, 2), '-b');
% legend('Path', 'Robot');
