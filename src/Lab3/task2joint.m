% Create a robot object
robot = Robot();
interpolate_time = 3;  % CHOOSE SOMETHING REASONABLE

% Define three [x, y, z, alpha] positions for your end effector that all
% lie upon the x-z plane of your robot

positions = [165 0 300 10; 165 0 260 20; 300 0 150 0];

% For each leg of your triangle, calculate the TASK SPACE trajectory
% between each vetex. Remember, to calculate a task space trajectory 
% you will need to create individual trajectories for x, y, z, and alpha.

% Move your robot to the first vertex

robot.interpolate_jp(robot.ik3001(positions(3, 1:4)), interpolate_time);
current_pos = positions(3, 1:4);
pause(interpolate_time); % Wait until done moving

waypointDataCount = 15; % How many data points to get per waypoint

totalDataCount = waypointDataCount * 3; % How many data points to get
dataTimeStep = interpolate_time / waypointDataCount; % Time between data steps

% The data matrix is a matrix with all the data in this form
data = zeros(totalDataCount, 5); % [th1, th2, th3, th4, timeStamp]
dataIndex = 1; % index of which is being inserted

currentTimestamp = 0;

tic;  % Start timer
for i=1:3
    
    waypoint = positions(i, 1:4);
    joints = robot.ik3001(waypoint);

    t0 = toc;

    traj1 = TrajGenerator.cubic_traj(t0, t0 + interpolate_time, current_pos(1), waypoint(1));
    traj2 = TrajGenerator.cubic_traj(t0, t0 + interpolate_time, current_pos(2), waypoint(2));
    traj3 = TrajGenerator.cubic_traj(t0, t0 + interpolate_time, current_pos(3), waypoint(3));
    traj4 = TrajGenerator.cubic_traj(t0, t0 + interpolate_time, current_pos(4), waypoint(4));

    while toc <= interpolate_time + t0

        traj_pos = [TrajGenerator.eval_traj(traj1, toc)
                    TrajGenerator.eval_traj(traj2, toc)
                    TrajGenerator.eval_traj(traj3, toc)
                    TrajGenerator.eval_traj(traj4, toc)];

        robot.servo_jp(robot.ik3001(traj_pos));
        
        robotMeasurements = robot.measure_js(true, false);
        position = robotMeasurements(1,:);

        if(toc - currentTimestamp >= dataTimeStep)
        
            currentTimestamp = toc;
            data(dataIndex,:) = [position, currentTimestamp];
            dataIndex = dataIndex + 1;

        end
    
    end
    
    current_pos = waypoint;
    disp(data);

end

% Loop through all collected data and convert it from joint-space to task
% space using your fk3001 function

% Plot the trajectory of your robot in x-y-z space using scatter3
% https://www.mathworks.com/help/matlab/ref/scatter3.html


% Plot the trajectory of your robot in theta2-theta3-theta-4 space using
% scatter3

task_space = zeros(totalDataCount, 4, 4);

for i = 1:totalDataCount

    task_space(i, :, :) = robot.fk_3001(data(i,1:4));

end

%% Plot the trajectory of your robot in x-y-z space using scatter3
clf;
figure; 
X = task_space(:, 1, 4); 
Y = task_space(:, 2, 4); 
Z = task_space(:, 3, 4); 

scatter3(X, Y, Z, 25, 'filled');
grid on; 
axis equal; 
xlabel('X (mm)'); 
ylabel('Y (mm)'); 
zlabel('Z (mm)'); 
title('Task Space Position (X, Y, Z) (mm) of End Effector at Timestep'); 
view(3);

%% Plot the trajectory of your robot in theta2-theta3-theta-4 space using scatter3
figure; 
X = data(:, 2); 
Y = data(:, 3); 
Z = data(:, 4); 

scatter3(X, Y, Z, 25, 'filled');
grid on; 
axis equal; 
xlabel('\theta_2 (Deg)'); 
ylabel('\theta_3 (Deg)'); 
zlabel('\theta_4 (Deg)'); 
title('Joint Space Position (Deg) of Joints at Timestep'); 
view(3);

TrajGenerator.plot_traj(0, 0 + interpolate_time, positions(2, 1), positions(3, 1));