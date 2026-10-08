% Create a robot object
robot = Robot();
interpolate_time = 2.0; % How long it takes to move
waypointMatrix = [
    0,-10,-67,67;
    0,20,-20,20;
    0,-40,60,60;
];
numOfWaypoints = size(waypointMatrix, 1);

lengths = [60, 36.32, 130.23, 124, 133.4];

robot.interpolate_jp([0,-40,60,60], interpolate_time); % Sends the robot to 0s
pause(interpolate_time); % Wait until done moving

waypointDataCount = 15; % How many data points to get per waypoint

totalDataCount = waypointDataCount * numOfWaypoints; % How many data points to get
dataTimeStep = interpolate_time / waypointDataCount; % Time between data steps

% The data matrix is a matrix with all the data in this form
data = zeros(totalDataCount, 5); % [th1, th2, th3, th4, timeStamp]
dataIndex = 1; % index of which is being inserted

tokIncrease = 0; % What to add to the timestamps because the timer gets restarted

tic;  % Start timer
for i=1:numOfWaypoints
    waypoint = waypointMatrix(i,1:4); % Waypoint is theta stars 

    robot.interpolate_jp(waypoint, interpolate_time);

        tic;  % restart timer for THIS segment
        while toc <= interpolate_time + (dataTimeStep/2)
            if dataIndex > size(data,1)
                break;
            end
        
            robotMeasurements = robot.measure_js(true, false);
            position = robotMeasurements(1,:);
        
            currentTimestamp = tokIncrease + toc;
            data(dataIndex,:) = [position, currentTimestamp];
            dataIndex = dataIndex + 1;
        
            pause(dataTimeStep);
        end
        
        disp(data);

        tokIncrease = tokIncrease + interpolate_time;  % advance by intended segment length

end


% Loop through all collected data and convert it from joint-space to task
% space using your fk3001 function

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
xlabel('X (cm)'); 
ylabel('Y (cm)'); 
zlabel('Z (cm)'); 
title('Task Space Position (X, Y, Z) (cm) of End Effector at Timestep'); 
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