%% USE THIS FILE TO GET SIGN OFF 3

robot = Robot();  % Create a robot object to use
interpolate_time = 3.0; % How long it takes to move

%% Send the robot to its 0 position
% We don't need to collect data yet, as we're just putting the robot at its
% starting point

robot.interpolate_jp([0,0,0,0], interpolate_time); % Sends the robot to 0s
pause(interpolate_time); % Wait until done moving

%% Prealicate Data
% We do this because allocating memory takes a lot of time, so we only want
% to do it once

dataCount = 15; % How many data points to get
dataTimeStep = interpolate_time / dataCount; % Time between data steps
% The data matrix is a matrix with all the data in this form
data = zeros(dataCount, 5); % [pos1, pos2, pos3, pos4, timeStamp]
dataIndex = 1; % index of which is being inserted


%% Send the robot to an arbitrary position
posToGive = [40, -45, 60, -15]; % Random pos
% Commands the robot to move
robot.interpolate_jp(posToGive, interpolate_time);

%% Collect data as the robot moves
tic;  % Start timer
while toc <= interpolate_time + (dataTimeStep / 2.0)
    % Read current joint positions (not velocities though!)
    % Store the positions and timesetamps in an array 
    robotMeasurements = robot.measure_js(true, false); % posisition and zero vector = [pos1, pos2, pos3, pos4; 0, 0, 0, 0]
    position = robotMeasurements(1, :); % just position = [pos1, pos2, pos3, pos4]

    currentTimestamp = toc; % seconds since timer started
    refinedMeasurement = [position, currentTimestamp]; % row for datapoint = [pos1, pos2, pos3, pos4, timestamp]


    data(dataIndex, :) = refinedMeasurement; % inserts the row at the dataIndex
    dataIndex = dataIndex + 1; % increases the index

    disp(data); % Prints the data matrix as it comes in 
    pause(dataTimeStep); % waits for the dataTimeStep time
end

%% Make your figure
% To use subfigures, you'll use the subplots feature of MATLAB.
% https://www.mathworks.com/help/matlab/ref/subplot.html

t = data(:,5);  % Time Column
y = data(:,1:4); % Pos Columns

figure 
for i = 1:4
    ax(i) = subplot (4,1,i); % Makes a subplot 
    h(i) = plot(t, y(:,i)); 
    ylabel(['Position ' num2str(i) ' (Deg)']); 
    grid on
end 
xlabel('Time (s)');
sgtitle('Robot Motion ([0, 0, 0, 0] to [' + strjoin(string(posToGive), ', ') + '])'); 
% In each subplot you create, you can use 'plot' to plot one joint value vs
% time
% https://www.mathworks.com/help/matlab/ref/plot.html
% Remember titles, labels, and units!


for i = 1:4
    set(h(i), 'xData', t, 'yData', y(:,i));  % idk what this does...
end 

%% Calculate time step statistics

timeSteps = diff(data(:, 5)); % Gets a vector thats the diffrence between each timestamp

meanTimeStep = mean(timeSteps);
medianTimeStep = median(timeSteps);
maxTimeStep = max(timeSteps);
minTimeStep = min(timeSteps);