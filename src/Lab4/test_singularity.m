% Move your robot to a point where Y=0, X > 150 and Z > 150

% Generate a trajectory that will move your robot in a straight line from
% your current (X, Y, Z) to the point (X, -Y, Z)

%% Execute the trajectory
travelTime = 100000000;

% Create a robot object
robot = Robot();
interpolate_time = 3;  % CHOOSE SOMETHING REASONABLE

% Define three [x, y, z, alpha] positions for your end effector that all
% lie upon the x-z plane of your robot

%positions = [100 -100 300 0; 0 0 300 0; 0 100 300 0; 0 100 100 0];

positions = [0 -100 250 0;0 0 250 0;0 50 250 0;0 100 250 0]

% For each leg of your triangle, calculate the TASK SPACE trajectory
% between each vetex. Remember, to calculate a task space trajectory 
% you will need to create individual trajectories for x, y, z, and alpha.

% Move your robot to the first vertex

robot.interpolate_jp(robot.ik3001(positions(1, 1:4)), interpolate_time);
current_pos = positions(1, 1:4);
pause(interpolate_time); % Wait until done moving

waypointDataCount = 15; % How many data points to get per waypoint

totalDataCount = waypointDataCount * 3; % How many data points to get
dataTimeStep = interpolate_time / waypointDataCount; % Time between data steps

% The data matrix is a matrix with all the data in this form
data = zeros(totalDataCount, 9); % [th1, th2, th3, th4, timeStamp, deteriminateOfJacobian, manipulatorX, manipulatorY, manipulatorZ]
dataIndex = 1; % index of which is being inserted

currentTimestamp = 0;

tic;  % Start timer

for i=2:size(positions,1) %For every waypoint
    
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
        
        robotMeasurements = robot.measure_js(true, true);
        position = robotMeasurements(1,:);
        velocity = robotMeasurements(2,:);
        disp(velocity);
        dkMatrix = robot.dk3001(position,velocity);

        cur_jacob = robot.jacob3001(position);
        determinate = det(cur_jacob(1:3, :)*cur_jacob(1:3, :).');

        if(toc - currentTimestamp >= dataTimeStep)
            
            currentTimestamp = toc;

            manipulatorPos = robot.fk_3001(position);
            disp(position)

            disp("manipulatorPos ");
            disp(manipulatorPos)
            data(dataIndex,:) = [position, currentTimestamp, determinate, manipulatorPos(1:3, 4).'];
            dataIndex = dataIndex + 1;
        end 


        if (robot.atSingularity(cur_jacob,0.2))
            %robot.servo_jp(position);
            disp('ERROCODE 67: FOUND A SINGULAERITY');
            pause(interpolate_time);
            robot.interpolate_jp(robot.fk_3001([0, 100, 150, 0]), interpolate_time);
            pause(interpolate_time);

            disp("data");
            disp(data)
            data = data(1:dataIndex-1,:);
            plot_data(data);
            return;
        end
    
    end
    
    current_pos = waypoint;
    disp(data);

end

plot_data(data);

function plot_data(data)

    %% Plot the trajectory of your robot in x-y-z space using scatter3
    clf;
    figure; 
    Y = data(:, 8);
    deter = data(:, 6);

    
    
    scatter(deter, Y, 25, 'filled');
    grid on; 
    axis equal; 
    xlabel('Y (mm)'); 
    ylabel('Determinate'); 
    title('Determinate of Jacobian vs. Y Position of End Effector (mm)'); 
    view(2);

end