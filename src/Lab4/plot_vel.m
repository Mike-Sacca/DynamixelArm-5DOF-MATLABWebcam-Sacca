% Create a robot object
robot = Robot();
interpolate_time = 3;  % CHOOSE SOMETHING REASONABLE

% Define three [x, y, z, alpha] positions for your end effector that all
% lie upon the x-z plane of your robot

positions = [165 0 300 10; 165 0 120 20; 300 0 150 0];

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
data = zeros(totalDataCount, 11); 
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
        
        robotMeasurements = robot.measure_js(true, true);
        position = robotMeasurements(1,:);
        velocity = robotMeasurements(2,:);
        % Used for target velocities
        dkMatrix = robot.dk3001(position,velocity);
        v = dkMatrix(1:3);
        if(toc - currentTimestamp >= dataTimeStep)

           

            manipulatorTransferMatrix = robot.fk_3001(position);

            disp("manipulatorTransferMatrix");
            disp(manipulatorTransferMatrix);
            manipulatorPos = manipulatorTransferMatrix(1:3,4).';
            disp("manipulatorPos");
            disp(manipulatorPos);
            
            %targetTransferMatrix = robot.fk_3001(traj_pos);
            %targetPos = targetTransferMatrix(1:3,4).';
            disp("targetPos ");
            disp(traj_pos );


            currentTimestamp = toc;
            data(dataIndex,:) = [currentTimestamp, traj_pos.', v.', manipulatorPos];
            dataIndex = dataIndex + 1;

        end % TrajGenerator
    
    end
    
    current_pos = waypoint;
    disp(data);
end


% Removes zero rows
data = data(1:dataIndex-1,:);
plot_data(data);

function plot_data(plotData)

    clf;
    figure;

    % ---- unpack data ----
    time = plotData(:,1);

    % target position
    xT = plotData(:,2);
    yT = plotData(:,3);
    zT = plotData(:,4);

    % actual velocity (dk)
    xAdot = plotData(:,6);
    yAdot = plotData(:,7);
    zAdot = plotData(:,8);

    % actual position
    xA = plotData(:,9);
    yA = plotData(:,10);
    zA = plotData(:,11);

    % ---- compute target velocity using gradients ----
    xTdot = gradient(xT, time);
    yTdot = gradient(yT, time);
    zTdot = gradient(zT, time);

    % ---- X ----
    subplot(3,2,1)
    plot(time,xT,time,xA)
    grid on
    title('X Position')
    legend('target','actual')

    subplot(3,2,2)
    plot(time,xTdot,time,xAdot)
    grid on
    title('X Velocity')
    legend('target','actual')

    % ---- Y ----
    subplot(3,2,3)
    plot(time,yT,time,yA)
    grid on
    title('Y Position')
    legend('target','actual')

    subplot(3,2,4)
    plot(time,yTdot,time,yAdot)
    grid on
    title('Y Velocity')
    legend('target','actual')

    % ---- Z ----
    subplot(3,2,5)
    plot(time,zT,time,zA)
    grid on
    title('Z Position')
    legend('target','actual')

    subplot(3,2,6)
    plot(time,zTdot,time,zAdot)
    grid on
    title('Z Velocity')
    legend('target','actual')

end