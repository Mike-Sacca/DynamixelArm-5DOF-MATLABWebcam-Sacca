% Make the robot class
robot = Robot();

% Navigate to home position to clear board for cam calibration
robot.interpolate_jp([0 -90 0 90], 2);
imgProcessor = ImageProcessor(debug=true);

% Possible Robot States:
% Searching, Picking 
% It just will go between the two
% Searching -> picking when a ball is detected by a camera and it saves the ball posisiton
% Picking -> searching when the robot is done picking up and sorting the ball
robotState = "Searching";

% Height for hovering above 
z_offset = 50;
z_ball = 20;

waitTime = 1;

ballColors = [];
ballPoses = [];

disp('Press any Key to Begin State Machine');
pause;

while true
    if robotState == "Searching"

        % Searches for a ball
        [ballColors, ballPoses] = imgProcessor.detect_balls();

        % If the ball Pose is not empty (ball(s) on board)
        if ~isempty(ballPoses) 
            robotState = "Picking"; % Go into picking
        end

        disp("Search Pause")
        pause(waitTime);

    elseif robotState == "Picking"

        % Iterate through all detected balls, picking them up and sorting
        % them out
        for i = 1:size(ballPoses, 1)

            ballPose = ballPoses(i,:);
            ballColor = ballColors(i);
            robot.pick_up_ball(ballPose, ballColor, z_offset, z_ball);

        end

        ballColors = [];
        ballPoses = [];
        
        % Return to searching state after all seen balls collected
        robotState = "Searching";
    end
end