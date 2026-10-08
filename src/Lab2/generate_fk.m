robot = Robot();  % Create a robot object to use
lengths = [60, 36.32, 130.23, 124, 133.4];

%% Step 1: Define your symbolic DH table

syms theta_1 theta_2 theta_3 theta_4;
dh_table = [
    0               ,lengths(1) ,0          ,0;
    theta_1         ,lengths(2) ,0          ,-90;
    theta_2-90+10.6 ,0          ,lengths(3) ,0;
    theta_3+90-10.6 ,0          ,lengths(4) ,0;
    theta_4         ,0          ,lengths(5) ,0
];

%% Step 2: Pass your symbolic DH table into dh2fk to get your symbolic 
% FK matrix

fk = robot.dh2fk(dh_table);

% Show this to an SA for SIGN-OFF #4

%% Step 3: Feed your symbolic FK matrix into 'matlabFunction' to turn it
% into a floating point precision function that runs fast.
% Write the fk_3001 function in Robot.m to complete sign-off #5

%matlabFunction(fk,"File","fk_func");

% Curiosity bonus (0 points): replicate the timeit experiment I did in
% sym_example.m to compare the matlabFunction FK function to just using
% subs to substitute the variables.


%% Calculate time step statistics

% timeSteps = diff(data(:, 5)); % Gets a vector thats the diffrence between each timestamp
% 
% meanTimeStep = mean(timeSteps);
% medianTimeStep = median(timeSteps);
% maxTimeStep = max(timeSteps);
% minTimeStep = min(timeSteps);

%% Lab 4 Section

lin_jacob = jacobian(fk(1:3, 4), [theta_1, theta_2, theta_3, theta_4]);

T_01 = robot.dh2fk(dh_table(1:2, :));
T_02 = robot.dh2fk(dh_table(1:3, :));
T_03 = robot.dh2fk(dh_table(1:4, :));
T_04 = robot.dh2fk(dh_table(1:5, :));

ang_jacob = [T_01(1:3, 3), T_02(1:3, 3), T_03(1:3, 3), T_04(1:3, 3)];

jacob = [lin_jacob; ang_jacob];

matlabFunction(jacob,"File","jacob_func");
