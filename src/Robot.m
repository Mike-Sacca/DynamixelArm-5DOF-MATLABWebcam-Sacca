% (c) 2023 Robotics Engineering Department, WPI
% Skeleton Robot class for OpenManipulator-X Robot for RBE 3001

classdef Robot < OM_X_arm
    % Many properties are abstracted into OM_X_arm and DX_XM430_W350. classes
    % Hopefully, you should only need what's in this class to accomplish everything.
    % But feel free to poke around!
    properties
        mDim; % Stores the robot link dimentions (mm)
        mOtherDim; % Stores extraneous second link dimensions (mm)
    end

    methods
        % Creates constants and connects via serial
        % Super class constructor called implicitly
        % Add startup functionality here
        function self = Robot(dummy)
            arguments
                dummy (1,1) logical = false
            end

            self = self@OM_X_arm(dummy);

            % Change robot to position mode with torque enabled by default
            % Feel free to change this as desired
            self.writeMode('p');
            self.writeMotorState(true);

            % Set the robot to move between positions with a 5 second profile
            % change here or call writeTime in scripts to change
            self.writeTime(2);
        end

        % Sends the joints to the desired angles
        % goals [1x4 double] - angles (degrees) for each of the joints to go to
        function writeJoints(self, goals)
            if checkSafe(goals)
                goals = mod(round(goals .* DX_XM430_W350.TICKS_PER_DEG + DX_XM430_W350.TICK_POS_OFFSET), DX_XM430_W350.TICKS_PER_ROT);
                self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.GOAL_POSITION, goals);
            end
        end

        % Creates a time based profile (trapezoidal) based on the desired times
        % This will cause writePosition to take the desired number of
        % seconds to reach the setpoint. Set time to 0 to disable this profile (be careful).
        % time [double] - total profile time in s. If 0, the profile will be disabled (be extra careful).
        % acc_time [double] - the total acceleration time (for ramp up and ramp down individually, not combined)
        % acc_time is an optional parameter. It defaults to time/3.
      
        function writeTime(self, time, acc_time)
            if (~exist("acc_time", "var"))
                acc_time = time / 3;
            end

            time_ms = time * DX_XM430_W350.MS_PER_S;
            acc_time_ms = acc_time * DX_XM430_W350.MS_PER_S;

            % disp("time");
            % disp(time_ms);
            % disp("acc time");
            % disp(acc_time_ms);

            self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, acc_time_ms);
            self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, time_ms);
        end
        
        % Sets the gripper to be open or closed
        % Feel free to change values for open and closed positions as desired (they are in degrees)
        % open [boolean] - true to set the gripper to open, false to close
        function writeGripper(self, open)
            if open
                self.gripper.writePosition(-35);
            else
                self.gripper.writePosition(55);
            end
        end

        % Sets position holding for the joints on or off
        % enable [boolean] - true to enable torque to hold last set position for all joints, false to disable
        function writeMotorState(self, enable)
            self.bulkReadWrite(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, enable);
        end

        % Supplies the joints with the desired currents
        % currents [1x4 double] - currents (mA) for each of the joints to be supplied
        function writeCurrents(self, currents)
            currentInTicks = round(currents .* DX_XM430_W350.TICKS_PER_mA);
            self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.GOAL_CURRENT, currentInTicks);
        end

        % Change the operating mode for all joints:
        % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
        % mode [string] - new operating mode for all joints
        % "current": Current Control Mode (writeCurrent)
        % "velocity": Velocity Control Mode (writeVelocity)
        % "position": Position Control Mode (writePosition)
        % Other provided but not relevant/useful modes:
        % "ext position": Extended Position Control Mode
        % "curr position": Current-based Position Control Mode
        % "pwm voltage": PWM Control Mode
        function writeMode(self, mode)
            switch mode
                case {'current', 'c'} 
                    writeMode = DX_XM430_W350.CURR_CNTR_MD;
                case {'velocity', 'v'}
                    writeMode = DX_XM430_W350.VEL_CNTR_MD;
                case {'position', 'p'}
                    writeMode = DX_XM430_W350.POS_CNTR_MD;
                case {'ext position', 'ep'} % Not useful normally
                    writeMode = DX_XM430_W350.EXT_POS_CNTR_MD;
                case {'curr position', 'cp'} % Not useful normally
                    writeMode = DX_XM430_W350.CURR_POS_CNTR_MD;
                case {'pwm voltage', 'pwm'} % Not useful normally
                    writeMode = DX_XM430_W350.PWM_CNTR_MD;
                otherwise
                    error("setOperatingMode input cannot be '%s'. See implementation in DX_XM430_W350. class.", mode)
            end

            lastVelTimes = self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL);
            lastAccTimes = self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC);

            self.writeMotorState(false);
            self.bulkReadWrite(DX_XM430_W350.OPR_MODE_LEN, DX_XM430_W350.OPR_MODE, writeMode);
            self.writeTime(lastVelTimes(1) / 1000, lastAccTimes(1) / 1000);
            self.writeMotorState(true);
        end

        % Gets the current joint positions, velocities, and currents
        % readings [3x4 double] - The joints' positions, velocities,
        % and efforts (deg, deg/s, mA)
        function readings = getJointsReadings(self)
            readings = zeros(3,4);
            
            readings(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            readings(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            readings(3, :) = self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT) ./ DX_XM430_W350.TICKS_PER_mA;
        end

        % Sends the joints at the desired velocites
        % vels [1x4 double] - angular velocites (deg/s) for each of the joints to go at
        function writeVelocities(self, vels)
            vels = round(vels .* DX_XM430_W350.TICKS_PER_ANGVEL);
            self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.GOAL_VELOCITY, vels);
        end
    
        %% LAB 1-----------------------------------------------------------
        % ALL code for lab 1 goes in this section ONLY

        function servo_jp(self, q)
        % SERVO_JP Send robot to a joint configuration
        % Inputs:
        %    q: a [1x4] vector containing the target joint positions
            
            self.writeTime(0.1); % Tells the motors to go quick
            self.writeJoints(q); % Writes q to the motors

        end
        
        function interpolate_jp(self, q, time)
        %INTERPOLATE_JP Send robot to a joint configuration over a period of time
        % Inputs:
        %    q: a [1x4] vector containing the target joint positions
        %    t: a scalar that tells how long to take to travel to the new position
        %       in milliseconds

            self.writeTime(time); % Tells the motor to go at the given break
            self.writeJoints(q); % Writes q to the motors
        end

        function q_curr = measure_js(robot, GETPOS, GETVEL)
        % MEASURED_JS Get the current position and velocity of the robot
        % Inputs:
        %    GETPOS: a boolean indicating whether or not to retrieve joint
        %            positions
        %    GETVEL: a boolean indicating whether or not to retrieve joint
        %            velocities
        % Outputs:
        %    q_curr: a [2x4] matrix whose top row contains joint positions (0s if
        %            GETPOS is false), and whose bottom row contains joint 
        %            velocities
        
            % Prealocates that data to 0s
            position = [0,0,0,0]; 
            velocity = [0,0,0,0];

            if (GETPOS)
               position = (robot.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG; % Sends the read request
            end

            if (GETVEL)
                velocity = (robot.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL); % Sends the read request
            end

            % Returns the position and velocity
            q_curr = [position;velocity]; % [posA, posB, posC, posD; velA, velB, velC, velD]
        end
        %% END LAB 1 CODE -------------------------------------------------
        %% BEGIN LAB 2 CODE -----------------------------------------------
        function ht = dh2mat(self, dh_row)
        %DH2MAT Gives the transformation matrix for a given row of a DH table
        % Inputs:
        %    dh_row: a [1x4] vector containing representing a single row of a DH
        %            table
        % Outputs: 
        %    ht: a [4x4] matrix representing the transformation defined by the DH
        %        row

            theta = dh_row(1);
            d = dh_row(2);
            a = dh_row(3);
            alpha = dh_row(4);

            ht = [cosd(theta) -sind(theta)*cosd(alpha)  sind(theta)*sind(alpha)  a*cosd(theta);
                  sind(theta)  cosd(theta)*cosd(alpha) -cosd(theta)*sind(alpha)  a*sind(theta);
                  0           sind(alpha)             cosd(alpha)             d           ;
                  0           0                      0                      1           ];
        end

        function ht = dh2fk(self, dh_tab)
        %DH2MAT_R Calculates FK from a DH table
        % Inputs:
        %    dh_tab: a [nx4] matrix representing a DH table
        % Outputs: 
        %    ht: a [4x4] matrix representing the transformation defined by the DH
        %        table

            currentTranslation = [
                1 0 0 0
                0 1 0 0
                0 0 1 0
                0 0 0 1
            ];

            % Gets the nubmer of rows in the dh_table
            numRows = size(dh_tab, 1);
            % Using number 3 and length caluclate:
            for rowIdx = 1:numRows
                nextTranslation = self.dh2mat(dh_tab(rowIdx, 1:4));
                currentTranslation = currentTranslation * nextTranslation;
            end

            ht = currentTranslation;

        end

        function fk = fk_3001(self, joint_angles)

            fk = fk_func(joint_angles(1), joint_angles(2), ...
                        joint_angles(3), joint_angles(4));

        end

        %% END LAB 2 CODE
        %% BEGIN LAB 3 CODE
        function qs = ik3001(self, pos)
            %IK3001_R Calculates IK for the OMX arm
            % Inputs:
            %    pos: a [nx4] matrix [x, y, z, alpha] representing a target pose in
            %    task space
            % Outputs: 
            %    qs: a [1x4] matrix representing the joint values needed to reach the
            %    target position
            
            x = pos(1);
            y = pos(2);
            z = pos(3);
            gamma = pos(4);

            qs = [];

            % Before running calculations, make sure
            %   1. The length of the x, y, z vector is less than the
            %      length of the extended robot arm
            %   2. The z-value is not negative

            if (sqrt(x^2+y^2+z^2) > 800)

                return
            
            end

            if (z < 0)
                
                return

            end
            
            qs = zeros(1, 4);
            
            % Theta 1 (Simple)
            qs(1) = atan2d(y, x);
            
            % Determine length of end effector in x-y plane based on
            % orientation in the z-axis
            l4 = 133.4;
            l4_xy = l4 * cosd(gamma);
            
            % Position to run IK on, at joint 3
            pos_prime = [x - l4_xy*cosd(qs(1)); y - l4_xy*sind(qs(1)); z + l4*sind(gamma)];
            
            % Geometric IK Formulas
            A = sqrt(x^2 + y^2) - l4_xy;
            B = pos_prime(3) - 96.326;
            C = sqrt(A^2 + B^2);
            
            % Lengths and angle between joint 2 and first armature
            l2 = 130.23;
            l3 = 124.0;
            phi = 10.6;

            % Utilize Law of Cosines for joints 2 and 3
            alpha = atan2d(B, A);
            beta_loc = (C^2 + l2^2 - l3^2) / (2.0*C*l2);
            
            if abs(beta_loc) > 1

                return

            end
            
            % atan2 over acos for consistency/sanity check
            beta = atan2d(sqrt(1 - beta_loc^2), beta_loc);
            qs(2) = 90 - alpha - beta - phi;

            th3_loc = (-C^2 + l2^2 + l3^2) / (2.0*l3*l2);

            if abs(th3_loc) > 1

                return
                
            end

            qs(3) = 90 + phi - atan2d(sqrt(1 - th3_loc^2), th3_loc);
            qs(4) = gamma - qs(2) - qs(3);
        
        end
        %% END LAB 3 CODE
        %% BEGIN LAB 4 CODE
        function j = jacob3001(self, qpos) 
            %JACOB3001 Calculates the jacobian of the OMX arm
            % Inputs:
            %   qpos: a [1x4] matrix composed of joint positions
            % Outputs:
            %   j: a [6x4] jacobian matrix of the robot in the given pos

            j = jacob_func(qpos(1), qpos(2), qpos(3), qpos(4));
            
        end

        function vs = dk3001(self, qpos, qvel)
            %DK3001 Calculates the forward velocity kinematics of the OMX
            %arm
            % Inputs:
            %   qpos: a [1x4] matrix composed of joint positions
            %   qvel: a [1x4] matrix composed of joint angular velocities
            % Outputs:
            %   vs: a [6x1] matrix representing the linear and angular 
            %       velocity of the end effector in the base frame of the 
            %       robot

            vs = jacob3001(self, qpos) * qvel.';
        end
            
        function isSingular = atSingularity (self, jacob, threshold) 

            % Determines if position is singular given Jacobian matrix, is
            % singular when det = 0 (with threshold)
            determinate = det(jacob(1:3, :)*jacob(1:3, :).');

            if (determinate < threshold) 
                isSingular = true;
            else 
                isSingular = false;
            end

        end
    %% END LAB 4 CODE
    %% BEGIN LAB 5 CODE
    function blocking_js_move(self, qpos, nvargs)
        arguments
            self Robot;
            qpos double;
            nvargs.time double = 2;
        end
        %BLOCKING_JS_MOVE moves the robot to a position in joint space
        %before exiting
        % Inputs:
        %   qpos: a [1x4] matrix of joint positions
        %   time: (optional): an integer representing desired travel time
        %         in seconds. Default time: 2 seconds.
        
        % Simple solution, rather than using trajectory generation for
        % joints (unnecessary), utilize existing function interpolate_jp,
        % and add blocking
        interpolate_jp(self, qpos, nvargs.time);
        pause(nvargs.time);

    end

    function blocking_ts_move(self, pos, nvargs)

        arguments
            self Robot;
            pos double;
            nvargs.time double = 2;
            nvargs.mode string = "cubic"
        end

        %BLOCKING_TS_MOVE moves the robot in a straight line in task space 
        %to the target position before exiting
        % Inputs:
        %   pos: a [1x4] matrix representing a target x, y, z, gamma
        %   time (optional): an integer representing desired travel time
        %         in seconds. Default time: 2 seconds.
        %   mode (optional): a string "cubic" or "quintic" indicating what 
        %                    type of trajectory to utilize

        % Determine current linear position XYZ, and Gamma, which is NOT
        % the measured angular position of joint 4, but the summation of
        % all joints which affect gamma (2, 3, and 4).
        measured_pos = self.measure_js(true, false);
        T = self.fk_3001(measured_pos(1, :));
        current_pos = [T(1,4), T(2,4), T(3,4), measured_pos(1, 2) + measured_pos(1, 3) + measured_pos(1, 4)];

        tic;
        t0 = toc;
        
        % Calculate desired trajectory, use t0 as reference for maximum
        % precision
        if nvargs.mode == "cubic"
            traj1 = TrajGenerator.cubic_traj(t0, t0+nvargs.time, current_pos(1), pos(1));
            traj2 = TrajGenerator.cubic_traj(t0, t0+nvargs.time, current_pos(2), pos(2));
            traj3 = TrajGenerator.cubic_traj(t0, t0+nvargs.time, current_pos(3), pos(3));
            traj4 = TrajGenerator.cubic_traj(t0, t0+nvargs.time, current_pos(4), pos(4));
        else
            traj1 = TrajGenerator.quinitic_traj(t0, t0+nvargs.time, current_pos(1), pos(1));
            traj2 = TrajGenerator.quinitic_traj(t0, t0+nvargs.time, current_pos(2), pos(2));
            traj3 = TrajGenerator.quinitic_traj(t0, t0+nvargs.time, current_pos(3), pos(3));
            traj4 = TrajGenerator.quinitic_traj(t0, t0+nvargs.time, current_pos(4), pos(4));
        end

        % Evaluate and follow trajectory
        while toc <= nvargs.time + t0

            traj_pos = [TrajGenerator.eval_traj(traj1, toc)
                        TrajGenerator.eval_traj(traj2, toc)
                        TrajGenerator.eval_traj(traj3, toc)
                        TrajGenerator.eval_traj(traj4, toc)];

            ik = self.ik3001(traj_pos);
            
            % Check for bad IK (sanity check)
            if exist('ik', 'var') && ~isempty(ik)
                
                self.servo_jp(ik);

            end
            
            % Necessary pause to avoid communication errors
            pause(0.04)

        end

    end

    % Simple Helper, block for safety
    function open_gripper(self)

        self.writeGripper(true);
        pause(1)

    end

    % Simple Helper, block for safety
    function close_gripper(self)
        
        self.writeGripper(false);
        pause(1)

    end

    function pick_up_ball(self, pos, color, z_offset, z_ball)
        %PICK_UP_BALL picks up a ball and deposits it in the correct color bin
        % Inputs:
        %   pos: a [1x2] matrix representing the position of the ball in the XY
        %        frame of the robot
        %   color: a string indicating what color bin the ball should be placed
        %          in
        %   z_offset (Optional): the z-position at which to begin the straight
        %                        vertical trajectory 
        %   z_ball (Optional): the z-posiiton at which to stop the vertical 
        %                      trajectory and grasp the ball
        
        x = pos(1);
        y = pos(2);
    
        % Waypoint positions
        above_ball = [x, y, z_offset, 90];  % Task Space
        at_ball = [x, y, z_ball, 90];       % Task Space
        home = [0, -90, 0, 90];             % Joint Space
    
        % Bin Locations
        switch lower(color)

            case "red"
                bin_xy = [0, -190]; 
            case "orange"
                bin_xy = [75, -190];
            case "yellow"
                bin_xy = [135, -190]; 
            case "green"
                bin_xy = [190, -190]; 
            otherwise 
                error("Unknown Bin Location"); 
    
        end
    
        above_bin = [bin_xy(1), bin_xy(2), 120, 45];
    
        % Blocking Movement for Picking Up Ball, placing it in its
        % corresponding bin, and returning to home position
        self.open_gripper();
        self.blocking_ts_move(above_ball, time = 1);
        self.blocking_ts_move(at_ball); 
        self.close_gripper();
        
        self.blocking_ts_move(above_ball);
        self.blocking_ts_move(above_bin, time = 1);
        self.open_gripper();
    
        self.blocking_js_move(home, time = 1);
    
        end
    
    end

end