classdef TrajGenerator < handle  
    methods(Static)
        % TODO: Fill in the arguments for these methods
        function coeffs = cubic_traj(t0, tf, q0, qf)

            %CUBIC_TRAJ Generates coefficients for a cubic trajectory
            % Inputs:
            %   TODO: Describe input args
            A = [
                1 t0 t0^2  t0^3;
                0 1  2*t0  3*t0^2;
                1 tf tf^2  tf^3;
                0 1  2*tf  3*tf^2
            ];
            b = [q0;0;qf;0];
            
            % Outputs:
            %   coeffs: a [1x4] matrix of cubic trajectory coefficients

            % YOUR CODE HERE
            % Hint: to solve the linear
            %  system of equations b = Ax,
            %       use x = A \ b

            
            coeffs = (A \ b).'; % .' Transposes it
        end

        % TODO: Fill in the arguments for these methods
        function coeffs = quinitic_traj(t0, tf, q0, qf)
        % Inputs:
        %   TODO: Describe input args
        % Outputs:
        %   coeffs: a [1x6] matrix of cubic trajectory coefficients



                A = [
                    1 t0 t0^2 t0^3 t0^4 t0^5;
                    0 1  2*t0  3*t0^2 4*t0^3 5*t0^4;
                    0 0  2     6*t0   12*t0^2 20*t0^3;
                    1 tf tf^2 tf^3 tf^4 tf^5;
                    0 1  2*tf  3*tf^2 4*tf^3 5*tf^4;
                    0 0  2     6*tf   12*tf^2 20*tf^3
                ];
                b = [q0; 0; 0; qf; 0; 0];
                coeffs = (A \ b).'; % .' transposes it  6x1
        end

        function state = eval_traj(coeff_mat, t) 
            %EVAL_TRAJ Evaluates multiple trajectories
            % Inputs:
            %   coeff_mat: a [nx4] or [nx6] matrix where each row is a set of
            %              cubic or quintic trajectory coefficients
            %   t: a time in seconds at which to evaluate the trajectories
            % Outputs:
            %   state: a [nx1] column vector containing the results of 
            %          evaluating the input trajectories at time t


            numCoeff = size(coeff_mat, 2);   % number of coefficients (4 or 6)

            % Build time vector dynamically
            T = zeros(numCoeff,1);
            for k = 1:numCoeff
                T(k) = t^(k-1);
            end

            state = coeff_mat * T;   % (n x order) * (order x 1) = (n x 1)

            % Just for cubic: 
            % T = [1; t; t^2; t^3];   % 4x1 vector
            % state = coeff_mat * T;  % (n x 4) * (4 x 1) = (n x 1)
        end

        function plot_traj(t0, tf, q0, qf)
            % demo_traj  Generates and plots a trajectory.
            %
            % Inputs:
            %   t0          start time (seconds)
            %   tf          end time (seconds)
            %   q0          initial angle (rad)
            %   qf          final angle (rad)
            %   useQuintic  (optional) true = quintic, false = cubic (default cubic)
            %
            % Example:
            %   demo_traj(0, 3, 0, pi/2)
            %   demo_traj(0, 3, 0, pi/2, true)

            cubicCoeffs = TrajGenerator.cubic_traj(t0, tf, q0, qf);
            quinticCoeffs = TrajGenerator.quinitic_traj(t0, tf, q0, qf);

            N  = 300; % number of data points to use
            ts = linspace(t0, tf, N).'; % array of evenly spaced dts
            
            % Nx3: [timestamp, cubic, quintic]
            positions     = zeros(N, 3);
            velocities    = zeros(N, 3);
            accelerations = zeros(N, 3);

            for i = 1:N
                t = ts(i);
                positions(i,1) = t;
                positions(i,2) = TrajGenerator.eval_traj(cubicCoeffs,   t);
                positions(i,3) = TrajGenerator.eval_traj(quinticCoeffs, t);
            end

            % Numeric derivatives
            velocities(:,1) = ts;
            velocities(:,2) = gradient(positions(:,2), ts);
            velocities(:,3) = gradient(positions(:,3), ts);

            accelerations(:,1) = ts;
            accelerations(:,2) = gradient(velocities(:,2), ts);
            accelerations(:,3) = gradient(velocities(:,3), ts);

            % Plot
            figure;

            subplot(3,1,1)
            plot(ts, positions(:,2), 'LineWidth', 2); hold on;
            plot(ts, positions(:,3), 'LineWidth', 2);
            grid on; ylabel('Position (rad)');
            legend('Cubic','Quintic');

            subplot(3,1,2)
            plot(ts, velocities(:,2), 'LineWidth', 2); hold on;
            plot(ts, velocities(:,3), 'LineWidth', 2);
            grid on; ylabel('Velocity (rad/s)');
            legend('Cubic','Quintic');

            subplot(3,1,3)
            plot(ts, accelerations(:,2), 'LineWidth', 2); hold on;
            plot(ts, accelerations(:,3), 'LineWidth', 2);
            grid on; ylabel('Acceleration (rad/s^2)');
            xlabel('Time (s)');
            legend('Cubic','Quintic');
        end
    end
end
