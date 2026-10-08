classdef ImageProcessor < handle
    properties 
        camera;         % The Camera object for this image processor
        debug;          % Debug Mode (True/False)
        static_mask;    % Static Mask to isolate checkerboard
    end

    methods
        function self = ImageProcessor(nvargs)
            arguments
                nvargs.debug logical = false;
            end

            self.debug = nvargs.debug;
            self.camera = Camera();
            self.static_mask = self.generate_static_mask();
        end

        function mask = generate_static_mask(self, nvargs)
            arguments
                self ImageProcessor;
                nvargs.margin double = 100;
            end
            %GENERATE_STATIC_MASK produces a binary mask that leaves only
            %the checkerboard and surrounding area visible
            % Inputs: 
            %   margin (optional): a number (what units?) indicating how
            %                      far from the edge of the checkerboard 
            %                      to keep in the mask
            % Outputs:
            %   mask: a binary mask that blacks out everything except a
            %         region of interest around the checkerboard

            img = self.camera.getImage();

            % Get image pixel size 
            [height, width, ~] = size(img);

            % Find checkerboard points
            [imagePoints, boardSize] = detectCheckerboardPoints(img);

            if isempty(imagePoints)
                disp("Checkerboard not detected. Make sure it is fully visible.");
                mask = poly2mask([1,1,width,width],[1,height,height],height,width);
                return
            end

            % Image points looks like [ x1 y1; x2 y2; x3 y3; ect] for each checkerboard corner
            % We want to get the min and max, x and y positions of the points to define the boundry 
            allCheckboardXs = imagePoints(:,1);
            allCheckboardYs = imagePoints(:,2);

            % This gets the values of the box within some margin
            maxX = max(allCheckboardXs) + nvargs.margin;
            maxY = max(allCheckboardYs) + nvargs.margin;
            minX = min(allCheckboardXs) - nvargs.margin;
            minY = min(allCheckboardYs) - nvargs.margin;

            % Clamp values so they stay inside image
            minX = max(1, round(minX));
            minY = max(1, round(minY));
            maxX = min(width, round(maxX));
            maxY = min(height, round(maxY));

            % Need to make a polygon (rectangle) out of the min and max points:
            x = [minX minX maxX maxX minX];
            y = [minY maxY maxY minY minY];
            mask = poly2mask(x,y,height,width);

            % Debug display
            if self.debug
                masked_img = img .* uint8(mask);
                figure
                imshow(masked_img)
                title("Static Mask Result")
            end
        end

        function p_robot = image_to_robot(self, uvpos)
            %IMAGE_TO_ROBOT transforms a point on the image to the
            % corresponding point in the frame of the robot
            % Inputs:
            %   uvpos: a [1x2] matrix representing an image (u, v) 
            %          coordinate
            % Outputs:
            %   p_robot: a [1x2] matrix representing the transformation of
            %            the input uvpos into the robot's base frame
            
            % Convert image coordinates to checkerboard frame
            xy_pos = pointsToWorld(self.camera.getCameraInstrinsics(), self.camera.getRotationMatrix(), ...
                        self.camera.getTranslationVector(), uvpos);
            
            % Convert checkerboard frame to robot frame (switch x-y,
            % perform translation between orientations)
            x_robot = 95 + xy_pos(2);
            y_robot = -112 + xy_pos(1);
            
            p_robot = [x_robot, y_robot];

        end

        function [colors, uv_centroids] = detect_centroids(self, image, nvargs)
            arguments
                self ImageProcessor;
                image uint8;
                nvargs.min_size double = 50;  % chooose a value
            end
            %DETECT_CENTROIDS detects the centroids of binary blobs of
            %large enough size
            % Iputs: 
            %   Image: an image of the environment that has already been
            %          masked for the environment and to isolate a single 
            %          color
            %   min_size (optional): the minimum size of a blob to consider
            %                        a ball
            % Outputs: 
            %   colors: a [1xn] matrix of strings indicating the color of
            %           the ball at each detected centroid
            %   uv_centroids: a [nx2] matrix of coordinates of valid
            %                 centroids in image coordinates
            
            % Apply first color mask, isolates balls from background WITH
            % colors maintained
            balls = image .* uint8(repmat(ballMask(image), 1, 1, 3));
    
            % Apply indiviudal, binary color masks
            rMasked = redMask(balls);
            oMasked = orangeMask(balls);
            yMasked = yellowMask(balls);
            gMasked = greenMask(balls);
            
            % Consolidate into matrix for iteration
            eachMasked(:, :, 1) = rMasked;
            eachMasked(:, :, 2) = oMasked;
            eachMasked(:, :, 3) = yMasked;
            eachMasked(:, :, 4) = gMasked;

            % Create vector for creating colors vector during iteration
            color_labels = ["Red", "Orange", "Yellow", "Green"];

            if self.debug
                
                %close all
                figure

            end

            uv_centroids = []
            colors = []

            % Iterate through each image and its corresponding color by
            % index
            for i = 1:4

                % Find centroids in image, extract information relating to
                % location and size
                stats = regionprops("table",eachMasked(:, :, i),"Centroid", ...
                    "MajorAxisLength","MinorAxisLength");
                centers = stats.Centroid;
                diameters = mean([stats.MajorAxisLength stats.MinorAxisLength], 2);
                radii = diameters/2;
                areas = radii .* pi;

                % Filter out centroids below minimum size to remove noise,
                % append coords to uv_centroids
                ball_centroids = centers(areas >= nvargs.min_size, :);
                uv_centroids = [uv_centroids; ball_centroids];
                
                % Determine number of balls of this color, append to the
                % colors vector
                num_balls = size(ball_centroids, 1)

                for j = 1:num_balls
                    
                    colors = [colors, color_labels(i)];

                end
                
                % Plot isolated colors and centroid detection if in debug mode
                if self.debug

                    subplot(2, 2, i); imshow(eachMasked(:, :, i)); title(color_labels(i));
                    hold on
                    viscircles(ball_centroids, radii(areas >= nvargs.min_size));
                    hold off
                
                end

                %disp(uv_centroids)

            end

        end

        function ts_centroids = correct_centroids(self, centroids, nvargs)
            arguments
                self ImageProcessor
                centroids double;
                nvargs.ball_z = 20;
            end

            %CORRECT_CENTROIDS transforms image coordinate centroids into
            %task-space coordinates for the ball
            % Inputs: 
            %   centroids: a [nx2] array of centroids in image coordinates
            %   ball_z (optional): how high the center of the ball is in
            %                      millimeters
            
            % Iterate through each centroid
            for i = 1:size(centroids, 1)
                
                % Convert image frame to robot frame, and then transform to
                % camera frame using translation in the x and reflection in
                % the y
                centroid_xy = self.image_to_robot([centroids(i, 1), centroids(i, 2)]);
                centroid_xy_cam = [360 - centroid_xy(1), -centroid_xy(2)];
    
                % Determine actual X-Y using Law of Similar Triangles
                Aprime = sqrt(centroid_xy_cam(1)^2 + centroid_xy_cam(2)^2);
                theta = atan2(centroid_xy_cam(1), centroid_xy_cam(2));
                B = nvargs.ball_z/2; %half of ball height
                Bprime = 172; %post height
    
                A = B/(Bprime/Aprime);
                Adoubleprime = Aprime - A;
    
                x = sin(theta) * Adoubleprime;
                y = cos(theta) * Adoubleprime;
    
                % Convert back to robot frame
                ts_centroids(i, :) = [360 - x, -y];
            
            end

        end
        
        function [ballColors, ballPoses] = detect_balls (self)
            %DETECT_BALLS finds the task space coordinates of all balls on the
            %checkerboard
            % Outputs:
            %   ts_coords: the task space coordinates of all balls in the
            %              workspace
            
            % Get image of board
            img = self.camera.getImage();

            % Mask to keep only the checkerboard region
            %staticMask = self.generate_static_mask();
            staticMask = self.static_mask;

            % Applying the mask to the image
            % maskedImg = img .* uint8(staticMask);
            maskedImg = img .* uint8(repmat(staticMask, 1, 1, 3));  % repmat to fix odd bug where 'Matrix dimensions must agree' - only on my device...
            [ballColors, uv_centroids] = self.detect_centroids(maskedImg);
            
            % If there are no balls return empty values
            if isempty(uv_centroids) 
                ballColors = [];
                ballPoses = [];
                return;
            end
            ballPoses = self.correct_centroids(uv_centroids);
        end
        
    end

end
