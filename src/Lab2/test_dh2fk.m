robot = Robot(true);  % enable dummy mode

dh_tab = zeros(1,4);  % TODO: update with your DH table
robot.dh2fk(dh_tab)
