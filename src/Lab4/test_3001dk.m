robot = Robot(true);  % enable dummy mode

qpos = [0 0 0 0];  % TODO: try different values here
qvel = [0 0 0 0];  % TODO: try different values here
robot.dk3001(qpos, qvel)
