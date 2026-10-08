%robot = Robot(true);

ang_test1 = [0 0 0 0];
ang_test2 = [-10 -130 70 9];
ang_test3 = [-20 -35 0 -10];

test1 = robot.fk_3001(ang_test1);
test2 = robot.fk_3001(ang_test2);
test3 = robot.fk_3001(ang_test3);

%ik_test1 = robot.ik3001([test1(1:3, 4).', ang_test1(4)]);
ik_test2 = robot.ik3001([test2(1:3, 4).', ang_test2(4)]);
% ik_test3 = robot.ik3001([test3(1:3, 4).', ang_test3(4)]);

%display([ang_test1; ik_test1])
display([ang_test2; ik_test2])

display(test2)
display(robot.fk_3001(ik_test2))
% 
% ik_fk_test1 = robot.fk_3001(ik_test1)
% ik_fk_test2 = robot.fk_3001(ik_test2);
% 
% x = [test1(1, 4), ik_fk_test1(1, 4)];
% y = [test1(2, 4), ik_fk_test1(2, 4)];
% z = [test1(3, 4), ik_fk_test1(3, 4)];
% 
% norm([x(1), y(1), z(1)] - [x(2), y(2), z(2)]);
% 
% figure
% scatter3(x, y, z, 'filled');
% hold on;
% plot3(x, y, z, 'r-', 'LineWidth', 2); % Draws line through points
% grid on;
% xlim([0 300]);
% xlabel('x');
% ylabel('y');
% zlabel('z');
% ylim([0 300]);
% zlim([0 300]);