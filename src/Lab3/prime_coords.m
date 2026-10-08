syms x y z gamma
l = 133.4;

l_xy = l * cos(gamma);
alpha = atan2(y, x);

xyz_prime = [x - abs(l_xy*cos(alpha)); y - abs(l_xy*sin(alpha)); z + l*sin(gamma)];

% matlabFunction(xyz_prime, "File", "get_prime");

% test = round(subs(xyz_prime, {x, y, z, gamma}, {82, -82, 5, 65}), 3)
% 
% x = [82, test(1)];
% y = [-82, test(2)];
% z = [5, test(3)];
% 
% norm([x(1), y(1), z(1)] - [x(2), y(2), z(2)])
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