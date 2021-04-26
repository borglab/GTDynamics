%% simplified, x/y/vx/vy model of cable robot at center location
clear
dt = 0.01;
m = 1;

A = [zeros(2), eye(2);...
     zeros(2), zeros(2)];
B_F = [zeros(2);...
       eye(2) / m];
F_T = [1, 1, -1, -1;...
       -1, 1, 1, -1] / sqrt(2);
B = B_F * F_T;
Q = diag([2, 3, 0, 0]);
R = diag(repmat([0.12], 4, 1));
[K, ~, ~] = lqrd(A, B, Q, R, dt);

format short
K
