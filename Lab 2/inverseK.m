function [theta1, theta2, theta3] = inverseK(x, y, z, L)
theta1 = atan2(y,x);
R = sqrt(x^2 + y^2);
d = sqrt(R^2 + z^2);
beta = atan2(z,R);
gamma = atan2(sqrt(L^2 - d^2/4), d/2);
theta2 = pi/2 - gamma - beta;
theta3 = pi/2 + gamma - beta;