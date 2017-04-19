function [x, y, z] = forwardK(theta1, theta2, theta3, L)
R = L * ( sin(theta2) + sin(theta3) );
z = L * ( cos(theta2) + cos(theta3) );
x = R .* cos(theta1);
y = R .* sin(theta1);
end