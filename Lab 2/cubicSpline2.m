t0 = 1;
tf = 2;
qf = 0;
q0 = 0.5;
v0 = 0;
vf = 0;

A = [1 t0 t0^2 t0^3;
    0 1 2*t0 3*t0^2;
    1 tf tf^2 tf^3;
    0 1 2*tf 3*tf^2;];
b = [q0 v0 qf vf]';
x = inv(A)*b

for i = 1:0.01:2
    y = x(1) + x(2)*i + x(3)*i^2 + x(4)*i^3;
    ydot = x(2) + 2*x(3)*i + 3*x(4)*i^2;
    yddot = 2*x(3) + 6*x(4)*i;
    plot(i,ydot,'b.');
    hold on;
end
