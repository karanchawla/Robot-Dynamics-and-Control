function [y, ydot, yddot] = spline(t)

for i = 0:0.01:1000
    if(i<1)
        x = [0;0;1.5000;-1.0000];
        y = x(1) + x(2)*i + x(3)*i^2 + x(4)*i^3;
        ydot = x(2) + 2*x(3)*i + 3*x(4)*i^2;
        yddot = 2*x(3) + 6*x(4)*i;
        plot(i,ydot,'b.');
        hold on;
    end
    
    if(i>=1 && i<=2)
        x = [-2.0000;6.0000;-4.5000;1.0000];
        y = x(1) + x(2)*i + x(3)*i^2 + x(4)*i^3;
        ydot = x(2) + 2*x(3)*i + 3*x(4)*i^2;
        yddot = 2*x(3) + 6*x(4)*i;
        plot(i,ydot,'r.');
        hold on;
    end
    
    if(i>2)
        y = 0;
        ydot = 0;
        yddot = 0;
    end
    
end

end

