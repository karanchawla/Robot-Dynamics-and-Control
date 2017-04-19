x0 = 13.5;
z0 = 13.5;
y0 = 0;

for t=0:0.05:10
   if t<2
      x = x0 - 0.5*t;
      y = y0;
      z = z0;
   end
   plot(x,z,'b+');
   hold on;
   if t>=2 && t<3
      x1 = x;
      y = y0;
      z = z0 + 0.5*(t-2);
   end 
    plot(x1,z1,'r+');
    if t>=3 && t<5
       x2 = 12.5250 + 0.5*(t-3);
       y = y0;
       z2 = z;
    end
    plot(x2,z2,'y+');
    
    
    
end