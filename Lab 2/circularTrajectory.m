xr0 = 10.0;
r0 = 4.0;
yr0 = 0;
zr0 = 8.0;
T = 10;

td1n = 0;
td2n = 0;
td3n = 0;
delta = 0.01;

td10 = 0;
td20 = 0;
td30 = 0;

figure;

%Infinity trajectory 
for t = 0:0.1:10
    phi = 2*pi*t/T;
    
    x = xr0 + r0*sin(phi);
    y = yr0;
    z = zr0 + r0*cos(phi);
    
    x0 = xr0 + r0*sin(phi - 2*pi*delta/T);
    y0= yr0;
    z0 = zr0 + r0*cos((phi - 2*pi*delta/T));
       
    x2 = xr0 + r0*sin(phi + 2*pi*delta/T);
    y2= yr0;
    z2 = zr0 + r0*cos((phi + 2*pi*delta/T));
    
    [td1,td2,td3] = inverseK(x, y, z, 10.0);
    [td10,td20,td30] = inverseK(x0, y0, z0, 10.0);
    [td1n,td2n,td3n] = inverseK(x2,y2,z2,10);
    
    td1dot = (td1 - td10)/delta/2;
    td2dot = (td2 - td20)/delta/2;
    td3dot = (td3 - td30)/delta/2;
    
    td1ddot = (td1 - 2*td1n + td10)/(delta*delta);
    td2ddot = (td2 - 2*td2n + td20)/(delta*delta);
    td3ddot = (td3 - 2*td3n + td30)/(delta*delta);
    
    subplot(3,1,1)     
    plot(t,td1,'r.');
    plot(t,td2,'g.');
    plot(t,td3,'b.');
    title('Theta dot')
    hold on;
    
    subplot(3,1,2) 
    plot(t,td1ddot,'r.');
    plot(t,td2ddot,'g.');
    plot(t,td3ddot,'b.');
    hold on;
    title('Theta doubledot')
   
    subplot(3,1,3)
    xlim([6,14]);
    ylim([4,12]);
    plot(x2,z2,'r.');
    title('Trajectory')
    hold on;
    
end


