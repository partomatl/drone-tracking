clear
clc
close all

load('recording2017_04_25_13_16.mat')

figure

subplot(2,2,1)
plot(time,x)
hold on
plot(time,xFiltered,'LineWidth',1)
hline = refline([0 0]);
hline.LineWidth=2;
xlabel('Time (s)')
title('x (cm)')
legend('raw x', 'filtered x')

subplot(2,2,2)
plot(time,y)
hold on
plot(time,yFiltered,'LineWidth',1)
hline = refline([0 0]);
hline.LineWidth=2;
xlabel('Time (s)')
title('y (cm)')
legend('raw y', 'filtered y')

subplot(2,2,3)
plot(time,z)
hold on
plot(time,zFiltered,'LineWidth',1)
hline = refline([0 40]);
hline.LineWidth=2;
xlabel('Time (s)')
title('z (cm)')
legend('raw z', 'filtered z')

subplot(2,2,4)
plot(time,angle)
hold on
plot(time,angleFiltered,'LineWidth',1)
hline = refline([0 0]);
hline.LineWidth=2;
xlabel('Time (s)')
title('angle (degrees)')
legend('raw angle', 'filtered angle')

figure

subplot(2,2,1)
plot(time,elevator)
xlabel('Time (s)')
title('Aileron command (PPM)')

subplot(2,2,2)
plot(time,aileron)
xlabel('Time (s)')
title('Elevator command (PPM)')

subplot(2,2,3)
plot(time,throttle)
xlabel('Time (s)')
title('Throttle command (PPM)')

subplot(2,2,4)
plot(time,rudder)
xlabel('Time (s)')
title('Rudder command (PPM)')

figure

subplot(2,2,1)
plot(time, xError)
xlabel('Time (s)')
title('x error (cm)')

subplot(2,2,2)
plot(time, yError)
xlabel('Time (s)')
title('y error (cm)')

subplot(2,2,3)
plot(time, zError)
xlabel('Time (s)')
title('z error (cm)')

subplot(2,2,4)
plot(time, angleError)
xlabel('Time (s)')
title('Angle error (cm)')

ecart=[];

for i=1:length(time)-1
    ecart=[ecart time(i+1)-time(i)];
end

mean(ecart)

figure
plot(time(1:end-1),ecart)

figure
plot3(xFiltered,yFiltered,zFiltered)
xlabel('x (cm)')
ylabel('y (cm)')
zlabel('z (cm)')