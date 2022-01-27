y = out.y.Data;
t = out.y.Time;



figure
subplot(3,2,1)
% plot([0 1 1.0001 3 3.0001 5 5.0001 7],[100 100 0 0 0 0 -100 -100],'LineWidth',2)
plot(out.sine.Time,out.sine.Data,'LineWidth',2)
hold on
plot(out.u.Time,out.u.Data,'LineWidth',2)
grid on
xlabel('time [s]')
ylabel('voltage [V]')
title('Supplied voltage')
legend('Desired','Actual')
set(gca,'FontSize',10)

subplot(3,2,2)
% plot([0 1 1.0001 3 3.0001 7],[0 0 10 10 0 0],'LineWidth',2)
plot(out.u1.Time,out.u1.Data,'LineWidth',2) 
grid on
xlabel('time [s]')
ylabel('brake pressure [-]')
title('Brake pressure')
set(gca,'FontSize',10)


subplot(3,2,[3 5])
plot(t,y(:,1),'LineWidth',2)
hold on
plot(t,y(:,2),'LineWidth',2)
plot(t,y(:,3),'LineWidth',2)
grid on
legend('q1','q2','q3')
xlabel('time [s]')
ylabel('gen coordinate [m]/[rad]')
title('generalized coordinates')
set(gca,'FontSize',10)

subplot(3,2,[4 6])
plot(t,y(:,4),'LineWidth',2)
hold on
plot(t,y(:,5),'LineWidth',2)
plot(t,y(:,6),'LineWidth',2)
grid on
legend('dq1','dq2','dq3')
xlabel('time [s]')
ylabel('gen coordinate velocity [m/s]/[rad/s]')
title('generalized velocities')
set(gca,'FontSize',10)

% figure(3)
% for i = 1:length(y)
%     if mod(i,round(length(y)/100)) == 0
%         q1 = y(i,1);
%         q2 = y(i,2);
%         q3 = y(i,3);
% 
%         cartX = q1; 
%         cartY = 0;
%         alpha = pi/2 - (2*pi - 2*q3)/4;
%         joint1X = cartX + sin(q2 - q3/2)*lBeam1;
%         joint1Y = cartY - cos(q2 - q3/2)*lBeam1;
%         joint2X = joint1X + sin(q2 + alpha)*lBeam2;
%         joint2Y = joint1Y - cos(q2 + alpha)*lBeam2;
% 
%         joint3X = cartX + sin(q2 + q3/2)*lBeam4;
%         joint3Y = cartY - cos(q2 + q3/2)*lBeam4;
% 
%         figure(3)
%         clf
%         plot([cartX;joint1X],[cartY;joint1Y],'b')
%         hold on
%         plot([joint1X;joint2X],[joint1Y;joint2Y],'b')
%         plot([cartX;joint3X],[cartY;joint3Y],'b')
%         plot([joint3X;joint2X],[joint3Y;joint2Y],'b')
%         xlim([-5 5])
%         ylim([-0.5 0.5])
%         drawnow
%     end
% end
