clear; close; clc;

%% symbolic variables
syms g t F real
syms mCart bWheels real
syms bJoint1 bJoint2 bJoint3 bJoint4 real
syms mJoint1 mJoint2 mJoint3 real
mJoint = [mJoint1 mJoint2 mJoint3]';
syms mBeam1 mBeam2 mBeam3 mBeam4 real
mBeam = [mBeam1 mBeam2 mBeam3 mBeam4]';
syms lBeam1 lBeam2 lBeam3 lBeam4 real
lBeam = [lBeam1 lBeam2 lBeam3 lBeam4]';
syms hBeam bBeam real
syms kSpring bSpring real
losses = [bJoint1 bJoint2 bJoint3 bJoint4 bWheels bSpring]';

syms q1 q2 q3 q4 q5 real
syms dq1 dq2 dq3 dq4 dq5 real
syms ddq1 ddq2 ddq3 ddq4 ddq5 real

%% symbolic derivations
IBeam1 = 1/12*mBeam1*(lBeam1^2 + hBeam^2);
IBeam2 = 1/12*mBeam2*(lBeam2^2 + hBeam^2);
IBeam3 = 1/12*mBeam3*(lBeam3^2 + hBeam^2);
IBeam4 = 1/12*mBeam4*(lBeam4^2 + hBeam^2);
IBeam = [IBeam1 IBeam2 IBeam3 IBeam4]';
 
%% define generalized coordinates
q = [q1 q2 q3 q4 q5]';
dq = [dq1 dq2 dq3 dq4 dq5]';
ddq = [ddq1 ddq2 ddq3 ddq4 ddq5]';

%% position of c.o.m. and their derivatives
comPositions;
if true
    eomFree;
end

%% values for system properties
mBeam1 = 1; mBeam2 = 1; mBeam3 = 1; mBeam4 = 1;
mCart = 2;
mJoint1 = 0.5; mJoint2 = 0.5; mJoint3 = 0.5;
lBeam1 = 0.2; lBeam2 = 0.2; lBeam3 = 0.2; lBeam4 = 0.2;
hBeam = 0.02; bBeam = 0.02;
kSpring = 10; bSpring = 500;
g = 9.81;

%% initial conditions
q10 = 0;                % position cart
q20 = deg2rad(15);
q30 = deg2rad(-15);
q40 = deg2rad(-15);
q50 = deg2rad(15);
y0 = [q10 q20 q30 q40 q50 0 0 0 0 0]';

%% simulate ODE
tspan = [0 4];
tVector = linspace(0,tspan(end),100);
% FVector = 100*sin(2*pi*10*tVector);
FVector = zeros(1,length(tVector));
FVector(1) = 200;
[t,y] = ode45(@(t,y) evaluateEoM(t,y,FVector,tVector),tspan,y0);

%% visualization
figure(1)
plot(t,y(:,1))
hold on
plot(t,y(:,2))
plot(t,y(:,3))
plot(t,y(:,4))
plot(t,y(:,5))
grid on
legend('q1','q2','q3','q4','q5')
xlabel('time [s]')
ylabel('gen coordinate [m]/[rad]')

figure(2)
for i = 1:length(y)
    q1 = y(i,1);
    q2 = y(i,2);
    q3 = y(i,3);
    q4 = y(i,4);
    q5 = y(i,5);
    
    cartX = q1; 
    cartY = 0;
    joint1X = cartX + sin(q4)*lBeam1;
    joint1Y = cartY - cos(q4)*lBeam1;
    joint2X = joint1X + sin(q5)*lBeam2;
    joint2Y = joint1Y - cos(q5)*lBeam2;
    
    joint3X = cartX + sin(q2)*lBeam4;
    joint3Y = cartY - cos(q2)*lBeam4;
    joint4X = joint3X + sin(q3)*lBeam3;
    joint4Y = joint3Y - cos(q3)*lBeam3;
    
    figure(2)
    clf
    plot([cartX;joint1X],[cartY;joint1Y],'b')
    hold on
    plot([joint1X;joint2X],[joint1Y;joint2Y],'b')
    plot([cartX;joint3X],[cartY;joint3Y],'b')
    plot([joint3X;joint4X],[joint3Y;joint4Y],'b')
    plot([joint1X;joint3X],[joint1Y;joint3Y],'g')
    xlim([-0.5 0.5])
    ylim([-0.5 0.5])
    drawnow
end