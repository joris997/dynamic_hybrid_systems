close; clear;

%% symbolic variables
syms g t real
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

%% kinetic energy
T = 0;

% kinetic energy cart
T = T + 1/2*mCart*dcom.cart.x^2;
T = T + 1/2*mCart*dcom.cart.y^2;

% kinetic energy beams
for i = 1:4
    T = T + 1/2*mBeam(i)*dcom.beam.x(i)^2;
    T = T + 1/2*mBeam(i)*dcom.beam.y(i)^2;
    T = T + 1/2*IBeam(i)*dcom.beam.p(i)^2;
end

for i = 1:3
    T = T + 1/2*mJoint(i)*dcom.joint.x(i)^2;
    T = T + 1/2*mJoint(i)*dcom.joint.y(i)^2;
end

%% potential energy
V = 0;

% potential energy beams
for i = 1:4
    V = V + mBeam(i)*g*com.beam.y(i);
end

% potential energy joints
for i = 1:3
    V = V + mJoint(i)*g*com.joint.y(i);
end

% potential energy spring
l0 = 0;
diffX = com.joint.x(3) - com.joint.x(1);
diffY = com.joint.y(3) + com.joint.y(1);
V = V + 1/2*kSpring*(sqrt(diffX^2 + diffY^2)-l0)^2;

%% damping

%% constraint
c1 = com.joint.x(2) - com.joint.x(4);
c2 = com.joint.y(2) - com.joint.y(4);
C = [c1, c2];

Cq = jacobian(C,q)'

%% external forces
Q = zeros(5,1);

%% motor equations
dT_ddq = jacobian(T,dq).';
ddT_ddqdt = jacobian(dT_ddq,t) + ...
            jacobian(dT_ddq,q)*dq + ...
            jacobian(dT_ddq,dq)*ddq;

dT_dq = jacobian(T,q).';

dV_dq = jacobian(V,q).';

EoM = simplify(ddT_ddqdt - dT_dq + dV_dq - Q);

[A,b] = equationsToMatrix(EoM,ddq);
A = simplify(A);
b = simplify(b);

%% values for system properties
mBeam1 = 1; mBeam2 = 1; mBeam3 = 1; mBeam4 = 1;
mCart = 2;
mJoint1 = 0.5; mJoint2 = 0.5; mJoint3 = 0.5;
lBeam1 = 0.2; lBeam2 = 0.2; lBeam3 = 0.2; lBeam4 = 0.2;
hBeam = 0.02; bBeam = 0.02;
kSpring = 10; bSpring = 500;
g = 9.81;

A = subs(A);
b = subs(b);

%% create function to evaluate eom
eom = matlabFunction([dq; simplify(A\b)],'Vars',{t,[q; dq]}, ...
    'File', 'evaluateEoM');

%% initial conditions
q10 = 0;                % position cart
q20 = deg2rad(15);
q30 = deg2rad(-15);
q40 = deg2rad(-15);
q50 = deg2rad(15);
y0 = [q10 q20 q30 q40 q50 0 0 0 0 0]';
% y0 = [0 deg2rad(45) deg2rad(-45) 0 0 0 0 0 0 0]';

%% simulate ODE
tspan = [0 5];
% tol = 1e-4;
% opt = odeset('RelTol',1e-11,'AbsTol',1e-4);
[t,y] = ode45(@evaluateEoM,tspan,y0);

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
    xlim([-0.5 0.5])
    ylim([-0.5 0.5])
    drawnow
end








