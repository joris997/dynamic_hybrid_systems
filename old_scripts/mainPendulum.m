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
q = [q1 q2]';
dq = [dq1 dq2]';
ddq = [ddq1 ddq2]';

%% position of c.o.m. and their derivatives
com.joint.x(1) = sin(q1)*lBeam1;
com.joint.y(1) = - cos(q1)*lBeam1;

com.joint.x(2) = com.joint.x(1) + sin(q2)*lBeam2;
com.joint.y(2) = com.joint.y(1) - cos(q2)*lBeam2;

dcom.joint.x = sym(zeros(2,1));
dcom.joint.y = sym(zeros(2,1));
for i = 1:length(q)
    for ii = 1:2
        dcom.joint.x(ii) = dcom.joint.x(ii) + diff(com.joint.x(ii),q(i))*dq(i);
        dcom.joint.y(ii) = dcom.joint.y(ii) + diff(com.joint.y(ii),q(i))*dq(i);
    end
end

%% kinetic energy
T = 0;
for i = 1:2
    T = T + 1/2*mJoint(i)*dcom.joint.x(i)^2;
    T = T + 1/2*mJoint(i)*dcom.joint.y(i)^2;
end

%% potential energy
V = 0;
% potential energy joints
for i = 1:2
    V = V + mJoint(i)*g*com.joint.y(i);
end

%% damping


%% external forces
Q = zeros(2,1);

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
mCart = 2;
mJoint1 = 5; mJoint2 = 0.5;
lBeam1 = 0.2; lBeam2 = 0.2;
hBeam = 0.02; bBeam = 0.02;
g = 9.81;

A = subs(A);
b = subs(b);

%% create function to evaluate eom
eom = matlabFunction([dq; simplify(A\b)],'Vars',{t,[q; dq]}, ...
    'File', 'evaluateEoM');

%% initial conditions
q10 = deg2rad(0);
q20 = deg2rad(15);
y0 = [q10 q20 0 0]';

%% simulate ODE
tspan = [0 5];
[t,y] = ode45(@evaluateEoM,tspan,y0);

%% visualization
figure(1)
plot(t,y(:,1))
hold on
plot(t,y(:,2))
grid on
legend('q1','q2')
xlabel('time [s]')
ylabel('gen coordinate [m]/[rad]')


figure(2)
for i = 1:length(y)
    q1 = y(i,1);
    q2 = y(i,2);

    joint1X = sin(q1)*lBeam1;
    joint1Y = - cos(q1)*lBeam1;
    joint2X = joint1X + sin(q2)*lBeam2;
    joint2Y = joint1Y - cos(q2)*lBeam2;

    
    figure(2)
    plot(joint1X,joint1Y,'b*')
    hold on
    plot(joint2X,joint2Y,'b*')
    xlim([-0.5 0.5])
    ylim([-0.5 0.5])
    drawnow
end