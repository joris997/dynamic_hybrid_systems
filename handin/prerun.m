close all; clear all;

%% symbolic variables
syms g t real
syms mCart bWheels rWheels real

syms bJoint1 bJoint2 bJoint3 bJoint4 real

syms mJoint1 mJoint2 mJoint3 real
mJoint = [mJoint1 mJoint2 mJoint3]';

syms mBeam1 mBeam2 mBeam3 mBeam4 real
mBeam = [mBeam1 mBeam2 mBeam3 mBeam4]';

syms lBeam1 lBeam2 lBeam3 lBeam4 real
lBeam = [lBeam1 lBeam2 lBeam3 lBeam4]';

syms hBeam bBeam real
syms kSpring bSpring real

syms R L F real

syms icur real % integral of the current, artificial state
syms q1 q2 q3 cur vol pres real
syms dq1 dq2 dq3 dcur real
syms ddq1 ddq2 ddq3 ddcur real

%% symbolic derivations
IBeam1 = 1/12*mBeam1*(lBeam1^2 + hBeam^2);
IBeam2 = 1/12*mBeam2*(lBeam2^2 + hBeam^2);
IBeam3 = 1/12*mBeam3*(lBeam3^2 + hBeam^2);
IBeam4 = 1/12*mBeam4*(lBeam4^2 + hBeam^2);
IBeam = [IBeam1 IBeam2 IBeam3 IBeam4]';
 
%% define generalized coordinates
q = [q1 q2 q3]';
dq = [dq1 dq2 dq3]';
ddq = [ddq1 ddq2 ddq3]';

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
l0 = 0.0;
V = V + 1/2*kSpring*(2*lBeam1*sin(q3/2)-l0)^2;

%% damping
B = diag([2.0,0.05,0.01]);
D = 1/2*dq'*B*dq + ...
    0.1*(lBeam1*cos(q3/2)*dq3)^2 + ...
    1*dq1^2*pres; % + ...
%     1/2*cur*R*cur;

% W = Wdamping + Wfriction;
% Q = jacobian(Wdamping,dq)';
%% external forces
Q = sym(zeros(length(q),1));

R = 1000;
L = 100;
kMotor = 10;
rWheels = 0.05;
Q(1) = Q(1) + (kMotor/rWheels)*cur;

% Q(3) = Q(3) + Wdamping;

%% motor equations
dT_ddq = jacobian(T,dq).';
ddT_ddqdt = jacobian(dT_ddq,t) + ...
            jacobian(dT_ddq,q)*dq + ...
            jacobian(dT_ddq,dq)*ddq;

dT_dq = jacobian(T,q).';

dV_dq = jacobian(V,q).';

dD_ddq = jacobian(D,dq).';

EoM = simplify(ddT_ddqdt - dT_dq + dV_dq - Q + dD_ddq);

[A,b] = equationsToMatrix(EoM,ddq);
A = simplify(A);
b = simplify(b);

%% values for system properties
mBeam1 = 0.5; mBeam2 = 0.5; mBeam3 = 0.5; mBeam4 = 0.5;
mCart = 4;
mJoint1 = 0.1; mJoint2 = 0.1; mJoint3 = 0.1;
lBeam1 = 0.2; lBeam2 = 0.2; lBeam3 = 0.2; lBeam4 = 0.2;
rJoint1 = 0.05; rJoint2 = 0.05; rJoint3 = 0.05;
hBeam = 0.02; bBeam = 0.02;
kSpring = 50; bSpring = 100;
bJoint1 = 0.6; bJoint2 = 0.6; bJoint3 = 0.6; bJoint4 = 0.6;
g = 9.81;

A = subs(A);
b = subs(b);

%% create function to evaluate eom
matlabFunction([dq; simplify(A\b); -R*cur/L + vol/L], ...
    'Vars',{t,[q; dq; cur],vol,pres}, ...
    'File', 'evaluateEoM');

matlabFunction([-R*cur/L + vol/L; 0], ...
    'Vars',{t,[cur;dcur],vol}, ...
    'File', 'evaluateEoDC');
    
%% initial conditions
q10 = 0;                % position cart
q20 = deg2rad(0);
q30 = deg2rad(0);
%     q1  q2  q3  dq1 dq2 dq3 i
y0 = [q10 q20 q30 0   0   0   0]';




