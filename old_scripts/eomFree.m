Ti = [com.cart.x com.cart.y ...
    com.beam.x(1) com.beam.y(1) com.beam.p(1) ...
    com.joint.x(1) com.joint.y(1) com.joint.p(1) ...
    com.beam.x(2)  com.beam.y(2) com.beam.p(2) ...
    com.joint.x(2) com.joint.y(2) com.joint.p(2) ...
    com.beam.x(4) com.beam.y(4) com.beam.p(4) ...
    com.joint.x(3) com.joint.y(3) com.joint.p(3) ...
    com.beam.x(3) com.beam.y(3) com.beam.p(3) ...
    com.joint.x(4) com.joint.y(4) com.joint.p(4)]';

%% generalized forces
% jacobian dT/dq
Tq = jacobian(Ti,q);
dT = Tq*dq;

% convective terms
h = jacobian(dT,q)*dq;
h = simplify(h);

% mass matrix
M = diag([mCart mCart mBeam1 mBeam1 IBeam1 ...
    mJoint1 mJoint1 0 mBeam2 mBeam2 IBeam2 ...
    mJoint2 mJoint2 0 mBeam4 mBeam4 IBeam4 ...
    mJoint3 mJoint3 0 mBeam3 mBeam3 IBeam3 ...
    mJoint2 mJoint2 0]);
M_bar = Tq'*M*Tq;
M_bar = simplify(M_bar);

% external force vector
F_ext = sym(zeros(26,1));
F_ext(1) = F;

% gravity
f = [0 -mCart*g 0 -mBeam1*g 0 ...
    0 -mJoint1*g 0 0 -mBeam2*g 0 ...
    0 -mJoint2*g 0 0 -mBeam4*g 0 ...
    0 -mJoint3*g 0 0 -mBeam3*g 0 ...
    0 -mJoint2*g 0]';

% spring
l0 = 0;
diffX = com.joint.x(3) - com.joint.x(1);
diffY = com.joint.y(3) - com.joint.y(1);
Cspring = sqrt(diffX^2 + diffY^2)-l0;
SigmaSpring = kSpring*Cspring;

% damping
Cjoint1 = q4 + q5;
Cjoint2 = q3 - q5;
Cjoint3 = q2 + q3;
Cjoint4 = q4 - q2;
Ccart = q1;
Cdamper = sqrt(diffX^2 + diffY^2)-l0;
Cdamping = [Cjoint1 Cjoint2 Cjoint3 Cjoint4 Ccart Cdamper]';


dCdamping = sym(zeros(size(Cdamping)));
for i = 1:length(q)
    for ii = 1:length(Cdamping)
        dCdamping(ii) = dCdamping(ii) + diff(Cdamping(ii),q(i))*dq(i);
    end
end
SigmaDamping = simplify(losses.*dCdamping);

% creation of rhs
Q = Tq'*F_ext;
Q_bar = simplify(Q + Tq'*(f - M*h ...
    - Tq*jacobian(Cspring,q)'*SigmaSpring ...
    - Tq*jacobian(Cdamping,q)'*SigmaDamping));

% constraints
c1 = com.joint.x(2) - com.joint.x(4);
c2 = com.joint.y(2) - com.joint.y(4);
C = [c1; c2];

% jacobian dC/dq
Cq = simplify(jacobian(C,q));

% convective terms C,qq*dq*dq which are determined by the definition
% according to d(dC/dt)/dq*dq
Cd = Cq*dq;
C2 = simplify(jacobian(Cd,q)*dq);

A = [M_bar Cq';
     Cq zeros(length(C))];
b = [Q_bar;
     -C2];

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
EoM = simplify(A\b);

%% create function to evaluate eom
eom = matlabFunction([dq; EoM(1:5)],'Vars',{t,[q; dq],F}, ...
    'File', 'evaluateEoM');