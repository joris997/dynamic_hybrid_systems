% com.cart.x = q1;
% com.cart.y = 0;
% com.cart.p = 0;
% 
% com.beam.x(1) = com.cart.x + sin(q4)*lBeam1/2;
% com.beam.y(1) = com.cart.y - cos(q4)*lBeam1/2;
% com.beam.p(1) = q4;
% 
% com.joint.x(1) = com.cart.x + sin(q4)*lBeam1;
% com.joint.y(1) = com.cart.y - cos(q4)*lBeam1;
% com.joint.p(1) = 0;
% 
% com.beam.x(2) = com.joint.x(1) + sin(q5)*lBeam2/2;
% com.beam.y(2) = com.joint.y(1) - cos(q5)*lBeam2/2;
% com.beam.p(2) = q5;
% 
% com.joint.x(2) = com.joint.x(1) + sin(q5)*lBeam2;
% com.joint.y(2) = com.joint.y(1) - cos(q5)*lBeam2;
% com.joint.p(2) = 0;
% 
% com.beam.x(4) = com.cart.x + sin(q2)*lBeam4/2;
% com.beam.y(4) = com.cart.y - cos(q2)*lBeam4/2;
% com.beam.p(4) = q2;
% 
% com.joint.x(3) = com.cart.x + sin(q2)*lBeam4;
% com.joint.y(3) = com.cart.y - cos(q2)*lBeam4;
% com.joint.p(3) = 0;
% 
% com.beam.x(3) = com.joint.x(3) + sin(q3)*lBeam3/2;
% com.beam.y(3) = com.joint.y(3) - cos(q3)*lBeam3/2;
% com.beam.p(3) = q3;
% 
% com.joint.x(4) = com.joint.x(3) + sin(q3)*lBeam3;
% com.joint.y(4) = com.joint.y(3) - cos(q3)*lBeam3;
% com.joint.p(4) = 0;

%% try again with 3 gen coordinates
com.cart.x = q1;
com.cart.y = 0;
com.cart.p = 0;

com.beam.x(1) = com.cart.x + sin(q2 - q3/2)*lBeam1/2;
com.beam.y(1) = com.cart.y - cos(q2 - q3/2)*lBeam1/2;
com.beam.p(1) = q2 - q3/2;

com.joint.x(1) = com.cart.x + sin(q2 - q3/2)*lBeam1;
com.joint.y(1) = com.cart.y - cos(q2 - q3/2)*lBeam1;
com.joint.p(1) = 0;

alpha = pi/2 - (2*pi - 2*q3)/4;

com.beam.x(2) = com.joint.x(1) + sin(q2 + alpha)*lBeam2/2;
com.beam.y(2) = com.joint.y(1) - cos(q2 + alpha)*lBeam2/2;
com.beam.p(2) = q2 + alpha;

com.joint.x(2) = com.joint.x(1) + sin(q2 + alpha)*lBeam2;
com.joint.y(2) = com.joint.y(1) - cos(q2 + alpha)*lBeam2;
com.joint.p(2) = 0;

com.beam.x(4) = com.cart.x + sin(q2 + q3/2)*lBeam4/2;
com.beam.y(4) = com.cart.y - cos(q2 + q3/2)*lBeam4/2;
com.beam.p(4) = q2 + q3/2;

com.joint.x(3) = com.cart.x + sin(q2 + q3/2)*lBeam4;
com.joint.y(3) = com.cart.y - cos(q2 + q3/2)*lBeam4;
com.joint.p(3) = 0;

com.beam.x(3) = com.joint.x(3) + sin(q2 - alpha)*lBeam3/2;
com.beam.y(3) = com.joint.y(3) - cos(q2 - alpha)*lBeam3/2;
com.beam.p(3) = q2 - alpha;

dcom.cart.x = sym(0); dcom.cart.y = sym(0); dcom.cart.p = sym(0);
dcom.beam.x = sym(zeros(4,1)); 
dcom.beam.y = sym(zeros(4,1)); 
dcom.beam.p = sym(zeros(4,1));
dcom.joint.x = sym(zeros(3,1)); 
dcom.joint.y = sym(zeros(3,1)); 
dcom.joint.p = sym(zeros(3,1));

for i = 1:length(q)
    dcom.cart.x = dcom.cart.x + diff(com.cart.x,q(i))*dq(i);

    for ii = 1:4
        dcom.beam.x(ii) = dcom.beam.x(ii) + diff(com.beam.x(ii),q(i))*dq(i);
        dcom.beam.y(ii) = dcom.beam.y(ii) + diff(com.beam.y(ii),q(i))*dq(i);
        dcom.beam.p(ii) = dcom.beam.p(ii) + diff(com.beam.p(ii),q(i))*dq(i);
    end
    for ii = 1:3
        dcom.joint.x(ii) = dcom.joint.x(ii) + diff(com.joint.x(ii),q(i))*dq(i);
        dcom.joint.y(ii) = dcom.joint.y(ii) + diff(com.joint.y(ii),q(i))*dq(i);
        dcom.joint.p(ii) = dcom.joint.p(ii) + diff(com.joint.p(ii),q(i))*dq(i);
    end
end
