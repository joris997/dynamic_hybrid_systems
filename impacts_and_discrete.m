function [value, isterminal,direction] = impacts_and_discrete(t,y)
lBeam1 = 0.2; lBeam2 = 0.2; lBeam3 = 0.2; lBeam4 = 0.2;
rJoint1 = 0.05; rJoint2 = 0.05; rJoint3 = 0.05;
hBeam = 0.02; bBeam = 0.02;

alpha = pi/2 - (2*pi - 2*y(3))/4;
value = [-cos(y(2) - y(3)/2)*lBeam1;
         -cos(y(2) + y(3)/2)*lBeam4;
         y(3)];
     
isterminal = [1;
              1;
              1];
          
direction = [1;
             -1;
             -1];
end