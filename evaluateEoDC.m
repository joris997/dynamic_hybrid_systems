function out1 = evaluateEoDC(t,in2,vol)
%EVALUATEEODC
%    OUT1 = EVALUATEEODC(T,IN2,VOL)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    01-Feb-2021 22:42:59

cur = in2(1,:);
out1 = [cur.*-1.0e+1+vol./1.0e+2;0.0];
