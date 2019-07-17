function y = normalizeAngle(u)
%#codegen
% normalizes angle to -pi to pi
y = u; 

y(u>pi) = mod(u(u>pi) + pi, 2*pi) - pi; 
y(u<-pi) = -(mod(-(u(u<-pi) - pi), 2*pi) - pi); 
