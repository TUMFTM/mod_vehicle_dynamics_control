function [z,A]=jaccsd_input(f, x, u, tS)
%__________________________________________________________________________
%% Documentation 
% 
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de) 
%               based on code from 
% https://de.mathworks.com/matlabcentral/fileexchange/18189-learning-the-extended-kalman-filter
% Copyright (c) 2009, Yi Cao
% All rights reserved.
% 
% Start Date:   15.02.2018
% 
% Description:  calculates the jacobian of the system dynamics x(k+1) = f(x(k), u(k))
%               with respect to the input u(k). Therefore complex step differentiation is used. 
%
% Inputs: 
%   f     function handle to system dynamics f(x(k), u(k))
%   x     State for which the jacobian is evaluated
%   u     Input for which the jacobian is evaluated
%   tS    Sample rate
% 
% Outputs: 
%   z     system udpate value x(k+1) = f(x(k), u(k))
%   A     Jacobian of the system dynamics df(x(k), u(k))/d(u(k))

%% Algorithm for complex step differentiation
z=f(x,u,tS);
n=numel(u);
m=numel(z);
A=zeros(m,n);
h=n*eps;
for k=1:n
    u1=complex(u);
    u1(k)=u1(k)+h*1i;
    A(:,k)=imag(f(x,u1,tS))/h;
end
end