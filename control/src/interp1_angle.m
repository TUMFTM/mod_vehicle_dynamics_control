function [ vq ] = interp1_angle( x, v, xq, method, extrapolation)
  % adaption of the standard matlab interp1 function to address the problems occuring with
  % interpolation of angular values due to discontinuities at -pi/pi%_________________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
% 
% Start Date:   27.01.2018
% 
% Description:  adaption of the standard matlab interp1 function to address
%               the problems occuring with interpolation of angular values 
%               due to discontinuities at -pi/pi
%         
% Inputs:
%   x               Interpolation basis points
%   v               Function values at basis points
%   xq              Vector with query points 
%   method          Interpolation method 
%   extrapolation   Extrapolation method
% 
%
% Outputs: 
%   vq              Vector with interpolation values at query points

vq = zeros(length(xq), 1); 
  
if(length(x) ~= length(v))
  error('Interpolation vectors are not the same size'); 
end

% next point idx (reference) 
idx_next = 2; 

for i = 1:1:length(xq)
  % iterate the next point until it is greater than the actual query point
  while(x(idx_next) < xq(i))
    idx_next = idx_next + 1; 
    if(idx_next > length(v))
      % if the end is reached, set the remaining values to zero
      vq(i:length(xq)) = 0; 
      break; 
    end
  end
  % if the distance change between two points at the path is larger than one pi, this 
  % indicates that a change from -pi to +pi or the other way round was mde 
  if((v(idx_next) - v(idx_next-1)) > pi)
    % in case the change is positive, remove 2pi from the second value
    % this leads to a local remap to an out of bounds (lower)  value. This is better for
    % interpolation. 
    vq(i) = interp1([x(idx_next-1), x(idx_next)], [v(idx_next-1), v(idx_next)-2*pi], xq(i), method, extrapolation); 
  elseif((v(idx_next) - v(idx_next-1)) < -pi)
    % in case the change is negative, add 2pi from the second value
    % this leads to a local remap to an out of bounds (upper) value. This is better for
    % interpolation. 
    vq(i) = interp1([x(idx_next-1), x(idx_next)], [v(idx_next-1), v(idx_next)+2*pi], xq(i), method, extrapolation); 
  else
    % in case none of this is true, interpolate normally
    vq(i) = interp1([x(idx_next-1), x(idx_next)], [v(idx_next-1), v(idx_next)], xq(i), method, extrapolation); 
  end
  % remap to -pi to + pi 
  vq(i) = normalizeAngle(vq(i)); 
end


