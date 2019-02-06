function [M_trans] = transCov2TrackOrientation(M, psi_track)
%__________________________________________________________________________ 
%% Documentation 
% 
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de) 
%               Madeline Wolz
% 
% Start Date:   18.02.2018
% 
% Description:  transforms covariance matrix M (in global cooridnates) 
%               to a coordinate system along the track 
% 
% Inputs:   
%   M               Covariance matrix which should be transformed
%   psi_Track_rad   Track orientation


% Transformation Matrix T based on the standard rotation matrix 
% as the covariance Matrix M also includes a third component (the orientation) 
% which shall not be transformed, the third dimension is added. 
T = [cos(psi_track+pi/2), -sin(psi_track+pi/2), 0;...
     sin(psi_track+pi/2),  cos(psi_track+pi/2), 0;... 
     0, 0, 1];

M_trans = T * M * T';

end

