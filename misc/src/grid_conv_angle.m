function gamma = grid_conv_angle(lat, lon)
% Calculates the grid convergance angle according to
% https://gis.stackexchange.com/questions/115531/calculating-grid-convergence-true-north-to-grid-north
%
% Reference:
% https://de.mathworks.com/matlabcentral/fileexchange/28813-gps-coordinate-transformations
%
%
% Description:
% 	Returns the grid convergance angle (meridian convergence angle) of the
%   spherical UTM projection (grid north) to true north.
%
% Usage:
%   gamma = grid_con_angle(lat, lon);
%
% Inputs:
%   lat    Latitude vector [deg]  +ddd.ddddd  WGS84
%   lon    Longitude vector [deg]  +ddd.ddddd  WGS84
%
% Outputs
%   gamma  Grid convergance angle [deg]
%

% longitude of UTM zone's central meridian
lon0 = floor(lon./6).*6+3;

% Grid convergance angle
gamma = rad2deg(atan(tan(deg2rad(lon - lon0)) * sin(deg2rad(lat))));
end