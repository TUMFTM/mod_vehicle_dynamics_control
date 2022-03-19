function [week, ms] = utc2gpstime(utc, leapseconds)
%% Converts a given utc datetime into gps time format
%
% Inputs:
%   utc           utc timestep [datetime]
%   leapseconds   number of leap seconds to account for
%                 (currently 18s state:10.06.2021) [int]
% Outputs
%   week          Number of weeks since 06.01.1980 [int]
%   ms            Number of milliseconds since the beginning 
%                 of the gps week [int]
%

% Set gps start time (06.01.1980 00:00:00)
Jan_6_1980 = datetime(1980, 1, 6, 0, 0, 0, 0, 'TimeZone','Z');

% Convert utc to gps time
gps = utc - Jan_6_1980 + seconds(leapseconds);

% Convert gps time to gps time format
week = floor(days(gps) / 7);
ms = milliseconds(gps - days(week * 7));
end