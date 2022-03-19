function  [first, second, third, hlength, id, type, port, mlength, seq, idletime, timestatus, week, ms, recstatus, reserved, version] = novatel_header(id, length, time, leapseconds)
%% Reference
% https://docs.novatel.com/OEM7/Content/Messages/Binary.htm
%
% Description:
% 	Binary message header of the Novatel GPS driver/sensor.
%
% Inputs:
%   id            Message ID number of the novatal log message [int]
%   length        The length in bytes of the body of the message, not
%                 including the header nor the CRC [int]
%   time          utc timestep [datetime]
%   leapseconds   Number of leap seconds to account for
%                 (currently 18s state:10.06.2021) [int]
%
% Outputs
%   first         First of the three sync bytes [uint8]
%   second        Second of the three sync bytes [uint8]
%   third         Thrid of the three sync bytes [uint8]
%   hlength       Length of the header in bytes [uint8]
%   type          Message Type (see documentation) [uint8]
%   port          Port Address (see documentation) [uint8]
%   seq           Sequence used for multiple related logs [uint16]
%   idletime      Time the processor is idle [uint8]
%   timestatus    Indicates the quality of the GPS reference time [uint8]
%   week          GPS reference week number [uint16]
%   ms            Milliseconds from the beginning of the GPS reference 
%                 week [uint32]
%   recstatus     32-bits representing the status of various hardware
%                 and software components of the receiver [uint32]
%   reserved      Reserved for internal use [uint32]
%   version       A value (0 - 65535) representing the receiver software
%                 build number [uint16]
%

% Define Constants
first = uint8(170);
second = uint8(68);
third = uint8(18);
hlength = uint8(28);
type = uint8(0);
port = uint8(0);
seq = uint16(0);
idletime = uint8(90);
timestatus = uint8(180);
recstatus = uint32(0);
reserved = uint16(0);
version = uint16(32768);

% Set variables
mlength = uint16(length);

% Get gps timestamp
% Error: "The 'datetime' class does not support code generation."
% [week, ms] = utc2gpstime(time, leapseconds);
% week = uint16(week);
% ms = uint32(ms);

week = uint16(1900);
ms = uint32(time);

end