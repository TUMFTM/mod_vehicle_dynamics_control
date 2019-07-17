function tcs =llh2tcsT(llh,origin)

%% taken from Charles Rino from 
% https://de.mathworks.com/matlabcentral/fileexchange/28813-gps-coordinate-transformations
% slight modification by AW to use degrees 

% Description:
% 	This function returns the x, y, and z topocentric (TCS)
%	coordinates of the point specified by llh [lat lon hgt],
%   relative to the input origin [lat lon alt].
%
% Usage:
%   tcs=llh2tcs_mks(llh,origin);
%
% Inputs:
%   llh   - 3xN Matrix of Vectors in LLH coordinates
%           where
%              llh(1,:) is latitude   (deg)
%              llh(2,:) is longitude  (deg)
%              llh(3,:) is height     (meters)
%   origin - 3xN Matrix of Vectors in LLH coordinates
%           where
%              origin(1,:) is latitude   (deg)
%              origin(2,:) is longitude  (deg)
%              origin(3,:) is height     (meters)
%
% Outputs
%   tcs   - 3XN Matrix of Vectors in TCS coordinates (meters)
%           where
%              tcs(1,:) is x
%              tcs(2,:) is y
%              tcs(3,:) is z
%

% convert coordinates 
origin_conv = zeros(3, 1); 
origin_conv(1) = origin(1)/(360/(2*pi));
origin_conv(2) = origin(2)/(360/(2*pi));
origin_conv(3) = origin(3);

% convert coordinates 
llh_conv = zeros(3, 1); 
llh_conv(1) = llh(1)/(360/(2*pi));
llh_conv(2) = llh(2)/(360/(2*pi));
llh_conv(3) = llh(3);


ecf = llh2ecfT(llh_conv);

uvw = ecf2uvwT(ecf,origin_conv);
tcs = zeros(3, 1); 
tcs = uvw2tcsT(uvw,origin_conv);
tcs = [tcs(1); tcs(2); tcs(3)]; 

end

function ecf=llh2ecfT(llh)
%
% Description:
% 		This function returns the x, y, and z earth centered fixed (ECF)
%		coordinates of the point specified by llh [lat lon hgt].  
%		Note that longitude is positive east of the Greenwich meridian.
%
% Usage:
%   ecf=llh2ecfT(llh)
%
% Inputs:
%   llh   - 3xN Matrix of Vectors
%           where
%              llh(1,:) is latitude positive (radians)
%              llh(2,:) is longitude positive East (radians)
%              llh(3,:) is height (meters)
%
% Outputs
%   ecf   - 3xN Matrix of Vectors in ECF coordinates (meters)
%           where
%              x(1,:) is Greenwich meridan; (0 lon, 0 lat)
%              y(2,:) is Easterly
%              z(3,:) is North Pole
%

if isvector(llh)
    llh = llh(:);
end

lat=llh(1,:);
lon=llh(2,:);
hgt=llh(3,:);

%  Set up WGS-84 constants.
[a,f] = EarthModel;

%  Convert lat,lon,hgt geographic coords to XYZ Earth Centered Earth Fixed coords.
%		N = a/sqrt( 1 - f*(2-f)*sin(lat)*sin(lat) )
%		X = (N + h)*cos(lat)*cos(lon)
%		Y = (N + h)*cos(lat)*sin(lon)
%		Z = ((1-f)^2 * N + h)*sin(lat)

%  Store some commonly used values.

slat = sin(lat);
N = a./sqrt(1 - f*(2-f) * slat.^2);
Nplushgtclat = (N + hgt) .* cos(lat);

x = Nplushgtclat .* cos(lon);
y = Nplushgtclat .* sin(lon);
z = ((1-f)^2 * N + hgt) .* slat;

ecf = [x; y; z];

end

function uvw = ecf2uvwT( ecf, origin )
%
% Description: 
%    This function will rotate ECF coordinates into UVW coordinates:
%    X axis (U axis) is colinear with longitude of origin
%
% Usage:
%  uvw = ecf2uvw( ecf, origin )
%
% Inputs:
%  ecf         3xN array of vectors in ECF coordinates
%
%                   x1 x2 x3 ... xN
%              ecf= y1 y2 y3 ... yN
%                   z1 z2 z3 ... zN
%
% Outputs:
%  uvw         3xN array of vectors in UVW coordinates 
%

if isvector(origin)
    origin = origin(:);
end
if isvector(ecf)
    ecf = ecf(:);
end

YAW_TYPE = 1;
DC = Get_DirCos_ForwardT (origin(2,:), YAW_TYPE);

sz  = size(origin);
sze = size(ecf);

if sz(2)==1
    uvw = DC*ecf;
elseif sze(2)==1
   uvw = zeros(3,sz(2));
   for n=1:sz(2)
       uvw(:,n)=DC(:,:,n)*ecf;
   end
elseif sz(2)==sze(2)
   uvw = zeros(3,sz(2));
   for n=1:sz(2)
       uvw(:,n)=DC(:,:,n)*ecf(:,n);
   end
else
    error('Inconsistent dimensions');
end

end

function tcs = uvw2tcsT(uvw, origin)
%
% Description:
%	This function will convert a position vector from
%	UVW to TCS coordinates relative to origin.
%
% Usage:
%  tcs = uvw2tcs ( uvw , origin)
%
% Inputs:
%  uvw      3xN array of vectors in geocentric coordinates
%
%           x1 x2 x3 ... xN
%     uvw = y1 y2 y3 ... yN
%           z1 z2 z3 ... zN
%
%     origin 3x1 or 3xN vector array (CLR August 2005)
%
% Outputs:
%  tcs      3xN array of vectors in topocentric coordinates 
%
if isvector(origin)
    origin = origin(:);
end
if isvector(uvw)
    uvw = uvw(:);
end

origin_ECF = llh2ecfT(origin);
origin_UVW = ecf2uvwT(origin_ECF,origin);

YAW_TYPE = 1;
PITCH_TYPE = 2;
ROLL_TYPE = 3;

DC1 = Get_DirCos_ForwardT (pi/2, ROLL_TYPE);
DC2 = Get_DirCos_ForwardT (pi/2, PITCH_TYPE);
DC3 = Get_DirCos_ForwardT (-origin(1,:), ROLL_TYPE);
DC21=DC2*DC1;
sz=size(DC3);
if length(sz)==2
    DC = DC3*DC21;
else
    DC=zeros(3,3,sz(3));
    for n=1:sz(3)
        DC(:,:,n)=DC3(:,:,n)*DC21;
    end
end
    
szuvw = size(uvw);
tcs = zeros(szuvw(1),szuvw(2));
tcs(1,:) = uvw(1,:) - origin_UVW(1,:);
tcs(2,:) = uvw(2,:) - origin_UVW(2,:);
tcs(3,:) = uvw(3,:) - origin_UVW(3,:);
if length(sz)==2
    tcs = DC * tcs;
else
    for n=1:sz(3)
        tcs(:,n)=DC(:,:,n)*tcs(:,n);
    end
end

end

function DC = Get_DirCos_ForwardT (A, MatrixFlavor);
%
% Description:
%	Fills a direction cosine matrix defined by positive right-hand rule Euler
%	angles that transforms from an INS type basis to a body type basis.    
%
% Usage:
%  DC = Get_DirCos_ForwardT (A, MatrixFlavor);
%
% Inputs:
%  A            - Angle in Radians
%  MatrixFlavor - Axis: ROLL_TYPE, PITCH_TYPE, YAW_TYPE
%
% Outputs:
%  DC           - Direction Cosine Matrix
%
% Modified by CLR to accept N vector of angles & return (3x3xN) rotation matrices
%

YAW_TYPE = 1;
PITCH_TYPE = 2;
ROLL_TYPE = 3;

cosA =  cos(A(:));
sinA =  sin(A(:));
DC   =  zeros(3,3,length(A));
   
switch (MatrixFlavor) 

	case YAW_TYPE,
	        DC(1,1,:) = cosA;
	        DC(1,2,:) = sinA;
	        %DC(1,3) =  0;
	        DC(2,1,:) = -sinA;
	        DC(2,2,:) = cosA;
	        %DC(2,3) =  0;
	        %DC(3,1) =  0;
	        %DC(3,2) =  0;
	        DC(3,3,:) =  1;

	case PITCH_TYPE,
           DC(1,1,:) = cosA;
           %DC(1,2) =  0;
           DC(1,3,:) = -sinA;
           %DC(2,1) =  0;
           DC(2,2,:) =  1;
           %DC(2,3) =  0;
           DC(3,1,:) = sinA;
           %DC(3,2) =  0;
           DC(3,3,:) = cosA;

	case ROLL_TYPE,
	        DC(1,1,:) =  1;
	        %DC(1,2) =  0;
	        %DC(1,3) =  0;
	        %DC(2,1) =  0;
	        DC(2,2,:) = cosA;
	        DC(2,3,:) = sinA;
	        %DC(3,1) =  0;
	        DC(3,2,:) = -sinA;
	        DC(3,3,:) = cosA;
end

end


function [a,f] = EarthModel
%
% Define the constants for the WGS-84 ellipsoidal Earth model.
%
% Outputs:
%   a - semi-major axis of the Earth ellipsoid model
%   f - flattening

a = 6378137.0; %meters
f = 1.0/298.257223563;

end