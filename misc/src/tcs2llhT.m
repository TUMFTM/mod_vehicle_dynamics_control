function llh =tcs2llhT(tcs,origin)

%% taken from Charles Rino from 
% https://de.mathworks.com/matlabcentral/fileexchange/28813-gps-coordinate-transformations
% slight modification by AW to use degrees instead of radians 

%%
% Description:
% 	This function returns the LLH coordinates of the point specified by 
%   x,y,z topocentric (TCS) coordinates relative to the input origin.
%
% Usage:
%   llh=tcs2llhT(tcs,origin);
%
% Inputs:
%   tcs   - Nx3 Matrix of Vectors in TCS coordinates (meters)
%           where
%              tcs(1,:) is x
%              tcs(2,:) is y
%              tcs(3,:) is z
%
% Outputs
%   llh   - Nx3 Matrix of Vectors in LLH coordinates
%           where
%              llh(1,:) is latitude   (degree)
%              llh(2,:) is longitude  (degree)
%              llh(3,:) is height     (meters)
if isvector(tcs)
    tcs = tcs(:);
end
if isvector(origin)
    origin = origin(:);
end

% convert coordinates 
origin_conv = zeros(3, 1); 
origin_conv(1) = origin(1)/(360/(2*pi));
origin_conv(2) = origin(2)/(360/(2*pi));
origin_conv(3) = origin(3);

uvw=tcs2uvwT(tcs,origin_conv);

llh = zeros(3, 1); 
llh_raw = uvw2llhT(uvw,origin_conv);
% convert back to degree
llh(1) = llh_raw(1)*(360/(2*pi));
llh(2) = llh_raw(2)*(360/(2*pi));
llh(3) = llh_raw(3);

end

function uvw = tcs2uvwT( tcs, origin)
%
% Description:
% 		This function will rotate TCS coordinates into UVW coordinates:
%       X axis (U axis) is colinear with longitude of origin.
%
% Usage:
%   uvw = tcs2uvwT(tcs, origin)
%
% Inputs:
%   tcs	3xN array of vectors
%   origin 3x1 or 3XN CLR August, 2005
%
% Outputs:
%   uvw	3xN array of vectors
%
if isvector(tcs)
    tcs = tcs(:);
end
if isvector(origin)
    origin = origin(:);
end

YAW_TYPE = 1;
PITCH_TYPE = 2;
ROLL_TYPE = 3;

origin_ECF = llh2ecfT(origin);             %origin_ECF is Nx3
origin_UVW = ecf2uvwT(origin_ECF,origin);

DC1 = Get_DirCos_ForwardT (pi/2, ROLL_TYPE);
DC2 = Get_DirCos_ForwardT (pi/2, PITCH_TYPE);
DC3 = Get_DirCos_ForwardT (-origin(1,:), ROLL_TYPE);
DC21=DC2*DC1;
sz=size(DC3);
if length(sz)==2
    DC = DC3*DC21;
    DCINV=DC^(-1);
    uvw = DCINV * tcs;
else
    DCINV=zeros(3,3,sz(3));
    uvw  =zeros(3,sz(3));
    for n=1:sz(3)
        DCINV(:,:,n)=DC3(:,:,n)*DC21;
        DCINV(:,:,n)=DCINV(:,:,n)^(-1);
        uvw(:,n) = DCINV(:,:,n) * tcs(:,n);
    end
end

uvw(1,:) = uvw(1,:) + origin_UVW(1,:);
uvw(2,:) = uvw(2,:) + origin_UVW(2,:);
uvw(3,:) = uvw(3,:) + origin_UVW(3,:);

end

% Notes:
% In the file uvw2tcs.m, we have the following transform for UVW --> TCS
%
% TCS = DC * ( UVW - origin_UVW )
%
% To get the inverse transform for TCS --> UVW,
%	DC^(-1) * TCS = DC^(-1)*DC * ( UVW - origin_UVW )
%	DC^(-1)*TCS = UVW - origin_UVW
%	UVW = DC^(-1)*TCS + origin_UVW

function llh = uvw2llhT ( uvw, origin )
%
% Description:
%   This function will convert a UVW coordinates vector to geodetic
%   LLH coordinates (Longitude, Latitude, Height)
%
%   Given a three-dimensional vector, uvw, represented in the Joint Stars
%   Geocentric system described below, uvw2llh calculates the corresponding
%   longitude, latitude (radians), and the height (meters).
%
% NOTE: This routine uses an iterative method; to compute the exact solution
%   for the spherical earth model, see the routine suvw2ecf.
%
% Usage:
%   llh=uvw2llhT(uvw,origin)
%
% Inputs:
%   origin - 3xN or 1xN vector [lat; lon; hgt] (radians,radians,meters)
%   uvw    - 3xN Vector, [x; y; z]
%
% Outputs:
%   llh   - 1xN Structure such that
%             llh.lon - longitude  (rad)
%             llh.lat - latitude   (rad)
%             llh.hgt - height above ellipsoid (meters)
%

%***********************************************************************
%* Convert Joint STARS geocentric to geodetic (radians)
%***********************************************************************
%*    Joint STARS UVW coordinate system:
%*      + W axis is coincident with ellipsoid axis of rotation.
%*      + W axis exits ellipsoid at 90 degrees north latitude
%*        (north pole).
%*      + U axis exits ellipsoid at the equator.
%*      + U axis exits ellipsoid at meridian of topocentric site center.
%*      + V axis exits ellipsoid at the equator.
%*      + V axis is 90 degrees east of U axis in longitude.
%*      + The axes form a right-handed (UVW order) cartesian system.
%*      + Values are specified in meters.
%*      + This is similar to a Universal Space Rectangular coordinate
%*        system.
%*
%*    Joint STARS generalized topocentric coordinate system XYZ:
%*      + XYZ axes from a right-handed (XYZ order) cartesian system.
%*      + Z axis is perpendicular to a plane tangent to the ellipsoid
%*        at the topocentric center point. It corresponds to the
%*        geodetic latitude vector at that point.
%*      + Y axis lies in the plane of the meridian of the topocentric
%*        site center.
%*
%*    The plane formed by the XY axes is usually taken to be tanget to
%*    to the ellipsoid at the topocentric site center.  It can be
%*    formed at any distance from the ellipsoid center along the z axis.
%*
%*    The Z axis is positive away from the spheroid.  Values are
%*    specified in meters.  This is local space rectangular coordinate
%*    system and is obtained by translation and rotation from uvw.
%*
%*    Joint STARS latitude, longitude, and elevation are defined in the
%*    usual manner with respect to the ellipsoid (i.e. geodetic).
%*    Thus, elevation is the distance above the ellisoid along the
%*    normal to a plane tangent to the ellipsoid at a point.
%*    Latitude is positive in the northern hemisphere.
%*    Longitude is positive in the eastern hemisphere.
%*
%***********************************************************************

% modified for one dimensional vector to suit code generation
llh = zeros(3, 1); 

if isvector(uvw)
    uvw = uvw(:);
end
if isvector(origin)
    origin = origin(:);
end

%  Set up WGS-84 constants.
[alpha,f] = EarthModel;

% eccentricity squared for WGS84.
ecc_sq = (2.0 - f) * f;

llhorigin.lat = origin(1);
llhorigin.lon = origin(2);
llhorigin.hgt = origin(3);
lorigin=length(llhorigin.lat);

% Radius of curvature of ellipsoid in a plane perpendicular to
% a meridian and perpendicular to a plane tangent to the surface
% The value N here is for the origin and is used as the initial
% value in the iterations in the geodetic to uvw transformation.

denom = 1.0 - ecc_sq*sin(llhorigin.lat).^2;
N = alpha./sqrt( denom );

% Compute the offset of the geodetic and geocentric centers - a magic
% number first guess.
esqNsin = ecc_sq* N.*sin(llhorigin.lat);

% Compute derivative of N with latitude as help for first guess
dNdlat = esqNsin.*cos(llhorigin.lat)./denom;

tmp1 = sqrt( uvw(1).^2 + uvw(2).^2 );

    if ( tmp1 == 0.0 )  % At North or South Pole.
        if lorigin==1
            llh(2) = llhorigin.lon;
        else
            llh(2) = llhorigin.lon(n);
        end
        if ( uvw(3) > 0.0 )
            llh(3) = uvw(3) - (alpha/sqrt(1.0 - ecc_sq));
            llh(1) = asin(1.0);
        else
            llh(3) = -uvw(3) - (alpha/sqrt(1.0 - ecc_sq));
            llh(1) = asin(-1.0);
        end

    else  % Position is NOT at the Pole.
        if lorigin==1
            llh(2) = llhorigin.lon + atan2( uvw(2), uvw(1) );

            % Take initial guess at latitude and effective radius, then iterate.
            lat = atan2( uvw(3) +esqNsin,tmp1);
            % radius of earth in meters
            re = N + dNdlat*(lat-llhorigin.lat);

            dlat = 1.0;
            % Go until roughly half meter error on surface i.e. 1e-7 * 6.38e6
            while ( dlat > 1.0e-7 )
                olatsav = lat;
                tmp2 = uvw(3) + ecc_sq*re*sin(lat);
                lat = atan2( tmp2, tmp1);
                re = alpha / sqrt(1.0 - ecc_sq * sin(lat)^2);
                dlat = abs(lat - olatsav);
            end
        else
            llh(2) = llhorigin(2) + atan2( uvw(2), uvw(1) );
            
            lat = atan2( uvw(3)+esqNsin,tmp1);
            re = N+dNdlat*(lat-llhorigin.lat);
            
            dlat = 1.0;
            % Go until roughly half meter error on surface i.e. 1e-7 * 6.38e6
            while ( dlat > 1.0e-7 )
                olatsav = lat;
                tmp2 = uvw(3) + ecc_sq*re*sin(lat);
                lat = atan2( tmp2, tmp1(n) );
                re = alpha / sqrt(1.0 - ecc_sq * sin(lat)^2);
                dlat = abs(lat - olatsav);
            end

        end

        llh(3) = tmp1 / cos(lat)  - re;		% height in meters
        llh(1) = lat;

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


function DC = Get_DirCos_ForwardT (A, MatrixFlavor)
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