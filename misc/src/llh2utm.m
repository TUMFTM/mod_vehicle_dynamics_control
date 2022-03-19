function  [x,y,zone_nbr,zone_ltr] = llh2utm(Lat,Lon)
% -------------------------------------------------------------------------
% [x,y,zone_nbr,zone_ltr] = deg2utm(Lat,Lon)
%
% Description: Function to convert lat/lon vectors into UTM coordinates (WGS84).
% Some code has been extracted from UTM.m function by Gabriel Ruiz Martinez.
%
% Inputs:
%    Lat: Latitude vector.   Degrees.  +ddd.ddddd  WGS84
%    Lon: Longitude vector.  Degrees.  +ddd.ddddd  WGS84
%
% Outputs:
%    x, y , zone_nbr, zone_ltr.   See example
%
% Example 1:
%    Lat=[40.3154333; 46.283900; 37.577833; 28.645650; 38.855550; 25.061783];
%    Lon=[-3.4857166; 7.8012333; -119.95525; -17.759533; -94.7990166; 121.640266];
%    [x,y,utmzone] = deg2utm(Lat,Lon);
%    fprintf('%7.0f ',x)
%       458731  407653  239027  230253  343898  362850
%    fprintf('%7.0f ',y)
%      4462881 5126290 4163083 3171843 4302285 2772478
%    zone_nbr = 30 32 11 28 15 51
%    zone_ltr = T T S R S R
%
% Example 2: If you have Lat/Lon coordinates in Degrees, Minutes and Seconds
%    LatDMS=[40 18 55.56; 46 17 2.04];
%    LonDMS=[-3 29  8.58;  7 48 4.44];
%    Lat=dms2deg(mat2dms(LatDMS)); %convert into degrees
%    Lon=dms2deg(mat2dms(LonDMS)); %convert into degrees
%    [x,y,utmzone] = deg2utm(Lat,Lon)
%
% Author: 
%   Rafael Palacios
%   Universidad Pontificia Comillas
%   Madrid, Spain
% Version: Aug/06
%-------------------------------------------------------------------------
% Argument checking
%
narginchk(2, 2);  %2 arguments required

% Main
sa = 6378137.000000; 
sb = 6356752.314245;

e2 = ( ( ( sa ^ 2 ) - ( sb ^ 2 ) ) ^ 0.5 ) / sb;
e2cuadrada = e2 ^ 2;
c = ( sa ^ 2 ) / sb;

lat = Lat * ( pi / 180 );
lon = Lon * ( pi / 180 );
zone_nbr = fix( ( Lon / 6 ) + 31);
S = ( ( zone_nbr * 6 ) - 183 );
deltaS = lon - ( S * ( pi / 180 ) );
if (Lat<-72), zone_ltr=67;  % C
elseif (Lat<-64), zone_ltr=68;  % D
elseif (Lat<-56), zone_ltr=69;  % E
elseif (Lat<-48), zone_ltr=70;  % F
elseif (Lat<-40), zone_ltr=71;  % G
elseif (Lat<-32), zone_ltr=72;  % H
elseif (Lat<-24), zone_ltr=74;  % J
elseif (Lat<-16), zone_ltr=75;  % K
elseif (Lat<-8), zone_ltr=76;  % L
elseif (Lat<0), zone_ltr=77;  % M
elseif (Lat<8), zone_ltr=78;  % N
elseif (Lat<16), zone_ltr=80;  % P
elseif (Lat<24), zone_ltr=81;  % Q
elseif (Lat<32), zone_ltr=82;  % R
elseif (Lat<40), zone_ltr=83;  % S
elseif (Lat<48), zone_ltr=84;  % T
elseif (Lat<56), zone_ltr=85;  % U
elseif (Lat<64), zone_ltr=86;  % V
elseif (Lat<72), zone_ltr=87;  % W
else, zone_ltr=88;  % X
end
a = cos(lat) * sin(deltaS);
epsilon = 0.5 * log( ( 1 +  a) / ( 1 - a ) );
nu = atan( tan(lat) / cos(deltaS) ) - lat;
v = ( c / ( ( 1 + ( e2cuadrada * ( cos(lat) ) ^ 2 ) ) ) ^ 0.5 ) * 0.9996;
ta = ( e2cuadrada / 2 ) * epsilon ^ 2 * ( cos(lat) ) ^ 2;
a1 = sin( 2 * lat );
a2 = a1 * ( cos(lat) ) ^ 2;
j2 = lat + ( a1 / 2 );
j4 = ( ( 3 * j2 ) + a2 ) / 4;
j6 = ( ( 5 * j4 ) + ( a2 * ( cos(lat) ) ^ 2) ) / 3;
alfa = ( 3 / 4 ) * e2cuadrada;
beta = ( 5 / 3 ) * alfa ^ 2;
gama = ( 35 / 27 ) * alfa ^ 3;
Bm = 0.9996 * c * ( lat - alfa * j2 + beta * j4 - gama * j6 );
x = epsilon * v * ( 1 + ( ta / 3 ) ) + 500000;
y = nu * v * ( 1 + ta ) + Bm;
if (y<0)
   y=9999999+y;
end

end