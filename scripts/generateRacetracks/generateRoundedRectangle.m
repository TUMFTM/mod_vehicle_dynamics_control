% generates a rounded rectangle as a test track for tracking control applications

l1 = 100; 
l2 = 20; 
lstep = 2; 
r = 12.5; 
phistep = 0.1; 
phi = phistep:phistep:(pi/2); 
b0 = 10; 

% build track snippets
trackparts{1} = [lstep:lstep:l1; zeros(1, l1/lstep)]; 
trackparts{2} = [lstep:lstep:l2; zeros(1, l2/lstep)]; 
trackparts{3} = [r*cos(phi+3/2*pi); r*sin(phi+3/2*pi)+r]; 

% order of track snippets
trackgenerator = [1, 3, 2, 3, 1, 3, 2, 3];
% rotation of track snippets
rotgenerator = [0, 0, pi/2, pi/2, pi, pi, 3*pi/2, 3*pi/2] + pi/2; 

% build up track 
x_m = 0; 
y_m = 0;
for i = 1:1:length(trackgenerator)
  R = [cos(rotgenerator(i)), -sin(rotgenerator(i));...
    sin(rotgenerator(i)), cos(rotgenerator(i))]; 
  % rotate track part to the right position 
  x_loc = R(1, :)*trackparts{trackgenerator(i)}; 
  y_loc = R(2, :)*trackparts{trackgenerator(i)};
  % transform trackpart origin to the last point
  x_m = [x_m, x_loc+x_m(end)]; 
  y_m = [y_m, y_loc+y_m(end)]; 
end
% generate width vector 
b_m = ones(1, length(x_m))*b0;

% write to csv
dlmwrite('roundedRectangle.csv', [x_m(1:end-1)',y_m(1:end-1)',b_m(1:end-1)'], ';'); 

figure; 
plot(x_m(1:end-1), y_m(1:end-1)); 
axis equal; 
grid on; 
