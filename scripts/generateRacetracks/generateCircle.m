% generates a circle as a test track for tracking control applications
close all; 

r = 25; 
phi = 0:0.1:2*pi; 
x_m = (cos(phi) - 1)*r; 
y_m = sin(phi)*r; 
b_m = 3*ones(1, length(x_m)); 
figure; 
plot(x_m, y_m); 
axis equal; 
grid on; 

% write to csv
dlmwrite('Circle.csv', [x_m',y_m',b_m'],';'); 


