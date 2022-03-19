% generates a circle as a test track for tracking control applications
close all; 

r = 50; 
phi = 0:0.05:2*pi; 
x_m = (cos(phi) - 1)*r; 
y_m = sin(phi)*r; 
s_m = [0, cumsum(sqrt(diff(x_m).^2+diff(y_m).^2))];
kappa_radpm = ones(length(x_m), 1)/r;
ax = zeros(length(x_m), 1); 
vx = ones(length(x_m), 1)*20;
banking = zeros(length(x_m), 1); 
figure; 
plot(x_m, y_m); 
axis equal; 
grid on; 

% write to csv
dlmwrite('Circle.csv', [s_m', x_m', y_m', phi', kappa_radpm, vx, ax, banking],';'); 


