% this script calculates the eigenvalues of the closed loop for the lateral acceleration controller

% Assumptions: 
% Two first order low pass filter in series
% first: Steering dynamics with low pass constant Ts
% second: Vehicle dynamics with low pass constant Tv
% 
% control law: -k1*ay/v^2 -k2*delta
% with ay being the lateral acceleration (error) 
% and delta being the steering angle (error) 

Ts = 0.04; 
Tv = 0.10; 
v = 50; 
k1 = 3; 
k2 = 0.3; 

% system dynamic matrix
A = [-1/Tv, v^2/Tv; ...
    0, -1/Ts]; 
% input matrix
B = [0; 1/Ts];
% controller
K = [k1/v^2, k2]; 
A_cl = A - B*K; 

% calculate eigenvalues 
sys = ss(A_cl, B, [1, 0], 0); 
figure; 
disp('Eigenvalues of closed loop: '); 
eig(A_cl) 
pzmap(sys); grid on; axis equal; 