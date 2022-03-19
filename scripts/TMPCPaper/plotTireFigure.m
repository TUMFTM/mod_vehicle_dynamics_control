% this script plots the tire figure 
alpha = -0.2:0.001:0.2; 

figure; grid on; hold on; box on; 
PacParam = [40,   1, 4500,  0];
PacParam2 = [25,   40/25, 4500,  0];
FyF_linear = PacParam(1)*PacParam(2)*PacParam(3).*alpha; 
FyF_nonlinear1 = PacParam(3).*sin(PacParam(2).*atan(PacParam(1).*alpha - PacParam(4).*(PacParam(1).*alpha - atan(PacParam(1).*alpha)))); 
FyF_nonlinear2 = PacParam2(3).*sin(PacParam2(2).*atan(PacParam2(1).*alpha - PacParam2(4).*(PacParam2(1).*alpha - atan(PacParam2(1).*alpha)))); 
plot(alpha, FyF_linear); 
plot(alpha, FyF_nonlinear1); 
plot(alpha, FyF_nonlinear2); 
xlabel('Side slip angle $\alpha$ in rad'); 
ylabel('Force $F_y$ in N'); 
ylim([-10000, 10000]); 
legend('Linear', 'Pacejka 1', 'Pacejka 2'); 
matlab2tikz('TireModelComparison.tex', 'standalone', true);