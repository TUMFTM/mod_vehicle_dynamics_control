% plots a basic tire model given via the vector PacParam 
function plotTireModel(PacParam, alpha_data, F_data)
  figure; 
  grid on; hold on; 
  if(nargin > 2)
    % if tire model data is available, plot it
    scatter(alpha_data, F_data, 10); 
    alphaMax = max(alpha_data); 
    alphaMin = min(alpha_data); 
  else
    alphaMax = 0.5; 
    alphaMin = -0.5;
  end
  alpha_sample = alphaMin:0.002:alphaMax; 
  FyF = PacParam(3).*sin(PacParam(2).*atan(PacParam(1).*alpha_sample - PacParam(4).*(PacParam(1).*alpha_sample - atan(PacParam(1).*alpha_sample)))); 
  scatter(alpha_sample, FyF, 'LineWidth', 2); 
  xlabel('Side slip angle in rad'); 
  ylabel('Tire force in N'); 
  ylim([-6000, 6000]); 
  hold on; 
  plot(alpha_sample, PacParam(3).*sin(PacParam(2).*atan(PacParam(1).*alpha_sample))); 
  legend('Detailed Model', 'Simple Model for Control'); 
end