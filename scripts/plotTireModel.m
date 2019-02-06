% plots a basic tire model given via the vector PacParam 
function plotTireModel(PacParam, Fz, alpha_data, F_data)
  figure; 
  grid on; hold on; 
  if(nargin > 2)
    % if tire model data is available, plot it
    scatter(alpha_data, F_data, 10); 
    alphaMax = max(alpha_data); 
    alphaMin = min(alpha_data); 
  else
    alphaMax = 0.3; 
    alphaMin = -0.3;
  end
  Fz_nom = 3000;
  alpha_sample = alphaMin:0.002:alphaMax; 
  %FyF = Fz./Fz_nom.*PacParam(3).*sin(PacParam(2).*atan(PacParam(1).*alpha_sample - PacParam(4).*(PacParam(1).*alpha_sample - atan(PacParam(1).*alpha_sample)))); 
  FyF = PacParam(3).*sin(PacParam(2).*atan(PacParam(1).*alpha_sample)); 
    scatter(alpha_sample, FyF, 'LineWidth', 2); 
  xlabel('Side slip angle in rad'); 
  ylabel('Tire force in N'); 
  ylim([-6000, 6000]); 
end