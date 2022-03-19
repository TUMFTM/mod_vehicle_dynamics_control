function y = normalizeAngle360(u)
% Normalize angle in degrees to [0 360]
y = mod(u, 360);
end

