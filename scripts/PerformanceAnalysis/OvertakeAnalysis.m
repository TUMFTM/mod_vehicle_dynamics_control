% this script generates visualisations for overtaking situations 

% length of the straight considered
s_straight_m = 1000; 
% opponent velocities to be analyzed
vel_op_mps = 45:10:85;
% difference velocities to be analyzed
diff_speed_mps = 0:0.5:10; 

% prepare plotting
figure; grid on; hold on; box on; 
xlabel('Overtaking distance in m'); 
ylabel('Mean difference speed in mps'); 

% calculate maximum required distance for overtaking
t_overtake_s = s_straight_m./vel_op_mps;
for i = 1:1:length(t_overtake_s) 
    plot(t_overtake_s(i)*diff_speed_mps, diff_speed_mps,  ...
        'DisplayName', ['Op. vel. ' num2str(vel_op_mps(i)) ' mps']);  
end

legend('Location', 'northwest'); 