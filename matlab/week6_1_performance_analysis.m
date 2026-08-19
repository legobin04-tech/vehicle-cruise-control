clc
close all

%% 1. Run Simulink model and load data
modelName = "week6_6_final_tuned_controller";

out = sim(modelName);
simData = out.simData;

%% 2. Validate logged data
assert(isa(simData, "timeseries"), ...
    "simData must be a timeseries object.");

assert(size(simData.Data, 2) == 6, ...
    "simData must contain exactly 6 signals.");

%% 3. Extract signals
t          = simData.Time;
v_ref      = simData.Data(:, 1);
v          = simData.Data(:, 2);
e          = simData.Data(:, 3);
theta_deg  = simData.Data(:, 4);
F_throttle = simData.Data(:, 5);
F_brake    = simData.Data(:, 6);

%% 4. Check physical consistency
tolerance = 1e-6;

assert(max(abs(e - (v_ref - v))) < tolerance, ...
    "The logged error does not equal v_ref - v.");

assert(all(F_throttle >= -tolerance), ...
    "F_throttle contains a negative value.");

assert(all(F_brake >= -tolerance), ...
    "F_brake contains a negative value.");

assert(all((F_throttle <= tolerance) | ...
           (F_brake <= tolerance)), ...
    "Throttle and brake are active simultaneously.");

disp("Data validation passed.");

%% 5. Define driving segments
idx_uphill   = (t >= 30) & (t < 60);
idx_downhill = (t >= 60) & (t < 90);
idx_return   = (t >= 90);
idx_disturb  = (t >= 30);

%% 6. Calculate performance metrics

% Speed extrema for each road condition
min_speed_uphill   = min(v(idx_uphill));
max_speed_downhill = max(v(idx_downhill));
min_speed_return   = min(v(idx_return));

% Time-weighted RMSE after the first grade disturbance
t_disturb = t(idx_disturb);
e_disturb = e(idx_disturb);

rmse_disturbance = sqrt( ...
    trapz(t_disturb, e_disturb.^2) / ...
    (t_disturb(end) - t_disturb(1)) );

% Maximum actuator forces after 30 seconds
max_throttle = max(F_throttle(idx_disturb));
max_brake    = max(F_brake(idx_disturb));

% Final values
final_speed = v(end);
final_error = e(end);

%% 7. Display results as a table
performanceMetrics = table( ...
    min_speed_uphill, ...
    max_speed_downhill, ...
    min_speed_return, ...
    rmse_disturbance, ...
    max_throttle, ...
    max_brake, ...
    final_speed, ...
    final_error);

performanceMetrics.Properties.VariableNames = { ...
    'MinSpeedUphill', ...
    'MaxSpeedDownhill', ...
    'MinSpeedReturn', ...
    'RMSE_Disturbance', ...
    'MaxThrottle', ...
    'MaxBrake', ...
    'FinalSpeed', ...
    'FinalError'};

disp(performanceMetrics);
%% 8. Plot performance results
figure( ...
    "Name", "Cruise Control Performance", ...
    "Color", "w");

layout = tiledlayout(3, 1, ...
    "TileSpacing", "compact", ...
    "Padding", "compact");

% Speed response
ax1 = nexttile;
plot(t, v_ref, "--", "LineWidth", 1.5);
hold on;
plot(t, v, "LineWidth", 1.8);
grid on;
ylabel("Speed (m/s)");
legend("V_{ref}", "V", "Location", "best");
title("Vehicle Speed Response");

% Road grade
ax2 = nexttile;
stairs(t, theta_deg, "LineWidth", 1.8);
grid on;
ylabel("Grade (deg)");
title("Road Grade Profile");

% Actuator forces
ax3 = nexttile;
plot(t, F_throttle, "LineWidth", 1.8);
hold on;
plot(t, F_brake, "LineWidth", 1.8);
grid on;
ylabel("Force (N)");
xlabel("Time (s)");
legend("Throttle", "Brake", "Location", "best");
title("Throttle and Brake Forces");

% Mark road-grade transitions
axes_list = [ax1, ax2, ax3];

for k = 1:numel(axes_list)
    xline(axes_list(k), 30, 'k--', 'HandleVisibility', 'off');
    xline(axes_list(k), 60, 'k--', 'HandleVisibility', 'off');
    xline(axes_list(k), 90, 'k--', 'HandleVisibility', 'off');
    xlim(axes_list(k), [0 150]);
end

linkaxes(axes_list, "x");
title(layout, "Cruise Control Grade-Disturbance Performance");

writetable(performanceMetrics, ...
    "week6_6_baseline_metrics.csv");

save("week6_6_baseline_results.mat", ...
    "performanceMetrics", ...
    "t", "v_ref", "v", "e", ...
    "theta_deg", "F_throttle", "F_brake");

exportgraphics(gcf, ...
    "week6_6_baseline_response.png", ...
    "Resolution", 300);