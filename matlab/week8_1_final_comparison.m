clc;
close all;

%% 1. 프로젝트 경로 및 생성 파일 저장 위치
scriptPath = mfilename('fullpath');

if isempty(scriptPath)
    error('스크립트를 week8_2_final_comparison.m으로 저장하세요.');
end

matlabFolder   = fileparts(scriptPath);
projectRoot    = fileparts(matlabFolder);
simulinkFolder = fullfile(projectRoot, 'simulink');
artifactFolder = fullfile(projectRoot, 'slprj');

if ~isfolder(simulinkFolder)
    error('Simulink 폴더를 찾을 수 없습니다: %s', simulinkFolder);
end

addpath(simulinkFolder);

Simulink.fileGenControl('set', ...
    'CacheFolder', artifactFolder, ...
    'CodeGenFolder', artifactFolder, ...
    'keepPreviousPath', false, ...
    'createDir', true);

fprintf('생성 파일 저장 위치: %s\n\n', artifactFolder);


%% 2. 공통 차량 변수
rho = 1.225;       % 공기 밀도 [kg/m^3]
A = 2.2;           % 전면 투영면적 [m^2]
Crr = 0.015;       % 구름저항계수 [-]
g = 9.81;          % 중력가속도 [m/s^2]

A_front = A;
Cr = Crr;

v0 = 20;           % 초기속도 [m/s]
v_ref = 30;        % 목표속도 [m/s]


%% 3. 최종 비교 조건
% 1. 기본 제어기, 기준 차량
% 2. 튜닝 제어기, 기준 차량
% 3. 튜닝 제어기, 내리막 및 제동 최악 조건
% 4. 튜닝 제어기, 구동 및 에너지 최악 조건

caseName = [
    "Baseline Nominal"
    "Tuned Nominal"
    "Tuned Heavy-Low Cd"
    "Tuned Heavy-High Cd"
];

modelNames = [
    "week6_1_performance_logging"
    "week6_6_final_tuned_controller"
    "week6_6_final_tuned_controller"
    "week6_6_final_tuned_controller"
];

massValues = [
    1500
    1500
    1800
    1800
];

CdValues = [
    0.30
    0.30
    0.24
    0.36
];

% 각 제어기의 PID 게인
pValues = [
    500
    1000
    1000
    1000
];

iValues = [
    25
    60
    60
    60
];

dValues = [
    500
    500
    500
    500
];

numCases = numel(caseName);


%% 4. 모델 확인 및 불러오기
uniqueModels = unique(modelNames);

for k = 1:numel(uniqueModels)
    currentModel = char(uniqueModels(k));

    modelFile = fullfile( ...
        simulinkFolder, ...
        [currentModel '.slx']);

    if ~isfile(modelFile)
        error('모델을 찾을 수 없습니다: %s', modelFile);
    end

    load_system(modelFile);
end


%% 5. 최종 비교 시뮬레이션
results = struct([]);

fprintf('=== Week 8-2 최종 비교 시뮬레이션 ===\n');

for k = 1:numCases
    currentModel = char(modelNames(k));

    fprintf('\n[%d/%d] %s\n', ...
        k, numCases, caseName(k));

    fprintf('모델: %s | m = %.0f kg | Cd = %.2f\n', ...
        currentModel, massValues(k), CdValues(k));

    simIn = Simulink.SimulationInput(currentModel);

    % 차량 변수
    simIn = simIn.setVariable('m', massValues(k));
    simIn = simIn.setVariable('Cd', CdValues(k));

    % PID 블록이 p, i, d를 사용하는 경우
    simIn = simIn.setVariable('p', pValues(k));
    simIn = simIn.setVariable('i', iValues(k));
    simIn = simIn.setVariable('d', dValues(k));

    % PID 블록이 Kp, Ki, Kd를 사용하는 경우
    simIn = simIn.setVariable('Kp', pValues(k));
    simIn = simIn.setVariable('Ki', iValues(k));
    simIn = simIn.setVariable('Kd', dValues(k));

    simIn = simIn.setModelParameter('StopTime', '150');

    simOut = sim(simIn);
    loggedData = simOut.get('simData');

    if isempty(loggedData)
        error('%s에서 simData를 찾을 수 없습니다.', caseName(k));
    end

    if ~isa(loggedData, 'timeseries')
        error('%s의 simData가 timeseries가 아닙니다.', caseName(k));
    end

    % 현재 조건의 KPI 계산
    currentResult = calculateFinalKPI(loggedData);

    % 첫 번째 결과로 구조체 필드를 생성한 후 나머지 결과 저장
    if k == 1
        results = currentResult;
    else
        results(k) = currentResult;
    end

    fprintf(['상태: %s | 전체 RMSE: %.4f m/s | ' ...
             '외란 RMSE: %.4f m/s | 최종오차: %.5f m/s\n'], ...
        results(k).Status, ...
        results(k).OverallRMSE, ...
        results(k).DisturbanceRMSE, ...
        results(k).FinalError);
end


%% 6. 결과 배열 생성
status = vertcat(results.Status);

timeTo99 = vertcat(results.TimeTo99);
startupOvershoot = vertcat(results.StartupOvershoot);

overallRMSE = vertcat(results.OverallRMSE);
disturbanceRMSE = vertcat(results.DisturbanceRMSE);

overallMAE = vertcat(results.OverallMAE);
IAE = vertcat(results.IAE);

uphillDrop = vertcat(results.UphillDrop);
downhillRise = vertcat(results.DownhillRise);
returnDrop = vertcat(results.ReturnDrop);
finalError = vertcat(results.FinalError);

maxThrottle = vertcat(results.MaxThrottle);
maxBrake = vertcat(results.MaxBrake);
meanFinalThrottle = vertcat(results.MeanFinalThrottle);
meanFinalPower = vertcat(results.MeanFinalPower);

tractionEnergy = vertcat(results.TractionEnergy);
brakeEnergy = vertcat(results.BrakeEnergy);


%% 7. 최종 속도 성능 표
finalSpeedKPI = table( ...
    caseName, ...
    modelNames, ...
    massValues, ...
    CdValues, ...
    status, ...
    timeTo99, ...
    startupOvershoot, ...
    overallRMSE, ...
    disturbanceRMSE, ...
    overallMAE, ...
    IAE, ...
    uphillDrop, ...
    downhillRise, ...
    returnDrop, ...
    finalError, ...
    'VariableNames', { ...
    'Case', ...
    'Model', ...
    'Mass_kg', ...
    'Cd', ...
    'Status', ...
    'Time_to_99pct_s', ...
    'Startup_Overshoot_mps', ...
    'Overall_RMSE_mps', ...
    'Disturbance_RMSE_mps', ...
    'Overall_MAE_mps', ...
    'IAE_m', ...
    'Uphill_Drop_mps', ...
    'Downhill_Rise_mps', ...
    'Return_Drop_mps', ...
    'Final_Error_mps'});

fprintf('\n\n=== 최종 속도 성능 비교 ===\n');
disp(finalSpeedKPI);


%% 8. 액추에이터 및 에너지 표
finalEnergyKPI = table( ...
    caseName, ...
    maxThrottle, ...
    maxBrake, ...
    meanFinalThrottle, ...
    meanFinalPower, ...
    tractionEnergy, ...
    brakeEnergy, ...
    'VariableNames', { ...
    'Case', ...
    'Max_Throttle_N', ...
    'Max_Brake_N', ...
    'Mean_Final_Throttle_N', ...
    'Mean_Final_Power_kW', ...
    'Traction_Energy_kWh', ...
    'Brake_Energy_kWh'});

fprintf('\n=== 최종 액추에이터 및 에너지 비교 ===\n');
disp(finalEnergyKPI);


%% 9. 기본 제어기 대비 튜닝 제어기 개선율
metricName = [
    "Time to 99%"
    "Startup overshoot"
    "Overall RMSE"
    "Disturbance RMSE"
    "Overall MAE"
    "IAE"
    "Uphill drop"
    "Downhill rise"
    "Return drop"
    "Final error"
];

baselineValue = [
    timeTo99(1)
    startupOvershoot(1)
    overallRMSE(1)
    disturbanceRMSE(1)
    overallMAE(1)
    IAE(1)
    uphillDrop(1)
    downhillRise(1)
    returnDrop(1)
    finalError(1)
];

tunedValue = [
    timeTo99(2)
    startupOvershoot(2)
    overallRMSE(2)
    disturbanceRMSE(2)
    overallMAE(2)
    IAE(2)
    uphillDrop(2)
    downhillRise(2)
    returnDrop(2)
    finalError(2)
];

reductionPercent = ...
    (baselineValue - tunedValue) ./ baselineValue * 100;

controllerImprovement = table( ...
    metricName, ...
    baselineValue, ...
    tunedValue, ...
    reductionPercent, ...
    'VariableNames', { ...
    'Metric', ...
    'Baseline', ...
    'Tuned', ...
    'ReductionPercent'});

fprintf('\n=== 기본 제어기 대비 튜닝 제어기 개선율 ===\n');
disp(controllerImprovement);


%% 10. 최종 속도 비교 그래프
figure( ...
    'Name', 'Week 8 Final Speed Comparison', ...
    'Color', 'w');

hold on;
grid on;

plot( ...
    results(1).Time, ...
    results(1).ReferenceSpeed, ...
    'k--', ...
    'LineWidth', 1.8, ...
    'DisplayName', 'Reference');

colors = [
    0.85 0.20 0.20
    0.10 0.45 0.85
    0.95 0.55 0.10
    0.50 0.25 0.75
];

lineStyles = {'-', '-', '--', '-.'};

for k = 1:numCases
    plot( ...
        results(k).Time, ...
        results(k).VehicleSpeed, ...
        'Color', colors(k, :), ...
        'LineStyle', lineStyles{k}, ...
        'LineWidth', 1.5, ...
        'DisplayName', caseName(k));
end

xline(30, ':', 'Uphill', ...
    'HandleVisibility', 'off');

xline(60, ':', 'Downhill', ...
    'HandleVisibility', 'off');

xline(90, ':', 'Return', ...
    'HandleVisibility', 'off');

xlabel('Time [s]');
ylabel('Vehicle Speed [m/s]');
title('Final Controller and Robustness Comparison');
legend('Location', 'best');
xlim([0 150]);

ax = gca;
ax.Color = 'w';
ax.XColor = 'k';
ax.YColor = 'k';


%% 11. 기준 제어기 대비 정규화 성능 그래프
performanceMatrix = [
    overallRMSE, ...
    disturbanceRMSE, ...
    uphillDrop, ...
    downhillRise, ...
    returnDrop, ...
    finalError
];

normalizedPerformance = ...
    performanceMatrix ./ performanceMatrix(1, :) * 100;

figure( ...
    'Name', 'Normalized Final Performance', ...
    'Color', 'w');

bar(normalizedPerformance.');

grid on;

xticks(1:6);
xticklabels({ ...
    'Overall RMSE', ...
    'Disturbance RMSE', ...
    'Uphill Drop', ...
    'Downhill Rise', ...
    'Return Drop', ...
    'Final Error'});

xtickangle(20);
ylabel('Baseline-normalized performance [%]');
title('Final Performance Normalized to Baseline');
legend(caseName, 'Location', 'best');

ax = gca;
ax.Color = 'w';
ax.XColor = 'k';
ax.YColor = 'k';


%% 12. 에너지 비교 그래프
figure( ...
    'Name', 'Final Energy Comparison', ...
    'Color', 'w');

bar(1:numCases, [tractionEnergy, brakeEnergy], 'grouped');

grid on;
xticks(1:numCases);
xticklabels(caseName);
xtickangle(20);

ylabel('Energy [kWh]');
title('Final Traction and Brake Energy Comparison');
legend('Traction Energy', 'Brake Energy', ...
    'Location', 'northwest');

ax = gca;
ax.Color = 'w';
ax.XColor = 'k';
ax.YColor = 'k';


%% 13. 결과 저장
resultFile = fullfile( ...
    artifactFolder, ...
    'week8_2_final_comparison_results.mat');

save(resultFile, ...
    'finalSpeedKPI', ...
    'finalEnergyKPI', ...
    'controllerImprovement', ...
    'results');

fprintf('\n=== Week 8-2 완료 ===\n');
fprintf('결과 저장 위치: %s\n', resultFile);


%% 지역 함수: 최종 KPI 계산
function result = calculateFinalKPI(loggedData)

    t = loggedData.Time(:);
    data = squeeze(loggedData.Data);

    if size(data, 1) ~= numel(t) && ...
            size(data, 2) == numel(t)
        data = data.';
    end

    if size(data, 1) ~= numel(t)
        error('Time과 Data의 행 개수가 일치하지 않습니다.');
    end

    if size(data, 2) < 6
        error('simData에는 최소 6개의 신호가 필요합니다.');
    end

    referenceSpeed = data(:, 1);
    vehicleSpeed   = data(:, 2);
    throttleForce  = data(:, 5);
    brakeForce     = data(:, 6);

    if any(~isfinite(t)) || ...
            any(~isfinite(data(:, 1:6)), 'all')
        error('결과에 NaN 또는 Inf가 포함되어 있습니다.');
    end

    targetSpeed = referenceSpeed(end);
    speedError = referenceSpeed - vehicleSpeed;

    startupMask = t < 30;
    disturbanceMask = t >= 30;
    uphillMask = t >= 30 & t < 60;
    downhillMask = t >= 60 & t < 90;
    returnMask = t >= 90;
    finalMask = t >= t(end) - 30;

    if any(vehicleSpeed < 0)
        result.Status = "비정상: 음수 속도";
    else
        result.Status = "정상";
    end

    index99 = find( ...
        vehicleSpeed >= 0.99 * targetSpeed, ...
        1, ...
        'first');

    if isempty(index99)
        result.TimeTo99 = NaN;
    else
        result.TimeTo99 = t(index99);
    end

    result.StartupOvershoot = max( ...
        max(vehicleSpeed(startupMask) - ...
            referenceSpeed(startupMask)), ...
        0);

    totalDuration = t(end) - t(1);

    result.OverallRMSE = sqrt( ...
        trapz(t, speedError.^2) / totalDuration);

    result.OverallMAE = ...
        trapz(t, abs(speedError)) / totalDuration;

    result.IAE = trapz(t, abs(speedError));

    disturbanceTime = t(disturbanceMask);
    disturbanceError = speedError(disturbanceMask);

    result.DisturbanceRMSE = sqrt( ...
        trapz( ...
            disturbanceTime, ...
            disturbanceError.^2) / ...
        (disturbanceTime(end) - disturbanceTime(1)));

    result.UphillDrop = max( ...
        targetSpeed - min(vehicleSpeed(uphillMask)), ...
        0);

    result.DownhillRise = max( ...
        max(vehicleSpeed(downhillMask)) - targetSpeed, ...
        0);

    result.ReturnDrop = max( ...
        targetSpeed - min(vehicleSpeed(returnMask)), ...
        0);

    result.FinalError = abs( ...
        targetSpeed - vehicleSpeed(end));

    result.MaxThrottle = max( ...
        throttleForce(disturbanceMask));

    result.MaxBrake = max( ...
        brakeForce(disturbanceMask));

    finalTime = t(finalMask);
    finalThrottle = throttleForce(finalMask);

    result.MeanFinalThrottle = ...
        trapz(finalTime, finalThrottle) / ...
        (finalTime(end) - finalTime(1));

    finalPower = ...
        max(throttleForce(finalMask), 0) .* ...
        max(vehicleSpeed(finalMask), 0);

    result.MeanFinalPower = ...
        trapz(finalTime, finalPower) / ...
        (finalTime(end) - finalTime(1)) / 1000;

    tractionPower = ...
        max(throttleForce, 0) .* max(vehicleSpeed, 0);

    brakePower = ...
        max(brakeForce, 0) .* max(vehicleSpeed, 0);

    result.TractionEnergy = ...
        trapz(t, tractionPower) / 3.6e6;

    result.BrakeEnergy = ...
        trapz(t, brakePower) / 3.6e6;

    result.Time = t;
    result.ReferenceSpeed = referenceSpeed;
    result.VehicleSpeed = vehicleSpeed;
end