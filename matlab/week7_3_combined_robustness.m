clc;
close all;

%% 1. 프로젝트 및 Simulink 생성 파일 경로 설정
% 이 스크립트가 다음 위치에 있다고 가정:
% vehicle-cruise-control\matlab\week7_3_combined_robustness.m

scriptPath = mfilename('fullpath');

if isempty(scriptPath)
    error(['스크립트를 week7_3_combined_robustness.m으로 저장한 뒤 ' ...
           '다시 실행하세요.']);
end

matlabFolder   = fileparts(scriptPath);
projectRoot    = fileparts(matlabFolder);
simulinkFolder = fullfile(projectRoot, 'simulink');
artifactFolder = fullfile(projectRoot, 'slprj');

if ~isfolder(simulinkFolder)
    error('Simulink 폴더를 찾을 수 없습니다: %s', simulinkFolder);
end

% 모델을 찾을 수 있도록 경로 추가
addpath(simulinkFolder);

% slprj 및 slxc 등 생성 파일 위치 지정
Simulink.fileGenControl('set', ...
    'CacheFolder', artifactFolder, ...
    'CodeGenFolder', artifactFolder, ...
    'keepPreviousPath', false, ...
    'createDir', true);

fileGenConfig = Simulink.fileGenControl('getConfig');

fprintf('=== Simulink 생성 파일 저장 위치 ===\n');
fprintf('Cache 저장 위치 : %s\n', fileGenConfig.CacheFolder);
fprintf('Code 생성 위치  : %s\n\n', fileGenConfig.CodeGenFolder);


%% 2. 차량 및 제어기 기본 변수
% 차량 변수
m     = 1500;      % 기준 질량 [kg]
rho   = 1.225;     % 공기 밀도 [kg/m^3]
Cd    = 0.30;      % 기준 공기저항계수 [-]
A     = 2.2;       % 전면 투영면적 [m^2]
Crr   = 0.015;     % 구름저항계수 [-]
g     = 9.81;      % 중력가속도 [m/s^2]

% 모델에서 다른 변수명을 사용하는 경우를 위한 동일 변수
A_front = A;
Cr      = Crr;

% 속도 조건
v0     = 20;       % 초기속도 [m/s]
v_ref  = 30;       % 목표속도 [m/s]

% 튜닝된 PID 게인
p = 1000;
i = 60;
d = 500;

Kp = p;
Ki = i;
Kd = d;


%% 3. Simulink 모델 설정
modelName = 'week6_6_final_tuned_controller';
modelFile = fullfile(simulinkFolder, modelName + ".slx");

if ~isfile(modelFile)
    error('Simulink 모델을 찾을 수 없습니다: %s', modelFile);
end

load_system(modelFile);


%% 4. 질량-Cd 복합 시험 조건
% Nominal 조건과 네 가지 경계 조건을 시험한다.

caseName = [
    "Nominal"
    "Light-Low Cd"
    "Light-High Cd"
    "Heavy-Low Cd"
    "Heavy-High Cd"
];

massValues = [
    1500
    1200
    1200
    1800
    1800
];

CdValues = [
    0.30
    0.24
    0.36
    0.24
    0.36
];

numCases = numel(caseName);


%% 5. 결과 저장 공간
status                    = strings(numCases, 1);
timeTo99                  = nan(numCases, 1);
overallRMSE               = nan(numCases, 1);
disturbanceRMSE           = nan(numCases, 1);
uphillDrop                = nan(numCases, 1);
downhillRise              = nan(numCases, 1);
returnDrop                = nan(numCases, 1);
finalError                = nan(numCases, 1);
maxThrottle               = nan(numCases, 1);
maxBrake                  = nan(numCases, 1);
meanFinalThrottle         = nan(numCases, 1);
meanFinalPower            = nan(numCases, 1);
tractionEnergy            = nan(numCases, 1);
brakeEnergy               = nan(numCases, 1);

timeData      = cell(numCases, 1);
referenceData = cell(numCases, 1);
speedData     = cell(numCases, 1);


%% 6. 복합조건 시뮬레이션
fprintf('=== 질량-Cd 복합 강건성 시험 시작 ===\n');

for k = 1:numCases
    fprintf('\n[%d/%d] %s: m = %.0f kg, Cd = %.2f\n', ...
        k, numCases, caseName(k), massValues(k), CdValues(k));

    % 각 시험 조건을 Simulink 모델에 전달
    simIn = Simulink.SimulationInput(modelName);

    simIn = simIn.setVariable('m', massValues(k));
    simIn = simIn.setVariable('Cd', CdValues(k));

    % 시뮬레이션 종료시간
    simIn = simIn.setModelParameter('StopTime', '150');

    % 시뮬레이션 실행
    simOut = sim(simIn);

    % To Workspace 블록의 변수 이름이 simData라고 가정
    loggedData = simOut.get('simData');

    if isempty(loggedData)
        error(['%s 조건에서 simData를 찾을 수 없습니다. ' ...
               'To Workspace 블록의 변수 이름을 확인하세요.'], ...
               caseName(k));
    end

    if ~isa(loggedData, 'timeseries')
        error('%s 조건의 simData가 timeseries 형식이 아닙니다.', ...
            caseName(k));
    end

    % KPI 계산
    result = calculateRobustnessKPI(loggedData);

    status(k)            = result.Status;
    timeTo99(k)          = result.TimeTo99;
    overallRMSE(k)       = result.OverallRMSE;
    disturbanceRMSE(k)   = result.DisturbanceRMSE;
    uphillDrop(k)        = result.UphillDrop;
    downhillRise(k)      = result.DownhillRise;
    returnDrop(k)        = result.ReturnDrop;
    finalError(k)        = result.FinalError;
    maxThrottle(k)       = result.MaxThrottle;
    maxBrake(k)          = result.MaxBrake;
    meanFinalThrottle(k) = result.MeanFinalThrottle;
    meanFinalPower(k)    = result.MeanFinalPower;
    tractionEnergy(k)    = result.TractionEnergy;
    brakeEnergy(k)       = result.BrakeEnergy;

    timeData{k}      = result.Time;
    referenceData{k} = result.ReferenceSpeed;
    speedData{k}     = result.VehicleSpeed;

    fprintf(['상태: %s | 최저속도: %.3f m/s | ' ...
             '최고속도: %.3f m/s | 최종속도: %.3f m/s\n'], ...
             result.Status, ...
             min(result.VehicleSpeed(result.Time >= 30)), ...
             max(result.VehicleSpeed(result.Time >= 30)), ...
             result.VehicleSpeed(end));
end


%% 7. 속도 성능 KPI 표
speedKPI = table( ...
    caseName, ...
    massValues, ...
    CdValues, ...
    status, ...
    timeTo99, ...
    overallRMSE, ...
    disturbanceRMSE, ...
    uphillDrop, ...
    downhillRise, ...
    returnDrop, ...
    finalError, ...
    'VariableNames', { ...
    'Case', ...
    'Mass_kg', ...
    'Cd', ...
    'Status', ...
    'Time_to_99pct_s', ...
    'Overall_RMSE_mps', ...
    'Disturbance_RMSE_mps', ...
    'Uphill_Drop_mps', ...
    'Downhill_Rise_mps', ...
    'Return_Drop_mps', ...
    'Final_Error_mps'});

fprintf('\n\n=== 질량-Cd 복합 속도 성능 KPI ===\n');
disp(speedKPI);


%% 8. 액추에이터 및 에너지 KPI 표
energyKPI = table( ...
    caseName, ...
    massValues, ...
    CdValues, ...
    maxThrottle, ...
    maxBrake, ...
    meanFinalThrottle, ...
    meanFinalPower, ...
    tractionEnergy, ...
    brakeEnergy, ...
    'VariableNames', { ...
    'Case', ...
    'Mass_kg', ...
    'Cd', ...
    'Max_Throttle_N', ...
    'Max_Brake_N', ...
    'Mean_Final_Throttle_N', ...
    'Mean_Final_Power_kW', ...
    'Traction_Energy_kWh', ...
    'Brake_Energy_kWh'});

fprintf('\n=== 질량-Cd 복합 액추에이터 및 에너지 KPI ===\n');
disp(energyKPI);


%% 9. 속도 그래프
figure('Name', 'Week 7-3 Combined Robustness', ...
       'Color', 'w');

hold on;
grid on;

colors = lines(numCases);

% 기준속도
plot(timeData{1}, referenceData{1}, ...
    'k--', ...
    'LineWidth', 1.8, ...
    'DisplayName', 'Reference');

% 각 복합조건 속도
for k = 1:numCases
    plot(timeData{k}, speedData{k}, ...
        'Color', colors(k, :), ...
        'LineWidth', 1.4, ...
        'DisplayName', caseName(k));
end

% 경사 변화 시점
xline(30, ':', '오르막 시작', ...
    'HandleVisibility', 'off');

xline(60, ':', '내리막 시작', ...
    'HandleVisibility', 'off');

xline(90, ':', '평지 복귀', ...
    'HandleVisibility', 'off');

xlabel('Time [s]');
ylabel('Vehicle Speed [m/s]');
title('Mass-C_d Combined Robustness Test');
legend('Location', 'best');
xlim([0 150]);


%% 10. RMSE 비교 그래프
figure('Name', 'Combined RMSE Comparison', ...
       'Color', 'w');

x = 1:numCases;

bar(x, [overallRMSE, disturbanceRMSE], 'grouped');

grid on;
xticks(x);
xticklabels(caseName);
xtickangle(20);

ylabel('RMSE [m/s]');
title('Mass-C_d Combined RMSE Comparison');
legend('Overall RMSE', 'Disturbance RMSE', ...
    'Location', 'best');


%% 11. 액추에이터 비교 그래프
figure('Name', 'Combined Actuator Comparison', ...
       'Color', 'w');

bar(x, [maxThrottle, maxBrake], 'grouped');

grid on;
xticks(x);
xticklabels(caseName);
xtickangle(20);

ylabel('Force [N]');
title('Maximum Actuator Force');
legend('Maximum Throttle', 'Maximum Brake', ...
    'Location', 'best');


%% 12. 에너지 비교 그래프
figure('Name', 'Combined Energy Comparison', ...
       'Color', 'w');

bar(x, [tractionEnergy, brakeEnergy], 'grouped');

grid on;
xticks(x);
xticklabels(caseName);
xtickangle(20);

ylabel('Energy [kWh]');
title('Mass-C_d Combined Energy Comparison');
legend('Traction Energy', 'Brake Energy', ...
    'Location', 'best');


%% 13. 결과 저장
resultFile = fullfile( ...
    artifactFolder, ...
    'week7_3_combined_robustness_results.mat');

save(resultFile, ...
    'speedKPI', ...
    'energyKPI', ...
    'timeData', ...
    'referenceData', ...
    'speedData');

fprintf('\n=== 시험 완료 ===\n');
fprintf('결과 저장 위치: %s\n', resultFile);


%% 지역 함수: KPI 계산
function result = calculateRobustnessKPI(loggedData)

    t = loggedData.Time(:);
    data = squeeze(loggedData.Data);

    % 데이터 방향 확인
    if size(data, 1) ~= numel(t) && size(data, 2) == numel(t)
        data = data.';
    end

    if size(data, 1) ~= numel(t)
        error('Time과 Data의 행 개수가 일치하지 않습니다.');
    end

    if size(data, 2) < 6
        error(['simData에는 최소 6개의 열이 필요합니다. ' ...
               '현재 열 개수: %d'], size(data, 2));
    end

    % Mux 신호 순서
    referenceSpeed = data(:, 1);
    vehicleSpeed   = data(:, 2);
    throttleForce = data(:, 5);
    brakeForce    = data(:, 6);

    if any(~isfinite(t)) || any(~isfinite(data(:, 1:6)), 'all')
        error('시뮬레이션 결과에 NaN 또는 Inf가 포함되어 있습니다.');
    end

    targetSpeed = referenceSpeed(end);
    speedError  = referenceSpeed - vehicleSpeed;

    % 시간 구간
    disturbanceMask = t >= 30;
    uphillMask      = t >= 30 & t < 60;
    downhillMask    = t >= 60 & t < 90;
    returnMask      = t >= 90;

    % 최종 30초 평균
    finalMask = t >= (t(end) - 30);

    if ~any(uphillMask) || ~any(downhillMask) || ~any(returnMask)
        error('경사 시험 구간을 찾을 수 없습니다.');
    end

    % 정상 여부
    if any(vehicleSpeed < 0)
        result.Status = "비정상: 음수 속도";
    else
        result.Status = "정상";
    end

    % 99% 목표속도 최초 도달시간
    index99 = find(vehicleSpeed >= 0.99 * targetSpeed, 1, 'first');

    if isempty(index99)
        result.TimeTo99 = NaN;
    else
        result.TimeTo99 = t(index99);
    end

    % 속도 KPI
    result.OverallRMSE = sqrt(mean(speedError.^2));

    result.DisturbanceRMSE = sqrt( ...
        mean(speedError(disturbanceMask).^2));

    result.UphillDrop = max( ...
        targetSpeed - min(vehicleSpeed(uphillMask)), 0);

    result.DownhillRise = max( ...
        max(vehicleSpeed(downhillMask)) - targetSpeed, 0);

    result.ReturnDrop = max( ...
        targetSpeed - min(vehicleSpeed(returnMask)), 0);

    result.FinalError = abs( ...
        targetSpeed - vehicleSpeed(end));

    % 액추에이터 KPI
    result.MaxThrottle = max( ...
        throttleForce(disturbanceMask));

    result.MaxBrake = max( ...
        brakeForce(disturbanceMask));

    result.MeanFinalThrottle = mean( ...
        throttleForce(finalMask));

    % 최종 평지 평균 구동 출력
    finalPower = ...
        max(throttleForce(finalMask), 0) .* ...
        max(vehicleSpeed(finalMask), 0);

    result.MeanFinalPower = mean(finalPower) / 1000;

    % 전체 시뮬레이션 에너지
    tractionPower = ...
        max(throttleForce, 0) .* max(vehicleSpeed, 0);

    brakePower = ...
        max(brakeForce, 0) .* max(vehicleSpeed, 0);

    result.TractionEnergy = ...
        trapz(t, tractionPower) / 3.6e6;

    result.BrakeEnergy = ...
        trapz(t, brakePower) / 3.6e6;

    % 그래프 저장용 데이터
    result.Time           = t;
    result.ReferenceSpeed = referenceSpeed;
    result.VehicleSpeed   = vehicleSpeed;
end