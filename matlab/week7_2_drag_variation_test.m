clc;
close all;

scriptFolder = fileparts(mfilename('fullpath'));
projectRoot  = fileparts(scriptFolder);
artifactFolder = fullfile(projectRoot, 'slprj');

Simulink.fileGenControl('set', ...
    'CacheFolder', artifactFolder, ...
    'CodeGenFolder', artifactFolder, ...
    'keepPreviousPath', false, ...
    'createDir', true);

% 설정 확인
fileGenConfig = Simulink.fileGenControl('getConfig');

fprintf('Cache 저장 위치  : %s\n', fileGenConfig.CacheFolder);
fprintf('Code 생성 위치   : %s\n', fileGenConfig.CodeGenFolder);

%% 공통 차량 파라미터
rho = 1.225;
A_front = 2.2;

m = 1500;
g = 9.81;

Crr = 0.015;
Cr = Crr;              % 구름저항계수

v0 = 20;
targetSpeed = 30;

%% 최종 PID 게인
Kp = 1000;
Ki = 60;
Kd = 500;

%% 시험 모델과 공기저항계수
modelName = "week7_2_drag_variation";

CdList = [0.24, 0.30, 0.36];

dragResults = cell(size(CdList));

%% Cd별 시뮬레이션
for k = 1:numel(CdList)

    currentCd = CdList(k);

    simInput = Simulink.SimulationInput(modelName);

    % 이번 시험에서 변경하는 변수
    simInput = simInput.setVariable( ...
        "Cd", currentCd);

    % 나머지 조건은 고정
    simInput = simInput.setVariable("m", m);
    simInput = simInput.setVariable("Cr", Crr);

    simInput = simInput.setVariable("Kp", Kp);
    simInput = simInput.setVariable("Ki", Ki);
    simInput = simInput.setVariable("Kd", Kd);

    fprintf( ...
        'Cd = %.2f 시뮬레이션 실행 중...\n', ...
        currentCd);

    dragResults{k} = sim(simInput);
end

fprintf('모든 Cd 조건의 시뮬레이션 완료\n');

%% Cd별 차량속도 비교
figure("Color", "w");
hold on;

% 기준속도
referenceData = dragResults{2}.simData;

plot( ...
    referenceData.Time, ...
    referenceData.Data(:, 1), ...
    "k--", ...
    "LineWidth", 1.5);

colors = lines(numel(CdList));

for k = 1:numel(CdList)

    simData = dragResults{k}.simData;

    plot( ...
        simData.Time, ...
        simData.Data(:, 2), ...
        "Color", colors(k, :), ...
        "LineWidth", 1.5);
end

xline(30, ":", "오르막 시작", ...
    "HandleVisibility", "off");

xline(60, ":", "내리막 시작", ...
    "HandleVisibility", "off");

xline(90, ":", "평지 복귀", ...
    "HandleVisibility", "off");

grid on;
xlabel("Time [s]");
ylabel("Vehicle Speed [m/s]");
title("Drag Coefficient Variation Test");

legendLabels = [ ...
    "Reference", ...
    compose("Cd = %.2f", CdList)];

legend(legendLabels, "Location", "best");
xlim([0 150]);

%% Cd별 기본 결과
fprintf('\n=== 공기저항계수 변화 시험 결과 ===\n');

formatSpec = [ ...
    'Cd = %.2f | %s | ' ...
    '30초 이후 최저속도 = %.3f m/s | ' ...
    '최고속도 = %.3f m/s | ' ...
    '최종속도 = %.3f m/s\n'];

for k = 1:numel(CdList)

    simData = dragResults{k}.simData;

    t = simData.Time;
    v = simData.Data(:, 2);

    idxAfter30 = t >= 30;

    if all(isfinite(v)) && all(v >= 0)
        status = '정상';
    else
        status = '확인 필요';
    end

    fprintf( ...
        formatSpec, ...
        CdList(k), ...
        status, ...
        min(v(idxAfter30)), ...
        max(v(idxAfter30)), ...
        v(end));
end

%% Cd별 성능 및 에너지 KPI

% Data Mux 열 번호
COL_VREF     = 1;
COL_V        = 2;
COL_THROTTLE = 5;
COL_BRAKE    = 6;

numberOfCases = numel(CdList);

%% 공통 시간 범위 확인
tStart = -inf;
tEnd = inf;

for k = 1:numberOfCases

    ts = dragResults{k}.simData;

    tStart = max(tStart, ts.Time(1));
    tEnd = min(tEnd, ts.Time(end));
end

%% 공통 0.01초 시간축
dt = 0.01;

tStartCommon = ceil(tStart / dt) * dt;
tEndCommon = floor(tEnd / dt) * dt;

tCommon = (tStartCommon:dt:tEndCommon)';

%% 시험 구간
idxStartup = tCommon >= 0 & tCommon < 30;
idxUphill = tCommon >= 30 & tCommon < 60;
idxDownhill = tCommon >= 60 & tCommon < 90;
idxReturn = tCommon >= 90 & tCommon <= 150;
idxDisturbance = tCommon >= 30;

% 평지 정상상태 동력 계산 구간
idxFinalFlat = tCommon >= 140 & tCommon <= 150;

%% 성능 KPI 저장 공간
timeTo99 = nan(numberOfCases, 1);
overallRMSE = nan(numberOfCases, 1);
disturbanceRMSE = nan(numberOfCases, 1);

uphillDrop = nan(numberOfCases, 1);
downhillRise = nan(numberOfCases, 1);
returnDrop = nan(numberOfCases, 1);
finalError = nan(numberOfCases, 1);

%% 액추에이터 및 에너지 KPI
maxThrottle = nan(numberOfCases, 1);
maxBrake = nan(numberOfCases, 1);

meanFinalThrottle = nan(numberOfCases, 1);
meanFinalPower_kW = nan(numberOfCases, 1);

tractionEnergy_kWh = nan(numberOfCases, 1);
brakeEnergy_kWh = nan(numberOfCases, 1);

alignedDragData = cell(numberOfCases, 1);

%% Cd별 KPI 계산
for k = 1:numberOfCases

    ts = dragResults{k}.simData;

    tOriginal = ts.Time;
    yOriginal = ts.Data;

    % 중복 시각이 있으면 마지막 값 사용
    [tUnique, uniqueIndex] = unique( ...
        tOriginal, "last");

    yUnique = yOriginal(uniqueIndex, :);

    % 공통 시간축으로 보간
    yCommon = interp1( ...
        tUnique, ...
        yUnique, ...
        tCommon, ...
        "linear");

    alignedDragData{k} = yCommon;

    %% 신호 분리
    vReference = yCommon(:, COL_VREF);
    vVehicle = yCommon(:, COL_V);

    throttleForce = yCommon(:, COL_THROTTLE);
    brakeForce = yCommon(:, COL_BRAKE);

    speedError = vReference - vVehicle;

    %% 목표속도 99% 최초 도달시간
    reachIndex = find( ...
        idxStartup & ...
        vVehicle >= 0.99 * targetSpeed, ...
        1, ...
        "first");

    if ~isempty(reachIndex)
        timeTo99(k) = tCommon(reachIndex);
    end

    %% 속도 오차 성능
    overallRMSE(k) = sqrt( ...
        mean(speedError.^2));

    disturbanceRMSE(k) = sqrt( ...
        mean(speedError(idxDisturbance).^2));

    uphillDrop(k) = max( ...
        0, ...
        targetSpeed - min(vVehicle(idxUphill)));

    downhillRise(k) = max( ...
        0, ...
        max(vVehicle(idxDownhill)) - targetSpeed);

    returnDrop(k) = max( ...
        0, ...
        targetSpeed - min(vVehicle(idxReturn)));

    finalError(k) = abs(speedError(end));

    %% 최대 액추에이터 힘
    maxThrottle(k) = max( ...
        throttleForce(idxDisturbance));

    maxBrake(k) = max( ...
        brakeForce(idxDisturbance));

    %% 평지 정상상태 평균 구동력
    meanFinalThrottle(k) = mean( ...
        throttleForce(idxFinalFlat));

    %% 기계적 동력
    % P = Fv, 단위: W
    tractionPower_W = throttleForce .* vVehicle;
    brakePower_W = brakeForce .* vVehicle;

    meanFinalPower_kW(k) = mean( ...
        tractionPower_W(idxFinalFlat)) / 1000;

    %% 150초간 기계적 에너지
    % 1 kWh = 3.6e6 J
    tractionEnergy_kWh(k) = trapz( ...
        tCommon, tractionPower_W) / 3.6e6;

    brakeEnergy_kWh(k) = trapz( ...
        tCommon, brakePower_W) / 3.6e6;
end

%% 속도 성능표
dragPerformanceTable = table( ...
    CdList', ...
    timeTo99, ...
    overallRMSE, ...
    disturbanceRMSE, ...
    uphillDrop, ...
    downhillRise, ...
    returnDrop, ...
    finalError, ...
    'VariableNames', { ...
    'Cd', ...
    'Time_to_99pct_s', ...
    'Overall_RMSE_mps', ...
    'Disturbance_RMSE_mps', ...
    'Uphill_Drop_mps', ...
    'Downhill_Rise_mps', ...
    'Return_Drop_mps', ...
    'Final_Error_mps'});

%% 액추에이터 및 에너지표
dragEnergyTable = table( ...
    CdList', ...
    maxThrottle, ...
    maxBrake, ...
    meanFinalThrottle, ...
    meanFinalPower_kW, ...
    tractionEnergy_kWh, ...
    brakeEnergy_kWh, ...
    'VariableNames', { ...
    'Cd', ...
    'Max_Throttle_N', ...
    'Max_Brake_N', ...
    'Mean_Final_Throttle_N', ...
    'Mean_Final_Power_kW', ...
    'Traction_Energy_kWh', ...
    'Brake_Energy_kWh'});

fprintf('\n=== Cd별 속도 성능 KPI ===\n');
disp(dragPerformanceTable);

fprintf('\n=== Cd별 액추에이터 및 에너지 KPI ===\n');
disp(dragEnergyTable);