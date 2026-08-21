clc
close all

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
Cd = 0.3;
A_front = 2.2;
g = 9.81;
Crr = 0.015;
Cr = Crr;

v0 = 20;
targetSpeed = 30;

%% 최종 PID 게인
Kp = 1000;
Ki = 60;
Kd = 500;

%% 시험 모델 및 질량 조건
modelName = "week7_1_mass_variation";

massList = [1200, 1500, 1800];

simulationResults = cell(size(massList));

%% 질량별 시뮬레이션
for k = 1:numel(massList)

    currentMass = massList(k);

    simInput = Simulink.SimulationInput(modelName);

    % 해당 시뮬레이션에만 질량 적용
    simInput = simInput.setVariable( ...
        "m", currentMass);

    % 모델에서 Cr을 사용하는 경우에도 질량에 맞게 갱신
    simInput = simInput.setVariable( ...
        "Cr", Crr);

    % PID 블록이 변수를 사용하는 경우를 위한 설정
    simInput = simInput.setVariable("Kp", Kp);
    simInput = simInput.setVariable("Ki", Ki);
    simInput = simInput.setVariable("Kd", Kd);

    fprintf("질량 %d kg 시뮬레이션 실행 중...\n", ...
        currentMass);

    simulationResults{k} = sim(simInput);
end

fprintf("모든 질량 조건의 시뮬레이션 완료\n");

%% 질량별 속도 결과 비교
figure("Color", "w");
hold on;

% 기준속도는 1500 kg 결과에서 가져옴
referenceData = simulationResults{2}.simData;

plot( ...
    referenceData.Time, ...
    referenceData.Data(:, 1), ...
    "k--", ...
    "LineWidth", 1.5);

colors = lines(numel(massList));

for k = 1:numel(massList)

    simData = simulationResults{k}.simData;

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
title("Vehicle Mass Variation Test");

legendLabels = [ ...
    "Reference", ...
    compose("m = %d kg", massList)];

legend(legendLabels, "Location", "best");
xlim([0 150]);

%% 질량별 기본 결과 확인
fprintf('\n=== 질량 변화 시험 결과 ===\n');

formatSpec = [ ...
    'm = %4d kg | %s | ' ...
    '30초 이후 최저속도 = %.3f m/s | ' ...
    '최고속도 = %.3f m/s | ' ...
    '최종속도 = %.3f m/s\n'];

for k = 1:numel(massList)

    simData = simulationResults{k}.simData;

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
        massList(k), ...
        status, ...
        min(v(idxAfter30)), ...
        max(v(idxAfter30)), ...
        v(end));
end

%% 질량별 KPI 정량 비교

% Data Mux 열 번호
COL_VREF     = 1;
COL_V        = 2;
COL_THROTTLE = 5;
COL_BRAKE    = 6;

numberOfCases = numel(massList);

%% 모든 결과가 공통으로 포함하는 시간 범위 확인
tStart = -inf;
tEnd = inf;

for k = 1:numberOfCases

    ts = simulationResults{k}.simData;

    tStart = max(tStart, ts.Time(1));
    tEnd = min(tEnd, ts.Time(end));
end

%% 0.01초 간격의 공통 시간축
dt = 0.01;

tStartCommon = ceil(tStart / dt) * dt;
tEndCommon = floor(tEnd / dt) * dt;

tCommon = (tStartCommon:dt:tEndCommon)';

%% 시험 구간 정의
idxStartup = tCommon >= 0 & tCommon < 30;
idxUphill = tCommon >= 30 & tCommon < 60;
idxDownhill = tCommon >= 60 & tCommon < 90;
idxReturn = tCommon >= 90 & tCommon <= 150;
idxDisturbance = tCommon >= 30;

%% KPI 저장 공간
timeTo99 = nan(numberOfCases, 1);

overallRMSE = nan(numberOfCases, 1);
disturbanceRMSE = nan(numberOfCases, 1);

uphillDrop = nan(numberOfCases, 1);
downhillRise = nan(numberOfCases, 1);
returnDrop = nan(numberOfCases, 1);

finalError = nan(numberOfCases, 1);

maxThrottle = nan(numberOfCases, 1);
maxBrake = nan(numberOfCases, 1);

alignedMassData = cell(numberOfCases, 1);

%% 질량별 계산
for k = 1:numberOfCases

    ts = simulationResults{k}.simData;

    tOriginal = ts.Time;
    yOriginal = ts.Data;

    % 동일 시각이 중복된 경우 마지막 값 사용
    [tUnique, uniqueIndex] = unique( ...
        tOriginal, "last");

    yUnique = yOriginal(uniqueIndex, :);

    % 공통 시간축으로 선형 보간
    yCommon = interp1( ...
        tUnique, ...
        yUnique, ...
        tCommon, ...
        "linear");

    alignedMassData{k} = yCommon;

     %% 신호 분리
    vReference = yCommon(:, COL_VREF);
    vVehicle = yCommon(:, COL_V);

    speedError = vReference - vVehicle;

    %% 목표속도의 99%에 처음 도달한 시간
    reachIndex = find( ...
        idxStartup & ...
        vVehicle >= 0.99 * targetSpeed, ...
        1, ...
        "first");

    if ~isempty(reachIndex)
        timeTo99(k) = tCommon(reachIndex);
    end

    %% 오차 지표
    overallRMSE(k) = sqrt( ...
        mean(speedError.^2));

    disturbanceRMSE(k) = sqrt( ...
        mean(speedError(idxDisturbance).^2));

    %% 경사 구간 최대 편차
    uphillDrop(k) = max( ...
        0, ...
        targetSpeed - min(vVehicle(idxUphill)));

    downhillRise(k) = max( ...
        0, ...
        max(vVehicle(idxDownhill)) - targetSpeed);

    returnDrop(k) = max( ...
        0, ...
        targetSpeed - min(vVehicle(idxReturn)));

    %% 최종 오차
    finalError(k) = abs(speedError(end));

    %% 외란 구간 최대 액추에이터 힘
    maxThrottle(k) = max( ...
        yCommon(idxDisturbance, COL_THROTTLE));

    maxBrake(k) = max( ...
        yCommon(idxDisturbance, COL_BRAKE));
end

%% 결과표 생성
massKpiTable = table( ...
    massList', ...
    timeTo99, ...
    overallRMSE, ...
    disturbanceRMSE, ...
    uphillDrop, ...
    downhillRise, ...
    returnDrop, ...
    finalError, ...
    maxThrottle, ...
    maxBrake, ...
    'VariableNames', { ...
    'Mass_kg', ...
    'Time_to_99pct_s', ...
    'Overall_RMSE_mps', ...
    'Disturbance_RMSE_mps', ...
    'Uphill_Drop_mps', ...
    'Downhill_Rise_mps', ...
    'Return_Drop_mps', ...
    'Final_Error_mps', ...
    'Max_Throttle_N', ...
    'Max_Brake_N'});

fprintf('\n=== 질량별 강건성 KPI ===\n');
disp(massKpiTable);

