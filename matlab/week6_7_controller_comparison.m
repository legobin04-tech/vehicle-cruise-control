clc;
close all;

targetSpeed = 30;
   
%% 비교할 Simulink 모델
baseModel  = "week6_1_performance_logging";
tunedModel = "week6_6_final_tuned_controller";

%% 기본 제어기
Kp = 500;
Ki = 25;
Kd = 500;

outBase = sim(baseModel);

%% 튜닝 제어기
Kp = 1000;
Ki = 60;
Kd = 500;

outTuned = sim(tunedModel);

%% To Workspace 데이터 가져오기
baseData  = outBase.simData;
tunedData = outTuned.simData;

%% 저장 형식과 크기 확인
fprintf("=== 기본 제어기 ===\n");
fprintf("데이터 형식: %s\n", class(baseData));
fprintf("데이터 크기: %s\n\n", mat2str(size(baseData)));

fprintf("=== 튜닝 제어기 ===\n");
fprintf("데이터 형식: %s\n", class(tunedData));
fprintf("데이터 크기: %s\n\n", mat2str(size(tunedData)));

%% 데이터 내부 구조 확인
if isa(baseData, "timeseries")

    fprintf("기본 제어기 Time 크기: %s\n", ...
        mat2str(size(baseData.Time)));
    fprintf("기본 제어기 Data 크기: %s\n", ...
        mat2str(size(baseData.Data)));

    fprintf("튜닝 제어기 Time 크기: %s\n", ...
        mat2str(size(tunedData.Time)));
    fprintf("튜닝 제어기 Data 크기: %s\n", ...
        mat2str(size(tunedData.Data)));

elseif isstruct(baseData)

    fprintf("기본 제어기 구조체 필드:\n");
    disp(fieldnames(baseData));

    fprintf("튜닝 제어기 구조체 필드:\n");
    disp(fieldnames(tunedData));

    if isfield(baseData, "signals")
        fprintf("기본 signals.values 크기: %s\n", ...
            mat2str(size(baseData.signals.values)));
        fprintf("튜닝 signals.values 크기: %s\n", ...
            mat2str(size(tunedData.signals.values)));
    end

elseif isnumeric(baseData)

    fprintf("기본 제어기 첫 행과 마지막 행:\n");
    disp(baseData([1 end], :));

    fprintf("튜닝 제어기 첫 행과 마지막 행:\n");
    disp(tunedData([1 end], :));

end

%% 신호 열 번호
COL_VREF     = 1;
COL_V        = 2;
COL_ERROR    = 3;
COL_GRADE    = 4;
COL_THROTTLE = 5;
COL_BRAKE    = 6;

%% 원본 데이터
tBase = baseData.Time;
yBase = baseData.Data;

tTuned = tunedData.Time;
yTuned = tunedData.Data;

%% 동일 시각이 중복 저장된 경우 마지막 값만 사용
[tBaseUnique, idxBase] = unique(tBase, "last");
yBaseUnique = yBase(idxBase, :);

[tTunedUnique, idxTuned] = unique(tTuned, "last");
yTunedUnique = yTuned(idxTuned, :);

%% 두 모델이 공통으로 포함하는 시간 구간
tStart = max(tBaseUnique(1), tTunedUnique(1));
tEnd   = min(tBaseUnique(end), tTunedUnique(end));

%% 0.01초 간격의 공통 시간축
dt = 0.01;
tCommon = (tStart:dt:tEnd)';

%% 연속적인 신호는 선형 보간
baseCommon = interp1( ...
    tBaseUnique, yBaseUnique, ...
    tCommon, "linear");

tunedCommon = interp1( ...
    tTunedUnique, yTunedUnique, ...
    tCommon, "linear");

%% 경사각은 계단 입력이므로 이전 값 유지 방식으로 보간
baseCommon(:, COL_GRADE) = interp1( ...
    tBaseUnique, yBaseUnique(:, COL_GRADE), ...
    tCommon, "previous");

tunedCommon(:, COL_GRADE) = interp1( ...
    tTunedUnique, yTunedUnique(:, COL_GRADE), ...
    tCommon, "previous");

%% 공통 시간축 변환 결과 확인
fprintf("\n=== 공통 시간축 변환 결과 ===\n");
fprintf("시간 범위: %.2f ~ %.2f s\n", tCommon(1), tCommon(end));
fprintf("시간 간격: %.3f s\n", dt);
fprintf("데이터 크기: %s\n", mat2str(size(baseCommon)));

%% 기본 제어기와 튜닝 제어기 속도 비교
figure("Color", "w");

plot(tCommon, ...
    baseCommon(:, COL_VREF), ...
    "k--", "LineWidth", 1.5);
hold on;

plot(tCommon, ...
    baseCommon(:, COL_V), ...
    "b", "LineWidth", 1.5);

plot(tCommon, ...
    tunedCommon(:, COL_V), ...
    "r", "LineWidth", 1.5);

xline(30, ":", "오르막 시작");
xline(60, ":", "내리막 시작");
xline(90, ":", "평지 복귀");

grid on;
xlabel("Time [s]");
ylabel("Vehicle Speed [m/s]");
title("Baseline vs Tuned Controller");
legend( ...
    "Reference", ...
    "Baseline", ...
    "Tuned", ...
    "Location", "best");

xlim([0 150]);

%% KPI 계산을 위한 신호 분리
vRef = baseCommon(:, COL_VREF);

vBase  = baseCommon(:, COL_V);
vTuned = tunedCommon(:, COL_V);

eBase  = vRef - vBase;
eTuned = vRef - vTuned;

%% 시험 구간 정의
idxStartup = tCommon >= 0  & tCommon < 30;
idxUphill  = tCommon >= 30 & tCommon < 60;
idxDownhill = tCommon >= 60 & tCommon < 90;
idxReturn  = tCommon >= 90 & tCommon <= 150;

% 초기 4000 N 포화를 제외하고 외란 대응 제어력을 평가
idxDisturbance = tCommon >= 30;

%% 전체 구간 오차 성능
rmseBase = sqrt(mean(eBase.^2));
rmseTuned = sqrt(mean(eTuned.^2));

maeBase = mean(abs(eBase));
maeTuned = mean(abs(eTuned));

iaeBase = trapz(tCommon, abs(eBase));
iaeTuned = trapz(tCommon, abs(eTuned));

%% 경사 외란 구간 RMSE
disturbanceRmseBase = sqrt( ...
    mean(eBase(idxDisturbance).^2));

disturbanceRmseTuned = sqrt( ...
    mean(eTuned(idxDisturbance).^2));

%% 구간별 최대 속도 편차
startupOvershootBase = max( ...
    0, max(vBase(idxStartup)) - targetSpeed);

startupOvershootTuned = max( ...
    0, max(vTuned(idxStartup)) - targetSpeed);

uphillDropBase = max( ...
    0, targetSpeed - min(vBase(idxUphill)));

uphillDropTuned = max( ...
    0, targetSpeed - min(vTuned(idxUphill)));

downhillRiseBase = max( ...
    0, max(vBase(idxDownhill)) - targetSpeed);

downhillRiseTuned = max( ...
    0, max(vTuned(idxDownhill)) - targetSpeed);

returnDropBase = max( ...
    0, targetSpeed - min(vBase(idxReturn)));

returnDropTuned = max( ...
    0, targetSpeed - min(vTuned(idxReturn)));

%% 최종 정상상태 오차
finalErrorBase = abs(eBase(end));
finalErrorTuned = abs(eTuned(end));

%% 외란 구간 최대 스로틀과 제동력
maxThrottleBase = max( ...
    baseCommon(idxDisturbance, COL_THROTTLE));

maxThrottleTuned = max( ...
    tunedCommon(idxDisturbance, COL_THROTTLE));

maxBrakeBase = max( ...
    baseCommon(idxDisturbance, COL_BRAKE));

maxBrakeTuned = max( ...
    tunedCommon(idxDisturbance, COL_BRAKE));

%% 비교할 KPI 배열
metricNames = [
    "Overall RMSE [m/s]"
    "Disturbance RMSE [m/s]"
    "MAE [m/s]"
    "IAE [m]"
    "Startup overshoot [m/s]"
    "Uphill maximum drop [m/s]"
    "Downhill maximum rise [m/s]"
    "Return maximum drop [m/s]"
    "Final absolute error [m/s]"
    "Maximum throttle after 30 s [N]"
    "Maximum brake after 30 s [N]"
];

baselineMetrics = [
    rmseBase
    disturbanceRmseBase
    maeBase
    iaeBase
    startupOvershootBase
    uphillDropBase
    downhillRiseBase
    returnDropBase
    finalErrorBase
    maxThrottleBase
    maxBrakeBase
];

tunedMetrics = [
    rmseTuned
    disturbanceRmseTuned
    maeTuned
    iaeTuned
    startupOvershootTuned
    uphillDropTuned
    downhillRiseTuned
    returnDropTuned
    finalErrorTuned
    maxThrottleTuned
    maxBrakeTuned
];

%% 기본 제어기 대비 감소율
reductionPercent = ...
    (baselineMetrics - tunedMetrics) ...
    ./ baselineMetrics * 100;

%% 결과표 생성
kpiTable = table( ...
    metricNames, ...
    baselineMetrics, ...
    tunedMetrics, ...
    reductionPercent, ...
    'VariableNames', { ...
    'Metric', ...
    'Baseline', ...
    'Tuned', ...
    'ReductionPercent'});

fprintf("\n=== 기본 제어기 vs 튜닝 제어기 KPI ===\n");
disp(kpiTable);