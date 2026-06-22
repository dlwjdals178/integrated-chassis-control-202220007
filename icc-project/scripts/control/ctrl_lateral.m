function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL 횡방향 통합 제어기 (AFS + ESC)
%
%   AFS: yaw rate 추종을 위한 보조 조향각 생성
%   ESC: slip angle이 커졌을 때 yaw moment 생성
%
%   Inputs:
%       yawRateRef - 목표 yaw rate [rad/s]
%       yawRate    - 실제 yaw rate [rad/s]
%       slipAngle  - 차체 슬립 앵글 beta [rad]
%       vx         - 종방향 속도 [m/s]
%       ctrlState  - 내부 상태
%       CTRL       - sim_params.m의 제어기 게인
%       LIM        - 제한값
%       dt         - 샘플링 시간 [s]
%
%   Outputs:
%       deltaAdd.steerAngle - AFS 보조 조향각 [rad]
%       deltaAdd.yawMoment  - ESC 요청 yaw moment [Nm]

    %% 0. ctrlState 초기화
if isempty(ctrlState)
    ctrlState = struct();
end

if ~isfield(ctrlState, 'intError')
    ctrlState.intError = 0;
end

if ~isfield(ctrlState, 'prevError')
    ctrlState.prevError = 0;
end

if ~isfield(ctrlState, 'prevDeriv')
    ctrlState.prevDeriv = 0;
end
    %% 1. sim_params.m에서 횡방향 PID 게인 가져오기
    Kp = CTRL.LAT.Kp;
    Ki = CTRL.LAT.Ki;
    Kd = CTRL.LAT.Kd;
    intMax = CTRL.LAT.intMax;

    %% 2. 기본 설계값 설정
    vxAbs = abs(vx);

    vRef = 20.0;                  % [m/s] 약 72 km/h 기준 속도
    betaTh = deg2rad(2);        % [rad] ESC 작동 시작 슬립각
    Kbeta = 70000;                % [Nm/rad] ESC yaw moment gain
    yawMomentMax = 4000;          % [Nm] yaw moment 제한
    steerLimit = deg2rad(9);    % [rad] AFS 보조 조향각 제한

    %% 3. 과제 제한값보다 크게 못 쓰게 제한
    betaTh = min(betaTh, LIM.MAX_SLIP_ANGLE);
    steerLimit = min(steerLimit, LIM.MAX_STEER_ANGLE);

    %% 4. yaw rate 오차 계산
    error = yawRateRef - yawRate;

    %% 5. 적분항 계산 + anti-windup 1차 제한
    intErrorNew = ctrlState.intError + error * dt;
    intErrorNew = max(-intMax, min(intMax, intErrorNew));

    %% 6. 미분항 계산
    if dt > 0
        derivRaw = (error - ctrlState.prevError) / dt;
    else
        derivRaw = 0;
    end

    %% 7. 미분항 저역통과 필터
    tauD = 0.03;
    alpha = tauD / (tauD + dt);
    derivFilt = alpha * ctrlState.prevDeriv + (1 - alpha) * derivRaw;

    %% 8. 속도별 AFS gain scheduling
    speedGainAFS = vxAbs / vRef;
    speedGainAFS = max(0.3, min(1.2, speedGainAFS));

    %% 9. PID로 AFS 보조 조향각 계산
    deltaRaw = speedGainAFS * (Kp * error + Ki * intErrorNew + Kd * derivFilt);

    %% 10. AFS 조향각 saturation
    deltaSat = max(-steerLimit, min(steerLimit, deltaRaw));

    %% 11. saturation 발생 시 적분항 증가 방지
    if abs(deltaRaw) > steerLimit
        intErrorUsed = ctrlState.intError;

        deltaRaw = speedGainAFS * (Kp * error + Ki * intErrorUsed + Kd * derivFilt);
        deltaSat = max(-steerLimit, min(steerLimit, deltaRaw));
    else
        intErrorUsed = intErrorNew;
    end

    %% 12. ESC yaw moment 계산
    if abs(slipAngle) > betaTh
        betaExcess = abs(slipAngle) - betaTh;

        speedGainESC = vxAbs / vRef;
        speedGainESC = max(0.5, min(2.0, speedGainESC));

        yawMoment = -Kbeta * sign(slipAngle) * betaExcess * speedGainESC;
    else
        yawMoment = 0;
    end

    %% 13. yaw moment saturation
    yawMoment = max(-yawMomentMax, min(yawMomentMax, yawMoment));

    %% 14. 출력 저장
    deltaAdd.steerAngle = deltaSat;
    deltaAdd.yawMoment = yawMoment;

    %% 15. 내부 상태 업데이트
    ctrlState.intError = intErrorUsed;
    ctrlState.prevError = error;
    ctrlState.prevDeriv = derivFilt;

end