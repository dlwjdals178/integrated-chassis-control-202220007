function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL Aggressive B1 ABS tuning for shorter stopping distance
%
% 목적:
%   현재 stop distance가 너무 길 때(예: 69.9 m),
%   slip ratio를 약간 희생하고 brake release를 줄여 정지거리를 줄이는 버전.
%
% 구조:
%   - 일반 시나리오: vxRef가 변하면 vxRef 기반 PI 제동 사용
%   - B1: vxRef 일정 + ax < 0이면 ABS mode latch
%         기본 B1 brake step은 runner/scenario에서 걸린다고 보고,
%         longitudinal은 absRelease/releaseRatio만 coordinator에 전달
%
% 주의:
%   이 코드는 이전에 쓰던 음수 release 지원 coordinator와 같이 써야 함.
%   즉 ctrl_coordinator가 lonCmd.absRelease, lonCmd.releaseRatio를 보고
%   brakeTorque를 음수로 빼주는 구조여야 함.

    %% 0. 상태 초기화
    if isempty(ctrlState) || ~isstruct(ctrlState)
        ctrlState = struct();
    end
    if ~isfield(ctrlState,'intError'),     ctrlState.intError = 0; end
    if ~isfield(ctrlState,'prevForce'),    ctrlState.prevForce = 0; end
    if ~isfield(ctrlState,'prevVxRef'),    ctrlState.prevVxRef = vxRef; end
    if ~isfield(ctrlState,'absMode'),      ctrlState.absMode = false; end
    if ~isfield(ctrlState,'absTimer'),     ctrlState.absTimer = 0; end
    if ~isfield(ctrlState,'releaseTimer'), ctrlState.releaseTimer = 0; end
    if ~isfield(ctrlState,'wheelSlip'),    ctrlState.wheelSlip = []; end

    %% 1. 기본 파라미터
    m  = 1500;      % [kg]
    rw = 0.31;      % [m]

    Kp     = CTRL.LON.Kp;
    Ki     = CTRL.LON.Ki;
    intMax = CTRL.LON.intMax;

    FmaxBrake = 4 * LIM.MAX_BRAKE_TRQ / rw;
    FmaxAx    = m * LIM.MAX_AX;
    Fmax      = min(FmaxBrake, FmaxAx);

    %% 2. vxRef 변화 감지 + 일반 PI
    err = vx - vxRef;   % 양수면 실제 속도가 목표보다 빠름

    vxRefRate = (vxRef - ctrlState.prevVxRef) / max(dt, 1e-6);
    vxRefChanging = abs(vxRefRate) > 0.05;

    F_pi = 0;

    % 일반 시나리오용: vxRef가 실제로 변할 때만 PI 제동 적극 사용
    if vxRefChanging && err > 0.2 && vx > 0.5
        P = Kp * err;

        % anti-windup
        if P < Fmax
            ctrlState.intError = ctrlState.intError + Kp * Ki * err * dt;
            ctrlState.intError = max(0, min(intMax, ctrlState.intError));
        end

        F_pi = P + ctrlState.intError;
    else
        if ax >= 0
            ctrlState.intError = 0;
        end
    end

    F_pi = max(0, min(Fmax, F_pi));

    %% 3. B1 ABS mode latch
    % B1은 vxRef가 일정이므로, ax < 0이 잡히면 제동 상황으로 보고 ABS mode 진입.
    if vx > 0.5 && ax < -0.15 && ~vxRefChanging
        ctrlState.absMode = true;
    end

    % 거의 정지하면 ABS mode 해제
    if vx < 0.5
        ctrlState.absMode = false;
    end

    %% 4. aggressive ABS release 판단
    absRelease   = false;
    releaseRatio = 0.0;

    if ctrlState.absMode
        ctrlState.absTimer = ctrlState.absTimer + dt;

        % wheelSlip이 ctrlState로 들어오는 경우 우선 사용
        slipMax = 0;
        if ~isempty(ctrlState.wheelSlip)
            slipMax = max(abs(ctrlState.wheelSlip(:)));
        end

        % ===============================================================
        % 튜닝 핵심:
        % 기존보다 release를 훨씬 덜 한다.
        % slip 0.12를 조금 넘는 것은 허용하고,
        % 0.18~0.22 이상으로 갈 때부터 본격적으로 release.
        % ===============================================================

        if slipMax > 0.35
            % 너무 큼. lock으로 가기 전에 강하게 풀기
            absRelease = true;
            releaseRatio = 0.45;
            ctrlState.releaseTimer = 0.010;   % 10 ms

        elseif slipMax > 0.22
            % 약간 과함. 중간 release
            absRelease = true;
            releaseRatio = 0.28;
            ctrlState.releaseTimer = 0.007;   % 7 ms

        elseif slipMax > 0.16
            % 0.12는 살짝 넘겨도 정지거리 때문에 허용
            absRelease = true;
            releaseRatio = 0.16;
            ctrlState.releaseTimer = 0.005;   % 5 ms

        elseif slipMax > 0.12
            % limit 살짝 초과는 아주 약하게만 release
            absRelease = true;
            releaseRatio = 0.08;
            ctrlState.releaseTimer = 0.003;   % 3 ms

        % wheelSlip이 안 들어오는 runner용 ax fallback
        % 더 늦게, 더 약하게 release해서 정지거리 줄임.
        elseif ax < -12.5
            absRelease = true;
            releaseRatio = 0.25;
            ctrlState.releaseTimer = 0.006;

        elseif ax < -11.0
            absRelease = true;
            releaseRatio = 0.12;
            ctrlState.releaseTimer = 0.004;

        else
            % slip 정보가 없거나 threshold 전이면 아주 약한 예방 pulse만 사용.
            % 기존 140 ms / 25% / 12 ms보다 훨씬 덜 풀도록 조정.
            if vx > 20
                pulsePeriod = 0.240;
                pulseRatio  = 0.07;
                pulseTime   = 0.003;

            elseif vx > 10
                pulsePeriod = 0.200;
                pulseRatio  = 0.06;
                pulseTime   = 0.003;

            elseif vx > 5
                pulsePeriod = 0.160;
                pulseRatio  = 0.05;
                pulseTime   = 0.002;

            else
                pulsePeriod = 0.120;
                pulseRatio  = 0.04;
                pulseTime   = 0.002;
            end

            if ctrlState.absTimer > pulsePeriod
                absRelease = true;
                releaseRatio = pulseRatio;
                ctrlState.releaseTimer = pulseTime;
                ctrlState.absTimer = 0;
            end
        end

        % release 유지 시간 처리
        if ctrlState.releaseTimer > 0
            absRelease = true;
            % 현재 설정된 releaseRatio가 너무 낮아도 최소 pulseRatio 수준 유지
            releaseRatio = max(releaseRatio, 0.04);
            ctrlState.releaseTimer = ctrlState.releaseTimer - dt;
        end

    else
        ctrlState.absTimer = 0;
        ctrlState.releaseTimer = 0;
    end

    %% 5. 출력 force
    % 일반 시나리오에서는 PI force 사용.
    % B1 ABS mode에서는 기본 brake step이 이미 걸려 있다고 보고,
    % 추가 양수 제동은 만들지 않는다. release만 coordinator로 보냄.
    F_cmd = F_pi;

    if ctrlState.absMode && ~vxRefChanging
        F_cmd = 0;
    end

    %% 6. jerk limit
    dFmax = m * LIM.MAX_JERK * dt;

    dF = F_cmd - ctrlState.prevForce;
    dF = max(-dFmax, min(dFmax, dF));

    F_cmd = ctrlState.prevForce + dF;
    F_cmd = max(0, min(Fmax, F_cmd));

    %% 7. 출력
    forceCmd.Fx_total   = F_cmd;
    forceCmd.brakeRatio = F_cmd / Fmax;

    % coordinator에서 음수 brakeTorque release를 만들기 위한 정보
    forceCmd.absMode      = ctrlState.absMode;
    forceCmd.absRelease   = absRelease;
    forceCmd.releaseRatio = releaseRatio;

    ctrlState.prevForce = F_cmd;
    ctrlState.prevVxRef = vxRef;
end