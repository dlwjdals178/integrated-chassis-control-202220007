function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL 수직방향 제어기 (Semi-active Skyhook CDC)

    %% 0. 내부 상태 초기화
    if isempty(ctrlState) || ~isstruct(ctrlState)
        ctrlState = struct();
    end

    if ~isfield(ctrlState, 'prevDamping')
        ctrlState.prevDamping = CTRL.VER.cMin * ones(4, 1);
    end

    %% 1. suspState 입력 보호
    if isfield(suspState, 'zs_dot')
        zs_dot = suspState.zs_dot(:);
    else
        zs_dot = zeros(4, 1);
    end

    if isfield(suspState, 'zu_dot')
        zu_dot = suspState.zu_dot(:);
    else
        zu_dot = zeros(4, 1);
    end

    if numel(zs_dot) ~= 4
        zs_dot = zeros(4, 1);
    end

    if numel(zu_dot) ~= 4
        zu_dot = zeros(4, 1);
    end

    %% 2. Skyhook on-off 제어
    relVel = zs_dot - zu_dot;

    cLow  = CTRL.VER.cMin;
    cHigh = CTRL.VER.cMax;

    dampingRaw = cLow * ones(4, 1);

    for i = 1:4
        if zs_dot(i) * relVel(i) > 0
            dampingRaw(i) = cHigh;
        else
            dampingRaw(i) = cLow;
        end
    end

    %% 3. damping 변화 완화
    alpha = 0.85;
    dampingCmd = alpha * ctrlState.prevDamping + ...
                 (1 - alpha) * dampingRaw;

    %% 4. saturation
    dampingCmd = max(CTRL.VER.cMin, min(CTRL.VER.cMax, dampingCmd));

    %% 5. 상태 업데이트
    ctrlState.prevDamping = dampingCmd;

end