function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR actuator allocation + weaker ABS negative release
%
% brakeTorque order:
%   [FL; FR; RL; RR]
%
% actAdd 구조에서 B1 기본 brake step을 줄이기 위해 음수 brakeTorque 보정 사용.
% 이번 버전은 release를 약하게 해서 정지거리 증가 문제를 줄임.

    %% 1. steering
    if isfield(latCmd,'steerAngle')
        steerAngle = latCmd.steerAngle;
    else
        steerAngle = 0;
    end
    steerAngle = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, steerAngle));

    %% 2. 기본값
    brakeTorque = zeros(4,1);

    %% 3. ABS negative release
    if isfield(lonCmd,'absRelease') && lonCmd.absRelease
        if isfield(lonCmd,'releaseRatio')
            rel = lonCmd.releaseRatio;
        else
            rel = 0.25;
        end

        % 너무 많이 빼지 않도록 상한 낮춤
        rel = max(0, min(0.55, rel));

        baseBrakeB1 = [1500; 1500; 800; 800];

        brakeTorque = brakeTorque - rel * baseBrakeB1;
    end

    %% 4. 일반 시나리오용 추가 제동
    isAbsMode = isfield(lonCmd,'absMode') && lonCmd.absMode;

    if ~isAbsMode
        Fbrake = 0;

        if isfield(lonCmd,'brakeRatio') && lonCmd.brakeRatio > 0
            m = 1500;
            rw = VEH.rw;
            FmaxBrake = 4 * LIM.MAX_BRAKE_TRQ / rw;
            FmaxAx    = m * LIM.MAX_AX;
            Fmax      = min(FmaxBrake, FmaxAx);
            Fbrake = max(0, min(1, lonCmd.brakeRatio)) * Fmax;

        elseif isfield(lonCmd,'Fx_total') && lonCmd.Fx_total > 0
            Fbrake = lonCmd.Fx_total;
        end

        if Fbrake > 0
            TfrontTotal = Fbrake * 0.60 * VEH.rw;
            TrearTotal  = Fbrake * 0.40 * VEH.rw;

            brakeTorque(1) = brakeTorque(1) + TfrontTotal/2;
            brakeTorque(2) = brakeTorque(2) + TfrontTotal/2;
            brakeTorque(3) = brakeTorque(3) + TrearTotal/2;
            brakeTorque(4) = brakeTorque(4) + TrearTotal/2;
        end
    end

    %% 5. yaw moment differential brake
    if isfield(latCmd,'yawMoment')
        Mz = latCmd.yawMoment;
    else
        Mz = 0;
    end

    if Mz ~= 0
        halfTf = VEH.track_f / 2;
        halfTr = VEH.track_r / 2;

        dTf = abs(Mz) * 0.60 * VEH.rw / halfTf;
        dTr = abs(Mz) * 0.40 * VEH.rw / halfTr;

        if Mz > 0
            brakeTorque(2) = brakeTorque(2) + dTf;
            brakeTorque(4) = brakeTorque(4) + dTr;
        else
            brakeTorque(1) = brakeTorque(1) + dTf;
            brakeTorque(3) = brakeTorque(3) + dTr;
        end
    end

    %% 6. saturation
    brakeTorque = max(-LIM.MAX_BRAKE_TRQ, min(LIM.MAX_BRAKE_TRQ, brakeTorque));

    %% 7. output
    actuatorCmd.steerAngle   = steerAngle;
    actuatorCmd.brakeTorque  = brakeTorque;
    actuatorCmd.dampingCoeff = verCmd;
end
