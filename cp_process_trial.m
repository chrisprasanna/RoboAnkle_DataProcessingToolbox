function rtn = cp_process_trial(trialName,varargin)
% This function processes vicon + embedded files

nVarArgs = length(varargin);

% Defaults
processEmb = 1; % yes = 1, no = 0
processVicon = 1; 

% Motor 3 datasheet params
k_t = 10.2/1000;
k_s = 940;
Vcc = 60;
R_T = 1800;
maxMotorAmp = 14;
maxMotorRpm = 80000;

% Default cutoff freqs and Thresholds
modelCutOff = 6;
GRFCutOff = 50;
embCutOff = 50;
eventThreshold = 50;

% Parse input arguemnts and interpret them 
for i=1:2:nVarArgs
    switch varargin{i}
        case 'processEmb'
            processEmb = varargin{i+1};
        case 'processVicon'
            processVicon = varargin{i+1};
        case 'k_t'
            % Torque Constant (Nm/A)
            k_t = varargin{i+1};
        case 'k_s'
            % Speed Constant (rpm/V)
            k_s = varargin{i+1};
        case 'Vcc'
            % Operating Voltage (V)
            Vcc = varargin{i+1};
        case 'R_T'
            % Total tranmission ratio
            R_T = varargin{i+1};
        case 'maxMotorAmp'
            % Max Motor Current (A) -- setting on ESCON 70/10 servo controller
            maxMotorAmp = varargin{i+1};
        case 'maxMotorRpm'
            % Max Motor Speed (Rpm) -- setting on ESCON 70/10 servo controller
            maxMotorRpm = varargin{i+1};
        case 'modelCutOff'
            modelCutOff = varargin{i+1};
        case 'GRFCutOff'
            GRFCutOff = varargin{i+1};
        case 'embCutOff'
            embCutOff = varargin{i+1};
        case 'eventThreshold'
            eventThreshold = varargin{i+1};
        otherwise
            fprintf('\n%s option not found!\n',varargin{i});
            return
    end
end

%% Process subject params
% rtn.subject = cp_vicon_process_subject(trialName);

%% Process inverse dynamics model
if (processVicon)

    model = cp_vicon_process_model(trialName, ...
        'cutOff',modelCutOff);

    rtn.info.model = model.info;
    rtn.params.model = model.params;
    rtn.time = model.time;
    rtn.model = model.id;
    rtn.markers = model.markers;

end

%% Process embedded data
if (processEmb)
    emb = cp_labview_process_data(trialName, ...
        'k_t',k_t, ...
        'k_s',k_s, ...
        'Vcc',Vcc, ...
        'R_T',R_T, ...
        'maxMotorAmp',maxMotorAmp, ...
        'maxMotorRpm',maxMotorRpm, ...
        'cutOff',embCutOff);
    
    rtn.params.emb = emb.params;
    rtn.control = emb.control;
    rtn.ge.emb = emb.ge;
    rtn.emb = emb.data;
    % rtn.info.emb = emb.info; % EDIT
    
end


%
%
%% Process vGRF

grf = cp_vicon_process_grf(trialName, ...
    'cutOff',GRFCutOff, ...
    'eventThreshold',eventThreshold,...
    'processVicon',processVicon,...
    'embData',emb.data);

rtn.info.grf = grf.info;
rtn.params.grf = grf.params;
rtn.grf.time = grf.time;
rtn.grf.r = grf.r;
rtn.grf.l = grf.l;
ge = grf.ge;

% Process HS
for foot = {'l','r'}

    % Remove TO before first HS
    ge.(foot{:}).to_time(ge.(foot{:}).to_time < ge.(foot{:}).hs_time(1)) = [];

    % Remove TO after last HS
    ge.(foot{:}).to_time(ge.(foot{:}).to_time > ge.(foot{:}).hs_time(end)) = [];

    % Reformat
    segment.(foot{:}).gait.time =  ...
        [ge.(foot{:}).hs_time(1:end-1), ge.(foot{:}).hs_time(2:end)];

    segment.(foot{:}).stance.time =  ...
        [ge.(foot{:}).hs_time(1:end-1), ge.(foot{:}).to_time(1:end)];

    segment.(foot{:}).swing.time =  ...
        [ge.(foot{:}).to_time(1:end), ge.(foot{:}).hs_time(2:end)];

end

rtn.ge.grf = ge;
rtn.segment.grf = segment;

%% Time normalize -- Whole Gait Cycle
%
%


% Gait cycle
gaitCycle = linspace(0,100,1001);
rtn.gait.gaitCycle = gaitCycle;

% Field names
if (processVicon)
    q_name = fieldnames(rtn.model);
end

q_grf = fieldnames(rtn.grf.l);

% Process embedded data
if (processEmb)
    q_control = fieldnames(rtn.control);
end

% Time Normalization
for foot = {'l','r'}
    
    hs_time = rtn.segment.grf.(foot{:}).gait.time;
    
    for i=1:numel(hs_time(:,1))
        
        if (processVicon)
            % If recorded hs occurs before model data, skip
            if (hs_time(i,1) < rtn.time(1)) | (hs_time(i,2) > rtn.time(end))
                continue;
            end

            % Find vicon time stamp corresponding to HS
            temp = rtn.time - hs_time(i,1);
            temp(temp < 0) = inf;
            [~,ii] = min(temp);
            hs1 = ii;

            temp = rtn.time - hs_time(i,2);
            temp(temp < 0) = inf;
            [~,ii] = min(temp);
            hs2 = ii;

            rawTime = rtn.time(hs1:hs2);
            rawGaitCylce = (rawTime - rawTime(1))./(rawTime(end) - rawTime(1))*100;

            rtn.gait.model.normalized.(foot{:}).time{i} = rawTime;

            % Interpolate inverse dynamics (vicon)
        
            for j=1:numel(q_name)
                rtn.gait.model.normalized.(foot{:}).(q_name{j}).X(:,i) = ...
                    interp1(rawGaitCylce, rtn.model.(q_name{j}).X(hs1:hs2), ...
                    gaitCycle, 'spline');
                rtn.gait.model.normalized.(foot{:}).(q_name{j}).Y(:,i) = ...
                    interp1(rawGaitCylce, rtn.model.(q_name{j}).Y(hs1:hs2), ...
                    gaitCycle, 'spline');
                rtn.gait.model.normalized.(foot{:}).(q_name{j}).Z(:,i) = ...
                    interp1(rawGaitCylce, rtn.model.(q_name{j}).Z(hs1:hs2), ...
                    gaitCycle, 'spline');
            end

        end
        
        % Find grf time stamp corresponding to HS
        temp = rtn.grf.time - hs_time(i,1);
        temp(temp < 0) = inf;
        [~,ii] = min(temp);
        hs1 = ii;

        temp = rtn.grf.time - hs_time(i,2);
        temp(temp < 0) = inf;
        [~,ii] = min(temp);
        hs2 = ii;

        rawTime = rtn.grf.time(hs1:hs2);
        rawGaitCylce = (rawTime - rawTime(1))./(rawTime(end) - rawTime(1))*100;

        rtn.gait.grf.normalized.(foot{:}).grf.time{i} = rawTime;

        % Interpolate vGRF
        for j=1:numel(q_grf)

            rtn.gait.grf.normalized.(foot{:}).(['L',q_grf{j}])(:,i) = ...
                interp1(rawGaitCylce, rtn.grf.l.(q_grf{j})(hs1:hs2), ...
                gaitCycle, 'spline');
            rtn.gait.grf.normalized.(foot{:}).(['R',q_grf{j}])(:,i) = ...
                interp1(rawGaitCylce, rtn.grf.r.(q_grf{j})(hs1:hs2), ...
                gaitCycle, 'spline');
        end
        
        
        
        % Interpolate embedded system data
        if (processEmb)
            
            % Find emb time stamp corresponding to HS
            temp = rtn.control.time - hs_time(i,1);
            temp(temp < 0) = inf;
            [~,ii] = min(temp);
            hs1 = ii;
            
            temp = rtn.control.time - hs_time(i,2);
            temp(temp < 0) = inf;
            [~,ii] = min(temp);
            hs2 = ii;
            
            rawTime = rtn.control.time(hs1:hs2);
            if isempty(rawTime) || sum(eq(rawTime,0))
                
            else
                rawGaitCylce = (rawTime - rawTime(1))./(rawTime(end) - rawTime(1))*100;
                
                rtn.gait.emb.normalized.(foot{:}).control.time{i} = rawTime;
                
                for j=2:numel(q_control)
                    rtn.gait.emb.normalized.(foot{:}).(q_control{j})(:,i) = ...
                        interp1(rawGaitCylce, rtn.control.(q_control{j})(hs1:hs2), ...
                        gaitCycle, 'linear');
                end
            end
        end
        
    end
    
    % Mean, std
    if (processVicon)
        for j=1:numel(q_name)
            jointVarMean_X =  ...
                mean(rtn.gait.model.normalized.(foot{:}).(q_name{j}).X, 2);
            jointVarMean_Y = ...
                mean(rtn.gait.model.normalized.(foot{:}).(q_name{j}).Y, 2);
            jointVarMean_Z =  ...
                mean(rtn.gait.model.normalized.(foot{:}).(q_name{j}).Z, 2);

            jointVarStd_X =  ...
                std(rtn.gait.model.normalized.(foot{:}).(q_name{j}).X')';
            jointVarStd_Y = ...
                std(rtn.gait.model.normalized.(foot{:}).(q_name{j}).Y')';
            jointVarStd_Z = ...
                std(rtn.gait.model.normalized.(foot{:}).(q_name{j}).Z')';

            rtn.gait.model.(foot{:}).(q_name{j}).X = ...
                [jointVarMean_X + jointVarStd_X, ...
                jointVarMean_X, ...
                jointVarMean_X - jointVarStd_X];

            rtn.gait.model.(foot{:}).(q_name{j}).Y = ...
                [jointVarMean_Y + jointVarStd_Y, ...
                jointVarMean_Y, ...
                jointVarMean_Y - jointVarStd_Y];

            rtn.gait.model.(foot{:}).(q_name{j}).Z = ...
                [jointVarMean_Z + jointVarStd_Z, ...
                jointVarMean_Z, ...
                jointVarMean_Z - jointVarStd_Z];
        end
    end
    
    % Mean, std
    for j=1:numel(q_grf)
        varMean = ...
            mean(rtn.gait.grf.normalized.(foot{:}).(['L',q_grf{j}]), 2);
        
        varStd = ...
            std(rtn.gait.grf.normalized.(foot{:}).(['L',q_grf{j}])')';
        
        rtn.gait.grf.(foot{:}).(['L',q_grf{j}]) = [varMean + varStd, ...
            varMean, ...
            varMean - varStd];
        
        varMean = ...
            mean(rtn.gait.grf.normalized.(foot{:}).(['R',q_grf{j}]), 2);
        
        varStd = ...
            std(rtn.gait.grf.normalized.(foot{:}).(['R',q_grf{j}])')';
        
        rtn.gait.grf.(foot{:}).(['R',q_grf{j}]) = [varMean + varStd, ...
            varMean, ...
            varMean - varStd];
    end
    
    % Mean, std
    if (processEmb)
        for j=2:numel(q_control)
            varMean = mean(rtn.gait.emb.normalized.(foot{:}).(q_control{j}), 2);
            varStd = std(rtn.gait.emb.normalized.(foot{:}).(q_control{j})')';
            
            rtn.gait.emb.(foot{:}).(q_control{j}) = [varMean + varStd, ...
                varMean, ...
                varMean - varStd];
        end
    end
    
end


%
%
%% Time normalize -- Stance Cycle
%
%


% Gait cycle
stanceCycle = linspace(0,100,1001);
rtn.stance.stanceCycle = stanceCycle;

% Time Normalization
for foot = {'l','r'}
    
    hs_time = rtn.segment.grf.(foot{:}).stance.time;
    
    for i=1:numel(hs_time(:,1))
        
        if (processVicon)
            % If recorded hs occurs before model data, skip
            if hs_time(i,1) < rtn.time(1)
                continue;
            end

            % Find vicon time stamp corresponding to HS
            temp = rtn.time - hs_time(i,1);
            temp(temp < 0) = inf;
            [~,ii] = min(temp);
            hs1 = ii;

            temp = rtn.time - hs_time(i,2);
            temp(temp < 0) = inf;
            [~,ii] = min(temp);
            hs2 = ii;

            rawTime = rtn.time(hs1:hs2);
            rawStanceCycle = (rawTime - rawTime(1))./(rawTime(end) - rawTime(1))*100;

            rtn.gait.model.normalized.(foot{:}).time{i} = rawTime;

            % Interpolate inverse dynamics (vicon)
            for j=1:numel(q_name)
                rtn.stance.model.normalized.(foot{:}).(q_name{j}).X(:,i) = ...
                    interp1(rawStanceCycle, rtn.model.(q_name{j}).X(hs1:hs2), ...
                    stanceCycle, 'spline');
                rtn.stance.model.normalized.(foot{:}).(q_name{j}).Y(:,i) = ...
                    interp1(rawStanceCycle, rtn.model.(q_name{j}).Y(hs1:hs2), ...
                    stanceCycle, 'spline');
                rtn.stance.model.normalized.(foot{:}).(q_name{j}).Z(:,i) = ...
                    interp1(rawStanceCycle, rtn.model.(q_name{j}).Z(hs1:hs2), ...
                    stanceCycle, 'spline');
            end
        end
        
        % Find grf time stamp corresponding to HS
        temp = rtn.grf.time - hs_time(i,1);
        temp(temp < 0) = inf;
        [~,ii] = min(temp);
        hs1 = ii;
        
        temp = rtn.grf.time - hs_time(i,2);
        temp(temp < 0) = inf;
        [~,ii] = min(temp);
        hs2 = ii;
        
        rawTime = rtn.grf.time(hs1:hs2);
        rawStanceCycle = (rawTime - rawTime(1))./(rawTime(end) - rawTime(1))*100;
        
        rtn.stance.grf.normalized.(foot{:}).grf.time{i} = rawTime;
        
        % Interpolate vGRF
        for j=1:numel(q_grf)
            
            rtn.stance.grf.normalized.(foot{:}).(['L',q_grf{j}])(:,i) = ...
                interp1(rawStanceCycle, rtn.grf.l.(q_grf{j})(hs1:hs2), ...
                stanceCycle, 'spline');
            rtn.stance.grf.normalized.(foot{:}).(['R',q_grf{j}])(:,i) = ...
                interp1(rawStanceCycle, rtn.grf.r.(q_grf{j})(hs1:hs2), ...
                stanceCycle, 'spline');
        end
        
        
        % Interpolate embedded system
        if (processEmb)
            
            % Find emb time stamp corresponding to HS
            temp = rtn.control.time - hs_time(i,1);
            temp(temp < 0) = inf;
            [~,ii] = min(temp);
            hs1 = ii;
            
            temp = rtn.control.time - hs_time(i,2);
            temp(temp < 0) = inf;
            [~,ii] = min(temp);
            hs2 = ii;
            
            rawTime = rtn.control.time(hs1:hs2);
            if isempty(rawTime) || sum(eq(rawTime,0))
                
            else
                rawStanceCycle = (rawTime - rawTime(1))./(rawTime(end) - rawTime(1))*100;
                
                rtn.stance.emb.normalized.(foot{:}).control.time{i} = rawTime;
                
                for j=2:numel(q_control)
                    rtn.stance.emb.normalized.(foot{:}).(q_control{j})(:,i) = ...
                        interp1(rawStanceCycle, rtn.control.(q_control{j})(hs1:hs2), ...
                        stanceCycle, 'linear');
                end
            end
        end
        
    end
    
    if (processVicon)
        % Mean, std
        for j=1:numel(q_name)
            jointVarMean_X =  ...
                mean(rtn.stance.model.normalized.(foot{:}).(q_name{j}).X, 2);
            jointVarMean_Y = ...
                mean(rtn.stance.model.normalized.(foot{:}).(q_name{j}).Y, 2);
            jointVarMean_Z =  ...
                mean(rtn.stance.model.normalized.(foot{:}).(q_name{j}).Z, 2);

            jointVarStd_X =  ...
                std(rtn.stance.model.normalized.(foot{:}).(q_name{j}).X')';
            jointVarStd_Y = ...
                std(rtn.stance.model.normalized.(foot{:}).(q_name{j}).Y')';
            jointVarStd_Z = ...
                std(rtn.stance.model.normalized.(foot{:}).(q_name{j}).Z')';

            rtn.stance.model.(foot{:}).(q_name{j}).X = ...
                [jointVarMean_X + jointVarStd_X, ...
                jointVarMean_X, ...
                jointVarMean_X - jointVarStd_X];

            rtn.stance.model.(foot{:}).(q_name{j}).Y = ...
                [jointVarMean_Y + jointVarStd_Y, ...
                jointVarMean_Y, ...
                jointVarMean_Y - jointVarStd_Y];

            rtn.stance.model.(foot{:}).(q_name{j}).Z = ...
                [jointVarMean_Z + jointVarStd_Z, ...
                jointVarMean_Z, ...
                jointVarMean_Z - jointVarStd_Z];
        end
    end
    
    % Mean, std
    for j=1:numel(q_grf)
        varMean = ...
            mean(rtn.stance.grf.normalized.(foot{:}).(['L',q_grf{j}]), 2);
        
        varStd = ...
            std(rtn.stance.grf.normalized.(foot{:}).(['L',q_grf{j}])')';
        
        rtn.stance.grf.(foot{:}).(['L',q_grf{j}]) = [varMean + varStd, ...
            varMean, ...
            varMean - varStd];
        
        varMean = ...
            mean(rtn.stance.grf.normalized.(foot{:}).(['R',q_grf{j}]), 2);
        
        varStd = ...
            std(rtn.stance.grf.normalized.(foot{:}).(['R',q_grf{j}])')';
        
        rtn.stance.grf.(foot{:}).(['R',q_grf{j}]) = [varMean + varStd, ...
            varMean, ...
            varMean - varStd];
    end
    
    
    if (processEmb)
        for j=2:numel(q_control)
            varMean = mean(rtn.stance.emb.normalized.(foot{:}).(q_control{j}), 2);
            varStd = std(rtn.stance.emb.normalized.(foot{:}).(q_control{j})')';
            
            rtn.stance.emb.(foot{:}).(q_control{j}) = [varMean + varStd, ...
                varMean, ...
                varMean - varStd];
        end
    end
    
end
%%
if (processVicon)
    figure;
    subplot(311); hold all;
    plot(rtn.gait.gaitCycle,rtn.gait.model.normalized.r.RAnkleAngles.X,'r', ...
        'lineWidth',0.1);
    plot(rtn.gait.gaitCycle,rtn.gait.model.normalized.l.LAnkleAngles.X,'k', ...
        'lineWidth',0.1);
    grid on
    subplot(312); hold all;
    plot(rtn.gait.gaitCycle,rtn.gait.model.normalized.r.RAnkleMoment.X,'r', ...
        'lineWidth',0.1);
    plot(rtn.gait.gaitCycle,rtn.gait.model.normalized.l.LAnkleMoment.X,'k', ...
        'lineWidth',0.1);
    grid on
    subplot(313); hold all;
    plot(rtn.gait.gaitCycle,rtn.gait.model.normalized.r.RAnklePower.X,'r', ...
        'lineWidth',0.1);
    plot(rtn.gait.gaitCycle,rtn.gait.model.normalized.l.LAnklePower.X,'k', ...
        'lineWidth',0.1);
    grid on
end
end
