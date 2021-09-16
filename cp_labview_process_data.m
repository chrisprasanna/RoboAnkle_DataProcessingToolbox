function rtn = cp_labview_process_data(trialName,varargin)
% Process logged data from LabVIEW Real-Time VIs
% note: need "convertTDMS" function from Matlab File Exchange 

nVarArgs = length(varargin);

% Defaults
k_t = 10.2/1000;      % 7.75/1000;        % torque constant - data sheet
%  k_s = 1230;          % speed constant - data sheet
k_s = 940; % 1223;    % maxon calibration (may be different than datasheet)
Vcc = 60;             % power supply voltage
maxMotorAmp = 14;     % setting in maxon
maxMotorRpm = 80000;  % setting in maxon
cutOff = 50;          % cutoff freq. for filtering
R_T = 1806;           % estimated from motor/ankle velocity data
syncVicon = 1;        % sync vicon data with sync pulse

% Collect and interpret input data
for i=1:2:nVarArgs
    switch varargin{i}
        case 'k_t'
            % torque constant (Nm/A)
            k_tau = varargin{i+1};
        case 'k_s'
            % speed constant (rpm/V)
            k_s = varargin{i+1};
        case 'Vcc'
            % operating voltage (V)
            Vcc = varargin{i+1};
        case 'R_T'
            % total tranmission ratio
            R_T = varargin{i+1};
        case 'maxMotorAmp'
            % max motor current (A) -- setting on ESCON 70/10 servo controller
            maxMotorAmp = varargin{i+1};
        case 'maxMotorRpm'
            % max motor speed (Rpm) -- setting on ESCON 70/10 servo controller
            maxMotorRpm = varargin{i+1};
        case 'cutOff'
            % filter cutoff frequency, 4-th order butterworth low-pass
            cutOff = varargin{i+1};
        case 'oldVersion'
            %  Old parameter format
            oldVersion = varargin{i+1};
        case 'syncVicon'
            %  sync pin
            syncVicon = varargin{i+1};
        otherwise
            fprintf('\n%s option not found!\n',varargin{i});
            return
    end
end

doplot = 1;

%
%% parse embedded parameters
%

% Check if metadata file exists
% metafile = ['./',trialName,'_metadata.tdms']; 
metafile = [trialName,'_metadata.tdms']; 
if exist(metafile,'file') ~= 2
    fprintf('\n\tMetadata file not found\n');
    rtn = []
    return
end
% [t,~,meta_ChanNames] = convertTDMS(false, 'filename', fullfile(pwd, metafile));
[t,~,meta_ChanNames] = convertTDMS(false, 'filename', metafile);
for ii = 3:length(t.Data.MeasuredData)
    group = extractBefore(t.Data.MeasuredData(ii).Name,'/');
    t.Data.MeasuredData(ii).Name = erase(t.Data.MeasuredData(ii).Name,[group,'/']);
end
for ii = 1:length(meta_ChanNames{1})
    group = extractBefore(meta_ChanNames{1}{ii},'/');
    meta_ChanNames{1}{ii} = erase(meta_ChanNames{1}{ii},[group,'/']);
end
meta_ChanNames = meta_ChanNames{1};
t.Data.MeasuredData = t.Data.MeasuredData(3:end);
% store data in struct
[~,q] = size(t.Data.MeasuredData);
for i=1:q
    name = strrep(t.Data.MeasuredData(i).Name, ' ', '_');
    name = regexp(name,'^\w+','once','match');
    if name(end) == '_'; name = name(1:end-1); end
    rtn.params.(name) = t.Data.MeasuredData(i).Data;
end

fs = rtn.params.Sampling_Freq;

% EDIT --> first line is date and time
% rtn.info.date = fscanf(fid,'#Date: %s',1);
% rtn.info.time = fscanf(fid,'%s\n',1);
% rtn.info.file = trialName;

% System parameters (inputted into this function)
rtn.params.motor.k_t = k_t;
rtn.params.motor.k_s = k_s;
rtn.params.motor.Vcc = Vcc;
rtn.params.motor.R_T = R_T;
rtn.params.motor.maxMotorAmp = maxMotorAmp;
rtn.params.motor.maxMotorRpm = maxMotorRpm;

%
%% read logged data
%

% Check file exists
% file = ['./',trialName,'_LabView.tdms']; % ['./',trialName];
file = [trialName,'_LabView.tdms'];
if exist(file,'file') ~= 2
    fprintf('\n\tEmbedded file not found\n');
    rtn = []
    return
end

% Open and Process Datafile
% [s,~,ChanNames,~,~] = convertTDMS(false, 'filename', fullfile(pwd, file));
[s,~,ChanNames,~,~] = convertTDMS(false, 'filename', file);
for ii = 3:length(s.Data.MeasuredData)
    group = extractBefore(s.Data.MeasuredData(ii).Name,'/');
    s.Data.MeasuredData(ii).Name = erase(s.Data.MeasuredData(ii).Name,[group,'/']);
end
for ii = 1:length(ChanNames{1})
    group = extractBefore(ChanNames{1}{ii},'/');
    ChanNames{1}{ii} = erase(ChanNames{1}{ii},[group,'/']);
end
ChanNames = ChanNames{1};
s.Data.MeasuredData = s.Data.MeasuredData(3:end);

% Size check
[~,n] = size(s.Data.MeasuredData);
nn = numel(ChanNames);

if nn ~= n
    [nn, n]
    error('Labels/column mismatch!');
    return
end


% Find if/where data bits were dropped
% Time stamp difference: dt = 1 for all points unless data chunk is missing
%
% This happens, I think, because the kernel interrupts during circular buffer
% writes.

% if(doplot)
%     figure;
%     plot(D(:,1))
%     xlabel('Data Point','fontsize',20)
%     ylabel('Time Stamp','fontsize',20)
% end
%
% dt = D(2:end,1) - D(1:end-1,1);
% startMissingData_index = find(dt ~= 1);
% endMissingData_index = startMissingData_index + 1;
%
% if(doplot)
%     figure;
%     plot(1:numel(D(1:end-1,1)),dt)
%     xlabel('Data Point','fontsize',20)
%     ylabel('Stamp[k+1] - Stamp[k]','fontsize',20)
% end

% calculate time vector
cnt0 = 0;
cntEnd = s.Data.MeasuredData(1).Total_Samples * 1/fs;
rtn.data.time = linspace(cnt0, cntEnd, s.Data.MeasuredData(1).Total_Samples)';
time = rtn.data.time;

% store data in struct
for i=1:n
    if contains(s.Data.MeasuredData(i).Name, 'STTTM') || contains(s.Data.MeasuredData(i).Name, 'Stiffness')
    else
        name = strrep(s.Data.MeasuredData(i).Name, ' ', '_');
        name = regexp(name,'^\w+','once','match');
        if name(end) == '_'; name = name(1:end-1); end
        rtn.data.(name) = s.Data.MeasuredData(i).Data;
    end
end

% butterworth 4th order
[b,a] = butter(4,2*(cutOff/fs));

% remove repeated hs
l_hs = find(rtn.data.Left_Heel_Strike_Boolean);
l_hs(l_hs < cnt0) = [];
r_hs = find(rtn.data.Right_Heel_Strike_Boolean);
r_hs(r_hs < cnt0) = [];

% collect HS indices
rtn.ge.l.hs_ind = l_hs;
rtn.ge.r.hs_ind = r_hs;

% find HS time
rtn.ge.l.hs_time = (l_hs - cnt0).*(1/fs);
rtn.ge.r.hs_time = (r_hs - cnt0).*(1/fs);

% Find TO from changes in state


% store important data
rtn.control.time = rtn.data.time;
rtn.control.u = rtn.data.Desired_Motor_Current;
rtn.control.i_m = rtn.data.True_Motor_Current;
rtn.control.omega_m_rpm = rtn.data.Motor_Velocity;
rtn.control.omega_m = rpm2radps(rtn.control.omega_m_rpm);
rtn.control.d_omega_m = gradient(rtn.control.omega_m)*fs;
rtn.control.tau_m = rtn.control.i_m.*k_t;
rtn.control.R_Tau = rtn.data.Transmission_Ratio;
rtn.control.tau_a = rtn.control.tau_m.*rtn.control.R_Tau;
rtn.control.V_m = rtn.control.omega_m_rpm/k_s;
rtn.control.P_m_elect = rtn.control.i_m.*rtn.control.V_m;
rtn.control.P_m_mech = rtn.control.tau_m.*rtn.control.omega_m;
rtn.control.ankPos = filtfilt(b,a,rtn.data.Ankle_Encoder_Angle);
rtn.control.d_ankPos = gradient(rtn.control.ankPos)*fs;
rtn.control.d2_ankPos = gradient(rtn.control.d_ankPos)*fs;
rtn.control.imu_roll = rtn.data.IMU_Roll_Angle;
rtn.control.imu_pitch = rtn.data.IMU_Pitch_Angle;
rtn.control.imu_yaw = rtn.data.IMU_Yaw_Angle;
rtn.control.thigh_angle = rtn.data.Thigh_Angle;
rtn.control.Phi = rtn.data.Phi;
rtn.control.GRF_l = rtn.data.Left_Ground_Reaction_Force;
rtn.control.GRF_r = rtn.data.Right_Ground_Reaction_Force;
rtn.control.GRF_l_filt = rtn.data.Filtered_Left_GRF;
rtn.control.GRF_r_filt = rtn.data.Filtered_Right_GRF;
rtn.control.dGRF_l = rtn.data.Left_GRF_Rate;
rtn.control.dGRF_r = rtn.data.Right_GRF_Rate;
rtn.control.theta_v = rtn.data.Desired_Ankle_Angle_Trajectory;
rtn.control.tau_ref = rtn.data.Torque_Reference;
rtn.control.tau_passive = rtn.data.Passive_Torque_Estimate;

end

