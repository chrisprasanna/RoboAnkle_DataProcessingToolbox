function rtn = cp_vicon_process_grf(trialName,varargin)


nVarArgs = length(varargin);

% Default input values
cutOff = 60;
eventThreshold = 50;

% Collect and interpret function inputs
for i=1:2:nVarArgs
    switch varargin{i}
        case 'cutOff'
            cutOff = varargin{i+1};
        case 'eventThreshold'
            eventThreshold = varargin{i+1};
        case 'processVicon'
            processVicon = varargin{i+1};
        case 'embData'
            emb = varargin{i+1};
        otherwise
            fprintf('\n%s option not found!\n',varargin{i});
            return
    end
end

% Collect params and info
rtn.params.processing_cutOff = cutOff;
rtn.params.eventThreshold = eventThreshold;

if (processVicon)
    
    rtn.info.useEmb = 0;
    
    % Parse model file
    file = ['./',trialName,'_GRF.csv'];
    rtn.info.file = file;
    % file = trialName;
    if (exist(file,'file') ~= 2)
        fprintf('\n\tGRF file not found: %s\n', file);
        rtn = []
        return
    end

    fid = fopen(file,'r');

    % skip line
    tline = fgetl(fid);
    fs =fscanf(fid,'%f', 1);

    rtn.params.fs = fs;

    fclose(fid);

    % Data
    D = csvread(file,5,0);
    frame_ = D(:,1);
    subframe_ = D(:,2);
    frame = (frame_ - 1).*10 + subframe_;
    
    % Butterworth 4th order
    [b,a] = butter(4,2*(cutOff/fs));

    % Filter/Store data
    rtn.time = frame.*(1/fs);
    rtn.r.Fx = filtfilt(b,a,D(:,3));
    rtn.r.Fy = filtfilt(b,a,D(:,4));
    rtn.r.Fz = filtfilt(b,a,D(:,5));
    rtn.l.Fx = filtfilt(b,a,D(:,6));
    rtn.l.Fy = filtfilt(b,a,D(:,7));
    rtn.l.Fz = filtfilt(b,a,D(:,8));
else
   
    rtn.info.useEmb = 1;
    
    % Butterworth 4th order
    rtn.time = emb.time;
    fs = 1/(rtn.time(2) - rtn.time(1));
    [b,a] = butter(4,2*(cutOff/fs));
    rtn.params.fs = fs;
    
    % Filter/Store data
    rtn.r.Fz = filtfilt(b,a,emb.Right_Ground_Reaction_Force);
    rtn.l.Fz = filtfilt(b,a,emb.Left_Ground_Reaction_Force);
    
end

% Gait events
threshold = (rtn.r.Fz < eventThreshold);
dthreshold = threshold(2:end) - threshold(1:end-1);

to = find(dthreshold == 1);
hs = find(dthreshold == -1);

if(1)
    figure; hold all;
    plot(rtn.time,rtn.r.Fz);
    plot(rtn.time(hs),rtn.r.Fz(hs),'ok')
    plot(rtn.time(to),rtn.r.Fz(to),'or')
end

rtn.ge.r.hs_time(:,1) = rtn.time(hs);
rtn.ge.r.to_time(:,1) = rtn.time(to);

threshold = (rtn.l.Fz < eventThreshold);
dthreshold = threshold(2:end) - threshold(1:end-1);

to = find(dthreshold == 1);
hs = find(dthreshold == -1);

if(1)
    figure; hold all;
    plot(rtn.time,rtn.l.Fz);
    plot(rtn.time(hs),rtn.l.Fz(hs),'ok')
    plot(rtn.time(to),rtn.l.Fz(to),'or')
end

% Collect gait event times and array indices
rtn.ge.l.hs_time(:,1) = rtn.time(hs);
rtn.ge.l.to_time(:,1) = rtn.time(to);
rtn.ge.l.hs_ind(:,1) = hs;
rtn.ge.l.to_ind(:,1) = to;


end
