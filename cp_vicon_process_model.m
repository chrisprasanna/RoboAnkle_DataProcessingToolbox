function rtn = cp_vicon_process_model(trialName,varargin)

% This function parses vicon model csv files

nVarArgs = length(varargin);

% Default Vicon cutoff frequency for filtering
cutOff = 6;

% Collect and interpret function inputs
for i=1:2:nVarArgs
    switch varargin{i}
        case 'cutOff'
            cutOff = varargin{i+1};
        otherwise
            fprintf('\n%s option not found!\n',varargin{i});
            return
    end
end

% Plots
debug = 0;

% Parse model file
file = fullfile(cd, [trialName,'_Model.csv']);
if exist(file,'file') ~= 2
    fprintf('\n\tModel file not found: %s\n', file);
    rtn = []
    return
end

fid = fopen(file,'r');

% Store trialName
rtn.info.file = file;
rtn.params.processing_cutOff = cutOff;

% skip two words, go next line, grab sample freq
fs = fscanf(fid, '%*s %*s\n %f', 1);

rtn.params.fs = fs;

% skip line
tline = fgetl(fid);

% parse labels and create struct fields names
tline = fgetl(fid);
labels = textscan(tline, '%s', 'delimiter', ',');
[m, n] = size(labels{1});
labelCnt = 0;
for i=1:m
    if ~(strcmp(labels{1}{i},''))
        % pasre header on label
        [token, remain] = strtok(labels{1}{i},':');
        labelCnt = labelCnt + 1;
        labelName{labelCnt} = remain(2:end);
    end
end

% find number of subfields
tline = fgetl(fid);
sublabels = textscan(tline, '%s', 'delimiter', ',');
[m, n] = size(sublabels{1});

% skip line of units
fgetl(fid);

% put data in matrix
D = zeros(1,m);
formatSpec = [repmat(['%f'],1,m-1), '%f\n'];
rowCount = 1;
while(1)
    C = textscan(fid, formatSpec, 'delimiter', ',');
    dataRow = cell2mat(C);
    if isempty(dataRow)
        break;
    end
    D(rowCount,:) = cell2mat(C);
    rowCount = rowCount + 1;
end

% Time = (frame number - 1) x dt
rtn.time = (D(:,1) - 1)*(1/fs);

% Butterworth 4th order
[b,a] = butter(4,2*(cutOff/fs));

% put data (X,Y,Z subfields per field)
% Fix units
% Pattern is (L/R)(joint)Angle, Moment, Power
for i=1:(labelCnt/3)
    
    % Angle -------------------------------------------------------------------
    jointAngle_X = deg2rad(D(:,3 + (9*i - 9)));
    jointAngle_Y = deg2rad(D(:,4 + (9*i - 9)));
    jointAngle_Z = deg2rad(D(:,5 + (9*i - 9)));
    
    % Set NaN to zero
    jointAngle_X(isnan(jointAngle_X)) = 0;
    jointAngle_Y(isnan(jointAngle_Y)) = 0;
    jointAngle_Z(isnan(jointAngle_Z)) = 0;
    
    % Filter
    jointAngle_X = filtfilt(b,a,jointAngle_X);
    jointAngle_Y = filtfilt(b,a,jointAngle_Y);
    jointAngle_Z = filtfilt(b,a,jointAngle_Z);
    
    d_jointAngle_X = gradient(jointAngle_X)*fs;
    d_jointAngle_Y = gradient(jointAngle_Y)*fs;
    d_jointAngle_Z = gradient(jointAngle_Z)*fs;
    
    d2_jointAngle_X = gradient(d_jointAngle_X)*fs;
    d2_jointAngle_Y = gradient(d_jointAngle_Y)*fs;
    d2_jointAngle_Z = gradient(d_jointAngle_Z)*fs;
    
    % Moment ------------------------------------------------------------------
    jointMoment_X = -D(:,6 + (9*i - 9))/1000;
    jointMoment_Y = -D(:,7 + (9*i - 9))/1000;
    jointMoment_Z = -D(:,8 + (9*i - 9))/1000;
    
    % Set NaN to zero
    jointMoment_X(isnan(jointMoment_X))= 0;
    jointMoment_Y(isnan(jointMoment_Y))= 0;
    jointMoment_Z(isnan(jointMoment_Z))= 0;
    
    % Filter
    jointMoment_X = filtfilt(b,a,jointMoment_X);
    jointMoment_Y = filtfilt(b,a,jointMoment_Y);
    jointMoment_Z = filtfilt(b,a,jointMoment_Z);
    
    d_jointMoment_X = gradient(jointMoment_X)*fs;
    d_jointMoment_Y = gradient(jointMoment_Y)*fs;
    d_jointMoment_Z = gradient(jointMoment_Z)*fs;
    
    d2_jointMoment_X = gradient(d_jointMoment_X)*fs;
    d2_jointMoment_Y = gradient(d_jointMoment_Y)*fs;
    d2_jointMoment_Z = gradient(d_jointMoment_Z)*fs;
    
    % Power (calculate) -------------------------------------------------------
    jointPower_X = d_jointAngle_X.*jointMoment_X;
    jointPower_Y = d_jointAngle_Y.*jointMoment_Y;
    jointPower_Z = d_jointAngle_Z.*jointMoment_Z;
    
    d_jointPower_X = gradient(jointPower_X)*fs;
    d_jointPower_Y = gradient(jointPower_Y)*fs;
    d_jointPower_Z = gradient(jointPower_Z)*fs;
    
    d2_jointPower_X = gradient(d_jointPower_X)*fs;
    d2_jointPower_Y = gradient(d_jointPower_Y)*fs;
    d2_jointPower_Z = gradient(d_jointPower_Z)*fs;
    
    % Store Angle info --------------------------------------------------------
    rtn.id.(labelName{1 + (3*i - 3)}).X = jointAngle_X;
    rtn.id.(labelName{1 + (3*i - 3)}).Y = jointAngle_Y;
    rtn.id.(labelName{1 + (3*i - 3)}).Z = jointAngle_Z;
    
    % 1st time derivative
    rtn.id.(['d_',labelName{1 + (3*i - 3)}]).X = d_jointAngle_X;
    rtn.id.(['d_',labelName{1 + (3*i - 3)}]).Y = d_jointAngle_Y;
    rtn.id.(['d_',labelName{1 + (3*i - 3)}]).Z = d_jointAngle_Z;
    
    % 2nd time derivative
    rtn.id.(['d2_',labelName{1 + (3*i - 3)}]).X = d2_jointAngle_X;
    rtn.id.(['d2_',labelName{1 + (3*i - 3)}]).Y = d2_jointAngle_Y;
    rtn.id.(['d2_',labelName{1 + (3*i - 3)}]).Z = d2_jointAngle_Z;
    
    if(debug)
        figure;
        subplot(311); hold all;
        title(['Saggital Plane ', labelName{1 + (3*i - 3)}],'fontsize',20)
        plot(jointAngle_X,'k')
        subplot(312); hold all;
        plot(d_jointAngle_X,'k')
        subplot(313); hold all;
        plot(d2_jointAngle_X,'k')
    end
    
    % Store Moment info --------------------------------------------------------
    rtn.id.(labelName{2 + (3*i - 3)}).X = jointMoment_X;
    rtn.id.(labelName{2 + (3*i - 3)}).Y = jointMoment_Y;
    rtn.id.(labelName{2 + (3*i - 3)}).Z = jointMoment_Z;
    
    % 1st time derivative
    rtn.id.(['d_',labelName{2 + (3*i - 3)}]).X = d_jointMoment_X;
    rtn.id.(['d_',labelName{2 + (3*i - 3)}]).Y = d_jointMoment_Y;
    rtn.id.(['d_',labelName{2 + (3*i - 3)}]).Z = d_jointMoment_Z;
    
    % 2nd time derivative
    rtn.id.(['d2_',labelName{2 + (3*i - 3)}]).X = d2_jointMoment_X;
    rtn.id.(['d2_',labelName{2 + (3*i - 3)}]).Y = d2_jointMoment_Y;
    rtn.id.(['d2_',labelName{2 + (3*i - 3)}]).Z = d2_jointMoment_Z;
    
    if(debug)
        figure;
        subplot(311); hold all;
        title(['Saggital Plane ', labelName{2 + (3*i - 3)}],'fontsize',20)
        plot(jointMoment_X,'k')
        subplot(312); hold all;
        plot(d_jointMoment_X,'k')
        subplot(313); hold all;
        plot(d2_jointMoment_X,'k')
    end
    
    % Store Power info --------------------------------------------------------
    rtn.id.(labelName{3 + (3*i - 3)}).X = jointPower_X;
    rtn.id.(labelName{3 + (3*i - 3)}).Y = jointPower_Y;
    rtn.id.(labelName{3 + (3*i - 3)}).Z = jointPower_Z;
    
    % 1st time derivative
    rtn.id.(['d_',labelName{3 + (3*i - 3)}]).X = d_jointPower_X;
    rtn.id.(['d_',labelName{3 + (3*i - 3)}]).Y = d_jointPower_Y;
    rtn.id.(['d_',labelName{3 + (3*i - 3)}]).Z = d_jointPower_Z;
    
    % 2nd time derivative
    rtn.id.(['d2_',labelName{3 + (3*i - 3)}]).X = d2_jointPower_X;
    rtn.id.(['d2_',labelName{3 + (3*i - 3)}]).Y = d2_jointPower_Y;
    rtn.id.(['d2_',labelName{3 + (3*i - 3)}]).Z = d2_jointPower_Z;
    
    if(debug)
        figure;
        subplot(311); hold all;
        title(['Saggital Plane ', labelName{3 + (3*i - 3)}],'fontsize',20)
        plot(jointPower_X,'k')
        subplot(312); hold all;
        plot(d_jointPower_X,'k')
        subplot(313); hold all;
        plot(d2_jointPower_X,'k')
    end
    
end

% Marker Trajectories
% skip line
tline = fgetl(fid);

% skip line
tline = fgetl(fid);

% parse labels and create struct fields names
tline = fgetl(fid);
labels = textscan(tline, '%s', 'delimiter', ',');
[m, n] = size(labels{1});
labelCnt = 0;
for i=1:m
    if ~(strcmp(labels{1}{i},''))
        % pasre header on label
        [token, remain] = strtok(labels{1}{i},':');
        labelCnt = labelCnt + 1;
        labelName{labelCnt} = remain(2:end);
    end
end

% find number of subfields
tline = fgetl(fid);
sublabels = textscan(tline, '%s', 'delimiter', ',');
[m, n] = size(sublabels{1});

% skip line of units
fgetl(fid);

% put data in matrix
D = zeros(1,m);
formatSpec = [repmat(['%f'],1,m-1), '%f\n'];
rowCount = 1;
while(1)
    C = textscan(fid, formatSpec, 'delimiter', ',');
    dataRow = cell2mat(C);
    if isempty(dataRow)
        break;
    end
    D(rowCount,:) = cell2mat(C);
    rowCount = rowCount + 1;
end

% put data (X,Y,Z subfields per field)
for i=1:labelCnt
    rtn.markers.(labelName{i}).X = D(:,3 + (3*i - 3));
    rtn.markers.(labelName{i}).Y = D(:,4 + (3*i - 3));
    rtn.markers.(labelName{i}).Z = D(:,5 + (3*i - 3));
end

fclose(fid);
end
