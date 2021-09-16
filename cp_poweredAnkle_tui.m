function rtn = cp_poweredAnkle_tui(varargin)
%%
% *** Restore marker trajectories and linear map encoder values ***

%%
% cp_poweredAnkle_tui  Text based user interface for iterative learning control.
%
%   cp_poweredAnkle_tui() default configuration.
%
%   cp_poweredAnkle_tui(Name,Value, ...) Name, Value pairs for adjustable settings.
%

% Count number of input arguments
nVarArgs = length(varargin);


% Defauts Settings
processVicon = 1;

% algorithm setting
gain = 0.5; % 1
maxharmonic = 10; % 10
smooth_vector = [90,5];
zeta = 1; % 1.5
alpha = 0.5;
adapt = 'gain';
model = 'average';
backstepping = 'off';

% Motor 3 settings
k_t = 10.2/1000;
k_s = 940;
Vcc = 60;
R_T = 1800;
maxMotorAmp = 14;
maxMotorRpm = 80000;
file_suffix = '';

% Zero-phase filter settings
gaitCycle = 0.1:0.1:100;
N = numel(gaitCycle);
fc_norm = (2*10)/N;

% Collect Arguments
% Override default settings when applicable
for i=1:2:nVarArgs
    switch varargin{i}
        case 'gain'
            gain = varargin{i+1};
        case 'maxharmonic'
            maxharmonic = varargin{i+1}
        case 'fc_norm'
            fc_norm = varargin{i+1}
        case 'smooth_vector'
            smooth_vector = varargin{i+1};
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
        otherwise
            fprintf('\n%s option not found!\n',varargin{i})
            rtn = [];
            return
    end
end

%----------------------------------------------------------------------------
% Initialize Plots
% ---------------------------------------------------------------------------
fig = figure;

% |E_k| v freq.
subplot(3,3,4); hold all;
h_E_bar_v_f_p_gamma = stem(0:maxharmonic, 0.*(0:maxharmonic),'--r');
h_E_bar_v_f = stem(0:maxharmonic, 0.*(0:maxharmonic),'k');
h_E_v_f = stem((0:maxharmonic)+0.2, 0.*(0:maxharmonic),'m');
ylabel('$| E |$','fontsize',17,'interpreter','latex');
xlabel('Harmonic','fontsize',17,'interpreter','latex');
grid on; box on;
xlim([-0.3,maxharmonic+0.5])
l = legend([h_E_bar_v_f,h_E_bar_v_f_p_gamma,h_E_v_f], ...
    {'$E^*_k$','$E^*_k + \gamma$', '$E_k$'});
set(l,'interpreter','latex', ...
    'Position', [0.0446 0.57 0.0571  0.0726],...
    'FontSize',10);


% |U_k| v freq.
subplot(3,3,7), hold all;
h_U_bar_v_f = stem(0:maxharmonic, 0.*(0:maxharmonic),'k');
h_U_v_f = stem((0:maxharmonic)+0.2, 0.*(0:maxharmonic),'m');
h_U_kp1_v_f = stem((0:maxharmonic)-0.2, 0.*(0:maxharmonic),'b');
ylabel('$| U |$','fontsize',17,'interpreter','latex');
xlabel('Harmonic','fontsize',17,'interpreter','latex');
grid on; box on;
xlim([-0.3,maxharmonic+0.5])
l = legend([h_U_bar_v_f,h_U_v_f,h_U_kp1_v_f], ...
    {'$U^*_k$','$U_k$','$U_{k+1}$'});
set(l,'interpreter','latex', ...
    'Position',[0.0581    0.2779    0.0400    0.0500],...
    'FontSize',10);



% rho v freq
subplot(3,3,1); hold all;
h_rho_v_f = stem(0:maxharmonic, 0.*(0:maxharmonic),'k');
ylabel('$\rho$','fontsize',17,'interpreter','latex');
xlabel('Harmonic','fontsize',17,'interpreter','latex');
grid on; box on;
xlim([-0.3,maxharmonic+0.5])

% y v t
subplot(3,3,2:3); hold all;
h_title = title(['Iteration = ', num2str(0)], ...
    'interpreter','latex','fontsize',17);
h_yd_v_t = plot(gaitCycle,0.*gaitCycle,'k');
h_y_v_t = plot(gaitCycle,0.*gaitCycle,'r');
h_u_k_v_t = plot(gaitCycle,0.*gaitCycle,'g');
h_u_kp1_v_t = plot(gaitCycle,0.*gaitCycle,'c');
h_e_kp1_v_t = plot(gaitCycle,0.*gaitCycle,'m');
h_stance_1_v_t = plot([0 0],[0,0],'--k');
h_pre_1_v_t = plot([0 0],[0,0],'--k');
ylabel('(rad)','fontsize',17,'interpreter','latex');
grid on; box on
l = legend([h_y_v_t,h_yd_v_t,h_u_k_v_t,h_u_kp1_v_t,h_e_kp1_v_t], ...
    {'$\tilde{\theta}_{p,k}$','$\tilde{\theta}_{b,k}$', ...
    '$\tilde{\theta}_{v,k}$', ...
    '$\tilde{\theta}_{v,k+1}$', ...
    '$\theta_{v,k+1}$'});
set(l,'interpreter','latex', ...
    'Position', [0.9122,0.7228,0.0742,0.22],...
    'FontSize',14);


% u v t
subplot(3,3,5:6); hold all;
title('$U_{k+1} = U^*_k + \rho E^*_k$','interpreter','latex')
h_e_v_t = plot(gaitCycle,0.*gaitCycle,'b');
h_u_v_t = plot(gaitCycle,0.*gaitCycle,'g');
h_u_filt_v_t = plot(gaitCycle,0.*gaitCycle,'m');
h_stance_2_v_t = plot([0 0],[0,0],'--k');
h_pre_2_v_t = plot([0 0],[0,0],'--k');
xlabel('\% gait','fontsize',17,'interpreter','latex');
ylabel('(deg)','fontsize',17,'interpreter','latex');
grid on; box on;

l = legend([h_e_v_t,h_u_v_t,h_u_filt_v_t], ...
    {'$\theta_{e,k}$','$\theta_{v,k+1}$','$\hat{\theta}_{v,k+1}$'});
set(l,'interpreter','latex', ...
    'Position',[0.9139,0.4661,0.0742,0.1233],...
    'FontSize',14);


% e v t | f
subplot(3,3,8:9); hold all
h_er_inf_t = plot(0,0,'k');
h_er_2_t = plot(0,0,'r');
h_er_inf_f = plot(0,0,'--b');
h_er_2_f = plot(0,0,'--g');
h_er_inf_t_min = plot(0,0,'ok');
h_er_2_t_min = plot(0,0,'or');
h_er_inf_f_min = plot(0,0,'ob');
h_er_2_f_min = plot(0,0,'og');
xlabel('Iteration','fontsize',17,'interpreter','latex');
ylabel('Error','fontsize',17,'interpreter','latex')
grid on; box on;
l = legend('$\|e\|_\infty$','$\|e\|_2$','$\|E\|_\infty$','$\|E\|_2$');
set(l,'interpreter','latex','box','off',...
    'FontSize',14);

fig.Position = [3 61 1426 738];
fig.PaperPosition = [-5.3542,1.3542,19.2083,8.2917];

% Filter for uff smoothing
[b,a] = butter(1,fc_norm);

% Collect settings
settings.maxharmonic = maxharmonic;
settings.gain = gain;
settings.fc_norm = fc_norm;
rtn.settings = settings;

fprintf('\n----------------------\n')
fprintf('Iterative Learning UI\n')
fprintf('-----------------------\n')
fprintf('\nSettings:\n')

fprintf('\tMaximum harmonic = %i\n',maxharmonic)
fprintf('\tLearning Gain = %.3f\n',gain)
fprintf('\tNormalized Cutoff Freq = %.3f\n',fc_norm)

% Error Vectors
Einf_t = [];
E2_t = [];
Einf_f = [];
E2_f = [];

% ILC Settings
maxHarmonic = maxharmonic;
initial_learning_gain = gain;

% Get directory
path = 'C:\Users\cpras\Documents\UW\Thesis\Testing';
% selpath = uigetdir(path); CHANGE
selpath = uigetdir(pwd);

% Learning Loop
k=1;
while(1)
    fprintf('\n................\n')
    fprintf('Iteration %i\n',k - 1);
    fprintf('................\n')
    
    % Get k trial
    while(1)
        fprintf('\n\t');
        trialName = input('Enter trial name: ','s');
        fprintf(['\n\t',trialName]);
        usr_input = input(' - Is this correct? [y/n] ','s');
        if strcmp('y',usr_input)
            break;
        end
    end
    
    % Data Collection
    trial  = fullfile(selpath, trialName);
    fprintf('\n\tParsing data...\n');
    rtn.T{k} = cp_process_trial(trial, ...
        'k_t',k_t, ...
        'k_s',k_s, ...
        'Vcc',Vcc, ...
        'R_T',R_T, ...
        'maxMotorAmp',maxMotorAmp, ...
        'maxMotorRpm',maxMotorRpm,...
        'processVicon',processVicon);
    
    % Get info on k=0 iteration
    if (k==1)
        
        % Which leg is prosthetic on?
        prostheticSide = rtn.T{k}.params.emb.Is_Prosthetic_Right;
        if prostheticSide
            prostheticSide = 'right';
        else
            prostheticSide = 'left';
        end
        % mass = rtn.T{k}.params.emb.Total_Mass; % CHANGE
        % rtn.settings.mass = mass; % CHANGE
        rtn.settings.prostheticSide = prostheticSide;        
        fs = rtn.T{k}.params.model.fs;
        
        fprintf('\tProsthetic Side = %s\n',prostheticSide);
        % fprintf('\tMass = %i\n',mass)
    end
    
    time = rtn.T{k}.time;
    grf.time = rtn.T{k}.grf.time;
    emb.time = rtn.T{k}.emb.time;
    
    % Desired output (biological) and real output (prosthetic)
    if strcmp(prostheticSide,'left')
        
        % Mean Vicon Kinematics
        yd_k = rtn.T{k}.gait.model.r.RAnkleAngles.X(:,2);
        y_k = rtn.T{k}.gait.model.l.LAnkleAngles.X(:,2);
        
        % all normalized trajectories
        yd_k_all = rtn.T{k}.gait.model.normalized.r.RAnkleAngles.X;
        y_k_all = rtn.T{k}.gait.model.normalized.l.LAnkleAngles.X;
        
        % Timing
        Tp = rtn.T{k}.segment.grf.l.gait.time(:,2) ...
            - rtn.T{k}.segment.grf.l.gait.time(:,1);
        
        stance_percent = mean((rtn.T{k}.segment.grf.l.stance.time(:,2) ...
            - rtn.T{k}.segment.grf.l.stance.time(:,1))./Tp).*100;
        
        % HS events
        grf.hs = rtn.T{k}.ge.grf.l.hs_time;
        emb.hs = rtn.T{k}.ge.emb.l.hs_time;
        
        % HS event indices
        grf.hs_ind = rtn.T{k}.ge.grf.l.hs_ind;
        emb.hs_ind = rtn.T{k}.ge.emb.l.hs_ind;
        
        % Torques
        tau_p = rtn.T{k}.gait.model.l.LAnkleMoment.X(:,2);
        tau_b = rtn.T{k}.gait.model.r.RAnkleMoment.X(:,2);
        
        % Encoder
        theta_e = rtn.T{k}.gait.emb.l.ankPos(2:end,2);
        
    else
        
        % mean torques
        yd_k = rtn.T{k}.gait.model.l.LAnkleAngles.X(:,2);
        y_k = rtn.T{k}.gait.model.r.RAnkleAngles.X(:,2);
        
        % all normalized trajectories
        yd_k_all = rtn.T{k}.gait.model.normalized.l.LAnkleAngles.X;
        y_k_all = rtn.T{k}.gait.model.normalized.r.RAnkleAngles.X;
        
        
        % Timing
        Tp = rtn.T{k}.segment.grf.r.gait.time(:,2) ...
            - rtn.T{k}.segment.grf.r.gait.time(:,1);
        
        stance_percent = mean((rtn.T{k}.segment.grf.r.stance.time(:,2) ...
            - rtn.T{k}.segment.grf.r.stance.time(:,1))./Tp).*100;
        
        % HS events
        grf.hs = rtn.T{k}.ge.grf.r.hs_time;
        emb.hs = rtn.T{k}.ge.emb.r.hs_time;
        
        % HS event indices
        grf.hs_ind = rtn.T{k}.ge.grf.r.hs_ind;
        emb.hs_ind = rtn.T{k}.ge.emb.r.hs_ind;
        
        % Torques
        tau_p = rtn.T{k}.gait.model.r.RAnkleMoment.X(:,2);
        tau_b = rtn.T{k}.gait.model.l.LAnkleMoment.X(:,2);
        
        % Encoder
        theta_e = rtn.T{k}.gait.emb.r.ankPos(2:end,2);
        
    end
    
    % Remove first point (0% == 100%) since we want N even for FFT
    yd_k = yd_k(2:end);
    y_k = y_k(2:end);
    
    while(1)
        
        % If first iteration, initialize learning
        if (k==1)
            fprintf('\n\tInitializing learning structures...\n');
            
            % First iteration, find y_0 std in freq. domain
            L = numel(y_k_all(2:end,1));
            f = (0:(L/2));
            Y_k_all = fft(y_k_all(2:end,:));
            Y_k_all = Y_k_all(1:(L/2)+1,:);
            Y_k_abs_mean = mean(abs(Y_k_all)')';
            Y_k_abs_std = std(abs(Y_k_all)')';
            
            % epsilon = 3.*Y_k_abs_std;
            epsilon = 0.*Y_k_abs_std;
            
            figure;
            title('Standard deviation of output','fontsize',20);
            plot(f,epsilon,'ok');
            xlabel('Harmonic','fontsize',20);
            ylabel('STD','fontsize',20)
            xlim([0,maxharmonic])
            
            pause
            
            rtn.S{k} = cp_ilc(yd_k, y_k, 0, ...
                'init','gain',initial_learning_gain, ...
                'epsilon', epsilon,...
                'model',model,...
                'adapt',adapt,...
                'backstepping',backstepping,...
                'maxHarm',maxHarmonic, ...
                'rho_max',inf, ...
                'zeta',zeta, ...
                'alpha',alpha, ...
                'fs', fs);
            
            % 'epsilon', 0,...
            
        else
            fprintf('\n\tLearning...\n');
            rtn.S{k} = cp_ilc(yd_k,y_k,rtn.S{k-1});
            
        end
        
        if(0)
            figure; hold all;
            plot(time,y);
            for i=1:numel(grf.hs)
                plot([grf.hs(i) grf.hs(i)], [min(y) max(y)],'-r')
            end
            for i=1:numel(emb.hs)
                plot([emb.hs(i) emb.hs(i)], [min(y) max(y)],'--r')
            end
        end
        
        % Find HS delay
        if (numel(grf.hs) ~= numel(emb.hs))
            fprintf('\t\t\tHS mismatch, cannot calculate delay');
            est_delay = nan;
        else
            est_delay = mean(emb.hs - grf.hs);
        end
        
        smooth_vector(1) = (stance_percent + 20);
        Tp_mean = mean(Tp);
        
        fprintf('\n\t\tTiming Info\n')
        fprintf('\t\t-------------\n')
        fprintf('\t\tEst. HS delay = %.3f (s), %.3f%%\n', ...
            est_delay, (est_delay/Tp_mean).*100)
        fprintf('\t\tTp = %.3f (s)\n',Tp_mean)
        fprintf('\t\tInital Smoothing = %.2f%%\n',smooth_vector(2))
        fprintf('\t\tStance = %.2f%%\n',smooth_vector(1))
        
        % Store Timing Info
        info.Tp_mean = Tp_mean;
        info.stance_percent = stance_percent;
        info.smooth_vector = smooth_vector;
        
        % Store the errors
        Einf_t(k) = rtn.S{k}.e_k_inf;
        E2_t(k) = rtn.S{k}.e_k_2;
        Einf_f(k) = rtn.S{k}.E_k_inf;
        E2_f(k) = rtn.S{k}.E_k_2;
        
        [~,iinf_t] = min(Einf_t(1:k)./Einf_t(1));
        [~,i2_t] = min(E2_t(1:k)./E2_t(1));
        [~,iinf_f] = min(Einf_f(1:k)./Einf_f(1));
        [~,i2_f] = min(E2_f(1:k)./E2_f(1));
        
        % Train MoCap to Encoder Mapper
        pred = [];
        resp = [];
        % -- Collect training data for all trials collected up to this point
        if strcmp(prostheticSide,'left')
            for ii = 1:k
                pred = [pred; rtn.T{ii}.model.LAnkleAngles.X];
                resp = [resp; interp1(rtn.T{ii}.emb.time,...
                rtn.T{ii}.emb.Ankle_Encoder_Angle, rtn.T{ii}.time)]; % CHANGE
                % resp = [resp; (1.2*rtn.T{ii}.model.LAnkleAngles.X)-0.05];
            end
        else
            for ii = 1:k
                pred = [pred; rtn.T{ii}.model.RAnkleAngles.X];
                resp = [resp; interp1(rtn.T{ii}.emb.time,...
                rtn.T{ii}.emb.Ankle_Encoder_Angle, rtn.T{ii}.time)]; % CHANGE
                % resp = [resp; (1.2*rtn.T{ii}.model.RAnkleAngles.X)-0.05];
            end
        end
        mdl = fitlm(pred, resp, 'linear', 'RobustOpts', 'off');
        if(0)
            ypred = predict(mdl,pred);
            figure; hold all;
            plot(resp)
            plot(ypred)
            legend('MoCap','Encoder','location','best')
        end
        theta_v = rtn.S{k}.u_kp1;
        thetaV_mapped = predict(mdl, theta_v);
        rtn.S{k}.u_kp1_e = thetaV_mapped;
        
        temp = fft(rtn.S{k}.u_kp1_e);
        rtn.S{k}.U_kp1_e = temp(1:(1000/2+1));
        
        % CHANGE
%         % Torque Check
%         if(0)
%             tau_ref = -mass*(theta_p_emb(2:end)- thetaV_mapped);
%             
%             figure()
%             plot(gaitCycle(2:end), tau_ref, 'r--')
%             hold on
%             plot(gaitCycle(2:end), tau_b(2:end)*mass, 'm')
%             plot(gaitCycle(2:end), tau_p(2:end)*mass, 'b')
%             plot(gaitCycle(2:end), tau_p(2:end)*mass + tau_ref, 'r')
%             xlabel('Gait Percent [%]')
%             ylabel('Ankle Torque [Nm]')
%             legend('Active', 'Biological (Vicon)','Prosthetic (Vicon)', ...
%                 'Prosthetic + Active','location', 'best')
%             title({'Desired Torque Trajectory and Vicon Torques',['Gain = ',num2str(-mass)]})
%         end
        
        % Phase Variable
        thigh_angle = rtn.T{k}.gait.emb.l.thigh_angle(2:end,2);
        Phi = cp_PHI(thigh_angle,gaitCycle);
        if(0)
            figure()
            plot(gaitCycle(2:end), Phi, '.')
            xlabel('Time [s]'); ylabel('\Phi', 'FontSize', 12);
            title('Mean Thigh Angle-Integral Phase Variable')
        end
        rtn.S{k}.phase.Mean_Phi = Phi;
        
        % Collected Phase Variable vs Offline Computation
        th = rtn.T{k}.emb.Thigh_Angle;
        t = rtn.T{k}.emb.time;
        ind = interp1(t,1:length(t),grf.hs,'nearest'); % HS indices (grf events --> emb times)
        
        % offline
        phi = zeros(length(ind(1):ind(end)),1);
        for i=1:numel(grf.hs)-1
            range = ind(i):ind(i+1);
            phi(range-ind(1)+1) = cp_PHI(th(range),t(range));
        end
        phi_meas = rtn.T{k}.emb.Phi(ind(1):ind(end)); % online
        
        if(0)
            t_r = t(ind(1):ind(end));
            figure(); plot(t_r,phi); hold on; plot(t_r,phi_meas)
            xlabel('Time [s]'); ylabel('Normalized Phase Angle')
            legend('Offline','Online')
        end
        rtn.S{k}.phase.Online_Phi = phi_meas;
        rtn.S{k}.phase.Offline_Phi = phi;
        
        % Filtering
        preFiltU_kp1 = rtn.S{k}.U_kp1;
        preFiltU_kp1_e = rtn.S{k}.U_kp1_e;
        
        % Filter uff  [rad]
        u_kp1_filt = rtn.S{k}.u_kp1;
        u_kp1_filt(1:(round(smooth_vector(2)*10)+1)) = 0;
        u_kp1_filt((round(smooth_vector(1)*10)+1):end) = 0;
        u_kp1_filt = filtfilt(b,a,u_kp1_filt);
        rtn.S{k}.u_kp1_filt = u_kp1_filt;
        % Store in frequency domain
        temp = fft(u_kp1_filt);
        rtn.S{k}.U_kp1 = temp(1:(1000/2+1));
        
        % Filter mapped uff  [rad]
        u_kp1_e_filt = rtn.S{k}.u_kp1_e;
        u_kp1_e_filt(1:(round(smooth_vector(2)*10)+1)) = 0;
        u_kp1_e_filt((round(smooth_vector(1)*10)+1):end) = 0;
        u_kp1_e_filt = filtfilt(b,a,u_kp1_e_filt);
        rtn.S{k}.u_kp1_e_filt = u_kp1_e_filt;
        % Store in frequency domain
        temp = fft(u_kp1_e_filt);
        rtn.S{k}.U_kp1_e = temp(1:(1000/2+1));
        
        if(0)
            figure; hold all;
            stem(abs(rtn.S{k}.U_kp1_e), '-k')
            stem(abs(preFiltU_kp1_e), '--r')
        end
        
        % update plots
        set(h_title,'string',['Iteration = ', num2str(k-1)]);
        set(h_e_v_t,'YData',rad2deg(theta_e)); % encoder angle (2,2) -- blue
        set(h_u_v_t,'YData',rad2deg(rtn.S{k}.u_kp1_e)); % virtual @ k+1 (2,2) -- green
        set(h_u_filt_v_t,'YData',rad2deg(rtn.S{k}.u_kp1_e_filt)); % filtered virtual @ k+1 (2,2) - purple
        set(h_stance_2_v_t,'XData',[smooth_vector(1),smooth_vector(1)], ...
            'YData',[min(rad2deg(rtn.S{k}.u_kp1_e_filt)),max(rad2deg(rtn.S{k}.u_kp1_e_filt))]);
        set(h_pre_2_v_t,'XData',[smooth_vector(2),smooth_vector(2)], ...
            'YData',[min(rad2deg(rtn.S{k}.u_kp1_e_filt)),max(rad2deg(rtn.S{k}.u_kp1_e_filt))]);
        
        set(h_e_kp1_v_t,'YData',rtn.S{k}.u_kp1_e_filt) % filtered virtual @ k+1 (2,1) -- purple
        
        if k > 1
            set(h_u_k_v_t, 'YData',rtn.S{k-1}.u_kp1_filt); % previous filtered virtual (2,1) -- green
            set(h_U_v_f,'YData',abs(rtn.S{k-1}.U_kp1(1:(maxharmonic+1))));
        end
        
        set(h_U_kp1_v_f,'YData',abs(rtn.S{k}.U_kp1(1:(maxharmonic+1)))); % (1,3)
        
        set(h_U_bar_v_f,'YData',abs(rtn.S{k}.U_bar_k(1:(maxharmonic+1)))); % (1,3)
        set(h_E_bar_v_f,'YData',abs(rtn.S{k}.E_bar_k(1:(maxharmonic+1)))); % (1,2)
        set(h_E_bar_v_f_p_gamma,'YData', ...
            abs(rtn.S{k}.E_bar_k(1:(maxharmonic+1))) ...
            + rtn.S{k}.params.epsilon(1:(maxharmonic+1)))  % (1,2)
        set(h_y_v_t,'YData', rtn.S{k}.y_k);  % (2,1)
        set(h_yd_v_t,'YData',rtn.S{k}.yd_k); % (2,1)
        
        set(h_u_kp1_v_t,'YData', rtn.S{k}.u_kp1_filt); % filtered virtual @ k+1 (2,1) -- cyan
        set(h_stance_1_v_t,'XData',[smooth_vector(1),smooth_vector(1)], ...
            'YData',[min(rtn.S{k}.yd_k),max(rtn.S{k}.yd_k)]);
        set(h_pre_1_v_t,'XData',[smooth_vector(2),smooth_vector(2)], ...
            'YData',[min(rtn.S{k}.yd_k),max(rtn.S{k}.yd_k)]);
        
        set(h_er_inf_t_min,'XData',iinf_t-1,'YData',Einf_t(iinf_t)./Einf_t(1)); % (2,3)
        set(h_er_2_t_min,'XData',i2_t-1,'YData',E2_t(i2_t)./E2_t(1));  % (2,3)
        set(h_er_inf_f_min,'XData',iinf_f-1,'YData',Einf_f(iinf_f)./Einf_f(1)); % (2,3)
        set(h_er_2_f_min,'XData',i2_f-1,'YData',E2_f(i2_f)./E2_f(1)); % (2,3)
        
        set(h_er_inf_t,'XData',(1:k)-1,'YData',Einf_t(1:k)./Einf_t(1));
        set(h_er_2_t,'XData',(1:k)-1,'YData',E2_t(1:k)./E2_t(1));
        set(h_er_inf_f,'XData',(1:k)-1,'YData',Einf_f(1:k)./Einf_f(1));
        set(h_er_2_f,'XData',(1:k)-1,'YData',E2_f(1:k)./E2_f(1));
        
        set(h_rho_v_f,'YData',abs(rtn.S{k}.rho_k(1:(maxharmonic+1))));
        set(h_E_v_f,'YData',abs(rtn.S{k}.E_k(1:(maxharmonic+1))));
        
        % Promt signal sufficient message
        fprintf('\n\tSignal learned!\n');
        fprintf('\t\tIs this signal sufficient?')
        usr_input = input('[y/n]', 's');
        if strcmp(usr_input,'n')
            % Make adjustments to learning
            while(1)
                fprintf('\t\tDo you want to change a parameter?\n')
                fprintf('\t\t 1 - gain\n')
                fprintf('\t\t 2 - earlysmooth\n')
                fprintf('\t\t 3 - latesmooth\n');
                fprintf('\t\t 4 - done\n')
                while(1)
                    fprintf('\t\t\t');
                    usr_input = input('Enter response: ');
                    if isscalar(usr_input)
                        break;
                    end
                end
                
                switch usr_input
                    case 1
                        fprintf('\t\t\t');
                        usr_input = input('Enter new gain value: ');
                        gain = usr_input;
                    case 2
                        fprintf('\t\t\t');
                        usr_input = input('Enter new earlysmooth: ');
                        smooth_vector(2) = usr_input;
                    case 3
                        fprintf('\t\t\t');
                        usr_input = input('Enter new latesmooth: ');
                        smooth_vector(1) = usr_input;
                    case 4
                        fprintf('\tLearning Gain = %.3f\n',gain)
                        fprintf('\tSmoothing = %i%% - %i%%\n', smooth_vector)
                        break;
                    otherwise
                        warning('Not an option');
                end
            end
        else
            break;
        end
    end
    
    rtn.S{k}.info = info;
    
    % End of Iteration Promt
    fprintf('\n\t');
    fprintf('Write feedforward signal to file?');
    usr_input = input('[y/n]','s');
    fprintf('\n');
    
    % Write lookup tables
    if strcmp(usr_input,'y')
        writeFlag = 1;
        file = ['./','uff_',file_suffix,num2str(k)];
        if exist(file,'file') == 2
            fprintf('\n\t\tFile already exist');
            usr_input = input(' overwrite? [y/n]','s');
            if strcmp('y',usr_input)
                writeFlag = 1;
                fid = fopen(file,'w');
                fprintf(fid,'%f\n',rtn.S{k}.u_kp1_e_filt);
                fclose(fid);
            else
                writeFlag = 0;
                fprintf('\n\tSignal not written to file.\n');
            end
        end
        if writeFlag
            folder = 'D:\';
            % Mean Phi
            writematrix(reshape(Phi, 1, []),[folder,'meanPhi.csv'],'Delimiter','comma')
            % Desired Theta
            writematrix(reshape(rtn.S{k}.u_kp1_e_filt, 1, []),[folder,'thetaV.csv'],'Delimiter','comma')
        end
    end
    
    fprintf('\n\t');
    usr_input = input('Continue with next k? [y/n]','s');
    if strcmp('n',usr_input)
        fprintf('\t\t')
        usr_input = input('Redo k? [y/n] (n will exit ui)','s');
        if strcmp('y',usr_input)
            continue;
        end
        break;
    end
    
    % Increment
    k = k + 1;
end
end


