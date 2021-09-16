function S_k = cp_ilc_edit(yd_k, y_k, S_km1, varargin)
% adaptiveILC Iterative learning control with adaptive learning gains.
%
% name-value pairs:
%
% 'model' options:
%          'off'
%          'interp' use G_inv (not working currently)
%          'fly' use U_k / Y_k
%          'average' average model
%          'error-inversion' invert with error signal (not currently working)
%
% 'adapt' options:
%          'off' no adapting
%          'gain' adapt learning gain only
%          'gain-phase' adapt learning gain and phase
%          'data-based' invert and update gain using error signal
%
% 'backstepping' options:
%          'off'
%          'on' use 'best-so-far' in update law
%
%   Use:
%
%   First iteration:
%   S_0 = ILC(yd_k, y_k, gain_k, 0, 'init', maxHarm)
%
%
%   All others:
%   S_k = ILC(yd_k, y_k, gain_k, S_km1)
%
%                         yd_k    -  k_th desired output
%                         y_k     -  k_th output
%                         epsilon -  error buffer (use variance of output k=0)
%                         gain_k  -  learning weight
%                         S_km1   -  {k-1}_th stucture
%
%                         S_k : Yd_k     - k_th desired output
%                               yd_k     - k_th desired output (time domain)
%                               Y_k      - k_th ouput
%                               y_k      - k_th ouput (time domain)
%                               rho_k    - k_th learning gains
%                               E_bar_k  - k_th update error
%                               U_bar_k  - k_th update input
%                               E_k      - k_th error
%                               U_kp1    - (k+1)_th input
%                               u_kp1    - (k+1)_th input (time domain)
%                               e_k_1    - k_th norm(yd_k - y_k,1)
%                               e_k_2    - k_th norm(yd_k - y_k,2)
%
% Example:
%   S_0 = ILC(yd_0, y_0, gain_k, 0, 'init', maxHarm);
%   S_1 = ILC(yd_1, y_1, gain_k, S_0);
%   S_2 = ILC(yd_2, y_2, gain_k, S_1);
%   .
%   .
%   .
%   S_k = ILC(yd_k, y_k, gain_k, S_km1);
%

% Inputs
L = numel(yd_k);

f = (0:(L/2))/L;
f = f(:);

Yd_k = fft(yd_k);
Yd_k = Yd_k(1:(floor(L/2)+1));

Y_k = fft(y_k);
Y_k = Y_k(1:(floor(L/2)+1));

% Default Options
init = 0;
adapt = 'off';                % adaption technique
model = 'off';                % model technique
backstepping = 'off';         % backstepping
gain = 1;                     % gain
epsilon = 1e-10;                  % padding
maxHarm = numel(L)-1;         % maximum learning harmonic
alpha = 0.5;                  % alpha - rate of learing decrease
zeta = 1 + alpha*0.75;        % zeta - rate of learning increase
phi = 45;                     % phase shift
rho_max = Inf;                  % max rho
fs = 1000;
init_struct = [];

if (nargin > 3)
    nVarArgs = length(varargin);
    if strcmp(varargin{1},'init')
        init = 1;
        for j=2:2:nVarArgs
            switch varargin{j}
                case 'gain'
                    gain = varargin{j+1};
                case 'alpha'
                    alpha = varargin{j+1};
                case 'zeta'
                    zeta = varargin{j+1};
                case 'phi'
                    phi = varargin{j+1};
                case 'epsilon'
                    epsilon = varargin{j+1};
                case 'maxHarm'
                    maxHarm = varargin{j+1};
                case 'model'
                    model = varargin{j+1};
                case 'adapt'
                    adapt = varargin{j+1};
                case 'backstepping'
                    backstepping = varargin{j+1};
                case 'rho_max'
                    rho_max = varargin{j+1};
                case 'G'
                    temp = varargin{j+1};
                    G = temp(:,1);
                    f_G = temp(:,2);
                case 'fs'
                    fs = varargin{j+1};
            end
        end
        fprintf('Adaptive ILC parameters:\n');
        fprintf('\t\tmax Harmonic = %i\n',maxHarm);
        fprintf('\t\tgain = %i\n',gain);
        fprintf('\t\talpha = %i\n',alpha);
        fprintf('\t\tzeta = %i\n',zeta);
        fprintf('\t\trho_max = %i\n',rho_max);
        fprintf('\t\tmodel = %s\n',model);
        fprintf('\t\tadapt = %s\n',adapt);
        fprintf('\t\tbackstepping = %s\n',backstepping);
        
        % Initialize rho
        rho_k = gain.*ones(numel(f),1);
        rho_k((maxHarm+2):end) = 0;
        rho_ang_bar_k  = 0.*f;
        
        % Error
        E_k = Yd_k - Y_k;
        e_k_inf = norm(yd_k - y_k,inf);
        e_k_2 = norm(yd_k - y_k,2);
        E_k_inf = norm(Yd_k - Y_k,inf);
        E_k_2 = norm(Yd_k - Y_k,2);
        
        if strcmp(model,'interp')
            % interpolate model
            G = interp1(f_G.*fs,abs(G),f.*fs,'pchip') ...
                .* exp(1j.*interp1(f_G.*fs,angle(G),f.*fs,'pchip'));
            G_inv_k = 1 ./ G;
        else
            G_inv_k = 1;
        end
        
        % First control signal
        U_kp1 = rho_k .* G_inv_k .* E_k;
        u_kp1 = real(ifft([U_kp1; conj(flipud(U_kp1(2:end-1)))]));
        
        S_k.params.model = model;
        S_k.params.adapt = adapt;
        S_k.params.backstepping = backstepping;
        S_k.params.fs = fs;
        S_k.params.f = f;
        S_k.params.maxHarm = maxHarm;
        S_k.params.epsilon = epsilon;
        S_k.params.gain = gain;
        S_k.params.rho_max = rho_max;
        S_k.params.zeta = zeta;
        S_k.params.alpha = alpha;
        S_k.G_inv_model = 1;
        S_k.G_inv_k = G_inv_k;
        S_k.G_inv_km1 = G_inv_k;
        S_k.k = 0;
        S_k.Yd_k = Yd_k;
        S_k.yd_k = yd_k;
        S_k.Y_k = Y_k;
        S_k.y_k = y_k;
        S_k.rho_k = rho_k;
        S_k.E_bar_k = E_k;
        S_k.E_hat_k = E_k;
        S_k.U_bar_k = zeros(numel(f),1);
        S_k.E_k = E_k;
        S_k.U_kp1 = U_kp1;
        S_k.u_kp1 = u_kp1;
        S_k.e_k_inf = e_k_inf;
        S_k.e_k_2 = e_k_2;
        S_k.E_k_inf = E_k_inf;
        S_k.E_k_2 = E_k_2;
        return
    end
else
    epsilon = S_km1.params.epsilon;
    gain = S_km1.params.gain;
    rho_max = S_km1.params.rho_max;
    zeta = S_km1.params.zeta;
    alpha = S_km1.params.alpha;
end

E_k = Yd_k - Y_k;
e_k_inf = norm(yd_k - y_k,inf);
e_k_2 = norm(yd_k - y_k,2);
E_k_inf = norm(Yd_k - Y_k,inf);
E_k_2 = norm(Yd_k - Y_k,2);
rho_km1 = S_km1.rho_k;
U_k = S_km1.U_kp1;
E_bar_km1 = S_km1.E_bar_k;
U_bar_km1 = S_km1.U_bar_k;
E_km1 = S_km1.E_k;
E_hat_km1 = S_km1.E_hat_k;

% Freq where mag of error has increased error + pad
E_incr = abs(E_k) > (abs(E_bar_km1) + epsilon);

% Freq the magnitude of error has decreased error - pad
E_decr = abs(E_k) < (abs(E_bar_km1) - epsilon);

E_inside_pad = ~(E_decr) & ~(E_incr);

% Deadzone
DZ = abs(E_k) < ((1/100).*abs(Yd_k));

% Backstepping
if strcmp(S_km1.params.backstepping,'on')
    E_bar_k = (E_no_decr .* E_bar_km1) + (E_decr.* E_k);
    U_bar_k = (E_no_decr.* U_bar_km1) + (E_decr .* U_k);
else
    E_bar_k = E_k;
    U_bar_k = U_k;
end

% Init vars
E_hat_k = (E_bar_km1 - E_k)./E_bar_km1;

% Model
if strcmp(S_km1.params.model,'fly')
    G_inv_k = U_k ./ Y_k;
    G_inv_k(isnan(G_inv_k)) = 0;
    G_inv_k(isinf(G_inv_k)) = 0;
    G_inv_model = U_k ./ Y_k;
    
elseif strcmp(S_km1.params.model,'error-inversion')
    G_inv_k = S_km1.G_inv_km1.*(E_bar_k .* rho_km1) ./ (E_bar_km1 - E_k);
    G_inv_k(isnan(G_inv_k)) = 0;
    G_inv_k(isinf(G_inv_k)) = 0;
    G_inv_model = U_k ./ Y_k;
    
elseif strcmp(S_km1.params.model,'average')
    G_inv_k = U_k ./ Y_k;
    G_inv_k(isnan(G_inv_k)) = 0;
    G_inv_k(isinf(G_inv_k)) = 0;
    avg_magG = abs(S_km1.G_inv_k);
    avg_angG = angle(S_km1.G_inv_k);
    n = (S_km1.k + 1);
    if n > 1
        new_magG = (abs(G_inv_k) + (n-1).*avg_magG) ./ n;
        new_angG = (angle(G_inv_k) + (n-1).*avg_angG) ./ n;
        % G_inv_k = new_angG .* exp(1j.*new_angG);
        G_inv_k = new_magG .* exp(1j.*new_angG);
    end
    G_inv_model = U_k ./ Y_k;
else
    G_inv_k = S_km1.G_inv_k;
    G_inv_model = U_k ./ Y_k;
end

% Updates
if strcmp(S_km1.params.adapt,'gain')

    rho_k = (~DZ).*((E_incr .* rho_km1*alpha)...
                  + (E_decr .* rho_km1*zeta) ...
                  + (E_inside_pad .* rho_km1)) ...
        + (DZ).*(rho_km1);
    rho_k(abs(rho_k) > rho_max) = rho_max ...
        *exp(1j.*angle(rho_k(abs(rho_k) > rho_max)));

elseif strcmp(S_km1.params.adapt,'gain-phase')
    phasor = exp(1j*-deg2rad(phi));
    rho_k = E_no_decr .* rho_km1*alpha.*phasor...
        +   E_no_decr.* rho_km1*alpha...
        +   E_decr.* rho_km1*zeta;
    rho_k(abs(rho_k) > rho_max) = rho_max ...
        *exp(1j.*angle(rho_k(abs(rho_k) > rho_max)));
    
elseif strcmp(S_km1.params.adapt,'data-based')
    E_hat_k = (E_bar_km1 - E_k) ./ E_bar_km1;
    rho_k = (1./abs(E_hat_k)).*abs(rho_km1) ...
        .* exp(1j.*(angle(rho_km1) - angle(E_hat_k)));
    rho_k(abs(rho_k) > rho_max) = rho_max ...
        *exp(1j.*angle(rho_k(abs(rho_k) > rho_max)));
else
    rho_k = rho_km1;
end

% Update Law
U_kp1 = U_bar_k + (~DZ).*(rho_k .* G_inv_k .* E_bar_k);
u_kp1 = real(ifft([U_kp1; conj(flipud(U_kp1(2:end-1)))]));

% Copy previous state
S_k = S_km1;
S_k.k = S_k.k + 1;
S_k.E_hat_k = E_hat_k;

% Update Struct Members
S_k.rho_k = rho_k;


S_k.Yd_k = Yd_k;
S_k.yd_k = yd_k;
S_k.Y_k = Y_k;
S_k.y_k = y_k;
S_k.E_bar_k = E_bar_k;
S_k.U_bar_k = U_bar_k;
S_k.E_k = E_k;
S_k.G_inv_k = G_inv_k;
S_k.G_inv_km1 = G_inv_k;
S_k.G_inv_model = G_inv_model;
S_k.U_kp1 = U_kp1;
S_k.u_kp1 = u_kp1;
S_k.e_k_inf = e_k_inf;
S_k.e_k_2 = e_k_2;
S_k.E_k_inf = E_k_inf;
S_k.E_k_2 = E_k_2;
end


