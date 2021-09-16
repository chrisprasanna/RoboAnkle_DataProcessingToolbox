function [Phi, Xs, Ys, k, ix0] = cp_PHI(x,t)
% PHI - is a function that computes the biomechanical phase variable PHI
% from the integral of the global thigh angle and the global thigh angle
% 
% INPUTS:
% x - Real (1 x n) -    is a measurement of the global thigh angle during a gait cycle
% t - Real (1 x n) -    is the time vector of the gait cycle (or number of samples)
% 
% OUTPUTS:
% Phi - Real (1 x n) -  is the normalized phase variable computed from the polar angle
%                       of the thigh angle-integral pgase portrait 
%% 

% Computing the integral of the hip joint
ix0 = trapz(x)/length(t); % shift in the hip angle for symmetry w.r.t the Y axis.
X = -(x - ix0); 

% Y = cumtrapz(X)/length(t); 
Y = cumtrapz(X); 

% Parameters to increase linearity in the phase variable 
k = abs(max(X)-min(X))/abs(max(Y)-min(Y)); % Scale factor
Xs = X; 
Ys = k*Y; 

% Phase Variable Phi
phi = atan2(Ys,Xs);             
% phi = unwrap(phi);       % Shift phase angles

% Normalized Phase Variable 
% Phi = (phi - phi(1))/(phi(end) - phi(1));
tol = -1e-2;
if all(diff(phi(3:end-2))>= tol)
    Phi = (phi + pi)/(2*pi); % works for stritly increasing phi
else
    % Phi = (phi + pi)/(2*pi); % works for stritly increasing phi
    Phi = (mod(phi,2*pi) - 0)/(2*pi); % works for phi with discontinuities 
end

% X = x;
% Y = cumtrapz(X)/length(t);
% 
% X_max = max(X); 
% X_min = min(X);
% Y_max = max(Y);
% Y_min = min(Y);
% 
% z = abs(X_max - X_min)/abs(Y_max - Y_min);
% gamma = -(X_max + X_min)/2;
% Gamma = -(Y_max + Y_min)/2;
% 
% Xs = X + gamma;
% Ys = z*(Y + Gamma);
% 
% phi = atan2(Ys,Xs);
% % phi = unwrap(phi) - 2*pi;
% 
% % Phi = (phi + pi)/(2*pi);
% Phi = (phi - phi(1))/(phi(end) - phi(1));

end