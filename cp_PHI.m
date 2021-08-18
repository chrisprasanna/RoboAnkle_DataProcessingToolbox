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

Y = cumtrapz(X); 

% Parameters to increase monotonicity
k = abs(max(X)-min(X))/abs(max(Y)-min(Y)); % Scale factor
Xs = X; 
Ys = k*Y; 

% Phase Variable Phi
phi = atan2(Ys,Xs);             
phi = unwrap(phi);       % Shift phase angles

% Normalized Phase Variable 
Phi = (phi - phi(1))/(phi(end) - phi(1));

end
