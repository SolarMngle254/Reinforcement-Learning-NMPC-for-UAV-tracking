% Author : Le Minh Nhat - K22 HCMUT Electrical & Electronics Faculty
% Inspiration : https://www.youtube.com/@atvu5238/ (Dat Vu)
% Date modified: 30/08/2026

function [X_ref, X_dot_ref, X_ddot_ref, ...
          Y_ref, Y_dot_ref, Y_ddot_ref, ...
          Z_ref, Z_dot_ref, Z_ddot_ref, ...
          psi_ref] = trajectory_generator(t, traj)
% TRAJECTORY_GENERATOR  Generate UAV trajectories and their derivatives (5 types).
%
%  INPUT:
%    t    : time vector, column (N+1 x 1) or row (1 x N+1)
%    traj : trajectory parameter struct with the following fields
%           (default values are assigned if fields are missing):
%           type  : 1 = circle
%                   2 = spiral
%                   3 = figure-eight
%                   4 = 3D Lissajous (butterfly)
%                   5 = wavy circle
%           scale : trajectory scaling factor (default = 1)
%           R0, kR, z0, kz, omega : parameters used depending on trajectory type
%
%  OUTPUT (each output is in [t, value] format):
%    X_ref, X_dot_ref, X_ddot_ref
%    Y_ref, Y_dot_ref, Y_ddot_ref
%    Z_ref, Z_dot_ref, Z_ddot_ref
%    psi_ref

% ----- Normalize time vector -----
t  = t(:);
if numel(t) < 2
    error('trajectory_generator: t must contain at least 2 elements.');
end
dt = t(2) - t(1);

% ----- Default trajectory parameters -----
if ~isfield(traj,'type'),  traj.type  = 1;    end
if ~isfield(traj,'scale'), traj.scale = 1.0;  end
if ~isfield(traj,'R0'),    traj.R0    = 5;    end
if ~isfield(traj,'kR'),    traj.kR    = 0.1;  end
if ~isfield(traj,'z0'),    traj.z0    = 1;    end
if ~isfield(traj,'kz'),    traj.kz    = 0.05; end
if ~isfield(traj,'omega'), traj.omega = 0.3;  end
typ   = traj.type;
scl   = traj.scale;

% ===== Generate trajectory based on the selected type =====
switch typ
    case 1  % ===== Type 1: Circle =====
        R     = scl * traj.R0 * ones(size(t));
        theta = traj.omega * t;
        X_val = R .* cos(theta);
        Y_val = R .* sin(theta);
        Z_val = traj.z0 + traj.kz * t;
        
    case 2  % ===== Type 2: Spiral =====
        R     = scl * (traj.R0 + traj.kR * t);
        theta = traj.omega * t;
        X_val = R .* cos(theta);
        Y_val = R .* sin(theta);
        Z_val = traj.z0 + traj.kz * t;
        
    case 3  % ===== Type 3: Figure-Eight =====
        a     = scl * traj.R0;
        w     = traj.omega;
        s     = w * t;
        X_val = a * sin(s);
        Y_val = a * 0.5 * sin(2*s);
        Z_val = traj.z0 + traj.kz * t * 0.2;
        
    case 4  % ===== Type 4: 3D Lissajous (Butterfly) =====
        X_val = 3 * sin(0.3 * t);
        Y_val = 3 * sin(0.6 * t);
        Z_val = 1.5 + 0.5 * sin(0.3 * t);
        
    case 5  % ===== Type 5: Wavy Circle =====
        X_val = 2.5 * cos(0.4 * t);
        Y_val = 2.5 * sin(0.4 * t);
        Z_val = 1.2 + 0.8 * sin(1.2 * t);
        
    otherwise
        error('trajectory_generator: traj.type must be one of (1,2,3,4,5).');
end

% ===== Compute yaw angle from the trajectory tangent =====
dx       = gradient(X_val, dt);
dy       = gradient(Y_val, dt);
psi_val  = atan2(dy, dx);

% ===== Compute first- and second-order derivatives =====
Xdot_val  = gradient(X_val, dt);
Xddot_val = gradient(Xdot_val, dt);
Ydot_val  = gradient(Y_val, dt);
Yddot_val = gradient(Ydot_val, dt);
Zdot_val  = gradient(Z_val, dt);
Zddot_val = gradient(Zdot_val, dt);

% ===== Package outputs as [t, value] for compatibility with legacy code =====
X_ref      = [t, X_val];
X_dot_ref  = [t, Xdot_val];
X_ddot_ref = [t, Xddot_val];
Y_ref      = [t, Y_val];
Y_dot_ref  = [t, Ydot_val];
Y_ddot_ref = [t, Yddot_val];
Z_ref      = [t, Z_val];
Z_dot_ref  = [t, Zdot_val];
Z_ddot_ref = [t, Zddot_val];
psi_ref    = [t, psi_val];
end
