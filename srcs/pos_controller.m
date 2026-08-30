% Author : Le Minh Nhat - K22 HCMUT Electrical & Electronics Faculty 
% Inspiration : https://www.youtube.com/@atvu5238/ (Dat Vu) 
% Date modified: 30/08/2026

function [Phi_ref, Theta_ref, U1] = pos_controller( ...
    X_ref, X_dot_ref, X_dot_dot_ref, ...
    Y_ref, Y_dot_ref, Y_dot_dot_ref, ...
    Z_ref, Z_dot_ref, Z_dot_dot_ref, ...
    Psi_ref, states)

%% ===== System Constants =====
constants = initial_constants();
m  = constants('m');   % Drone mass [kg]
g  = constants('g');   % Gravitational acceleration [m/s^2]
px = constants('px');  % Desired poles for X-axis control
py = constants('py');  % Desired poles for Y-axis control
pz = constants('pz');  % Desired poles for Z-axis control

%% ===== Current State Variables =====
% State vector: [u, v, w, p, q, r, x, y, z, phi, theta, psi]
u     = states(1);   % Velocity along the X-axis in the body frame [m/s]
v     = states(2);   % Velocity along the Y-axis in the body frame [m/s]
w     = states(3);   % Velocity along the Z-axis in the body frame [m/s]
x     = states(7);   % X-position in the inertial frame [m]
y     = states(8);   % Y-position in the inertial frame [m]
z     = states(9);   % Z-position in the inertial frame [m]
phi   = states(10);  % Roll angle [rad]
theta = states(11);  % Pitch angle [rad]
psi   = states(12);  % Yaw angle [rad]

%% ===== Rotation Matrix: Body-Frame Velocity to Inertial-Frame Velocity =====
R_matrix = [ cos(theta)*cos(psi),  sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi),  cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
             cos(theta)*sin(psi),  sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi),  cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
            -sin(theta),           sin(phi)*cos(theta),                              cos(phi)*cos(theta) ];

% Velocity expressed in the inertial frame
x_dot = R_matrix(1,:) * [u; v; w]; % X-axis velocity
y_dot = R_matrix(2,:) * [u; v; w]; % Y-axis velocity
z_dot = R_matrix(3,:) * [u; v; w]; % Z-axis velocity

%% ===== Position and Velocity Errors =====
ex     = X_ref - x;         % X-position error
ex_dot = X_dot_ref - x_dot; % X-velocity error

ey     = Y_ref - y;         % Y-position error
ey_dot = Y_dot_ref - y_dot; % Y-velocity error

ez     = Z_ref - z;         % Z-position error
ez_dot = Z_dot_ref - z_dot; % Z-velocity error

%% ===== Compute K1, K2 Gains and ux, uy, uz for Position Subsystem Stabilization =====
% X-axis
kx1 = (px(1) - (px(1) + px(2))/2).^2 - (px(1) + px(2)).^2 / 4;
kx2 = px(1) + px(2);
kx1 = real(kx1);
kx2 = real(kx2);

% Y-axis
ky1 = (py(1) - (py(1) + py(2))/2).^2 - (py(1) + py(2)).^2 / 4;
ky2 = py(1) + py(2);
ky1 = real(ky1);
ky2 = real(ky2);

% Z-axis
kz1 = (pz(1) - (pz(1) + pz(2))/2).^2 - (pz(1) + pz(2)).^2 / 4;
kz2 = pz(1) + pz(2);
kz1 = real(kz1);
kz2 = real(kz2);

% Temporary position control inputs based on the linearized model
ux = kx1 * ex + kx2 * ex_dot;
uy = ky1 * ey + ky2 * ey_dot;
uz = kz1 * ez + kz2 * ez_dot;

%% ===== Compute Desired Accelerations vx, vy, vz =====
vx = X_dot_dot_ref - ux;
vy = Y_dot_dot_ref - uy;
vz = Z_dot_dot_ref - uz;

%% ===== Compute Desired Pitch Angle (Theta_ref) =====
a = vx / (vz + g);
b = vy / (vz + g);
c = cos(Psi_ref);
d = sin(Psi_ref);

tan_theta = a*c + b*d;
Theta_ref = atan(tan_theta);

%% ===== Handle Psi_ref Singularity =====
if Psi_ref >= 0
    Psi_ref_singularity = Psi_ref - floor(abs(Psi_ref) / (2*pi)) * 2*pi;
else
    Psi_ref_singularity = Psi_ref + floor(abs(Psi_ref) / (2*pi)) * 2*pi;
end

%% ===== Compute Desired Roll Angle (Phi_ref) =====
if or( ...
       or(abs(Psi_ref_singularity) < pi/4, abs(Psi_ref_singularity) > 7*pi/4), ...
       and(abs(Psi_ref_singularity) > 3*pi/4, abs(Psi_ref_singularity) < 5*pi/4) )
    tan_phi = cos(Theta_ref) * (tan(Theta_ref)*d - b) / c;
else
    tan_phi = cos(Theta_ref) * (a - tan(Theta_ref)*c) / d;
end

Phi_ref = atan(tan_phi);

%% ===== Compute Total Thrust U1 =====
U1 = (vz + g) * m / (cos(Phi_ref) * cos(Theta_ref));

end
