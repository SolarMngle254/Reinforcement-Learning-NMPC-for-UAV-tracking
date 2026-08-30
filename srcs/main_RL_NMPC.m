% Author : Le Minh Nhat - K22 HCMUT Electrical & Electronics Faculty 
% Inspiration : https://www.youtube.com/@atvu5238/ (Dat Vu) 
% Date modified: 30/08/2026

clear all       % Clear all variables from the workspace
close all       % Close all figure windows
clc             % Clear the command window
warning off     % Disable warning messages

%% ===== RL + NMPC CONTROL PROGRAM (WITHOUT LPV) =====

%% 1. LOAD THE REINFORCEMENT LEARNING POLICY
% This file is generated after successfully running train_RL.m
disp('Loading RL policy (RL_NMPC_Policy.mat)...');
load('RL_NMPC_Policy.mat', 'agent');
disp('RL policy loaded successfully!');

%% 2. LOAD SYSTEM CONSTANTS
constants = initial_constants();
Ts                = constants('Ts');
controlled_states = constants('controlled_states');
innerDyn_length   = constants('innerDyn_length');
hz                = constants('hz');
ct = constants('ct'); 
cq = constants('cq'); 
l  = constants('l');

% NMPC control limits
omega_min = constants('omega_min');
omega_max = constants('omega_max');
U1_min = ct * 4 * omega_min.^2;
U1_max = ct * 4 * omega_max.^2;

y_min = [ct * l * (omega_min.^2 - omega_max.^2);
         ct * l * (omega_min.^2 - omega_max.^2);
         cq * (-2*omega_max.^2 + 2*omega_min.^2)];

y_max = [ct * l * (omega_max.^2 - omega_min.^2);
         ct * l * (omega_max.^2 - omega_min.^2);
         cq * (-2*omega_min.^2 + 2*omega_max.^2)];

%% 3. SIMULATION TIME AND REFERENCE TRAJECTORY
% Strict synchronization: update the controller at the exact sampling
% period Ts = 0.1 s.
t = (0 : Ts : 100)';   
           
traj_choice = 2;      
traj.type  = traj_choice;
traj.scale = 0.5;     
traj.R0    = 4;       
traj.kR    = 0.05;    
traj.z0    = 0.5;
traj.kz    = 0.02;
traj.omega = 0.3;

% Precompute the reference trajectory over the entire simulation time.
[X_ref, Xd_ref, Xdd_ref, Y_ref, Yd_ref, Ydd_ref, ...
    Z_ref, Zd_ref, Zdd_ref, psi_ref] = trajectory_generator(t, traj);

plotl = length(t);

%% 4. INITIAL STATE
states = [Xd_ref(1,2), Yd_ref(1,2), Zd_ref(1,2), 0, 0, 0, ...
          X_ref(1,2), Y_ref(1,2), Z_ref(1,2), 0, 0, psi_ref(1,2)];

states_total = states; 
velocityXYZ_total = [Xd_ref(1,2), Yd_ref(1,2), Zd_ref(1,2)];

omega1 = 110*pi/3; 
omega2 = 110*pi/3; 
omega3 = 110*pi/3; 
omega4 = 110*pi/3; 

U1 = ct * (omega1^2 + omega2^2 + omega3^2 + omega4^2);          
U2 = 0; 
U3 = 0; 
U4 = 0;

UTotal = [U1, U2, U3, U4]; 

global omega_total;
omega_total = omega1 - omega2 + omega3 - omega4;

U_prev = [U2; U3; U4]; 

%% 5. ANIMATION SETUP
bnd = constants('bounds');
zmax_vis = max([Z_ref(:,2); 0]) + 1;

fig_anim = figure('Name','UAV RL+NMPC Animation','Color','w');
ax_anim  = axes(fig_anim); 
hold(ax_anim,'on'); 
grid(ax_anim,'on'); 
axis(ax_anim,'equal');

xlabel(ax_anim,'x [m]'); 
ylabel(ax_anim,'y [m]'); 
zlabel(ax_anim,'z [m]');

xlim(ax_anim, bnd(1:2)); 
ylim(ax_anim, bnd(3:4)); 
zlim(ax_anim, [0 zmax_vis]);

view(ax_anim, 35, 25);

plot3(ax_anim, X_ref(:,2), Y_ref(:,2), Z_ref(:,2), ...
    '--b', 'LineWidth', 1.5);

hTrail = animatedline(ax_anim, ...
    'Color',[0.85 0.1 0.1], 'LineWidth',1.5);

hUAV = makeUAV(ax_anim, 0.25); 

%% 6. MAIN CONTROL LOOP (SINGLE-LOOP SYNCHRONIZED IMPLEMENTATION)
disp('Starting RL + NMPC simulation...');

fmin_opts = optimoptions('fmincon','Algorithm','sqp', ...
    'Display','off','MaxIterations',20, ...
    'OptimalityTolerance',1e-2,'StepTolerance',1e-3);

% Run a single control loop synchronized with the sampling period Ts.
for k = 1 : plotl-1
    
    % --- GET CURRENT REFERENCE ---
    xr = X_ref(k+1,2); 
    yr = Y_ref(k+1,2); 
    zr = Z_ref(k+1,2);
    psi_r_now = psi_ref(k+1,2);
    
    % --- OUTER LOOP (UPDATED AT EVERY TIME STEP) ---
    [phi_ref, theta_ref, U1] = pos_controller( ...
        xr, Xd_ref(k+1,2), Xdd_ref(k+1,2), ...
        yr, Yd_ref(k+1,2), Ydd_ref(k+1,2), ...
        zr, Zd_ref(k+1,2), Zdd_ref(k+1,2), ...
        psi_r_now, states);

    U1 = max(min(U1, U1_max), U1_min);
    
    % --- INNER LOOP (NMPC) ---
    Phi_h   = phi_ref * ones(hz,1);
    Theta_h = theta_ref * ones(hz,1);
    Psi_h   = psi_r_now * ones(hz,1);
    
    x_att0 = [states(10:12)'; states(4:6)']; 
    lb = repmat(y_min, hz, 1);
    ub = repmat(y_max, hz, 1);
    u_seq0 = repmat(U_prev, hz, 1);
    
    consts_local = initial_constants();
    
    cost_fun = @(u_seq) attitude_cost_nmpc( ...
        u_seq, x_att0, Phi_h, Theta_h, Psi_h, ...
        U_prev, hz, Ts, consts_local, omega_total);
    
    [u_seq_opt, ~, exitflag] = fmincon( ...
        cost_fun, u_seq0, [],[],[],[], lb, ub, [], fmin_opts);
    
    if isempty(u_seq_opt) || exitflag <= 0
        U_nmpc = U_prev;
    else
        U_nmpc = u_seq_opt(1:3);
    end
    
    U_prev = U_nmpc;
    
    % --- QUERY THE RL POLICY FOR THE RESIDUAL CONTROL SIGNAL ---
    ex = (xr - states(7)) / 2.0;
    ey = (yr - states(8)) / 2.0;
    ez = (zr - states(9)) / 2.0;
    epsi = wrapToPi(psi_r_now - states(12)) / pi;
    
    uvw = states(1:3)' / 5.0;
    pqr = states(4:6)' / 5.0;
    
    % Reuse the precomputed trajectory for fast lookahead feature extraction.
    idx1 = min(k + 1 + 3, plotl);
    idx2 = min(k + 1 + 6, plotl);
    idx3 = min(k + 1 + 10, plotl);
    
    la1 = ([X_ref(idx1,2); Y_ref(idx1,2); Z_ref(idx1,2)] ...
        - states(7:9)') / 2.0;
    
    la2 = ([X_ref(idx2,2); Y_ref(idx2,2); Z_ref(idx2,2)] ...
        - states(7:9)') / 2.0;
    
    la3 = ([X_ref(idx3,2); Y_ref(idx3,2); Z_ref(idx3,2)] ...
        - states(7:9)') / 2.0;
    
    obs = [ex; ey; ez; epsi; uvw; pqr; la1; la2; la3];
    
    % Query the RL policy and restore the physical control signal.
    dU_cell = getAction(agent, {obs});
    dU = double(dU_cell{1}) * 0.01; 
    
    U_final = U_nmpc + dU;
    
    UTotal = [UTotal; U1, U_final(1), U_final(2), U_final(3)];
    U_total_vector = [U1; U_final];
    
    % --- ODE45 STATE PROPAGATION ---
    Tspan = [0 Ts];
    
    [~, x_traj] = ode45( ...
        @(t,x) nonlinear_drone_model(t,x,U_total_vector), ...
        Tspan, states');
    
    states = x_traj(end,:);
    states_total = [states_total; states];

    % Update the physical rotor-speed equivalent.
    U1_sat = max(min(U1, U1_max), U1_min);
    omega_total = sqrt(U1_sat / ct); 
    
    % Compute the actual inertial velocity.
    phi_now = states(10); 
    theta_now = states(11); 
    psi_now = states(12);
    
    R_mat = [ cos(theta_now)*cos(psi_now), ...
              sin(phi_now)*sin(theta_now)*cos(psi_now) - cos(phi_now)*sin(psi_now), ...
              cos(phi_now)*sin(theta_now)*cos(psi_now) + sin(phi_now)*sin(psi_now);
              
              cos(theta_now)*sin(psi_now), ...
              sin(phi_now)*sin(theta_now)*sin(psi_now) + cos(phi_now)*cos(psi_now), ...
              cos(phi_now)*sin(theta_now)*sin(psi_now) - sin(phi_now)*cos(psi_now);
              
             -sin(theta_now), ...
              sin(phi_now)*cos(theta_now), ...
              cos(phi_now)*cos(theta_now) ];
    
    vel_inertial = R_mat * [states(1); states(2); states(3)];
    velocityXYZ_total = [velocityXYZ_total; vel_inertial'];
    
    % Update the 3D UAV visualization.
    pos_now = states(7:9); 
    eul_now = states(10:12);
    
    updateUAV(hUAV, pos_now, eul_now);
    addpoints(hTrail, pos_now(1), pos_now(2), pos_now(3));
    drawnow limitrate;
end

disp('Simulation completed!');

%% 7. PLOT SIMULATION RESULTS

% 7.1. 3D trajectory tracking
figure('Name','3D Trajectory Tracking', 'Color', 'w');

plot3(X_ref(:,2), Y_ref(:,2), Z_ref(:,2), ...
    '--b', 'LineWidth', 2); 
hold on;

plot3(states_total(:, 7), states_total(:, 8), states_total(:, 9), ...
    'r', 'LineWidth', 1.5);

grid on;

xlabel('x-position [m]', 'FontSize', 12);
ylabel('y-position [m]', 'FontSize', 12);
zlabel('z-position [m]', 'FontSize', 12);

legend({'Reference', 'RL+NMPC Actual'}, ...
    'Location', 'best', 'FontSize', 12);

title('3D Zero-Shot Trajectory Tracking');

% 7.2. Position tracking (X, Y, Z)
figure('Name','Position Tracking Error', 'Color', 'w');

subplot(3,1,1);
plot(t, X_ref(:,2), '--b', 'LineWidth', 1.5); 
hold on;
plot(t, states_total(:,7), 'r', 'LineWidth', 1.5);

ylabel('X [m]', 'FontSize', 12); 
grid on;

legend('Reference', 'Actual');
title('Position Tracking Performance');

subplot(3,1,2);
plot(t, Y_ref(:,2), '--b', 'LineWidth', 1.5); 
hold on;
plot(t, states_total(:,8), 'r', 'LineWidth', 1.5);

ylabel('Y [m]', 'FontSize', 12); 
grid on;

subplot(3,1,3);
plot(t, Z_ref(:,2), '--b', 'LineWidth', 1.5); 
hold on;
plot(t, states_total(:,9), 'r', 'LineWidth', 1.5);

ylabel('Z [m]', 'FontSize', 12); 
xlabel('Time [s]', 'FontSize', 12); 
grid on;

% 7.3. Attitude tracking (Roll, Pitch, Yaw)
figure('Name','Attitude Tracking', 'Color', 'w');

subplot(3,1,1);
plot(t, states_total(:,10), 'r', 'LineWidth', 1.5);

ylabel('\phi (Roll) [rad]', 'FontSize', 12); 
grid on;

title('Attitude Regulation');

subplot(3,1,2);
plot(t, states_total(:,11), 'r', 'LineWidth', 1.5);

ylabel('\theta (Pitch) [rad]', 'FontSize', 12); 
grid on;

subplot(3,1,3);
plot(t, psi_ref(:,2), '--b', 'LineWidth', 1.5); 
hold on;

plot(t, states_total(:,12), 'r', 'LineWidth', 1.5);

ylabel('\psi (Yaw) [rad]', 'FontSize', 12); 
xlabel('Time [s]', 'FontSize', 12); 
grid on;

legend('Reference', 'Actual');

% 7.4. Control inputs and RL residual signals
figure('Name','Control Inputs', 'Color', 'w');

subplot(4,1,1);
plot(t, UTotal(:,1), 'k', 'LineWidth', 1.5);

ylabel('U_1 [N]', 'FontSize', 12); 
grid on;

title('Control Effort (Total Thrust & Residual Torques)');

subplot(4,1,2);
plot(t, UTotal(:,2), 'b', 'LineWidth', 1.5);

ylabel('U_2 [Nm]', 'FontSize', 12); 
grid on;

subplot(4,1,3);
plot(t, UTotal(:,3), 'r', 'LineWidth', 1.5);

ylabel('U_3 [Nm]', 'FontSize', 12); 
grid on;

subplot(4,1,4);
plot(t, UTotal(:,4), 'g', 'LineWidth', 1.5);

ylabel('U_4 [Nm]', 'FontSize', 12); 
xlabel('Time [s]', 'FontSize', 12); 
grid on;
