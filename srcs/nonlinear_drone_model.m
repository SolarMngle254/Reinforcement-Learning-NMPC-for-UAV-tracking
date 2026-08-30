
function dx = nonlinear_drone_model(t, states, U)
    % ===== Nonlinear drone dynamic model =====
    % In this simulation, the body-fixed coordinate frame is used together
    % with the rotation matrix that transforms vectors from the body frame
    % to the inertial frame.
    % This approach avoids using a hybrid reference frame and is suitable
    % for ode45, which requires the nonlinear system to be expressed as
    % a set of first-order ordinary differential equations.

    %% ===== System constants =====
    constants = initial_constants();
    Ix  = constants('Ix');  % Moment of inertia about the X-axis [kg·m^2]
    Iy  = constants('Iy');  % Moment of inertia about the Y-axis [kg·m^2]
    Iz  = constants('Iz');  % Moment of inertia about the Z-axis [kg·m^2]
    m   = constants('m');   % Drone mass [kg]
    g   = constants('g');   % Gravitational acceleration [m/s^2]
    Jtp = constants('Jtp'); % Rotor moment of inertia [N·m·s^2 = kg·m^2]
    
    %% ===== State vector =====
    % states = [u, v, w, p, q, r, x, y, z, phi, theta, psi]
    % u, v, w    : Translational velocities in the body frame [m/s]
    % p, q, r    : Angular velocities about the body-frame axes [rad/s]
    % x, y, z    : Position in the inertial frame [m]
    % phi        : Roll angle [rad]
    % theta      : Pitch angle [rad]
    % psi        : Yaw angle [rad]
    u     = states(1);
    v     = states(2);
    w     = states(3);
    p     = states(4);
    q     = states(5);
    r     = states(6);
    x     = states(7);
    y     = states(8);
    z     = states(9);
    phi   = states(10);
    theta = states(11);
    psi   = states(12);
    
    %% ===== Control input vector =====
    % U1: Total thrust force [N]
    % U2: Torque about the X-axis [N·m]
    % U3: Torque about the Y-axis [N·m]
    % U4: Torque about the Z-axis [N·m]
    U1 = U(1);
    U2 = U(2);
    U3 = U(3);
    U4 = U(4);
    
    %% ===== Rotation matrix from body frame to inertial frame =====
    % Used to transform [u; v; w] into [x_dot; y_dot; z_dot]
    % expressed in the inertial frame.
    R_matrix = [ cos(theta)*cos(psi),  sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi),  cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
                 cos(theta)*sin(psi),  sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi),  cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
                -sin(theta),           sin(phi)*cos(theta),                              cos(phi)*cos(theta) ];
    
    %% ===== Transformation matrix from body angular rates
    %          [p, q, r] to Euler angle rates =====
    T_matrix = [ 1,  sin(phi)*tan(theta),   cos(phi)*tan(theta);
                 0,  cos(phi),             -sin(phi);
                 0,  sin(phi)*sec(theta),   cos(phi)*sec(theta) ];
    
    %% ===== Global variables =====
    global omega_total % Total rotor angular velocity used to account for
                       % the gyroscopic reaction torque
    
    %% ===== Nonlinear dynamic equations =====
    % dx(i) represents the time derivative of the i-th state variable.

    dx(1,1)  = (v*r - w*q) + g*sin(theta);                     % u_dot   - Acceleration along the body X-axis
    dx(2,1)  = (w*p - u*r) - g*cos(theta)*sin(phi);            % v_dot   - Acceleration along the body Y-axis
    dx(3,1)  = (u*q - v*p) - g*cos(theta)*cos(phi) + U1/m;     % w_dot   - Acceleration along the body Z-axis
    dx(4,1)  = q*r*(Iy-Iz)/Ix - (Jtp/Ix)*q*omega_total + U2/Ix; % p_dot   - Angular acceleration about the X-axis
    dx(5,1)  = p*r*(Iz-Ix)/Iy + (Jtp/Iy)*p*omega_total + U3/Iy; % q_dot   - Angular acceleration about the Y-axis
    dx(6,1)  = p*q*(Ix-Iy)/Iz + U4/Iz;                         % r_dot   - Angular acceleration about the Z-axis
    dx(7,1)  = R_matrix(1,:) * [u; v; w];                      % x_dot   - Inertial velocity along the X-axis
    dx(8,1)  = R_matrix(2,:) * [u; v; w];                      % y_dot   - Inertial velocity along the Y-axis
    dx(9,1)  = R_matrix(3,:) * [u; v; w];                      % z_dot   - Inertial velocity along the Z-axis
    dx(10,1) = T_matrix(1,:) * [p; q; r];                      % phi_dot     - Roll angle rate
    dx(11,1) = T_matrix(2,:) * [p; q; r];                      % theta_dot   - Pitch angle rate
    dx(12,1) = T_matrix(3,:) * [p; q; r];                      % psi_dot     - Yaw angle rate
end
