% Author : Le Minh Nhat - K22 HCMUT Electrical & Electronics Faculty 
% Inspiration : https://www.youtube.com/@atvu5238/ (Dat Vu) 
% Date modified: 30/08/2026

function constants = initial_constants()

    % ===== Physical constants =====
    Ix = 0.0034;  Iy = 0.0034;  Iz = 0.006;
    m  = 0.698;   g  = 9.81;    Jtp = 1.302e-6;
    Ts = 0.1;

    % ===== MPC weighting matrices =====
    Q = diag([10 10 10]);   S = diag([10 10 10]);   R = diag([10 10 10]);

    % ===== Motor and airframe parameters =====
    ct = 7.6184e-08 * (60/(2*pi))^2;
    cq = 2.6839e-09 * (60/(2*pi))^2;
    l  = 0.171;

    % ===== Controller parameters =====
    controlled_states = 3;
    hz = 4;
    innerDyn_length = 4;
    px = [-1 -2]; py = [-1 -2]; pz = [-1 -2];

    % ===== Trajectory and obstacle configuration (used for RRT) =====
    H_cruise = 3.0;                        % Cruise altitude H (m)
    radars   = [                           % 2D radar avoidance zones (disks)
       -1.0  1.5  0.9
        1.8  2.4  1.0
       -2.5 -0.5  0.8
        2.5 -1.5  0.9
    ];                                     % Each row: [xc yc r]
    bounds   = [-6 6 -6 6];                % Plotting bounds [xmin xmax ymin ymax]

    % ===== Miscellaneous =====
    trajectory = 1;                        % Kept for backward compatibility
    omega_min = 110*pi/3;
    omega_max = 860*pi/3;
    C_cm = [0 0 0 0 0 0 1 0 0;
            0 0 0 0 0 0 0 1 0;
            0 0 0 0 0 0 0 0 1];

    % ===== Store all constants in a Map =====
    keySet = {'Ix','Iy','Iz','m','g','Jtp','Ts','Q','S','R', ...
              'ct','cq','l','controlled_states','hz','innerDyn_length', ...
              'px','py','pz','trajectory','omega_min','omega_max','C_cm', ...
              'H_cruise','radars','bounds'};

    constants_list = {Ix, Iy, Iz, m, g, Jtp, Ts, Q, S, R, ...
                      ct, cq, l, controlled_states, hz, innerDyn_length, ...
                      px, py, pz, trajectory, omega_min, omega_max, C_cm, ...
                      H_cruise, radars, bounds};

    constants = containers.Map(keySet, constants_list);
end
