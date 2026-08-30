
function J = attitude_cost_nmpc(u_seq, x_att0, Phi_h, Theta_h, Psi_h, ...
                                u_prev, hz, Ts, constants, omega_total)
%ATTITUDE_COST_NMPC  NMPC cost function for UAV attitude control (phi, theta, psi).
%
%  u_seq       : control sequence [U2(1); U3(1); U4(1); ... ;
%                 U2(hz); U3(hz); U4(hz)]
%  x_att0      : current attitude state [phi;theta;psi;p;q;r]
%  Phi_h       : phi reference vector over the prediction horizon (hz x 1)
%  Theta_h     : theta reference vector over the prediction horizon
%  Psi_h       : psi reference vector over the prediction horizon
%  u_prev      : previous control input [U2_prev;U3_prev;U4_prev]
%  hz          : prediction horizon length
%  Ts          : inner-loop sampling time
%  constants   : struct/containers.Map containing system constants
%                from initial_constants()
%  omega_total : current total rotor angular velocity, used for
%                gyroscopic torque calculation

Ix  = constants('Ix');
Iy  = constants('Iy');
Iz  = constants('Iz');
Jtp = constants('Jtp');

nu = 3;
u_seq = u_seq(:);
x = x_att0(:);

% Weighting matrices
Qe   = diag([50 50 20]);     % Attitude angle error [phi theta psi]
Qer  = diag([1 1 1]);        % Angular rate [p q r]
Ru   = diag([1e-3 1e-3 1e-3]);
Rdu  = diag([1e-4 1e-4 1e-4]);

J = 0;

for k = 1:hz
    u_k = u_seq((k-1)*nu+1 : k*nu);   % [U2; U3; U4]

    phi   = x(1);
    theta = x(2);
    psi   = x(3);
    p     = x(4);
    q     = x(5);
    r     = x(6);

    % Transformation matrix from body angular rates [p,q,r]
    % to Euler angle rates [phi_dot,theta_dot,psi_dot]
    Tm = [1,  sin(phi)*tan(theta),   cos(phi)*tan(theta);
          0,  cos(phi),             -sin(phi);
          0,  sin(phi)/cos(theta),   cos(phi)/cos(theta)];

    euler_dot = Tm * [p;q;r];
    phi_dot   = euler_dot(1);
    theta_dot = euler_dot(2);
    psi_dot   = euler_dot(3);

    % Body angular dynamics
    p_dot = q*r*(Iy-Iz)/Ix - (Jtp/Ix)*q*omega_total + u_k(1)/Ix;
    q_dot = p*r*(Iz-Ix)/Iy + (Jtp/Iy)*p*omega_total + u_k(2)/Iy;
    r_dot = p*q*(Ix-Iy)/Iz + u_k(3)/Iz;

    x_dot = [phi_dot; theta_dot; psi_dot; p_dot; q_dot; r_dot];

    % Discrete-time state propagation using forward Euler integration
    x = x + Ts * x_dot;

    % Reference values at prediction step k
    phi_ref   = Phi_h(k);
    theta_ref = Theta_h(k);
    psi_ref   = Psi_h(k);

    e_angle = [wrapToPi(phi - phi_ref);
               wrapToPi(theta - theta_ref);
               wrapToPi(psi - psi_ref)];
    e_rate  = [p; q; r];

    if k == 1
        du = u_k - u_prev;
    else
        u_prev_k = u_seq((k-2)*nu+1 : (k-1)*nu);
        du = u_k - u_prev_k;
    end

    J = J + e_angle.'*Qe*e_angle + ...
            e_rate.' *Qer*e_rate + ...
            u_k.'   *Ru *u_k   + ...
            du.'    *Rdu*du;
end

end
