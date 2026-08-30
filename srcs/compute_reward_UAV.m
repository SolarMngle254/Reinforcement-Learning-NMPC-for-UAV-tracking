% Author : Le Minh Nhat - K22 HCMUT Electrical & Electronics Faculty
% Inspiration : https://www.youtube.com/@atvu5238/ (Dat Vu)
% Date modified: 30/08/2026

function r = compute_reward_UAV(x, u, Xr, Yr, Zr, psir_ref, prmR)
%  COMPUTE_REWARD_UAV  Compute the UAV trajectory tracking reward.
%
%  prmR fields:
%    prmR.w_pos  : position error weight (x, y, z)
%    prmR.w_yaw  : yaw error weight
%    prmR.w_att  : roll/pitch error weight (to maintain a level attitude)
%    prmR.w_u    : control input effort weight

u1 = x(1); %#ok<NASGU> % Reserved for potential future use
v  = x(2);
w  = x(3);
X  = x(7);
Y  = x(8);
Z  = x(9);
phi_  = x(10);
theta = x(11);
psi   = x(12);

ex   = X - Xr;
ey   = Y - Yr;
ez   = Z - Zr;
epsi = wrapToPi(psi - psir_ref);

pos_err2  = ex^2 + ey^2 + ez^2;
yaw_err2  = epsi^2;
att_err2  = phi_^2 + theta^2;
u_norm2   = u(:).' * u(:);

% Assign default values if fields are not provided
if ~isfield(prmR,'w_pos'), prmR.w_pos = 5;   end
if ~isfield(prmR,'w_yaw'), prmR.w_yaw = 2;   end
if ~isfield(prmR,'w_att'), prmR.w_att = 1;   end
if ~isfield(prmR,'w_u'),   prmR.w_u   = 1e-2; end

cost = prmR.w_pos*pos_err2 + ...
       prmR.w_yaw*yaw_err2 + ...
       prmR.w_att*att_err2 + ...
       prmR.w_u*u_norm2;

r = -cost;

end
