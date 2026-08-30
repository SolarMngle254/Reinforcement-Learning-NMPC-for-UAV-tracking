% Author : Le Minh Nhat - K22 HCMUT Electrical & Electronics Faculty 
% Inspiration : https://www.youtube.com/@atvu5238/ (Dat Vu) 
% Date modified: 30/08/2026

function phi = build_features_UAV(x, Xr, Yr, Zr, psir_ref)
%BUILD_FEATURES_UAV  Construct the feature vector phi(x, ref) for RL.
%
%  x = [u v w p q r x y z phi theta psi]^T
%  ref = (Xr, Yr, Zr, psir_ref)
%
%  The selected feature vector has 10 dimensions:
%     [ex, ey, ez, epsi, u, v, w, p, q, r]^T

u     = x(1);
v     = x(2);
w     = x(3);
p     = x(4);
q     = x(5);
r     = x(6);
X     = x(7);
Y     = x(8);
Z     = x(9);
phi_  = x(10); %#ok<NASGU> % Reserved for potential future use
theta = x(11); %#ok<NASGU> % Reserved for potential future use
psi   = x(12);

ex   = X - Xr;
ey   = Y - Yr;
ez   = Z - Zr;
epsi = wrapToPi(psi - psir_ref);

phi = [ex; ey; ez; epsi; u; v; w; p; q; r];

end
