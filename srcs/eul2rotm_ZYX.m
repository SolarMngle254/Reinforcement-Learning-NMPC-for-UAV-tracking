% Author : Le Minh Nhat - K22 HCMUT Electrical & Electronics Faculty 
% Inspiration : https://www.youtube.com/@atvu5238/ (Dat Vu) 
% Date modified: 30/08/2026

function R = eul2rotm_ZYX(psi,theta,phi)
% R = Rz(psi)*Ry(theta)*Rx(phi)
cpsi=cos(psi); spsi=sin(psi);
cth =cos(theta); sth =sin(theta);
cph =cos(phi);   sph =sin(phi);
Rz = [cpsi -spsi 0; spsi cpsi 0; 0 0 1];
Ry = [cth 0 sth; 0 1 0; -sth 0 cth];
Rx = [1 0 0; 0 cph -sph; 0 sph cph];
R = Rz*Ry*Rx;
end
