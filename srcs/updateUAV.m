% Author : Le Minh Nhat - K22 HCMUT Electrical & Electronics Faculty 
% Inspiration : https://www.youtube.com/@atvu5238/ (Dat Vu) 
% Date modified: 30/08/2026

function updateUAV(h, pos, eul)

% pos = [x y z], eul = [phi theta psi] (rad)

phi = eul(1);
theta = eul(2);
psi = eul(3);

R = eul2rotm_ZYX(psi, theta, phi); % ZYX rotation: yaw -> pitch -> roll

% Compute the endpoints of the 4 arms in the WORLD frame
ends_world = (R * h.arm_body) + pos(:); % 3x4

% Arm 1: +X and -X

set(h.arm(1), 'XData',[ends_world(1,1) ends_world(1,2)], ...
              'YData',[ends_world(2,1) ends_world(2,2)], ...
              'ZData',[ends_world(3,1) ends_world(3,2)]);

% Arm 2: +Y and -Y

set(h.arm(2), 'XData',[ends_world(1,3) ends_world(1,4)], ...
              'YData',[ends_world(2,3) ends_world(2,4)], ...
              'ZData',[ends_world(3,3) ends_world(3,4)]);

% Redraw the 2 diagonal arms to make the "X" shape visually thicker
% using the same endpoint data

set(h.arm(3), 'XData',[ends_world(1,1) ends_world(1,4)], ...
              'YData',[ends_world(2,1) ends_world(2,4)], ...
              'ZData',[ends_world(3,1) ends_world(3,4)]);

set(h.arm(4), 'XData',[ends_world(1,3) ends_world(1,2)], ...
              'YData',[ends_world(2,3) ends_world(2,2)], ...
              'ZData',[ends_world(3,3) ends_world(3,2)]);

% Update the central body position

set(h.body, 'XData',pos(1),'YData',pos(2),'ZData',pos(3));

end
