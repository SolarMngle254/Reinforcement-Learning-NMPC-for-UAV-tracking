% Author : Le Minh Nhat - K22 HCMUT Electrical & Electronics Faculty 
% Inspiration : https://www.youtube.com/@atvu5238/ (Dat Vu) 
% Date modified: 30/08/2026

function h = makeUAV(ax, s)

% Create a simple X-shaped UAV visualization: 4 arms + central body

% s: scale factor

arms = s * [ % Each column represents an arm endpoint in the body frame
     1  0  0;   -1  0  0;   0  1  0;   0 -1  0
]';

arms = reshape(arms, [3,4]);

% Create 4 line objects for the 4 arms
h.arm(1) = plot3(ax, [0 0],[0 0],[0 0],'k-','LineWidth',2); % Updated later
h.arm(2) = plot3(ax, [0 0],[0 0],[0 0],'k-','LineWidth',2);
h.arm(3) = plot3(ax, [0 0],[0 0],[0 0],'k-','LineWidth',2);
h.arm(4) = plot3(ax, [0 0],[0 0],[0 0],'k-','LineWidth',2);

% Central body represented by a circular marker
h.body = plot3(ax, 0,0,0,'o','MarkerSize',6,'MarkerFaceColor',[0 0.4 1],'MarkerEdgeColor','none');

% Store arm endpoints in the body frame to avoid recreating them
h.arm_body = arms;

end
