%% 3D arm with the base and the sideways offsets
% Convention: X = forward, Y = height, Z = depth (to the right when qb = 0)
% Arm frame (before the base rotation): [a ; h ; c] = [reach ; height ; side]
clear; clc; close all;

L1 = 153.86;   % shoulder -> elbow (mm)
L2 = 120.39;   % elbow -> wrist (mm)
L3 = 122;      % wrist -> tip (mm)
r0 = 10.25;    % shoulder in front of the base axis (mm)
h0 = 107.6;    % shoulder height (mm)
d_shoulder = 6.68;   % offset to the right at the 2nd motor (shoulder)
d_elbow    = 7.26;   % offset to the right at the elbow
c_tip = d_shoulder + d_elbow;   % total side offset of the arm tip

X = 200; Y = 150; Z = 80;      % target
phi = 0;                       % hand angle (0 = flat)

[qb, t1, t2, t3] = ik_full(X, Y, Z, phi, L1, L2, L3, r0, h0, c_tip);
P = fk3(t1, t2, t3, L1, L2, L3, qb, r0, h0, d_shoulder, d_elbow);
q_deg = mod(rad2deg([qb t1 t2 t3]) + 180, 360) - 180;   % brings each angle between -180 and 180
fprintf('angles (deg): base %.2f | shoulder %.2f | elbow %.2f | wrist %.2f\n', q_deg);
fprintf('arm tip: (%.2f ; %.2f ; %.2f)   target: (%.2f ; %.2f ; %.2f)\n', P(:,end), X, Y, Z);

figure;
plot3(P(1,:), P(3,:), P(2,:), 'o-', 'LineWidth', 2)   % X, Z, then Y upwards
hold on
plot3(X, Z, Y, 'rx', 'MarkerSize', 12, 'LineWidth', 2)
hold off
axis equal; grid on; view(35, 25)
xlabel('X (forward)'); ylabel('Z (depth)'); zlabel('Y (height)'); title('My arm')

%% ======================= functions =======================
function [qb, t1, t2, t3] = ik_full(X, Y, Z, phi, L1, L2, L3, r0, h0, c_tip)
rho = sqrt(X^2 + Z^2);                 % base -> target distance (seen from above)
if rho < abs(c_tip), error('Target too close to the base axis'); end
a  = sqrt(rho^2 - c_tip^2);            % real reach in the arm plane
qb = atan2(Z, X) - atan2(c_tip, a);    % aim at the target, then correct for the offset
[t1, t2, t3] = ik_3links(L1, L2, L3, a - r0, Y - h0, phi);
end

function [theta1, theta2, theta3] = ik_3links(L1, L2, L3, x, y, theta)
xw = x - L3*cos(theta);
yw = y - L3*sin(theta);
[theta1, theta2] = ik_2links(L1, L2, xw, yw);
theta3 = theta - theta1 - theta2;
end

function [theta1, theta2] = ik_2links(L1, L2, x, y)
alpha  = acos((L1^2 + L2^2 - x^2 - y^2) / (2*L1*L2));
theta2 = pi + alpha;
theta1 = atan2(y, x) - atan2(L2*sin(theta2), L1 + L2*cos(theta2));
end

function P = fk3(q2, q3, q4, L1, L2, L3, qb, r0, h0, d_shoulder, d_elbow)
% 1) points in the arm frame: [reach ; height ; side]
%    at each offset, a point shifted along the motor axis is added
e  = [r0; h0; 0];                                   % shoulder (on top of the base)
e2 = e  + [0; 0; d_shoulder];                       % offset at the 2nd motor
u  = e2 + L1*[cos(q2); sin(q2); 0];                 % elbow
u2 = u  + [0; 0; d_elbow];                          % offset at the elbow
v  = u2 + L2*[cos(q2+q3); sin(q2+q3); 0];           % wrist
w  = v  + L3*[cos(q2+q3+q4); sin(q2+q3+q4); 0];     % tip
B  = [[0;0;0] e e2 u u2 v w];                       % one column per point, from the floor

% 2) base rotation about Y: (a, c) rotate, the height does not change
A = B(1,:); H = B(2,:); C = B(3,:);
P = [A*cos(qb) - C*sin(qb);
     H;
     A*sin(qb) + C*cos(qb)];
end
