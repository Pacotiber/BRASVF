figure;
% dimensions measured in the first arm assembly (mm)
L1 = 153.86;       % shoulder axis -> 3rd motor axis
L2 = 120.39;       % 3rd motor axis -> last servo axis
L3 = 122.65;       % last servo -> fingertips, along the gripper axis
h_gripper = 14.00; % finger plane offset perpendicular to the gripper axis (+ = upper side when the gripper is flat)
opening = 20;      % gap between the two fingers (mm), for the drawing only
r0 = 10.25;        % shoulder axis in front of the base axis
h0 = 107.6;        % height of the shoulder axis above the floor
d_shoulder = 7.25; % offset to the right at the 2nd motor (shoulder)
d_elbow    = 7.25; % offset to the right at the 3rd motor
d_gripper  = 7.25; % offset to the right at the last servo (the gripper is rigid with it)
c_tip = d_shoulder + d_elbow + d_gripper;   % 21.75: total offset of the tip (mid-plane of the gripper)
qmin = deg2rad([-90  -10 -150 -120]);   % limits: base, shoulder, 3rd motor, last servo
qmax = deg2rad([ 90  170  150  120]);
r_motor = 12;      % radius of the cylinders (mm)
l_motor = 40;      % length of the cylinders (mm)

% check: current pose of the CAD model -> the axis of the last servo must land at (198.59 ; 306.92)
[~,~,~,~,v_cad] = fk3(deg2rad(46.107), deg2rad(1.176), deg2rad(-15.407), L1, L2, L3, h_gripper, 0, r0, h0, d_shoulder, d_elbow, d_gripper);
disp(v_cad')

X = 200; Y = 150; Z = 80;    % target
[qb,theta1,theta2,theta3,ok] = ik_safe(X, Y, Z, L1, L2, L3, h_gripper, r0, h0, d_shoulder, d_elbow, d_gripper, qmin, qmax);
if ~ok, error('Target cannot be reached'); end
disp(rad2deg([qb theta1 theta2 theta3]))   % angles in degrees
[e,e2,u,u2,v,v2,p,w] = fk3(theta1, theta2,theta3, L1, L2,L3,h_gripper,qb,r0,h0,d_shoulder,d_elbow,d_gripper);
disp(w)    % must give the target back

Pts = [[0;0;0] e e2 u u2 v v2 p w];   % one column per point, from the floor
plot3(Pts(1,:), Pts(3,:), Pts(2,:), 'o-', 'LineWidth', 2)   % X, Z, then Y upwards
axis equal
hold on                            % keep what is already displayed
plot3(X, Z, Y, 'rx', 'MarkerSize', 12, 'LineWidth', 2)
patch([-300 300 300 -300], [-300 -300 300 300], [0 0 0 0], [0.9 0.9 0.9], 'FaceAlpha', 0.5)   % the floor

% motors: one cylinder on the axis of each joint
side = [-sin(qb); 0; cos(qb)];                                       % axis of the arm motors
cylinder_at([0; h0/2; 0], [0; 1; 0], r_motor, l_motor, [0.2 0.2 0.2])   % base (vertical axis)
cylinder_at((e+e2)/2,   side, r_motor, l_motor, [0.2 0.2 0.2])          % 2nd motor (shoulder)
cylinder_at((u+u2)/2,   side, r_motor, l_motor, [0.2 0.2 0.2])          % 3rd motor
cylinder_at((v+v2)/2,   side, r_motor, l_motor, [0.2 0.2 0.2])          % last servo (gripper)

% gripper: frame and fingers with their real dimensions (gripper frame: along, perpendicular, side)
q = theta1 + theta2 + theta3;                                        % angle of the gripper axis
box_at(v2, q, qb, [29.25 79.45], [6.5 24.72], [-26.01 26.01], [0.6 0.6 0.6])                    % frame
box_at(v2, q, qb, [L3-40.9 L3], [h_gripper-1.5 h_gripper+1.5], [ opening/2  opening/2+9.5], [0.9 0.5 0.1])   % right finger
box_at(v2, q, qb, [L3-40.9 L3], [h_gripper-1.5 h_gripper+1.5], [-opening/2-9.5 -opening/2], [0.9 0.5 0.1])   % left finger
hold off
zlim([-50 450])
set(gca, 'YDir', 'reverse')   % puts Z back in the right direction on screen (cancels the mirror effect)
camlight; lighting gouraud    % a little light to see the shape of the cylinders

grid on
xlabel('X (forward)'); ylabel('Z (depth)'); title('My arm');zlabel('Y (height)')
legend('arm', 'target', 'floor')

function cylinder_at(centre, axis_dir, radius, len, colour)
% draws a cylinder centred on "centre", oriented along "axis_dir" (coordinates [X;Y;Z])
[xc, yc, zc] = cylinder(radius, 30);       % MATLAB cylinder: vertical axis, height 0 to 1
zc = (zc - 0.5) * len;                     % centre it and give it its length
axis_dir = axis_dir / norm(axis_dir);
n1 = cross(axis_dir, [1; 0; 0]);           % two directions perpendicular to the axis
if norm(n1) < 0.1, n1 = cross(axis_dir, [0; 1; 0]); end
n1 = n1 / norm(n1);
n2 = cross(axis_dir, n1);
pts = n1*xc(:)' + n2*yc(:)' + axis_dir*zc(:)' + centre;   % rotate then move each point
Xc = reshape(pts(1,:), size(xc));
Yc = reshape(pts(2,:), size(xc));
Zc = reshape(pts(3,:), size(xc));
surf(Xc, Zc, Yc, 'FaceColor', colour, 'EdgeColor', 'none', 'HandleVisibility', 'off')   % X, Z, Y
fill3(Xc(1,:), Zc(1,:), Yc(1,:), colour, 'EdgeColor', 'none', 'HandleVisibility', 'off')  % caps
fill3(Xc(2,:), Zc(2,:), Yc(2,:), colour, 'EdgeColor', 'none', 'HandleVisibility', 'off')
end

function box_at(v2, q, qb, A, H, C, colour)
% draws a box in the gripper frame: A = [start end] along the gripper,
% H = [start end] perpendicular, C = [start end] sideways, starting from point v2
along = [cos(q)*cos(qb);  sin(q);  cos(q)*sin(qb)];     % gripper axis
perp  = [-sin(q)*cos(qb); cos(q); -sin(q)*sin(qb)];     % perpendicular (in the arm plane)
side  = [-sin(qb); 0; cos(qb)];                         % side
corners = zeros(3, 8); k = 0;
for c = C, for h = H, for a = A
    k = k + 1; corners(:,k) = v2 + a*along + h*perp + c*side;
end, end, end
faces = [1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];
patch('Vertices', corners([1 3 2],:)', 'Faces', faces, 'FaceColor', colour, 'EdgeColor', 'k', 'HandleVisibility', 'off')   % X, Z, Y
end

function [qb,a] = ik3D( x, z)
qb = atan2(z, x);
a=sqrt(x^2+z^2);
end

function [u,v]= fk2(q2, q3, L1, L2)
u=L1*[cos(q2);sin(q2)];
v=u+L2*[cos(q2+q3);sin(q2+q3)];
end

function [qb,t1,t2,t3,ok] = ik_safe(X, Y, Z, L1, L2, L3, h_gripper, r0, h0, d_shoulder, d_elbow, d_gripper, qmin, qmax)
c_tip = d_shoulder + d_elbow + d_gripper;
ok = false; qb = NaN; t1 = NaN; t2 = NaN; t3 = NaN;
if sqrt(X^2 + Z^2) < c_tip, return; end       % too close to the base axis
phis = deg2rad([0 -5 5 -10 10 -15 15 -20 20 -30 30 -45 45 -60 60 -75 75 -90 90]);
for phi = phis                                % first the gripper flat, then tilted more and more
    for elbow = [-1 1]                        % both elbow solutions
        [qb, t1, t2, t3] = ik_full(X, Y, Z, phi, L1, L2, L3, h_gripper, r0, h0, c_tip, elbow);
        q = mod([qb t1 t2 t3] + pi, 2*pi) - pi;    % angles between -pi and pi
        qb = q(1); t1 = q(2); t2 = q(3); t3 = q(4);
        if any(isnan(q)) || any(q < qmin) || any(q > qmax), continue; end   % limits
        [e,e2,u,u2,v,v2,p,w] = fk3(t1,t2,t3,L1,L2,L3,h_gripper,qb,r0,h0,d_shoulder,d_elbow,d_gripper);
        if min([e(2) e2(2) u(2) u2(2) v(2) v2(2) p(2) w(2)]) < 0, continue; end   % below the floor
        ok = true; return
    end
end
end

function [qb, t1, t2, t3] = ik_full(X, Y, Z, phi, L1, L2, L3, h_gripper, r0, h0, c_tip, elbow)
rho = sqrt(X^2 + Z^2);                 % 1) base -> target distance
a   = sqrt(rho^2 - c_tip^2);           % 2) distance in front of the arm (Pythagoras)
qb  = atan2(Z, X) - atan2(c_tip, a);   % 3) turn the base, corrected for the offset
xb = a - r0;                           % 4) distance measured from the shoulder
yb = Y - h0;                           %    height measured from the shoulder
[t1, t2, t3] = ik_3links(L1, L2, L3, h_gripper, xb, yb, phi, elbow);   % 5) the planar arm
end

function [theta1,theta2,theta3] = ik_3links(L1,L2,L3,h_gripper,x,y,theta,elbow)
% theta = angle of the gripper axis relative to the horizontal
xw=x-L3*cos(theta)+h_gripper*sin(theta);   % step back along the gripper...
yw=y-L3*sin(theta)-h_gripper*cos(theta);   % ...then remove the perpendicular offset
[theta1,theta2]=ik_2links(L1,L2,xw,yw,elbow);
theta3=theta-theta1-theta2;
end

function [theta1,theta2] = ik_2links(L1,L2,x,y,elbow)
D = (L1^2+L2^2-x^2-y^2)/(2*L1*L2);
if abs(D) > 1                          % out of reach: acos would give a complex number
    theta1 = NaN; theta2 = NaN; return
end
alpha = acos(D);
theta2 = elbow*(pi-alpha);             % elbow = -1: the (pi+alpha) solution, elbow = +1: the other one
theta1 = atan2(y,x) - atan2(L2*sin(theta2), L1 + L2*cos(theta2));
end

function [e,e2,u,u2,v,v2,p,w]= fk3(q2, q3,q4,L1, L2, L3,h_gripper,qb,r0,h0,d_shoulder,d_elbow,d_gripper)
side = [-sin(qb); 0; cos(qb)];     % "to the right" direction of the arm, after the base rotation
q = q2+q3+q4;                      % angle of the gripper axis relative to the horizontal
e  = [r0*cos(qb); h0; r0*sin(qb)];                                       % 2nd motor (shoulder)
e2 = e  + d_shoulder*side;                                               % offset at the shoulder
u  = e2 + L1*[cos(q2)*cos(qb);        sin(q2);        cos(q2)*sin(qb)];   % 3rd motor
u2 = u  + d_elbow*side;                                                  % offset at the 3rd motor
v  = u2 + L2*[cos(q2+q3)*cos(qb);     sin(q2+q3);     cos(q2+q3)*sin(qb)];   % last servo
v2 = v  + d_gripper*side;                                                % sideways offset of the gripper
p  = v2 + h_gripper*[-sin(q)*cos(qb);   cos(q);         -sin(q)*sin(qb)];  % perpendicular offset of the gripper
w  = p  + L3*[cos(q)*cos(qb);         sin(q);         cos(q)*sin(qb)];   % fingertips
end
