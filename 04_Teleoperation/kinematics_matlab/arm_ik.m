function [q_deg, ok, msg] = arm_ik(target, P, elbow, phi_deg)
%ARM_IK Analytical inverse kinematics (closed-form formulas).
%   [q_deg, ok, msg] = ARM_IK(target, P, elbow, phi_deg)
%   target  : [x y z] in mm, in the base frame
%             = centre of the wrist if P.L3 = 0, tip of the tool otherwise
%   P       : parameters (arm_params)
%   elbow   : 'up' (default) or 'down'
%   phi_deg : wanted tool pitch relative to the horizontal (default 0)
%             used for q4, and to step back from the tool tip to the wrist if P.L3 > 0
%   q_deg   : [q1 q2 q3 q4] in degrees
%   ok      : false if the target is out of reach or outside the limits

if nargin < 2 || isempty(P), P = arm_params(); end
if nargin < 3 || isempty(elbow), elbow = 'up'; end
if nargin < 4 || isempty(phi_deg), phi_deg = 0; end
q_deg = nan(1, 4); ok = false; msg = '';
x = target(1); y = target(2); z = target(3);
phi = deg2rad(phi_deg);

% total sideways offset of the aimed point (along y, constant)
d = P.y_shoulder + P.y_shoulder_elbow + P.y_elbow_wrist;
if P.L3 > 0, d = d + P.y_wrist_tool; end

% 1) base: take the sideways offset d into account
rho2 = x^2 + y^2;
if rho2 < d^2
    msg = 'target too close to the base axis'; return
end
r = sqrt(rho2 - d^2);
q1 = atan2(y, x) - atan2(d, r);

% 2) arm plane: step back from the tool tip to the wrist
r_w = r - P.L3 * cos(phi);
z_w = z - P.L3 * sin(phi);
u = r_w - P.r0;
v = z_w - P.z0;

% 3) elbow: law of cosines
D = (u^2 + v^2 - P.L1^2 - P.L2^2) / (2 * P.L1 * P.L2);
if abs(D) > 1
    msg = sprintf('out of reach (shoulder-wrist distance %.1f mm, reach %.1f to %.1f mm)', ...
                  hypot(u, v), abs(P.L1 - P.L2), P.L1 + P.L2);
    return
end
s = sqrt(1 - D^2);
if strcmpi(elbow, 'up'), s = -s; end
q3 = atan2(s, D);

% 4) shoulder
q2 = atan2(v, u) - atan2(P.L2 * sin(q3), P.L1 + P.L2 * cos(q3));

% 5) wrist: wanted tool pitch
q4 = phi - q2 - q3;

q_deg = rad2deg([q1 q2 q3 q4]);
q_deg(1) = mod(q_deg(1) + 180, 360) - 180;
outside = q_deg < P.q_min | q_deg > P.q_max;
if any(outside)
    msg = sprintf('outside the limits: joint(s) %s', mat2str(find(outside)));
    return
end
ok = true;
end
