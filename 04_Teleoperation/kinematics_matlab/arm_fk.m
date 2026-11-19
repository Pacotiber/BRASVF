function [T, P_tool] = arm_fk(q_deg, P)
%ARM_FK Forward kinematics of the arm.
%   [T, P_tool] = ARM_FK(q_deg, P)
%   q_deg : [q1 q2 q3] or [q1 q2 q3 q4] in degrees (q4 = 0 if missing)
%   P     : parameters (arm_params)
%   T     : cell array of 4x4 homogeneous matrices, expressed in the base frame
%             T{1} base (floor)     T{2} after the base rotation
%             T{3} shoulder axis    T{4} elbow axis
%             T{5} wrist axis       T{6} tool (= wrist if P.L3 = 0)
%           For frames 3 to 6: x along the arm, y = pitch axis, z = x ^ y
%   P_tool : position [x y z] of the tool (centre of the wrist if P.L3 = 0)

if nargin < 2, P = arm_params(); end
q = deg2rad(q_deg(:).');
if numel(q) < 4, q(4) = 0; end

T = cell(1, 6);
T{1} = eye(4);
T{2} = T{1} * Rz(q(1));
T{3} = T{2} * Tr(P.r0, P.y_shoulder, P.z0) * Ry(-q(2));
T{4} = T{3} * Tr(P.L1, P.y_shoulder_elbow, 0) * Ry(-q(3));
T{5} = T{4} * Tr(P.L2, P.y_elbow_wrist, 0) * Ry(-q(4));
if P.L3 > 0
    T{6} = T{5} * Tr(P.L3, P.y_wrist_tool, 0);
else
    T{6} = T{5};
end
P_tool = T{6}(1:3, 4).';
end

% ---------------------------------------------------------------------------
function M = Rz(a)
M = [cos(a) -sin(a) 0 0; sin(a) cos(a) 0 0; 0 0 1 0; 0 0 0 1];
end

function M = Ry(a)
% rotation about y by an angle a; Ry(-q) raises the x axis by q
M = [cos(a) 0 sin(a) 0; 0 1 0 0; -sin(a) 0 cos(a) 0; 0 0 0 1];
end

function M = Tr(a, b, c)
M = [1 0 0 a; 0 1 0 b; 0 0 1 c; 0 0 0 1];
end
