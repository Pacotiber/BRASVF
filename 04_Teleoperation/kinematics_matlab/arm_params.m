function P = arm_params()
%ARM_PARAMS Arm dimensions, measured in the CAD model (first version of the arm assembly).
%   All lengths are in mm.
%
%   Base frame (frame 0):
%     origin : on the rotation axis of the base, at floor level
%     z      : up (axis of the base servo)
%     x      : forward (direction of the arm when q1 = 0)
%     y      : z ^ x = direction of the pitch axes (shoulder, elbow, wrist)
%
%   Angles:
%     q1 : base rotation about z
%     q2 : shoulder, angle of arm 1 relative to the horizontal, positive upwards
%     q3 : elbow, angle of arm 2 relative to arm 1, positive upwards
%     q4 : wrist, angle of the tool relative to arm 2, positive upwards

P.r0 = 10.25;      % base axis -> shoulder axis distance (common perpendicular)
P.z0 = 107.6;      % height of the shoulder axis above the floor
P.L1 = 153.86;     % shoulder axis -> elbow axis
P.L2 = 120.39;     % elbow axis -> wrist axis
P.L3 = 0;          % wrist -> tool (0 = no gripper; 125.27 = fingertips)

% sideways offsets along y (pitch axes), measured at the servo horns
P.y_shoulder       = -32.0;   % shoulder relative to the base axis
P.y_shoulder_elbow = +49.5;   % elbow relative to the shoulder
P.y_elbow_wrist    = -7.25;   % wrist relative to the elbow
P.y_wrist_tool     = -30.62;  % gripper tip relative to the wrist (if L3 = 125.27)

% pose of the CAD model (deg), to check the computations
P.q_cad = [0, 57.03, 8.62, -54.03];

% limits (deg): to be measured on the real arm, provisional values
P.q_min = [-90, -10, -150, -120];
P.q_max = [ 90, 170,  150,  120];
end
