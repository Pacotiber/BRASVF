function h = arm_plot(q_deg, P, ax, frame_size)
%ARM_PLOT Draws the arm and the frame of each joint.
%   h = ARM_PLOT(q_deg, P, ax, frame_size)
%   Frames: x in red, y in green, z in blue.
%   The rotation axis of each joint is z for the base and y for the pitch joints.

if nargin < 2 || isempty(P), P = arm_params(); end
if nargin < 3 || isempty(ax), figure('Color', 'w'); ax = axes; end
if nargin < 4, frame_size = 40; end

[T, P_tool] = arm_fk(q_deg, P);
cla(ax); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
xlabel(ax, 'x (mm)'); ylabel(ax, 'y (mm)'); zlabel(ax, 'z (mm)');

% floor and base axis
patch(ax, [-150 300 300 -150], [-150 -150 150 150], [0 0 0 0], [0.92 0.92 0.92], ...
      'EdgeColor', 'none', 'FaceAlpha', 0.6);
plot3(ax, [0 0], [0 0], [0 P.z0], 'k--', 'LineWidth', 1);

% links (through the origins of the frames)
pts = cell2mat(cellfun(@(M) M(1:3, 4).', T([1 3 4 5 6]), 'UniformOutput', false).');
pts = [0 0 0; 0 0 P.z0 * 0.6; pts(2:end, :)];
h.links = plot3(ax, pts(:, 1), pts(:, 2), pts(:, 3), '-', 'Color', [0.2 0.2 0.2], 'LineWidth', 4);
h.joints = plot3(ax, pts(3:end-1, 1), pts(3:end-1, 2), pts(3:end-1, 3), 'o', ...
      'MarkerSize', 9, 'MarkerFaceColor', [1 0.8 0], 'MarkerEdgeColor', 'k');
plot3(ax, P_tool(1), P_tool(2), P_tool(3), 'p', 'MarkerSize', 14, ...
      'MarkerFaceColor', 'm', 'MarkerEdgeColor', 'k');

% frames
names = {'0 base', '1', '2 shoulder', '3 elbow', '4 wrist', '5 tool'};
colours = {'r', [0 0.6 0], 'b'};
to_draw = [1 3 4 5];
if P.L3 > 0, to_draw = [to_draw 6]; end
for k = to_draw
    o = T{k}(1:3, 4);
    for c = 1:3
        v = T{k}(1:3, c) * frame_size;
        quiver3(ax, o(1), o(2), o(3), v(1), v(2), v(3), 0, 'Color', colours{c}, ...
                'LineWidth', 2, 'MaxHeadSize', 0.5);
    end
    text(ax, o(1), o(2), o(3) + 12, names{k}, 'FontSize', 9, 'FontWeight', 'bold');
end

title(ax, sprintf('q = [%.1f  %.1f  %.1f  %.1f] deg   |   tool = (%.1f, %.1f, %.1f) mm', ...
      [q_deg(:).' zeros(1, 4 - numel(q_deg))], P_tool));
view(ax, 35, 20);
xlim(ax, [-150 350]); ylim(ax, [-200 200]); zlim(ax, [0 450]);
hold(ax, 'off');
end
