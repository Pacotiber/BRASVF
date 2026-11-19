%DEMO_ARM Arm kinematic model: checks + interactive interface.
%   Run this script from this folder:  >> demo_arm
clear; clc;
P = arm_params();

%% 1) Check with the pose of the CAD model
[~, wrist] = arm_fk(P.q_cad(1:3), P);
fprintf('CAD pose q = [%.2f %.2f %.2f] deg\n', P.q_cad(1:3));
fprintf('  computed wrist centre : (%.2f, %.2f, %.2f) mm\n', wrist);
fprintf('  expected (CAD measure): (143.62, 10.25, 346.35) mm\n');

[q_ik, ok] = arm_ik(wrist, P, 'down');
fprintf('Inverse kinematics of this point (elbow ''down''): q = [%.2f %.2f %.2f] deg  (ok = %d)\n\n', ...
        q_ik(1:3), ok);

%% 2) Round-trip test on random poses
rng(1); err_max = 0; n = 0;
for k = 1:500
    q = P.q_min(1:3) + rand(1, 3) .* (P.q_max(1:3) - P.q_min(1:3));
    [~, p] = arm_fk(q, P);
    for c = {'up', 'down'}
        [q2, ok] = arm_ik(p, P, c{1});
        if ok
            [~, p2] = arm_fk(q2(1:3), P);
            err_max = max(err_max, norm(p2 - p)); n = n + 1;
        end
    end
end
fprintf('Round trip forward -> inverse -> forward: %d solutions, max error %.2e mm\n\n', n, err_max);

%% 3) Interface: one slider per joint + target for the inverse kinematics
fig = figure('Name', 'Arm kinematics', 'Color', 'w', 'Position', [100 100 1100 700]);
ax = axes('Parent', fig, 'Position', [0.05 0.25 0.6 0.7]);
state.q = P.q_cad;
state.P = P;
guidata(fig, state);

names = {'q1 base', 'q2 shoulder', 'q3 elbow', 'q4 wrist'};
for k = 1:4
    uicontrol(fig, 'Style', 'text', 'Units', 'normalized', 'Position', [0.68 0.88-0.08*k 0.1 0.04], ...
              'String', names{k}, 'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
    uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', 'Position', [0.78 0.88-0.08*k 0.18 0.04], ...
              'Min', P.q_min(k), 'Max', P.q_max(k), 'Value', state.q(k), 'Tag', sprintf('q%d', k), ...
              'Callback', @update_sliders);
end

uicontrol(fig, 'Style', 'text', 'Units', 'normalized', 'Position', [0.68 0.40 0.3 0.04], ...
          'String', 'Target (x y z in mm) for the inverse kinematics:', 'BackgroundColor', 'w', ...
          'HorizontalAlignment', 'left');
uicontrol(fig, 'Style', 'edit', 'Units', 'normalized', 'Position', [0.68 0.34 0.2 0.05], ...
          'String', sprintf('%.1f %.1f %.1f', wrist), 'Tag', 'target');
uicontrol(fig, 'Style', 'popupmenu', 'Units', 'normalized', 'Position', [0.89 0.34 0.08 0.05], ...
          'String', {'up', 'down'}, 'Tag', 'elbow');
uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.68 0.27 0.29 0.05], ...
          'String', 'Go to the target', 'Callback', @go_to_target);
uicontrol(fig, 'Style', 'text', 'Units', 'normalized', 'Position', [0.68 0.15 0.3 0.1], ...
          'String', '', 'Tag', 'message', 'BackgroundColor', 'w', 'HorizontalAlignment', 'left');

arm_plot(state.q, P, ax);

% ---------------------------------------------------------------------------
function update_sliders(src, ~)
fig = ancestor(src, 'figure'); state = guidata(fig);
for k = 1:4, state.q(k) = get(findobj(fig, 'Tag', sprintf('q%d', k)), 'Value'); end
guidata(fig, state);
arm_plot(state.q, state.P, findobj(fig, 'Type', 'axes'));
end

function go_to_target(src, ~)
fig = ancestor(src, 'figure'); state = guidata(fig);
target = sscanf(get(findobj(fig, 'Tag', 'target'), 'String'), '%f').';
choices = get(findobj(fig, 'Tag', 'elbow'), 'String');
elbow = choices{get(findobj(fig, 'Tag', 'elbow'), 'Value')};
[q, ok, msg] = arm_ik(target, state.P, elbow, 0);
msgh = findobj(fig, 'Tag', 'message');
if ~ok
    set(msgh, 'String', ['Impossible: ' msg], 'ForegroundColor', [0.8 0 0]); return
end
set(msgh, 'String', sprintf('q = [%.1f  %.1f  %.1f  %.1f] deg', q), 'ForegroundColor', [0 0.5 0]);
state.q = q; guidata(fig, state);
for k = 1:4, set(findobj(fig, 'Tag', sprintf('q%d', k)), 'Value', q(k)); end
arm_plot(state.q, state.P, findobj(fig, 'Type', 'axes'));
end
