function lim = collect_joint_limits(robot)
% collect_joint_limits:
% Collect non-fixed joint limits from rigidBodyTree.

lim = zeros(0, 2);
for i = 1:robot.NumBodies
    jointObj = robot.Bodies{i}.Joint;
    if strcmp(jointObj.Type, 'fixed')
        continue;
    end

    l = jointObj.PositionLimits;
    if ~isfinite(l(1)) || ~isfinite(l(2)) || (l(1) >= l(2))
        l = [-pi, pi];
    end
    lim = [lim; l]; %#ok<AGROW>
end
end

