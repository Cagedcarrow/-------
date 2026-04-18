function [robot, ik, jointLimits, qHome, info] = load_robot_and_base(opts)
% load_robot_and_base:
% Import robot model, apply base transform, and initialize IK context.

if nargin < 1
    opts = struct();
end

if ~isfield(opts, 'urdfFile') || isempty(opts.urdfFile)
    ctx = setup_project_paths();
    opts.urdfFile = ctx.urdfFile;
end
if ~isfield(opts, 'baseFrame') || isempty(opts.baseFrame)
    opts.baseFrame = 'world_base';
end
if ~isfield(opts, 'baseLink') || isempty(opts.baseLink)
    opts.baseLink = 'ur10';
end
if ~isfield(opts, 'endEffector') || isempty(opts.endEffector)
    opts.endEffector = 'shovel_tip';
end
if ~isfield(opts, 'baseXYZ') || isempty(opts.baseXYZ)
    opts.baseXYZ = [0, 0, 1.40];
end
if ~isfield(opts, 'baseYPR') || isempty(opts.baseYPR)
    opts.baseYPR = [0, 0, pi];
end

fprintf('[common.load_robot] URDF=%s\n', opts.urdfFile);
if ~isfile(opts.urdfFile)
    error('[common.load_robot] URDF not found: %s', opts.urdfFile);
end

robot = importrobot(opts.urdfFile);
robot.DataFormat = 'row';

if ~any(strcmp(robot.BodyNames, opts.baseLink))
    error('[common.load_robot] Base link not found: %s', opts.baseLink);
end
if ~any(strcmp(robot.BodyNames, opts.endEffector))
    error('[common.load_robot] End effector not found: %s', opts.endEffector);
end

T_world_to_baseLink = trvec2tform(opts.baseXYZ) * eul2tform(opts.baseYPR, 'ZYX');
baseBody = getBody(robot, opts.baseLink);
setFixedTransform(baseBody.Joint, T_world_to_baseLink);

ik = inverseKinematics('RigidBodyTree', robot);
jointLimits = collect_joint_limits(robot);
qHome = clamp_to_limits(homeConfiguration(robot), jointLimits);

info = struct();
info.baseFrame = opts.baseFrame;
info.baseLink = opts.baseLink;
info.endEffector = opts.endEffector;
info.baseXYZ = opts.baseXYZ;
info.baseYPR = opts.baseYPR;
info.T_world_to_baseLink = T_world_to_baseLink;
end

