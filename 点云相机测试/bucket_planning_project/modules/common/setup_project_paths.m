function ctx = setup_project_paths()
% setup_project_paths:
% Add module paths and return frequently used project paths.

thisFile = mfilename('fullpath');
commonDir = fileparts(thisFile);
modulesDir = fileparts(commonDir);
projectRoot = fileparts(modulesDir);

addpath(modulesDir);
addpath(genpath(modulesDir));

ctx = struct();
ctx.projectRoot = projectRoot;
ctx.modulesDir = modulesDir;
ctx.mainDir = fullfile(projectRoot, 'main');
ctx.dataDir = fullfile(projectRoot, 'data');
ctx.legacyDir = fullfile(projectRoot, 'legacy');
ctx.urdfFile = fullfile(projectRoot, 'ur10_world.urdf');

if ~isfolder(ctx.dataDir)
    mkdir(ctx.dataDir);
end
if ~isfolder(ctx.legacyDir)
    mkdir(ctx.legacyDir);
end
end

