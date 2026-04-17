clc;
clear;
close all;

% synthetic_cloud_gen:
% 生成模拟场景点云并保存，便于在无真实相机时调试处理流程。

rng(42);
outPath = fullfile(fileparts(mfilename("fullpath")), "synthetic_scene.pcd");

% 1) 地面点云 (x-y 平面, z≈0)
groundN = 12000;
gx = -2 + 4 * rand(groundN, 1);
gy = -2 + 4 * rand(groundN, 1);
gz = 0.002 * randn(groundN, 1);
ground = [gx, gy, gz];

% 2) 立方体障碍 A
cubeA.center = [0.6, -0.4, 0.3];
cubeA.size = [0.5, 0.4, 0.6];
cubeAPoints = sampleBoxSurface(cubeA.center, cubeA.size, 3500);

% 3) 立方体障碍 B
cubeB.center = [-0.7, 0.8, 0.45];
cubeB.size = [0.35, 0.35, 0.9];
cubeBPoints = sampleBoxSurface(cubeB.center, cubeB.size, 3000);

% 4) 圆柱障碍 C
cyl.center = [-0.2, -0.9, 0.4];
cyl.radius = 0.22;
cyl.height = 0.8;
cylPoints = sampleCylinderSurface(cyl.center, cyl.radius, cyl.height, 3200);

% 5) 合并并加测量噪声
xyz = [ground; cubeAPoints; cubeBPoints; cylPoints];
xyz = xyz + 0.003 * randn(size(xyz));

ptCloud = pointCloud(xyz);
pcwrite(ptCloud, outPath, "Encoding", "ascii");

fprintf("[synthetic_cloud_gen] Saved: %s\n", outPath);
fprintf("[synthetic_cloud_gen] Total points: %d\n", ptCloud.Count);

figure("Name", "Synthetic Scene", "Color", "w");
pcshow(ptCloud);
title("Synthetic Obstacle Point Cloud");
xlabel("X (m)"); ylabel("Y (m)"); zlabel("Z (m)");
view(3); axis equal;

function pts = sampleBoxSurface(c, s, n)
% 在长方体6个面上均匀采样
half = s / 2;
faceId = randi(6, n, 1);
u = rand(n, 1);
v = rand(n, 1);
pts = zeros(n, 3);

for i = 1:n
    switch faceId(i)
        case 1 % +x
            pts(i, :) = [half(1), (u(i)-0.5)*s(2), (v(i)-0.5)*s(3)];
        case 2 % -x
            pts(i, :) = [-half(1), (u(i)-0.5)*s(2), (v(i)-0.5)*s(3)];
        case 3 % +y
            pts(i, :) = [(u(i)-0.5)*s(1), half(2), (v(i)-0.5)*s(3)];
        case 4 % -y
            pts(i, :) = [(u(i)-0.5)*s(1), -half(2), (v(i)-0.5)*s(3)];
        case 5 % +z
            pts(i, :) = [(u(i)-0.5)*s(1), (v(i)-0.5)*s(2), half(3)];
        case 6 % -z
            pts(i, :) = [(u(i)-0.5)*s(1), (v(i)-0.5)*s(2), -half(3)];
    end
end

pts = pts + c;
end

function pts = sampleCylinderSurface(c, r, h, n)
% 侧面+上下底面采样
nSide = round(0.7 * n);
nCap = n - nSide;

theta = 2*pi*rand(nSide, 1);
zSide = -h/2 + h*rand(nSide, 1);
xSide = r*cos(theta);
ySide = r*sin(theta);
sidePts = [xSide, ySide, zSide];

nTop = floor(nCap/2);
nBot = nCap - nTop;

thetaTop = 2*pi*rand(nTop, 1);
rhoTop = r*sqrt(rand(nTop, 1));
topPts = [rhoTop.*cos(thetaTop), rhoTop.*sin(thetaTop), (h/2)*ones(nTop, 1)];

thetaBot = 2*pi*rand(nBot, 1);
rhoBot = r*sqrt(rand(nBot, 1));
botPts = [rhoBot.*cos(thetaBot), rhoBot.*sin(thetaBot), (-h/2)*ones(nBot, 1)];

pts = [sidePts; topPts; botPts] + c;
end
