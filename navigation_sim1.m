function navigation_sim1()
clear; clc; close all;

%% ====== MAP / ROBOT PARAMETERS ======
Nx = 30;           
Ny = 20;           
robotPos = [3, 3]; % [x,y] in grid coordinates 
robotStep = 0.35;  % movement speed per iteration
waypointTol = 0.4; % how close to waypoint before advancing
maxIters = 999999;

% occupancy map (0 free, 1 obstacle)
map = zeros(Ny, Nx);

% obstacles
map(5:10, 10:15) = 1;     % center-ish
map(12:15, 3:8)  = 1;     % bottom-left-ish
map(3:6, 20:26)  = 1;     % top-right-ish

%% ====== PYTHON GAZE INPUT ======
% read screen_position.txt in the same folder as this .m file
baseDir  = fileparts(mfilename('fullpath'));
gazeFile = fullfile(baseDir, 'screen_position.txt');

% gaze smoothing + dwell/lock
alpha = 0.35;            % EMA smoothing (0..1). 
dwellSec = 0.60;         % how long gaze must be stable to lock target
stableRadiusCells = 2;   % stability threshold in grid cells
onlyLockWhenIdle = true; % don't change target while robot is moving

% State
gazeTarget = [];         % moving gaze cell [x,y]
targetPos  = [];         % locked target cell [x,y]
plannedPath = [];        % Nx2 list of waypoints [x,y]
pathIndex  = 1;

% For dwell
lastGazeCell = [];
stableStartT = NaN;

% For smoothing
uvSmooth = [0.5, 0.5];

%% ====== LIDAR SIM PARAMS ======
anglesDeg = 0:10:350;
lidarMaxRange = 12; % cells

%% ====== FIGURE SETUP ======
figure('Name','Gaze-Controlled Navigation','Color','none');
set(gcf,'Position',[100,100,1200,520]);

%% ====== MAIN LOOP ======
for iter = 1:maxIters

    %% 1) READ (u,v) FROM FILE
    [ok, u, v] = readGazeUV(gazeFile);
    if ok
        % EMA smoothing to reduce jitter
        uvSmooth = alpha*[u,v] + (1-alpha)*uvSmooth;
        uS = uvSmooth(1);
        vS = uvSmooth(2);

        % Map normalized gaze to grid cell
        gazeTarget = uvToGrid(uS, vS, Nx, Ny);
    else
        gazeTarget = [];
    end

    %% 2) DWELL-LOCK TARGET (only if gaze valid)
    if ~isempty(gazeTarget)

        % If we only lock when idle, skip locking while following a path
        if ~(onlyLockWhenIdle && ~isempty(plannedPath))

            if isempty(lastGazeCell)
                lastGazeCell = gazeTarget;
                stableStartT = tic;
            else
                if norm(gazeTarget - lastGazeCell) <= stableRadiusCells
                    % stable
                    if toc(stableStartT) >= dwellSec
                        % Candidate locked target
                        candidate = gazeTarget;

                        % Reject if inside obstacle
                        if isOccupied(map, candidate)
                            % ignore
                        else
                            % Accept new target
                            targetPos = candidate;

                            % Plan path
                            startCell = round(robotPos);
                            goalCell  = targetPos;
                            plannedPath = astarGrid(map, startCell, goalCell);
                            pathIndex = 1;

                            % If no path found, clear target
                            if isempty(plannedPath)
                                targetPos = [];
                            end
                        end

                        % Reset dwell timer to avoid re-triggering constantly
                        lastGazeCell = gazeTarget;
                        stableStartT = tic;
                    end
                else
                    % moved too much, restart dwell
                    lastGazeCell = gazeTarget;
                    stableStartT = tic;
                end
            end
        end
    else
        lastGazeCell = [];
        stableStartT = NaN;
    end

    %% 3) FOLLOW PATH
    if ~isempty(plannedPath)
        % If finished, stop
        if pathIndex > size(plannedPath,1)
            plannedPath = [];
            pathIndex = 1;
        else
            nextCell = plannedPath(pathIndex, :);

            % Move toward next waypoint
            robotPos = robotPos + robotStep * (nextCell - robotPos);

            % Advance waypoint if close enough
            if norm(robotPos - nextCell) < waypointTol
                pathIndex = pathIndex + 1;
            end
        end
    end

    %% 4) DRAW MAP + ROBOT + LIDAR + TARGETS
    subplot(1,2,1); cla;

    imagesc(map);
    colormap(gray);
    axis equal tight;
    set(gca,'YDir','normal');
    title('Gaze-Controlled Navigation');
    hold on;

    % Draw planned path
    if ~isempty(plannedPath)
        plot(plannedPath(:,1), plannedPath(:,2), 'y-', 'LineWidth', 2);
    end

    % Draw robot
    plot(robotPos(1), robotPos(2), 'bo', 'MarkerSize', 12, 'LineWidth', 2);

    % Draw moving gaze (dot)
    if ~isempty(gazeTarget)
        plot(gazeTarget(1), gazeTarget(2), 'r.', 'MarkerSize', 22);
    end

    % Draw locked target (circle)
    if ~isempty(targetPos)
        plot(targetPos(1), targetPos(2), 'ro', 'MarkerSize', 12, 'LineWidth', 2);
    end

    % Draw LiDAR rays
    drawLidar(map, robotPos, anglesDeg, lidarMaxRange);

    hold off;

    %% 5) DRAW GAZE INPUT PANEL (normalized)
    subplot(1,2,2); cla;
    title('Python Gaze Input');
    axis([0 1 0 1]);
    grid on;
    hold on;

    if ok
        plot(uS, 1-vS, 'r.', 'MarkerSize', 25); % invert for display
        text(0.02, 0.95, sprintf("u=%.2f  v=%.2f", uS, vS), 'Color','k');
    else
        text(0.02, 0.5, "No gaze data", 'Color','k');
    end

    if ~isempty(targetPos)
        text(0.02, 0.90, "Target: LOCKED", 'Color','k');
    else
        text(0.02, 0.90, "Target: none", 'Color','k');
    end

    hold off;

    drawnow;

    pause(0.03); % ~30 FPS loop
end

end

%% ====== HELPERS ======

function [ok, u, v] = readGazeUV(gazeFile)
ok = false; u = NaN; v = NaN;
if ~isfile(gazeFile), return; end
s = strtrim(fileread(gazeFile));
if isempty(s), return; end
parts = split(s, ',');
if numel(parts) < 3, return; end
u = str2double(parts{2});
v = str2double(parts{3});
if any(isnan([u v])), return; end
if ~(u>=0 && u<=1 && v>=0 && v<=1), return; end
ok = true;
end

function cellXY = uvToGrid(u, v, Nx, Ny)
% u: left->right (0..1)
% v: top->bottom (0..1)
tx = round(u * Nx);
ty = round((1 - v) * Ny);   % invert vertical for map coords
tx = max(1, min(Nx, tx));
ty = max(1, min(Ny, ty));
cellXY = [tx, ty];
end

function occ = isOccupied(map, cellXY)
x = cellXY(1); y = cellXY(2);
occ = map(y,x) == 1;
end

function drawLidar(map, robotPos, anglesDeg, maxRange)
% Simple ray-casting in grid
Ny = size(map,1); Nx = size(map,2);
rx = robotPos(1); ry = robotPos(2);

for a = anglesDeg
    ang = deg2rad(a);
    dx = cos(ang); dy = sin(ang);

    hitX = rx; hitY = ry;
    for r = 0:0.2:maxRange
        x = rx + r*dx;
        y = ry + r*dy;

        xi = round(x); yi = round(y);
        if xi < 1 || xi > Nx || yi < 1 || yi > Ny
            hitX = x; hitY = y;
            break;
        end

        if map(yi, xi) == 1
            hitX = x; hitY = y;
            break;
        end

        hitX = x; hitY = y;
    end

    plot([rx hitX], [ry hitY], 'g-', 'LineWidth', 1);
end
end

function path = astarGrid(map, startCell, goalCell)
% A* on a 4-connected grid
% map: NyxNx (0 free, 1 obstacle)
Ny = size(map,1); Nx = size(map,2);

sx = startCell(1); sy = startCell(2);
gx = goalCell(1);  gy = goalCell(2);

% Validate start/goal
if sx<1||sx>Nx||sy<1||sy>Ny||gx<1||gx>Nx||gy<1||gy>Ny
    path = [];
    return;
end
if map(sy,sx)==1 || map(gy,gx)==1
    path = [];
    return;
end

% Costs
INF = 1e9;
gScore = INF*ones(Ny, Nx);
fScore = INF*ones(Ny, Nx);
cameFromX = zeros(Ny, Nx);
cameFromY = zeros(Ny, Nx);
openSet = false(Ny, Nx);
closedSet = false(Ny, Nx);

gScore(sy,sx) = 0;
fScore(sy,sx) = heuristic(sx,sy,gx,gy);
openSet(sy,sx) = true;

% 4-neighborhood
nbr = [1 0; -1 0; 0 1; 0 -1];

while any(openSet(:))
    % Find open node with smallest fScore
    [cy,cx] = find(openSet);
    bestIdx = 1;
    bestF = fScore(cy(1),cx(1));
    for k=2:numel(cx)
        fk = fScore(cy(k),cx(k));
        if fk < bestF
            bestF = fk;
            bestIdx = k;
        end
    end
    x = cx(bestIdx); y = cy(bestIdx);

    if x==gx && y==gy
        path = reconstructPath(cameFromX, cameFromY, startCell, goalCell);
        return;
    end

    openSet(y,x) = false;
    closedSet(y,x) = true;

    for i=1:size(nbr,1)
        xn = x + nbr(i,1);
        yn = y + nbr(i,2);

        if xn<1||xn>Nx||yn<1||yn>Ny, continue; end
        if closedSet(yn,xn), continue; end
        if map(yn,xn)==1, continue; end

        tentativeG = gScore(y,x) + 1;

        if ~openSet(yn,xn)
            openSet(yn,xn) = true;
        elseif tentativeG >= gScore(yn,xn)
            continue;
        end

        cameFromX(yn,xn) = x;
        cameFromY(yn,xn) = y;
        gScore(yn,xn) = tentativeG;
        fScore(yn,xn) = tentativeG + heuristic(xn,yn,gx,gy);
    end
end

path = []; % no route
end

function h = heuristic(x,y,gx,gy)
h = abs(x-gx) + abs(y-gy); % Manhattan
end

function path = reconstructPath(cameFromX, cameFromY, startCell, goalCell)
gx = goalCell(1); gy = goalCell(2);
sx = startCell(1); sy = startCell(2);

path = [gx gy];
x = gx; y = gy;

while ~(x==sx && y==sy)
    px = cameFromX(y,x);
    py = cameFromY(y,x);
    if px==0 && py==0
        path = [];
        return;
    end
    x = px; y = py;
    path = [x y; path]; 
end
end
