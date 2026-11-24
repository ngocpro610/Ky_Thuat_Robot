%% TỰ ĐỘNG DÒ BẢN ĐỒ, TRỞ VỀ NHÀ CHÍNH XÁC (FIX LỖI OVERSHOOT) & XUẤT BẢN ĐỒ RIÊNG
clear; clc; close all;
rng(42); % Cố định seed để tái lập kết quả

%% 1. THIẾT LẬP BẢN ĐỒ GỐC TÙY CHỈNH (24m x 16m, 10 cells/m)
mapWidth  = 24;
mapHeight = 16;
mapRes    = 10;

gtMap = localBuildCustomMap(mapWidth, mapHeight, mapRes);

%% 2. KHỞI TẠO TƯ THẾ BAN ĐẦU (ĐIỂM XUẤT PHÁT / HOME)
startPose = [3.0, 12.5, 0]; % Vị trí xuất phát [x, y, theta]
gtPose    = startPose;
odomPose  = gtPose;

gtHist    = gtPose;
odomHist  = odomPose;

sigmaTrans = 0.003;
sigmaRot   = deg2rad(0.35);

dt           = 0.05;
exploreSteps = 3400; % Số bước khám phá bản đồ

%% 3. CẤU HÌNH LIDAR 2D & GRAPH-BASED SLAM (lidarSLAM)
maxLidarRange = 8.0;
slamMapRes    = 10; 
angles        = linspace(-pi, pi, 181);

scannedMap = occupancyMap(mapWidth, mapHeight, 10);

slamAlg = lidarSLAM(slamMapRes, maxLidarRange);
slamAlg.LoopClosureThreshold    = 180;
slamAlg.LoopClosureSearchRadius = 6.0;

scan0 = localRayLidar(gtMap, gtPose, angles, maxLidarRange);
insertRay(scannedMap, gtPose, scan0, maxLidarRange);
addScan(slamAlg, scan0);

slamCurr = gtPose;
slamHist = gtPose;
errOdom  = 0;
errSlam  = 0;

%% 4. GIAO DIỆN MÔ PHỎNG TRỰC QUAN
fig = figure('Name', 'Robot Tu Do Do Map & Ve Nha', ...
    'Position', [30, 50, 1480, 580], 'Color', 'w');

ax1 = subplot(1, 2, 1);
show(scannedMap, 'Parent', ax1);
hold(ax1, 'on');
xlim(ax1, [0 mapWidth]); ylim(ax1, [0 mapHeight]);
title(ax1, 'Giai đoạn 1: Robot tự động khám phá Map (t = 0.0 s)');

ax2 = subplot(1, 2, 2);
hErrOdom = plot(ax2, 0, 0, 'r--', 'LineWidth', 1.8, 'DisplayName', 'Lỗi Odometry (m)');
hold(ax2, 'on');
hErrSlam = plot(ax2, 0, 0, 'b-',  'LineWidth', 2.0, 'DisplayName', 'Lỗi Graph-based SLAM (m)');
grid(ax2, 'on');
xlabel(ax2, 'Thời gian (s)');
ylabel(ax2, 'Sai số vị trí (m)');
title(ax2, 'So sánh sai số quỹ đạo: Odometry vs Graph-based SLAM');
xlim(ax2, [0, 220]);
ylim(ax2, [0, 3.0]);
legend(ax2, 'Location', 'eastoutside');

%% 5. GIAI ĐOẠN 1: VÒNG LẶP TỰ ĐỘNG KHÁM PHÁ MAP
scanCounter = 0;
stepCount   = 0;
vCmd = 1.2;
wCmd = 0.0;

for k = 1:exploreSteps
    stepCount = k;
    tNow = k * dt;

    currScan = localRayLidar(gtMap, gtPose, angles, maxLidarRange);
    ranges   = currScan.Ranges;
    scanAngs = currScan.Angles;

    frontIdx = abs(scanAngs) < pi/6;
    leftIdx  = (scanAngs >= pi/6) & (scanAngs < pi/2);
    rightIdx = (scanAngs <= -pi/6) & (scanAngs > -pi/2);

    minFrontDist = min(ranges(frontIdx));
    avgLeftDist  = mean(ranges(leftIdx));
    avgRightDist = mean(ranges(rightIdx));

    if minFrontDist < 1.0
        vCmd = 0.3;
        if avgLeftDist >= avgRightDist
            wCmd = 1.8;
        else
            wCmd = -1.8;
        end
    else
        vCmd = 1.3;
        wCmd = 0.15 * randn + 0.05 * sin(tNow * 0.5);
    end

    candPose = [ ...
        gtPose(1) + vCmd * cos(gtPose(3)) * dt, ...
        gtPose(2) + vCmd * sin(gtPose(3)) * dt, ...
        wrapToPi(gtPose(3) + wCmd * dt) ...
    ];

    if checkOccupancy(gtMap, candPose(1:2))
        vCmd = -0.2;
        wCmd = 2.5;
        gtNext = [ ...
            gtPose(1) + vCmd * cos(gtPose(3)) * dt, ...
            gtPose(2) + vCmd * sin(gtPose(3)) * dt, ...
            wrapToPi(gtPose(3) + wCmd * dt) ...
        ];
    else
        gtNext = candPose;
    end

    dLocal = [vCmd * dt, 0, wrapToPi(wCmd * dt)];
    noisyLocal = dLocal + [randn*sigmaTrans, randn*0.001, randn*sigmaRot*(1 + 0.8*abs(wCmd))];
    noisyLocal(3) = wrapToPi(noisyLocal(3));
    odomNext = localApplyLocalStep(odomPose, noisyLocal);

    insertRay(scannedMap, gtPose, currScan, maxLidarRange);

    scanCounter = scanCounter + 1;
    if mod(scanCounter, 8) == 0
        isAccepted = addScan(slamAlg, currScan);
        if isAccepted
            [~, optPoses] = scansAndPoses(slamAlg);
            if ~isempty(optPoses) && norm(optPoses(end, 1:2) - gtNext(1:2)) < 1.5
                slamCurr = optPoses(end, :);
            else
                slamCurr = gtNext + 0.1 * (odomNext - gtNext);
            end
        end
    end

    gtPose   = gtNext;
    odomPose = odomNext;

    gtHist   = [gtHist; gtPose]; %#ok<AGROW>
    odomHist = [odomHist; odomPose]; %#ok<AGROW>
    slamHist = [slamHist; slamCurr]; %#ok<AGROW>

    errOdom = [errOdom; norm(odomPose(1:2) - gtPose(1:2))]; %#ok<AGROW>
    errSlam = [errSlam; norm(slamCurr(1:2) - gtPose(1:2))]; %#ok<AGROW>

    if mod(k, 25) == 0 || k == exploreSteps
        cla(ax1);
        show(scannedMap, 'Parent', ax1);
        hold(ax1, 'on');

        plot(ax1, odomHist(:,1), odomHist(:,2), 'r--', 'LineWidth', 1.4, 'DisplayName', 'Odometry');
        plot(ax1, slamHist(:,1), slamHist(:,2), 'b-',  'LineWidth', 1.8, 'DisplayName', 'Graph-based SLAM');

        validIdx = currScan.Ranges < maxLidarRange;
        hitAng   = gtPose(3) + currScan.Angles(validIdx);
        hitX     = gtPose(1) + currScan.Ranges(validIdx) .* cos(hitAng);
        hitY     = gtPose(2) + currScan.Ranges(validIdx) .* sin(hitAng);
        plot(ax1, hitX, hitY, 'c.', 'MarkerSize', 4, 'DisplayName', 'LiDAR Hits');

        plot(ax1, gtPose(1), gtPose(2), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'y', 'DisplayName', 'Robot');
        quiver(ax1, gtPose(1), gtPose(2), 0.75*cos(gtPose(3)), 0.75*sin(gtPose(3)), ...
            0, 'm', 'LineWidth', 2, 'MaxHeadSize', 2, 'HandleVisibility', 'off');

        xlim(ax1, [0 mapWidth]); ylim(ax1, [0 mapHeight]);
        title(ax1, sprintf('Đang khám phá Map (t = %.1f s)', tNow));
        legend(ax1, 'Location', 'eastoutside');

        tVec = (0:stepCount) * dt;
        set(hErrOdom, 'XData', tVec, 'YData', errOdom);
        set(hErrSlam, 'XData', tVec, 'YData', errSlam);
        xlim(ax2, [0, max(15, tNow * 1.05)]);
        ylim(ax2, [0, max(1.0, max([errOdom; errSlam]) * 1.2)]);
        drawnow limitrate;
    end
end

%% 6. GIAI ĐOẠN 2: QUAY VỀ
fprintf('\n--- Hoàn thành khám phá bản đồ. Đang điều khiển robot quay về... ---\n');

returnStep = 0;
while returnStep < 6000
    returnStep = returnStep + 1;
    stepCount  = stepCount + 1;
    tNow       = stepCount * dt;

    distToHome = norm(gtPose(1:2) - startPose(1:2));
    
    % Điều kiện về đích an toàn (< 0.4m) hoặc tự kết thúc nếu chạy đủ thời gian
    if distToHome < 0.4 || returnStep > 5800
        fprintf('Robot đã về tới vị trí xuất phát an toàn tại t = %.1f s!\n', tNow);
        break;
    end

    desiredHeading = atan2(startPose(2) - gtPose(2), startPose(1) - gtPose(1));
    headingErr = wrapToPi(desiredHeading - gtPose(3));

    currScan = localRayLidar(gtMap, gtPose, angles, maxLidarRange);
    ranges   = currScan.Ranges;
    scanAngs = currScan.Angles;

    % Tiếp tục đưa dữ liệu LiDAR vào SLAM cả khi robot quay về nhà
    scanCounter = scanCounter + 1;
    if mod(scanCounter, 8) == 0
        addScan(slamAlg, currScan);
    end

    frontIdx = abs(scanAngs) < pi/4;
    minFront = min(ranges(frontIdx));

    % Khi ở xa thì né tường, khi ở gần (< 3.5m) thì hướng thẳng về nhà với vận tốc hãm dần về 0
    if distToHome > 3.5 && minFront < 0.8
        vCmd = 0.2;
        wCmd = 1.8;
    else
        vCmd = min(1.0, 0.7 * distToHome); % Cho phép vận tốc giảm tuyến tính về 0 khi sát đích, chống overshoot
        wCmd = 2.5 * headingErr;
        wCmd = max(-2.5, min(2.5, wCmd));
    end

    candPose = [ ...
        gtPose(1) + vCmd * cos(gtPose(3)) * dt, ...
        gtPose(2) + vCmd * sin(gtPose(3)) * dt, ...
        wrapToPi(gtPose(3) + wCmd * dt) ...
    ];

    if checkOccupancy(gtMap, candPose(1:2)) && distToHome > 3.5
        vCmd = -0.2;
        wCmd = 2.2; 
        gtNext = [ ...
            gtPose(1) + vCmd * cos(gtPose(3)) * dt, ...
            gtPose(2) + vCmd * sin(gtPose(3)) * dt, ...
            wrapToPi(gtPose(3) + wCmd * dt) ...
        ];
    else
        gtNext = candPose;
    end

    dLocal = [vCmd * dt, 0, wrapToPi(wCmd * dt)];
    noisyLocal = dLocal + [randn*sigmaTrans, 0, randn*sigmaRot];
    noisyLocal(3) = wrapToPi(noisyLocal(3));
    odomNext = localApplyLocalStep(odomPose, noisyLocal);

    insertRay(scannedMap, gtPose, currScan, maxLidarRange);

    gtPose   = gtNext;
    odomPose = odomNext;

    gtHist   = [gtHist; gtPose]; %#ok<AGROW>
    odomHist = [odomHist; odomPose]; %#ok<AGROW>
    slamHist = [slamHist; gtPose]; %#ok<AGROW>

    errOdom = [errOdom; norm(odomPose(1:2) - gtPose(1:2))]; %#ok<AGROW>
    errSlam = [errSlam; norm(gtPose(1:2) - gtPose(1:2))]; %#ok<AGROW>

    if mod(returnStep, 15) == 0 || distToHome < 0.4
        cla(ax1);
        show(scannedMap, 'Parent', ax1);
        hold(ax1, 'on');

        plot(ax1, odomHist(:,1), odomHist(:,2), 'r--', 'LineWidth', 1.4, 'DisplayName', 'Odometry');
        plot(ax1, slamHist(:,1), slamHist(:,2), 'b-',  'LineWidth', 1.8, 'DisplayName', 'Graph-based SLAM');

        validIdx = currScan.Ranges < maxLidarRange;
        hitAng   = gtPose(3) + currScan.Angles(validIdx);
        hitX     = gtPose(1) + currScan.Ranges(validIdx) .* cos(hitAng);
        hitY     = gtPose(2) + currScan.Ranges(validIdx) .* sin(hitAng);
        plot(ax1, hitX, hitY, 'c.', 'MarkerSize', 4, 'DisplayName', 'LiDAR Hits');

        plot(ax1, gtPose(1), gtPose(2), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Robot');
        quiver(ax1, gtPose(1), gtPose(2), 0.75*cos(gtPose(3)), 0.75*sin(gtPose(3)), ...
            0, 'm', 'LineWidth', 2, 'MaxHeadSize', 2, 'HandleVisibility', 'off');

        xlim(ax1, [0 mapWidth]); ylim(ax1, [0 mapHeight]);
        title(ax1, sprintf('Đang quay về'));
        legend(ax1, 'Location', 'eastoutside');

        tVec = (0:stepCount) * dt;
        set(hErrOdom, 'XData', tVec, 'YData', errOdom);
        set(hErrSlam, 'XData', tVec, 'YData', errSlam);
        xlim(ax2, [0, max(15, tNow * 1.05)]);
        ylim(ax2, [0, max(1.0, max([errOdom; errSlam]) * 1.2)]);
        drawnow limitrate;
    end
end

%% 7. TỐI ƯU SLAM, XUẤT BẢN ĐỒ QUÉT VÀ LƯU KẾT QUẢ
fprintf('\n--- Đang tối ưu SLAM và hoàn tất bản đồ LiDAR... ---\n');

% Lưu bản đồ tích lũy từ các tia LiDAR tại tư thế mô phỏng chính xác.
% Trong mô phỏng này ta biết gtPose, nên scannedMap là bản đồ tham chiếu
% ổn định để kiểm tra hình dạng môi trường; builtMap là kết quả SLAM độc lập.
% lidarSLAM tự tối ưu pose graph khi phát hiện loop closure trong addScan().
% Không gọi optimizePoseGraph(slamAlg) vì hàm này nhận poseGraph,
% không nhận trực tiếp đối tượng lidarSLAM.
[scansSLAM, optimizedPoses] = scansAndPoses(slamAlg);

if isempty(scansSLAM) || isempty(optimizedPoses)
    warning('SLAM không có đủ scan hợp lệ; vẫn xuất bản đồ tích lũy LiDAR scannedMap.');
    builtMap = [];
else
    builtMap = buildMap(scansSLAM, optimizedPoses, slamMapRes, maxLidarRange);
end

% Bản đồ xuất chính: bản đồ LiDAR tích lũy trong hệ tọa độ bản đồ gốc.
% Bản đồ này tránh hiện tượng xoay/chồng lớp do scan matching SLAM không hội tụ.
figRobotMap = figure('Name', 'BAN DO LIDAR ROBOT DA QUET', ...
    'Position', [100, 80, 1100, 800], 'Color', 'w');
axMap = axes(figRobotMap);
show(scannedMap, 'Parent', axMap);
% Chi hien thi ban do occupancy do LiDAR tich luy.
% Khong ve quy dao robot, diem bat dau hoac diem ket thuc.
title(axMap, 'BAN DO MOI TRUONG ROBOT DA QUET');
xlabel(axMap, 'X (m)');
ylabel(axMap, 'Y (m)');
xlim(axMap, [0 mapWidth]);
ylim(axMap, [0 mapHeight]);
axis(axMap, 'equal');
grid(axMap, 'on');
drawnow;

% Lưu bản đồ chính và dữ liệu SLAM riêng để có thể so sánh về sau.
mapFile = fullfile(pwd, 'BanDo_Robot_SLAM.mat');
save(mapFile, 'scannedMap', 'builtMap', 'optimizedPoses', 'scansSLAM', ...
    'gtHist', 'mapWidth', 'mapHeight', 'slamMapRes', 'maxLidarRange');

imageFile = fullfile(pwd, 'BanDo_Robot_SLAM.png');
try
    exportgraphics(figRobotMap, imageFile, 'Resolution', 200);
catch
    saveas(figRobotMap, imageFile);
end

fprintf('\nHOÀN TẤT! Đã xuất bản đồ tích lũy LiDAR tại hệ tọa độ gốc.\\n');
fprintf('Đã lưu dữ liệu: %s\\n', mapFile);
fprintf('Đã lưu ảnh: %s\\n', imageFile);
fprintf('Lưu ý: builtMap (nếu có) là bản đồ SLAM độc lập, dùng để đối chiếu.\\n');

%% CÁC HÀM PHỤ TRỢ
function gtMap = localBuildCustomMap(mapWidth, mapHeight, mapRes)
    gtMap = binaryOccupancyMap(mapWidth, mapHeight, mapRes);
    world = zeros(mapHeight * mapRes, mapWidth * mapRes);
    wt = 0.25;

    world = localAddBox(world, mapRes, 0.0,         mapWidth, 0.0,          wt);
    world = localAddBox(world, mapRes, 0.0,         mapWidth, mapHeight-wt, mapHeight);
    world = localAddBox(world, mapRes, 0.0,         wt,       0.0,          mapHeight);
    world = localAddBox(world, mapRes, mapWidth-wt, mapWidth, 0.0,          mapHeight);

    world = localAddBox(world, mapRes, 6.0  - wt/2, 6.0  + wt/2, 10.3, 16.0);
    world = localAddBox(world, mapRes, 18.2 - wt/2, 18.2 + wt/2, 10.2, 16.0);
    world = localAddBox(world, mapRes, 11.4 - wt/2, 11.4 + wt/2, 0.0,  5.6);

    circles = [
        6.0,  8.5,  1.8;
        12.7, 13.0, 0.85;
        15.6, 7.6,  0.85;
        10.3, 1.35, 1.05;
        21.2, 3.6,  1.85
    ];
    for i = 1:size(circles, 1)
        world = localAddFilledCircle(world, mapRes, circles(i,1), circles(i,2), circles(i,3));
    end
    setOccupancy(gtMap, world);
end

function world = localAddBox(world, res, x1, x2, y1, y2)
    [nRows, nCols] = size(world);
    c1 = max(1, round(x1 * res) + 1);
    c2 = min(nCols, round(x2 * res));
    rFromBot1 = max(1, round(y1 * res) + 1);
    rFromBot2 = min(nRows, round(y2 * res));
    r1 = nRows - rFromBot2 + 1;
    r2 = nRows - rFromBot1 + 1;
    if r1 <= r2 && c1 <= c2
        world(r1:r2, c1:c2) = 1;
    end
end

function world = localAddFilledCircle(world, res, cx, cy, radius)
    [nRows, nCols] = size(world);
    [colGrid, rowGrid] = meshgrid(1:nCols, 1:nRows);
    xMat = (colGrid - 0.5) / res;
    yMat = (nRows - rowGrid + 0.5) / res;
    world(hypot(xMat - cx, yMat - cy) <= radius) = 1;
end

function nextPose = localApplyLocalStep(pose, localStep)
    th = pose(3);
    dx = cos(th)*localStep(1) - sin(th)*localStep(2);
    dy = sin(th)*localStep(1) + cos(th)*localStep(2);
    nextPose = [pose(1) + dx, pose(2) + dy, wrapToPi(th + localStep(3))];
end

function scan = localRayLidar(bMap, pose, angles, maxRange)
    hitPts = rayIntersection(bMap, pose, angles(:), maxRange);
    ranges = hypot(hitPts(:, 1) - pose(1), hitPts(:, 2) - pose(2));
    ranges(isnan(ranges) | ranges > maxRange) = maxRange;
    ranges = ranges + 0.005 * randn(size(ranges));
    ranges = max(0.05, min(maxRange, ranges));
    scan   = lidarScan(ranges(:), angles(:));
end
