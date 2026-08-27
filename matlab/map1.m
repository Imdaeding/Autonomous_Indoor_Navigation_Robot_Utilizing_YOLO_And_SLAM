%% 1. ros2 bag 로드
bag = ros2bag('./my_wall_mapping_data');

%% 2. /map 추출 및 벽면 이진화/연결 처리
mapBag = select(bag, 'Topic', '/map');
mapMsgs = readMessages(mapBag);
if isempty(mapMsgs)
    error('/map 토픽 데이터가 비어 있습니다.');
end
latestMsg = mapMsgs{end};
width = double(latestMsg.info.width);
height = double(latestMsg.info.height);
resolution = double(latestMsg.info.resolution);
originX = double(latestMsg.info.origin.position.x);
originY = double(latestMsg.info.origin.position.y);

rawMap = reshape(double(latestMsg.data), [width, height])';
wallBinary = (rawMap > 50);
se = strel('disk', 2);
wallConnected = imclose(wallBinary, se);
wallConnected = imdilate(wallConnected, strel('disk', 1));
boundaries = bwboundaries(wallConnected, 'noholes');

%% 3. 로봇 주행 궤적 및 타임스탬프(/tf) 추출
tfBag = select(bag, 'Topic', '/tf');
tfMsgs = readMessages(tfBag);
robotX = [];
robotY = [];
robotTime = [];

for i = 1:length(tfMsgs)
    transforms = tfMsgs{i}.transforms;
    for j = 1:length(transforms)
        tf = transforms(j);
        childFrame = strtrim(string(tf.child_frame_id));
        if contains(childFrame, "base_link") || contains(childFrame, "base_footprint")
            trans = tf.transform.translation;
            robotX(end+1) = double(trans.x);
            robotY(end+1) = double(trans.y);
            
            % 초 단위 시간 계산 (sec + nanosec)
            tSec = double(tf.header.stamp.sec) + double(tf.header.stamp.nanosec)*1e-9;
            robotTime(end+1) = tSec;
        end
    end
end

%% 4. 시작점 근처 정지 구간(속도 ≈ 0) 자동 감지 알고리즘
% 시작점(robotX(1), robotY(1))으로부터의 거리 계산
distFromStart = hypot(robotX - robotX(1), robotY - robotY(1));

% 각 프레임 간 이동 속도(m/s) 계산
dt = [diff(robotTime), 0.1];
dt(dt <= 0) = 0.05; % 0 나누기 방지
dx = [diff(robotX), 0];
dy = [diff(robotY), 0];
speed = hypot(dx, dy) ./ dt;

% 조건: 전체 주행의 30% 이후 시점 + 시작점 반경 0.35m 이내 + 속도가 0.02m/s 이하로 정지한 구간
minSearchIdx = round(length(robotX) * 0.3);
candidateIdx = find(minSearchIdx:length(robotX)) + minSearchIdx - 1;

stopPoints = candidateIdx(distFromStart(candidateIdx) < 0.35 & speed(candidateIdx) < 0.02);

if isempty(stopPoints)
    % 정지 구간을 못 찾을 경우 시작점 최단 근접 지점으로 분할
    [~, splitIdx] = min(distFromStart(minSearchIdx:end));
    splitIdx = splitIdx + minSearchIdx - 1;
    fprintf('[INFO] 정지 감지 대체: 시작점 최단 복귀 지점(Index: %d)으로 분할합니다.\n', splitIdx);
else
    % 정지해 있던 시간의 중간 지점을 1차 종료/2차 시작 분기점으로 지정
    splitIdx = stopPoints(round(length(stopPoints)/2));
    fprintf('[SUCCESS] 1차 완주 후 정지 구간 발견! 분할 지점 Index: %d\n', splitIdx);
end

% 1차 / 2차 궤적 분할
traj1_X = robotX(1:splitIdx);
traj1_Y = robotY(1:splitIdx);

traj2_X = robotX(splitIdx:end);
traj2_Y = robotY(splitIdx:end);

%% 5. 보고서 및 포스터용 플롯 출력
figure('Color', 'w', 'Position', [150, 150, 850, 750]);
hold on;

% 1) 빨간색 연속 벽면 플롯
wallLegendShown = false;
for k = 1:length(boundaries)
    boundary = boundaries{k};
    if size(boundary, 1) > 5
        bCol = boundary(:, 2);
        bRow = boundary(:, 1);
        bX = originX + (bCol - 1) * resolution;
        bY = originY + (bRow - 1) * resolution;
        
        if ~wallLegendShown
            plot(bX, bY, 'r-', 'LineWidth', 2.2, 'DisplayName', 'Wall Boundary');
            wallLegendShown = true;
        else
            plot(bX, bY, 'r-', 'LineWidth', 2.2, 'HandleVisibility', 'off');
        end
    end
end

% 2) 1차 주행 궤적 (파란색 실선)
plot(traj1_X, traj1_Y, 'b.-', 'LineWidth', 1.3, 'MarkerSize', 5, 'DisplayName', '1st Run: Mapping');

% 3) 2차 주행 궤적 (보라색 점선)
plot(traj2_X, traj2_Y, 'm.-', 'LineWidth', 1.5, 'MarkerSize', 5, 'DisplayName', '2nd Run: SLAM Path Following & Detection');

% 4) 주요 지점 마커 표시
plot(robotX(1), robotY(1), 'go', 'MarkerSize', 9, 'LineWidth', 2.2, 'DisplayName', '1st Lap Start');
plot(traj1_X(end), traj1_Y(end), 'c^', 'MarkerSize', 9, 'LineWidth', 2.2, 'DisplayName', '1st Lap End Point');
plot(traj2_X(1), traj2_Y(1), 'yp', 'MarkerSize', 11, 'LineWidth', 2.2, 'MarkerFaceColor', 'y', 'DisplayName', '2nd Lap Start Point');
plot(robotX(end), robotY(end), 'ks', 'MarkerSize', 9, 'LineWidth', 2.2, 'DisplayName', 'Final End Point');

hold off;
axis equal;
grid on;
box on;
title('UGV02 Mapping & SLAM with Detection', 'FontSize', 13, 'FontWeight', 'bold');
xlabel('X [m]', 'FontSize', 11);
ylabel('Y [m]', 'FontSize', 11);
legend('Location', 'best', 'FontSize', 10);