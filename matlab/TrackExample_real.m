clear; close all; clc;
rosshutdown;
rosinit('http://localhost:11311')
tftree = rostf;
pause(3);

% Parameter//============================================================
roi = [0, 7, -7, 7, -1, 2];

clusterThreshold = 0.3; % Cluster distance
cuboidTreshold = 0.0020; % Ignore smaller than 0.003 cuboid (cone: 0.0215) 
waypointTreshold = 3; % make a waypoint before 3m 


pp=controllerPurePursuit;
pp.LookaheadDistance=1.3; % m
pp.DesiredLinearVelocity=1.5; % m/s
pp.MaxAngularVelocity = 2.0; % rad/s

% init//==================================================================
waypoints = [];
markerIdPath = 0;
markerIdClusters = 0;
params = lidarParameters('OS1Gen1-64',512);

% Ouster 64ch sub
lidarSub = rossubscriber('/os1_cloud_node/points',"DataFormat","struct"); 
% Cone visualization for Rviz
[pubClusters, markerArrayMsg] = rospublisher('/clusters_marker', 'visualization_msgs/MarkerArray',DataFormat='struct');
% Path visualization for Rviz
pubPath = rospublisher('/path_marker', 'visualization_msgs/Marker','DataFormat','struct');
% Gps sub
gpsSub = rossubscriber("/ublox_gps/fix","sensor_msgs/NavSatFix","DataFormat","struct");
% imu sub
imuSub = rossubscriber("/imu/data","sensor_msgs/Imu","DataFormat","struct");
% Yolo Client
client = rossvcclient('/Activate_yolo','cob_object_detection_msgs/DetectObjects',DataFormat='struct');
request_l = rosmessage(client);
request_l.ObjectName.Data = 'left';
request_r = rosmessage(client);
request_r.ObjectName.Data = 'right';

load("camera1.mat"); 
tform_l = tform;
tformCamera_l = invert(tform_l);
load("camera2.mat");
tform_r = tform;
tformCamera_r = invert(tform_r);
load("cameraParams.mat")

figure;

while true % ctrl + c to stop
    tic;
    
    vehiclePose = getVehiclePose(modelStatesSub); % get pose data from gps, imu
    broadcastTftree(vehiclePose,tftree); % broadcasting TF tree

    if isempty(pp.Waypoints) || norm(worldWaypoints(end,:)-[vehiclePose(1), vehiclePose(2)]) < waypointTreshold  % Considering only x and y for the distance
        disp("Make new waypoints");
        
        try
            lidarData = receive(lidarSub);
            bboxData_r = call(client, request_r);
            bboxData_l = call(client, request_l);

            % 지면제거, roi설정, 노이즈 제거
            roiPtCloud = preprocess_lidar_data(lidarData, params, roi);
            
            % bboxData에서 y_cone, r_cone 추출
            [y_coneBboxs_l, b_coneBboxs_l] = extractConesBboxs(bboxData_l.ObjectList);
            [y_coneBboxs_r, b_coneBboxs_r] = extractConesBboxs(bboxData_r.ObjectList);
            
            % Bbox 속 pt 클러스터링
            [bboxesLidar_l,~,boxesUsed_l] = bboxCameraToLidar([y_coneBboxs_l; b_coneBboxs_l],roiPtCloud,cameraParams,tformCamera_l,'ClusterThreshold',clusterThreshold);
            [bboxesLidar_r,~,boxesUsed_r] = bboxCameraToLidar([y_coneBboxs_r; b_coneBboxs_r],roiPtCloud,cameraParams,tformCamera_r,'ClusterThreshold',clusterThreshold);
            
            % 검출된 콘 중 y_cone, b_cone 분류
            [y_coneBboxesLidar_l, b_coneBboxesLidar_l] = splitConesBboxes(y_coneBboxs_l,bboxesLidar_l,boxesUsed_l);
            [y_coneBboxesLidar_r, b_coneBboxesLidar_r] = splitConesBboxes(y_coneBboxs_r,bboxesLidar_r,boxesUsed_r);
            
            % 임계치 이상의 부피를 가진 Cuboid Box의 중심점 추출
            innerConePosition = extractConePositions(cuboidTreshold, b_coneBboxesLidar_l, b_coneBboxesLidar_r);
            outerConePosition = extractConePositions(cuboidTreshold, y_coneBboxesLidar_l, y_coneBboxesLidar_r);
            
            % 중복점 제거, 정렬
            innerConePosition = unique_rows(innerConePosition);
            outerConePosition = unique_rows(outerConePosition);

            % y, b콘 개수 맞추기 (적은 콘 기준)
            [innerConePosition, outerConePosition] = match_array_lengths(innerConePosition, outerConePosition);
            
            % 검출된 콘 시각화
            hold off;
            scatter(innerConePosition(:,1),innerConePosition(:,2),'blue');
            hold on;
            scatter(outerConePosition(:,1),outerConePosition(:,2),'red');
            
            % 경로 생성
            waypoints = generate_waypoints_del(innerConePosition, outerConePosition);
            
            % waypoint 차량 좌표계에서 월드 좌표계로 변환
            worldWaypoints = transformWaypointsToWorld(waypoints, vehiclePose);
            
            % 경로, 콘 시각화 Rviz
            [markerArrayMsg, markerIdClusters] = generate_clusters_marker(innerConePosition, outerConePosition, 'base_footprint', markerIdClusters);
            send(pubClusters, markerArrayMsg);
            [pathMarkerMsg, markerIdPath] = generate_path_marker(worldWaypoints, 'hunter2_base', markerIdPath);
            send(pubPath, pathMarkerMsg);
     
            pp.Waypoints = worldWaypoints;

        catch
            disp("Fail to make new waypoints");
            continue; % 다음 while문 반복으로 넘어감
            
        end
    end

    [v, w] = pp(vehiclePose);  % Pass the current vehicle pose to the path planner
    [pub, msg] = publish_twist_command(v, w, '/ackermann_steering_controller/cmd_vel');
    send(pub, msg);
    toc;
end



%%


function vehiclePose = getVehiclePose(gpsSub, imuSub)
    gpsMsg = receive(gpsSub);
    imuMsg = receive(imuSub);
    quat = [imuMsg.Orientation.W,imuMsg.Orientation.X,imuMsg.Orientation.Y,imuMsg.Orientation.Z];
    euler = quat2eul(quat);
    % UTM 좌표계로 변환 (Korea: 32652)
    [x_utm, y_utm] = projfwd(projcrs(32652), gpsMsg.latitude, gpsMsg.longitude);

    vehiclePose = [x_utm, y_utm, euler(1)];
end

function broadcastTftree(vehiclePose,tftree)
    % TF 메시지 생성 및 설정
    tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
    tfStampedMsg.ChildFrameId = 'base_link';
    tfStampedMsg.Header.FrameId = 'hunter2_base';
    tfStampedMsg.Header.Stamp = rostime('now');
    tfStampedMsg.Transform.Translation.X = vehiclePose(1);
    tfStampedMsg.Transform.Translation.Y = vehiclePose(2);
    tfStampedMsg.Transform.Rotation.Z = sin(vehiclePose(3)/2);
    tfStampedMsg.Transform.Rotation.W = cos(vehiclePose(3)/2);

    sendTransform(tftree, tfStampedMsg);
end

function roiPtCloud = preprocess_lidar_data(lidarData, params, roi)
    xyzData = rosReadXYZ(lidarData);
    ptCloud = pointCloud(xyzData);

    ptCloudOrg = pcorganize(ptCloud, params); 

    groundPtsIdx = segmentGroundFromLidarData(ptCloudOrg);
    nonGroundPtCloud = select(ptCloudOrg, ~groundPtsIdx, 'OutputSize', 'full'); % 지면제거

    indices = findPointsInROI(nonGroundPtCloud, roi);
    roiPtCloud = select(nonGroundPtCloud, indices); % roi 

    roiPtCloud = pcdenoise(roiPtCloud, 'PreserveStructure', true); % 노이즈 제거
end

function [y_coneBboxs, b_coneBboxs] = extractConesBboxs(bboxData)
    % 수신한 Bbox 데이터에서 y_cone, b_cone 분리
    % BoundingBoxes_ 대신 Detections의 길이로 메모리 공간을 미리 할당
    numBboxes = numel(bboxData.Detections);

    % y_cone 및 b_cone에 대한 임시 저장 공간
    temp_y_coneBboxs = zeros(numBboxes, 4);
    temp_b_coneBboxs = zeros(numBboxes, 4);

    y_count = 0;
    b_count = 0;

    for i = 1:numBboxes
        currentBbox = bboxData.Detections(i, 1).Mask.Roi;
        
        % 변경된 데이터 형식에 따라 BoundingBoxes_ 대신 Mask.Roi 사용
        x = currentBbox.X;
        y = currentBbox.Y;
        w = currentBbox.Width;
        h = currentBbox.Height;
        
        if strcmp(bboxData.Detections(i, 1).Label, 'y_cone')
            y_count = y_count + 1;
            temp_y_coneBboxs(y_count, :) = [x, y, w, h];
        else
            b_count = b_count + 1;
            temp_b_coneBboxs(b_count, :) = [x, y, w, h];
        end
    end

    % 최종 결과
    y_coneBboxs = temp_y_coneBboxs(1:y_count, :);
    b_coneBboxs = temp_b_coneBboxs(1:b_count, :);
end

function [y_coneBboxesLidar, b_coneBboxesLidar] = splitConesBboxes(y_coneBboxs,bboxesLidar,boxesUsed)
    % y_cone의 개수만 계산
    numY_cone = sum(boxesUsed(1:size(y_coneBboxs,1)));
    
    % bboxesLidar에서 y_cone와 b_cone의 bbox 분류
    y_coneBboxesLidar = bboxesLidar(1:numY_cone, :);
    b_coneBboxesLidar = bboxesLidar(numY_cone+1:end, :);
end

function conePosition = extractConePositions(cuboidTreshold, coneBboxesLidar_l, coneBboxesLidar_r)
    % Extract xlen, ylen, zlen from the bounding boxes
    volumes_l = prod(coneBboxesLidar_l(:, 4:6), 2);
    volumes_r = prod(coneBboxesLidar_r(:, 4:6), 2);

    % Find indices where volumes are smaller than cuboidThreshold
    indices_l = volumes_l > cuboidTreshold;
    indices_r = volumes_r > cuboidTreshold;

    % Combine the inner cone positions from left and right into a single array
    conePosition = [coneBboxesLidar_l(indices_l, 1:2);coneBboxesLidar_r(indices_r, 1:2)];
end

function uniqueArray = unique_rows(array)
    [~, uniqueIdx] = unique(array, 'rows');
    uniqueArray = array(uniqueIdx, :);
    uniqueArray = sortrows(uniqueArray);
end

function [out1, out2] = match_array_lengths(arr1, arr2)
    len1 = size(arr1, 1); % Get the number of rows
    len2 = size(arr2, 1); % Get the number of rows

    if len1 > len2
        out1 = arr1(1:len2, :); % Keep only the first len2 rows
        out2 = arr2;
    elseif len2 > len1
        out1 = arr1;
        out2 = arr2(1:len1, :); % Keep only the first len1 rows
    else
        out1 = arr1;
        out2 = arr2;
    end
end

function waypoints = generate_waypoints_del(innerConePosition, outerConePosition)
    [m,nc] = size(innerConePosition); % size of the inner/outer cone positions data
    kockle_coords = zeros(2*m,nc); % initiate a P matrix consisting of inner and outer coordinates
    kockle_coords(1:2:2*m,:) = innerConePosition;
    kockle_coords(2:2:2*m,:) = outerConePosition; % merge the inner and outer coordinates with alternate values
    xp = []; % create an empty numeric xp vector to store the planned x coordinates after each iteration
    yp = []; 

    
    interv=size(innerConePosition,1)*2;
    %step 1 : delaunay triangulation
    tri=delaunayTriangulation(kockle_coords);
    pl=tri.Points;
    cl=tri.ConnectivityList;
    [mc, nc]=size(pl);
		    
    % inner and outer constraints when the interval is even
    if rem(interv,2) == 0
     cIn = [2 1;(1:2:mc-3)' (3:2:(mc))'; (mc-1) mc];
     cOut = [(2:2:(mc-2))' (4:2:mc)'];
    else
    % inner and outer constraints when the interval is odd
    cIn = [2 1;(1:2:mc-2)' (3:2:(mc))'; (mc-1) mc];
    cOut = [(2:2:(mc-2))' (4:2:mc)'];
    end
    
		    %step 2 : 외부 삼각형 거
    C = [cIn;cOut];
    TR=delaunayTriangulation(pl,C);
    TRC=TR.ConnectivityList;
    TL=isInterior(TR);
    TC =TR.ConnectivityList(TL,:);
    [~, pt]=sort(sum(TC,2));
    TS=TC(pt,:);
    TO=triangulation(TS,pl);
		    
		    %step 3 : 중간 waypoint 생성
    xPo=TO.Points(:,1);
    yPo=TO.Points(:,2);
    E=edges(TO);
    iseven=rem(E,2)==0;
    Eeven=E(any(iseven,2),:);
    isodd=rem(Eeven,2)~=0;
    Eodd=Eeven(any(isodd,2),:);
    xmp=((xPo((Eodd(:,1))) + xPo((Eodd(:,2))))/2);
    ymp=((yPo((Eodd(:,1))) + yPo((Eodd(:,2))))/2);
    Pmp=[xmp ymp];
    waypoints = Pmp;

		    %step 4 : waypoint 보간
    %distancematrix = squareform(pdist(Pmp));
    %distancesteps = zeros(length(Pmp)-1,1);
    %for j = 2:length(Pmp)
    %    distancesteps(j-1,1) = distancematrix(j,j-1);
    %end
    %totalDistance = sum(distancesteps); % total distance travelled
    %distbp = cumsum([0; distancesteps]); % distance for each waypoint
    %gradbp = linspace(0,totalDistance,100);
    %xq = interp1(distbp,xmp,gradbp,'spline'); % interpolate x coordinates
    %yq = interp1(distbp,ymp,gradbp,'spline'); % interpolate y coordinates
    %xp = [xp xq]; % store obtained x midpoints after each iteration
    %yp = [yp yq]; % store obtained y midpoints after each iteration
    
		    %step 5 : 최종 waypoint 생성
    %waypoints=[xp', yp'];
end


function odomWaypoints = transformWaypointsToWorld(waypoints, vehiclePose)
    % Initialize transformed waypoints
    odomWaypoints = zeros(size(waypoints));

    % Extract the vehicle's yaw angle
    theta = vehiclePose(3);

    % Create the 2D rotation matrix
    R = [cos(theta), -sin(theta);
         sin(theta), cos(theta)];

    % Transform each waypoint
    for i = 1:size(waypoints,1)
        % Rotate the waypoint considering the vehicle's yaw
        rotatedPoint = R * waypoints(i,:)';

        % Translate considering the vehicle's position in the odom frame
        odomWaypoints(i,:) = rotatedPoint' + vehiclePose(1:2)';
    end
end

function [markerArrayMsg, markerID] = generate_clusters_marker(b_coneCluster, y_coneCluster, frameId, markerID)
    % Concatenate the clusters for easier looping
    combinedClusters = [b_coneCluster; y_coneCluster];
    clusterColors = [repmat([0 0 1], size(b_coneCluster, 1), 1); % Blue for b_coneCluster
                     repmat([1 1 0], size(y_coneCluster, 1), 1)]; % Yellow for y_coneCluster

    markerArrayMsg = rosmessage('visualization_msgs/MarkerArray','DataFormat','struct');
    
    for i = 1:height(combinedClusters)
        markerMsg = rosmessage('visualization_msgs/Marker','DataFormat','struct');
        markerMsg.Header.FrameId = frameId;
        markerMsg.Id = int32(markerID); % Cast 'markerID' to int32
        markerID = markerID + 1;  % Increment markerID by 1 for each new marker
        markerMsg.Type = int32(3);
        markerMsg.Action = int32(0);
        markerMsg.Pose.Position.X = double(combinedClusters(i,1));
        markerMsg.Pose.Position.Y = double(combinedClusters(i,2));
        markerMsg.Pose.Position.Z = 0;
        markerMsg.Pose.Orientation.W = 1.0;
        markerMsg.Scale.X = 0.3;
        markerMsg.Scale.Y = 0.3;
        markerMsg.Scale.Z = 0.5;
        
        % Set Color
        markerMsg.Color.R = single(clusterColors(i, 1));
        markerMsg.Color.G = single(clusterColors(i, 2));
        markerMsg.Color.B = single(clusterColors(i, 3));
        markerMsg.Color.A = single(0.5);
        
        markerArrayMsg.Markers(i) = markerMsg;
    end
end

function [markerMsg, markerID] = generate_path_marker(waypoints, frameId, markerID)
    markerMsg = rosmessage('visualization_msgs/Marker','DataFormat','struct');
    markerMsg.Header.FrameId = frameId;
    markerMsg.Id = int32(markerID);  % Cast 'markerID' to int32
    markerID = markerID + 1;  % Increment markerID by 1 for each new marker
    markerMsg.Type = int32(4); % LINE_STRIP
    markerMsg.Action = int32(0);
    markerMsg.Pose.Orientation.W = 1.0;
    markerMsg.Scale.X = 0.05;  % Specify a suitable scale
    markerMsg.Color.R = single(1.0); % red
    markerMsg.Color.G = single(0.0); % green
    markerMsg.Color.B = single(0.0); % blue
    markerMsg.Color.A = single(1.0); % alpha

    % Add the waypoints to the points array of the marker
    for i = 1:size(waypoints, 1)
        point = rosmessage('geometry_msgs/Point','DataFormat','struct');
        point.X = double(waypoints(i, 1));
        point.Y = double(waypoints(i, 2));
        point.Z = 0;
        markerMsg.Points(i) = point;
    end
end

function [pub, msg] = publish_twist_command(v, w, topicName)
    pub = rospublisher(topicName, 'geometry_msgs/Twist','DataFormat','struct');
    msg = rosmessage(pub);
    msg.Linear.X = v;
    msg.Angular.Z = w;
end