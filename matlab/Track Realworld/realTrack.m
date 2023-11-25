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
pp.LookaheadDistance=1; % m
pp.DesiredLinearVelocity=0.1; % m/s
pp.MaxAngularVelocity = 0.3; % rad/s

LidarCam = true;
GpsImu = true;

cnt = 0;
% init//==================================================================
waypoints = [];
markerIdPath = 0;
markerIdClusters = 0;
params = lidarParameters('OS1Gen1-64',512);

if LidarCam
    % Ouster 64ch sub
    sub.Lidar = rossubscriber('/ouster/points',"DataFormat","struct");
    % Yolo Client
    client = rossvcclient('/Activate_yolo','cob_object_detection_msgs/DetectObjects','struct');
    request_l = rosmessage(client);
    request_l.ObjectName.Data = 'left';
    request_r = rosmessage(client);
    request_r.ObjectName.Data = 'right';
    load("tform_left.mat"); 
    tformCamera_l = invert(tform_l);
    load("tform_right.mat");
    tformCamera_r = invert(tform_r);
    load("cameraParams_left.mat")
    load("cameraParams_right.mat")
end
if GpsImu
    % Gps sub
    sub.Gps = rossubscriber("/ublox_gps/fix","sensor_msgs/NavSatFix","DataFormat","struct");
    % imu sub
    sub.Imu = rossubscriber("/imu/data","sensor_msgs/Imu","DataFormat","struct");
    disp("Waiting for GPS, IMU data..")
    vehiclePose_origin = getVehiclePose_gps(sub);
end

% Publish Command
pub.Cmd = rospublisher('/cmd_vel', 'geometry_msgs/Twist','DataFormat','struct');
% Cone visualization for Rviz
[pub.Clusters, markerArrayMsg] = rospublisher('/clusters_marker', 'visualization_msgs/MarkerArray',DataFormat='struct');
% Path visualization for Rviz
pub.Path = rospublisher('/path_marker', 'visualization_msgs/Marker','DataFormat','struct');

sub.Odom = rossubscriber("/odom","nav_msgs/Odometry","DataFormat","struct");


figure;

while true % ctrl + c to stop
    tic;
    
    if GpsImu
        vehiclePose = getVehiclePose_gps(sub); % get pose data from gps, imu
        vehiclePose(1) = vehiclePose(1) - vehiclePose_origin(1);
        vehiclePose(2) = vehiclePose(2) - vehiclePose_origin(2);
        worldFrame = 'hunter_gps';
    else % odom 
        vehiclePose_odom = getVehiclePose_odom(sub); % get pose from odom
        worldFrame = 'hunter_odom';
    end
    
    broadcastTftree(vehiclePose,tftree,worldFrame); % broadcasting TF tree

    if isempty(pp.Waypoints) || norm(worldWaypoints(end,:)-[vehiclePose(1), vehiclePose(2)]) < waypointTreshold  % Considering only x and y for the distance
        disp("Make new waypoints");
        
        try
            if LidarCam
                lidarData = receive(sub.Lidar);
                bboxData_r = call(client, request_r);
                bboxData_l = call(client, request_l);
            
                % 콘 검출
                [innerConePosition, outerConePosition] = detectCone(lidarData,params,roi,bboxData_l,bboxData_r,cameraParams_l,cameraParams_r, ...
                                                                        tformCamera_l,tformCamera_r,clusterThreshold,cuboidTreshold);
            else
                if cnt / 2 ==1
                    [innerConePosition, outerConePosition] = Right();
                    cnt = cnt +1;
                else
                    [innerConePosition, outerConePosition] = Straight();
                    cnt = cnt + 1;
                end
            end

            % 경로 생성
            waypoints = generate_waypoints_del(innerConePosition, outerConePosition);
            
            % waypoint 차량 좌표계에서 월드 좌표계로 변환
            worldWaypoints = transformWaypointsToWorld(waypoints, vehiclePose);
            

            % 검출된 콘 시각화
            hold off;
            scatter(innerConePosition(:,1),innerConePosition(:,2),'blue');
            hold on;
            scatter(outerConePosition(:,1),outerConePosition(:,2),'red');
            
            % 경로, 콘 시각화 Rviz
            [markerArrayMsg, markerIdClusters] = generate_clusters_marker(innerConePosition, outerConePosition, 'base_link', markerIdClusters);
            send(pub.Clusters, markerArrayMsg);
            [pathMarkerMsg, markerIdPath] = generate_path_marker(worldWaypoints, worldFrame, markerIdPath);
            send(pub.Path, pathMarkerMsg);
     
            pp.Waypoints = worldWaypoints;

        catch
            disp("Fail to make new waypoints");
            continue; % 다음 while문 반복으로 넘어감
            
        end
    end
    [v, w] = pp(vehiclePose);  % Pass the current vehicle pose to the path planner
    cmdMsg = publish_twist_command(v, w, pub.Cmd);
    send(pub.Cmd, cmdMsg);


    toc;
end



%%
function vehiclePose = getVehiclePose_odom(sub)
    msg = receive(sub.Odom);

    % 위치 정보 추출
    x = msg.Pose.Pose.Position.X;
    y = msg.Pose.Pose.Position.Y;

    % 방향(오리엔테이션) 정보 추출 및 Yaw 각도 계산
    quat = [msg.Pose.Pose.Orientation.W, msg.Pose.Pose.Orientation.X, ...
            msg.Pose.Pose.Orientation.Y, msg.Pose.Pose.Orientation.Z];
    eul = quat2eul(quat);
    yaw = eul(1); % Yaw는 보통 오일러 각의 첫 번째 요소입니다.
    vehiclePose = [x,y,yaw];
end


function vehiclePose = getVehiclePose_gps(sub)
    gpsMsg = receive(sub.Gps);
    imuMsg = receive(sub.Imu);
    quat = [imuMsg.Orientation.W,imuMsg.Orientation.X,imuMsg.Orientation.Y,imuMsg.Orientation.Z];
    euler = quat2eul(quat);
    % UTM 좌표계로 변환 (Korea: 32652)
    [x_utm, y_utm] = projfwd(projcrs(32652), gpsMsg.Latitude, gpsMsg.Longitude);

    vehiclePose = [x_utm, y_utm, euler(1)];
end

function vehiclePose = getVehiclePose_TF(tree, sourceFrame, targetFrame)
    % ex) vehiclePose = getVehiclePose_TF(tftree, 'odom', 'base_link'); % get vehiclePose from Odom
    % This function returns the pose of the vehicle in the odom frame.

    % Check if the frames are available in the tree
    if ~any(strcmp(tree.AvailableFrames, sourceFrame))
        error('Source frame is not available in the tree');
    end
    if ~any(strcmp(tree.AvailableFrames, targetFrame))
        error('Target frame is not available in the tree');
    end

    % Wait for the transformation to be available
    waitForTransform(tree, sourceFrame, targetFrame); 

    % Get the transformation
    tf = getTransform(tree, sourceFrame, targetFrame);

    % Extract the vehicle's pose
    trans = [tf.Transform.Translation.X,
            tf.Transform.Translation.Y];

    quat = [tf.Transform.Rotation.W;
            tf.Transform.Rotation.X;
            tf.Transform.Rotation.Y;
            tf.Transform.Rotation.Z];

    eul = quat2eul(quat');  % Get the euler angles in ZYX order (yaw, pitch, roll)

    vehiclePose = [trans, eul(1)];  % Vehicle's pose in [x, y, theta(yaw)]
end

function broadcastTftree(vehiclePose,tftree,worldFrame)
    % TF 메시지 생성 및 설정
    tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
    tfStampedMsg.ChildFrameId = 'base_link';
    tfStampedMsg.Header.FrameId = worldFrame;
    tfStampedMsg.Header.Stamp = rostime('now');
    tfStampedMsg.Transform.Translation.X = vehiclePose(1);
    tfStampedMsg.Transform.Translation.Y = vehiclePose(2);
    tfStampedMsg.Transform.Rotation.Z = sin(vehiclePose(3)/2);
    tfStampedMsg.Transform.Rotation.W = cos(vehiclePose(3)/2);

    sendTransform(tftree, tfStampedMsg);
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
        odomWaypoints(i,:) = rotatedPoint' + vehiclePose(1:2);
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

function cmdMsg = publish_twist_command(v, w, pub)
    cmdMsg = rosmessage(pub);
    cmdMsg.Linear.X = v;
    cmdMsg.Angular.Z = w;
end


function [innerConePosition, outerConePosition] = Right()
    innerConePosition = [
               0, -1;
               1.5, -1;
               2.5, -2;
               3, -3;
               3, -4;
               3, -5];
    outerConePosition = [
               0, 1;
               2, 1;
               3.5, 0;
               4.5, -1;
               5, -3;
               5, -5];
end

function [innerConePosition, outerConePosition] = Straight()
    innerConePosition = [
               1, -1;
               2, -1;
               3, -1;
               4, -1;
               5, -1;
               6, -1];
    outerConePosition = [
               1,1;
               2, 1;
               3, 1;
               4, 1;
               5, 1;
               6, 1];

end
