clear; close all; clc;
rosshutdown;
rosinit('http://localhost:11311')
tftree = rostf;
pause(2);
bag = rosbag("2023-11-26-19-12-29.bag");
% imuMsg = select(bag,"Topic","/imu/data");
gpsMsg = select(bag,"Topic","/ublox_gps/fix");
% imuMsg = select(bag,"Topic","/imu/data");

gpsData = readMessages(gpsMsg);
% imuData = readMessages(imuMsg);
%% 

[x_utm, y_utm] = projfwd(projcrs(32652), gpsData{1}.Latitude, gpsData{1}.Longitude);
% quat = [imuMsg.Orientation.W,imuMsg.Orientation.X,imuMsg.Orientation.Y,imuMsg.Orientation.Z];
% euler = quat2eul(quat);

vehiclePose_origin = [x_utm, y_utm, 0];
% figure
for i = 1:length(gpsData)

    [x_utm, y_utm] = projfwd(projcrs(32652), gpsData{i}.Latitude, gpsData{i}.Longitude);

    vehiclePose = [x_utm, y_utm,0];
    delta_x = vehiclePose(1) - vehiclePose_origin(1);
    delta_y = vehiclePose(2) - vehiclePose_origin(2);

    % 초기 위치에서 현재 위치로의 회전 변위 계산
    delta_yaw = vehiclePose(3) - vehiclePose_origin(3);

    % 회전 행렬을 사용하여 회전 변환 수행
    R = [cos(delta_yaw), -sin(delta_yaw); sin(delta_yaw), cos(delta_yaw)];
    delta_xy_rotated = R * [delta_x; delta_y];

    % 상대적인 위치 및 회전 변위 계산
    relative_x = delta_xy_rotated(1);
    relative_y = delta_xy_rotated(2);
    relative_yaw = delta_yaw;

    % 결과를 [x_rel, y_rel, yaw_rel] 형식으로 반환
    vehiclePose = [relative_x, relative_y, relative_yaw];
    a(i,:) = vehiclePose;
    b(i,:) = [delta_x,delta_y];
    worldFrame = 'hunter_gps';
    broadcastTftree(vehiclePose,tftree,worldFrame);
    % plot(vehiclePose(1),vehiclePose(2),'r.');
    % hold on

end



function broadcastTftree(vehiclePose,tftree,worldFrame)
    % TF 메시지 생성 및 설정
    tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
    tfStampedMsg.ChildFrameId = 'base_link';
    tfStampedMsg.Header.FrameId = worldFrame;
    tfStampedMsg.Header.Stamp = rostime('now');
    tfStampedMsg.Transform.Translation.X = vehiclePose(1);
    tfStampedMsg.Transform.Translation.Y = vehiclePose(2);
    % tfStampedMsg.Transform.Rotation.Z = sin(vehiclePose(3)/2);
    % tfStampedMsg.Transform.Rotation.W = cos(vehiclePose(3)/2);
    q = eul2quat([0 0 vehiclePose(3)]);
    tfStampedMsg.Transform.Rotation.X = q(2);
    tfStampedMsg.Transform.Rotation.Y = q(3);
    tfStampedMsg.Transform.Rotation.Z = q(4);
    tfStampedMsg.Transform.Rotation.W = q(1);
    sendTransform(tftree, tfStampedMsg);
end