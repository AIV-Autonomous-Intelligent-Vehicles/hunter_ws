clear; close all; clc;
rosshutdown;
rosinit('http://localhost:11311')
lidarSub = rossubscriber('/ouster/points', "DataFormat", "struct");
cameraSub = rossubscriber('/camera_left/usb_cam_left/image_raw/compressed','DataFormat','struct');
figure;
load("Final_left_cal.mat"); %라이다 카메라 칼리브레이션 파일
load("Final_left_cameraParam.mat") % 카메라 칼리브레이션 파일
while true
    lidarData = receive(lidarSub);
    imageMsg = receive(cameraSub);
	imageData = rosReadImage(imageMsg);
    xyzData = rosReadXYZ(lidarData);
    ptCloud = pointCloud(xyzData);
    imPts = projectLidarPointsOnImage(ptCloud,cameraParams,tform);
    imshow(imageData)
    hold on
    plot(imPts(:,1),imPts(:,2),'.','Color','r','LineWidth',0.35)
    hold off
end