clear; close all; clc;
rosshutdown;
rosinit('http://localhost:11311')
lidarSub = rossubscriber('/ouster/points', "DataFormat", "struct");
cameraSub = rossubscriber('/camera_left/usb_cam_0left/image_raw/compressed','DataFormat','struct');
figure;
load("tform_left.mat"); %라이다 카메라 칼리브레이션 파일
load("cameraParams_left.mat") % 카메라 칼리브레이션 파일
while true
    lidarData = receive(lidarSub);
    imageMsg = receive(cameraSub);
	imageData = rosReadImage(imageMsg);
    xyzData = rosReadXYZ(lidarData);
    ptCloud = pointCloud(xyzData);
    imPts = projectLidarPointsOnImage(ptCloud,cameraParams_l,tform_l);
    imshow(imageData)
    hold on
    plot(imPts(:,1),imPts(:,2),'.','Color','r','LineWidth',0.35)
    hold off
end