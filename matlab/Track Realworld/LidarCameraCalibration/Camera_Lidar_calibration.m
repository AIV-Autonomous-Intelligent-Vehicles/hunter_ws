clear;
clc;
%%

% rosbag 객체 생성
bag=rosbag('right_F.bag'); %위 코드를 실행하여 rosbag파일을 새로 생성했다면 파일명을 꼭 변경해야한다.

%ros lidar, camera 데이터 읽기
imageBag=select(bag,'Topic','/camera_right/usb_cam_right/image_raw/compressed');

pcBag=select(bag, 'Topic','/ouster/points');

imageMsgs=readMessages(imageBag);
pcMsgs=readMessages(pcBag);
ts1 = timeseries(imageBag);
ts2 = timeseries(pcBag);
t1 = (ts1.Time)*50;
t2 = (ts2.Time)*50;

k = 1;
if size(t2,1) > size(t1,1)
    for i = 1:size(t1,1)
        [val,indx] = min(abs(t1(i) - t2));
        if val <= 0.1
            idx(k,:) = [i indx];
            k = k + 1;
        end
    end
else
    for i = 1:size(t2,1)
        [val,indx] = min(abs(t2(i) - t1));
        if val <= 0.1
    
            idx(k,:) = [indx i];
            k = k + 1;
        end
    end
end

pcFilesPath = fullfile(tempdir,'PointClouds');
imageFilesPath = fullfile(tempdir,'Images');
if ~exist(imageFilesPath,'dir')
    mkdir(imageFilesPath);
end
if ~exist(pcFilesPath,'dir')
    mkdir(pcFilesPath);
end

for i = 1:length(idx)
    I = readImage(imageMsgs{idx(i,1)});
    pc = pointCloud(readXYZ(pcMsgs{idx(i,2)}));
    n_strPadded = sprintf('%04d',i) ;
    pcFileName = strcat(pcFilesPath,'/',n_strPadded,'.pcd');
    imageFileName = strcat(imageFilesPath,'/',n_strPadded,'.png');
    imwrite(I,imageFileName);
    pcwrite(pc,pcFileName);
end

checkerSize=92; % 체크박스의 전체 사이즈 / 체크무늬 패턴의 수 , 1000(1m) / 8, 체크박스 1개 파라미터, mm단위로 입력한다.
padding=[0 0 0 0];

lidarCameraCalibrator(imageFilesPath,pcFilesPath,checkerSize,padding)


function rosbagFile = helperDownloadRosbag()
% Download the data set from the given URL.
rosbagZipFile = matlab.internal.examples.downloadSupportFile( ...
    'lidar','data/lccSample.zip');
[outputFolder,~,~] = fileparts(rosbagZipFile);
rosbagFile = fullfile(outputFolder,'lccSample.bag');
if ~exist(rosbagFile,'file')
    unzip(rosbagZipFile,outputFolder);
end
end