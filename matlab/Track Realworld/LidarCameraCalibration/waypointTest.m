clear; close all; clc;
rosshutdown;
rosinit('http://localhost:11311')
%tftree = rostf;
pause(3);


roi = [0, 10, -10, 10, -2, 3];
params = lidarParameters('OS1Gen1-64',1024);
clusterThreshold = 1.5; % Cluster distance
cuboidTreshold = 0.0001; % Ignore smaller than 0.003 cuboid (cone: 0.0215) 

lidarSub = rossubscriber('/ouster/points','sensor_msgs/PointCloud2',"DataFormat","struct"); 
client = rossvcclient('/Activate_yolo','cob_object_detection_msgs/DetectObjects','DataFormat','struct');
request_l = rosmessage(client);
request_l.ObjectName.Data = 'left';
request_r = rosmessage(client);
request_r.ObjectName.Data = 'right';

load("left_tform.mat"); 
tformCamera_l = invert(tform);

load("bestCal.mat");
tformCamera_r = invert(tform);

load("bestCameraParams.mat");
load("left_cameraParams.mat");

%% 

figure;

while 1
    lidarData = receive(lidarSub);
    bboxData_r = call(client, request_r);
    bboxData_l = call(client, request_l);
    
    [innerConePosition, outerConePosition] = detectCone(lidarData,...
                                                        params,roi,...
                                                        bboxData_l, bboxData_r, ...
                                                        cameraParams_left, cameraParams_r,...
                                                        tformCamera_l, tformCamera_r, clusterThreshold,cuboidTreshold);
    
    hold off;
    scatter(innerConePosition(:,1),innerConePosition(:,2),'red');
    hold on;
    scatter(outerConePosition(:,1),outerConePosition(:,2),'blue');

    waypoints = generate_waypoints_del(innerConePosition, outerConePosition);
    plot(waypoints(:,1),waypoints(:,2),'r-');
    pause(0.01);
end

function waypoints = generate_waypoints_del(innerConePosition, outerConePosition)
    [m,nc] = size(innerConePosition); 
    cone_coords = zeros(2*m,nc);  
    cone_coords(1:2:2*m,:) = innerConePosition; 
    cone_coords(2:2:2*m,:) = outerConePosition; 
    
    xp = []; 
    yp = []; 

    interv=size(innerConePosition,1)*2;

    %step 1 : delaunay triangulation
    interv = size(innerConePosition,1)*2;
    tri = delaunayTriangulation(cone_coords);
    pl=tri.Points;
    [mc, nc]=size(pl);
	
    if isempty(pl)
        disp("err1\n")
        return
    end


   %step 2 : Remove exterior triangles
    if rem(interv,2) == 0
        cIn = [2 1;(1:2:mc-3)' (3:2:(mc))'; (mc-1) mc];
        cOut = [(2:2:(mc-2))' (4:2:mc)'];
    else
        cIn = [2 1;(1:2:mc-2)' (3:2:(mc))'; (mc-1) mc];
        cOut = [(2:2:(mc-2))' (4:2:mc)'];
    end

    C = [cIn; cOut];
    if isempty(C)
        disp("err2\n");
        return
    end

    TR = delaunayTriangulation(pl,C);
    if isempty(TR.ConnectivityList)
        disp("err3\n");
        return
    end
    
	TL = isInterior(TR);
    TC =TR.ConnectivityList(TL,:);
    [~, pt] = sort(sum(TC,2));

    TS = TC(pt,:);
    if isempty(TS)
        disp("err4\n");
        return
    end
    
    % Check error point
    if max(max(TS)) > size(pl,1)
       disp("err5\n");
       return;
    end

    TO=triangulation(TS,pl);
	    
    %step 3 : Find midpoints of internal edges
    xPo = TO.Points(:,1);
    yPo = TO.Points(:,2);
    E = edges(TO);
    iseven = rem(E,2) == 0;
    Eeven = E(any(iseven,2),:);
    isodd = rem(Eeven,2) ~= 0;
    Eodd = Eeven(any(isodd,2),:);
    xmp = ((xPo((Eodd(:,1))) + xPo((Eodd(:,2))))/2);
    ymp = ((yPo((Eodd(:,1))) + yPo((Eodd(:,2))))/2);
    Pmp = [xmp ymp];

	%  Interpolate midpoints
    distancematrix = squareform(pdist(Pmp));
    distancesteps = zeros(length(Pmp)-1,1);
    for j = 2:length(Pmp)
        distancesteps(j-1,1) = distancematrix(j,j-1);
    end
    totalDistance = sum(distancesteps); % total distance travelled
    distbp = cumsum([0; distancesteps]); % distance for each waypoint
    gradbp = linspace(0,totalDistance,50);
    xq = interp1(distbp,xmp,gradbp,'spline'); % interpolate x coordinates (1xn)
    yq = interp1(distbp,ymp,gradbp,'spline'); % interpolate y coordinates (1xn)

    waypoints=[xq', yq'];
end
