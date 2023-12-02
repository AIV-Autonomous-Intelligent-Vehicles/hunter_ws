function  [wp_x, wp_y] = generateWaypoints(PtCloudData, y_coneBboxs_l, b_coneBboxs_l, y_coneBboxs_r, b_coneBboxs_r) %#codegen
    
    %% Camera-Lidar calibration

    clusterThreshold = 0.3; % Cluster distance
    cuboidTreshold = 0.0020; % Ignore smaller than 0.003 cuboid (cone: 0.0215) 
    waypoints = [];
    tform1 = load("camera1.mat","tform1");
    tform2 = load("camera2.mat","tform2");
    cameraParams = load("cameraParams.mat","cameraParams");
    tformCamera1 = invert(tform1.tform1);
    tformCamera2 = invert(tform2.tform2); 
    
    x = PtCloudData(:,1);
    y = PtCloudData(:,2);
    z = PtCloudData(:,3);
    roiPtCloud = pointCloud([x y z]);


    [bboxesLidar_l, ~, boxesUsed_l] = bboxCameraToLidar([y_coneBboxs_l; b_coneBboxs_l], roiPtCloud, cameraParams.cameraParams, tformCamera2, 'ClusterThreshold',clusterThreshold);
    [bboxesLidar_r, ~, boxesUsed_r] = bboxCameraToLidar([y_coneBboxs_r; b_coneBboxs_r], roiPtCloud, cameraParams.cameraParams, tformCamera1, 'ClusterThreshold',clusterThreshold);
  
    %% split cone bbox

    idx = sum(boxesUsed_l(1:size(y_coneBboxs_l,1)));
    y_coneBboxesLidar_l = bboxesLidar_l(1:idx, :);
    b_coneBboxesLidar_l = bboxesLidar_l(idx+1:end, :);
    
    idx = sum(boxesUsed_r(1:size(y_coneBboxs_r,1)));
    y_coneBboxesLidar_r = bboxesLidar_r(1:idx, :);
    b_coneBboxesLidar_r = bboxesLidar_r(idx+1:end, :);
    
  
    %% extract cone pos
    
    % Extract xlen, ylen, zlen from the bounding boxes
    volumes_l = prod(b_coneBboxesLidar_l(:, 4:6), 2);
    volumes_r = prod(b_coneBboxesLidar_r(:, 4:6), 2);

    % Find indices where volumes are smaller than cuboidThreshold
    indices_l = volumes_l > cuboidTreshold;
    indices_r = volumes_r > cuboidTreshold;

    % Combine the inner cone positions from left and right into a single array
    innerConePosition = [b_coneBboxesLidar_l(indices_l, 1:2);b_coneBboxesLidar_r(indices_r, 1:2)];

    % Extract xlen, ylen, zlen from the bounding boxes
    volumes_l = prod(y_coneBboxesLidar_l(:, 4:6), 2);
    volumes_r = prod(y_coneBboxesLidar_r(:, 4:6), 2);

    % Find indices where volumes are smaller than cuboidThreshold
    indices_l = volumes_l > cuboidTreshold;
    indices_r = volumes_r > cuboidTreshold;

    % Combine the inner cone positions from left and right into a single array
    outerConePosition = [y_coneBboxesLidar_l(indices_l, 1:2);y_coneBboxesLidar_r(indices_r, 1:2)];
    
    %% match array length 
    [~, uniqueIdx] = unique(innerConePosition, 'rows');
    innerConePosition = innerConePosition(uniqueIdx,:);
    innerConePosition = sortrows(innerConePosition);

    [~, uniqueIdx] = unique(outerConePosition, 'rows');
    outerConePosition = outerConePosition(uniqueIdx,:);
    outerConePosition = sortrows(outerConePosition);

    len1 = size(innerConePosition, 1); % Get the number of rows
    len2 = size(outerConePosition, 1); % Get the number of rows

    if len1 > len2
        innerConePosition = innerConePosition(1:len2, :); % Keep only the first len2 rows
    elseif len1 < len2
        outerConePosition = outerConePosition(1:len1, :); % Keep only the first len1 rows
    else
        
    end
    
    if size(innerConePosition,1) < 2 || size(outerConePosition,1) < 2
        wp_x = zeros(50,1);
        wp_y = zeros(50,1);
        disp("Cone error\n")
        return
    end
    %% generate waypoint
    
    % Preprocessing the data 
    [m,nc] = size(innerConePosition); % size of the inner/outer cone positions data
    cone_coords = zeros(2*m,nc);  % initiate a cone_coords matrix consisting of inner and outer coordinates
    cone_coords(1:2:2*m,:) = innerConePosition; 
    cone_coords(2:2:2*m,:) = outerConePosition; 

    xp = []; % create an empty numeric xp vector to store the planned x coordinates 
    yp = []; % create an empty numeric yp vector to store the planned y coordinates 



    % step 1 : Form triangles
    interv = size(innerConePosition,1)*2;
    tri = delaunayTriangulation(cone_coords);
    pl=tri.Points;
    [mc, nc]=size(pl);
    
    if isempty(pl)
        wp_x = zeros(50,1);
        wp_y = zeros(50,1);
        disp("error1\n")
        return
    end
    %step 2 : Remove exterior triangles
    % Define constraints
    if rem(interv,2) == 0
        cIn = [2 1;(1:2:mc-3)' (3:2:(mc))'; (mc-1) mc];
        cOut = [(2:2:(mc-2))' (4:2:mc)'];
    else
    % inner and outer constraints when the interval is odd
        cIn = [2 1;(1:2:mc-2)' (3:2:(mc))'; (mc-1) mc];
        cOut = [(2:2:(mc-2))' (4:2:mc)'];
    end
    C = [cIn; cOut];
    if isempty(C)
        wp_x = zeros(50,1);
        wp_y = zeros(50,1);
        disp("error2\n")
        return
    end
    TR = delaunayTriangulation(pl,C);
    if isempty(TR.ConnectivityList)
        wp_x = zeros(50,1);
        wp_y = zeros(50,1);
        disp("error3\n")
        return
    end
    TL = isInterior(TR);
    TC =TR.ConnectivityList(TL,:);
    [~, pt] = sort(sum(TC,2));
    TS = TC(pt,:);
    if isempty(TS)
        wp_x = zeros(50,1);
        wp_y = zeros(50,1);
        disp("error4\n")
        return
    end
    
    % Check error point
    if max(max(TS)) > size(pl,1)
       wp_x = zeros(50,1);
       wp_y = zeros(50,1);
       disp("error5\n")
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

    wp_x = waypoints(:,1);
    wp_y = waypoints(:,2);
    check = 0;
end