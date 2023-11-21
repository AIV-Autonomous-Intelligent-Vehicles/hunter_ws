function [innerConePosition, outerConePosition] = detectCone(lidarData,params,bboxData_l,bboxData_r,cameraParams,tformCamera_l,tformCamera_r,clusterThreshold)
    xyzData = rosReadXYZ(lidarData);
    ptCloud = pointCloud(xyzData);

    ptCloudOrg = pcorganize(ptCloud, params); 

    groundPtsIdx = segmentGroundFromLidarData(ptCloudOrg);
    nonGroundPtCloud = select(ptCloudOrg, ~groundPtsIdx, 'OutputSize', 'full'); % 지면제거

    indices = findPointsInROI(nonGroundPtCloud, roi);
    roiPtCloud = select(nonGroundPtCloud, indices); % roi 

    roiPtCloud = pcdenoise(roiPtCloud, 'PreserveStructure', true); % 노이즈 제거
    
    % bboxData에서 y_cone, r_cone 추출
    [y_coneBboxs_l, b_coneBboxs_l] = extractConesBboxs(bboxData_l.ObjectList);
    [y_coneBboxs_r, b_coneBboxs_r] = extractConesBboxs(bboxData_r.ObjectList);

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
