%%
classdef ros_rate < matlab.System
   
    % Public, tunable properties
    properties
        RATE;
        
    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)
        rateObj;
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            %obj.RateObj = robotics.Rate(1/obj.SampleTime);
            obj.rateObj = robotics.Rate(obj.RATE);
        end

        function stepImpl(obj)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            % roiPtCloud = pointCloud(xyzData);
            % [bboxesLidar_l, ~, boxesUsed_l] = bboxCameraToLidar([L_Y_ConeBboxs; L_B_ConeBboxs], roiPtCloud, CameraParam, Camera2, 'ClusterThreshold', 0.3);
            obj.rateObj.waitfor();

        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
end
