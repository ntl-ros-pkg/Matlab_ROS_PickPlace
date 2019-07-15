classdef ObjectDetectorYOLOv2 < matlab.System & matlab.system.mixin.Propagates
    % Object Detector using YOLO v2
    % Author:  Tohru Kikawada
    % Copyright 2019 The MathWorks, Inc.
     
    % Public, tunable properties
    properties
        roi = [20 240 600 240]; % Regions of interest (ROI)
        tform = eye(4);         % Transformation matrix
    end
    
    % Public, non-tunable properties
    properties(Nontunable)
        
    end
    
    properties(DiscreteState)
        
    end
       
    % Pre-computed constants
    properties(Access = private)
        detector;
    end
    
    properties(Constant)
        ARM_ID = 1;
        DISK_ID = 2;
        BRACE_ID = 3;
        
        LABEL_STR = ["arm_part","disk_part","t_brace_part"];
    end
    
    methods
        % Constructor
        function obj = ObjectDetectorYOLOv2(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            d = load('trainedYOLOv2Detector.mat');
            obj.detector = d.detector;
        end
        
        function [predictedLabels,bboxes,xyzPoints,labels_id,roi] = stepImpl(obj,im,pts)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
        
            % Pre-allocate variables
            % Classify
            [bboxes,scores,labels] = detect(obj.detector,im);
            % labels_id = zeros(numel(labels));
            for k = 1:numel(labels)
                if labels(k) == categorical(obj.LABEL_STR(obj.ARM_ID))
                    labels_id(k) = obj.ARM_ID;
                elseif labels(k) == categorical(obj.LABEL_STR(obj.DISK_ID))
                    labels_id(k) = obj.DISK_ID;
                elseif labels(k) == categorical(obj.LABEL_STR(obj.BRACE_ID))
                    labels_id(k) = obj.BRACE_ID;
                end
            end
            if numel(labels) > 3
                labels_id = labels_id(1:3);
            end
            
            % Filtering with the ROI
            innerIds = ...
                bboxes(:,1:2) > obj.roi(1:2) & ...
                obj.roi(1:2)+obj.roi(3:4) > bboxes(:,1:2)+bboxes(:,3:4);
            innerIds = innerIds(:,1) & innerIds(:,2);
            bboxes = double(bboxes(innerIds,:));
            scores = double(scores(innerIds,:));
            
            centroids = round(bboxes(:,1:2)+bboxes(:,3:4)/2);
            numObjects = size(bboxes,1);
            xyzPoints = zeros(numObjects,3);
            [~,predictedLabels] = ismember(labels,obj.detector.ClassNames);
            predictedLabels = predictedLabels(innerIds,:);
            
            %predictedLabels = predictedLabels';
            
            for k = 1:numObjects                
                % Acquire position in Kinect coordinate system
                xyzPoints(k,:) = pts(centroids(k,2),centroids(k,1),:);
            end
            if ~isempty(xyzPoints)
                homxyz = cart2hom(xyzPoints);
                tfxyz = homxyz * obj.tform.';
                xyzPoints = tfxyz(:,1:3);
            end
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function num = getNumOutputsImpl(~)
            num = 4;
        end
        
        function [sz1, sz2, sz3, sz4] = getOutputSizeImpl(~)
            sz1 = [10 1];
            sz2 = [10 4];
            sz3 = [10 3];
            sz4 = [1  3];
        end
        
        function [fz1, fz2, fz3, fz4] = isOutputFixedSizeImpl(~)
            fz1 = false;
            fz2 = false;
            fz3 = false;
            fz4 = false;
        end
        
        function [dt1, dt2, dt3, dt4] = getOutputDataTypeImpl(~)
            dt1 = 'double';
            dt2 = 'double';
            dt3 = 'double';
            dt4 = 'double';
        end
        
        function [cp1, cp2, cp3, cp4] = isOutputComplexImpl(~)
            cp1 = false;
            cp2 = false;
            cp3 = false;
            cp4 = false;
        end
        
    end
end
