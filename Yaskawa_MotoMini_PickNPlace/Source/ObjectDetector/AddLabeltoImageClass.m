classdef AddLabeltoImageClass < matlab.System & matlab.system.mixin.Propagates
    % Object Detector using YOLO v2
    % Author:  Tohru Kikawada
    % Copyright 2019 The MathWorks, Inc.
     
    % Public, tunable properties
    properties

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
        function obj = AddLabeltoImageClass(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            %d = load('trainedYOLOv2Detector.mat');
            %obj.detector = d.detector;
        end
        
        function [image_label] = stepImpl(obj,image_raw,bboxes,labels_id)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            predict_labels_int = [0 0 0];
            for k = 1:numel(predict_labels)
                if strcmp(predict_labels(k),obj.LABEL_STR(obj.ARM_ID))
                    predict_labels_int(k) = obj.ARM_ID;
                elseif strcmp(predict_labels(k),obj.LABEL_STR(obj.DISK_ID))
                    predict_labels_int(k) = obj.DISK_ID;
                elseif strcmp(predict_labels(k),obj.LABEL_STR(obj.BRACE_ID))
                    predict_labels_int(k) = obj.BRACE_ID;
                end
            end
            
            % F•t‚¯•\Ž¦
            class_num = numel(labels_id);
            cmaps = im2uint8(jet(class_num));
            image_label = image_raw;
            for k = 1: class_num
                if ~isempty(bboxes)
                    image_label = insertObjectAnnotation(image_label,...
                        'rectangle',bboxes(k,:),obj.LABEL_STR(k),'Color',cmaps(k,:));
                end
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
            sz4 = [10 3];
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
