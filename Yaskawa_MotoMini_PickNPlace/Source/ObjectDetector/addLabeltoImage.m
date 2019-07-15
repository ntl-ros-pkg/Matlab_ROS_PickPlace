%function image_label = addLabeltoImage(image_raw)
clear all force; close all; clc;
image_raw = imread("trainingData/image_00001.png");
%figure, imshow(image_raw)

labels = ["arm_part" "disk_part" "t_brace_part"]';
LABEL_ARM = 1;
LABEL_DISK = 2;
LABEL_BRACE = 3;

predict_labels = ["t_brace_part" "disk_part" "arm_part"]';
predict_labels_int = [0 0 0];

for k = 1:numel(predict_labels)
    if strcmp(predict_labels(k),"arm_part")
    	predict_labels_int(k) = LABEL_ARM;
    elseif strcmp(predict_labels(k),"disk_part")
    	predict_labels_int(k) = LABEL_DISK;
    elseif strcmp(predict_labels(k),"t_brace_part")
    	predict_labels_int(k) = LABEL_BRACE;
    end
end

disp(predict_labels_int)

% image_label=image_raw;
%figure, imshow(image_label);
%saveas(image_label, "image_label.png")
index = 1;
% êFïtÇØï\é¶
class_num = numel(labels);
cmaps = im2uint8(jet(class_num));
image_label = image_raw; %imread(gTruth.DataSource.Source{index});
bboxes = [179 236 50 48;
          277 232 55 47;
          225 245 52 63];
for k = 1: class_num
    if ~isempty(bboxes)
        image_label = insertObjectAnnotation(image_label,'rectangle',bboxes(k,:),labels(k),'Color',cmaps(k,:));
    end
end

figure, imshow(image_label);