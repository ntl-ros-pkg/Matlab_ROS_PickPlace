%% YOLO v2 物体検出器の学習

%% 初期化
clear; close all; clc; rng('default');

%% 画像データのロード
% imageDatastoreによる大規模画像の取り扱い
% イメージブラウザーでimageDatastoreの内容を確認することも可能
load DemoDir;
imDir = fullfile(DemoDir,'Source','ObjectDetector','trainingData');
imds = imageDatastore(imDir); % imageDatastoreオブジェクトを活用

%% ラベル定義の生成
% ワークのオブジェクト名
labelNames = {'arm_part','t_brace_part','disk_part'};
ldc = labelDefinitionCreator();
for k = 1:numel(labelNames)
    addLabel(ldc,labelNames{k},labelType.Rectangle);
end
labelDefs = create(ldc);
numClasses = numel(labelNames);

%% アノテーションの読み込み
d = load(fullfile(DemoDir,'Source','ObjectDetector','trainingData','groundTruthPoses'));
gtArrayForLabelData = permute(reshape(d.groundTruthPoses.bboxesImg',4*3,[]),[2 1]);
labelData = table(gtArrayForLabelData(:,1:4),...
    gtArrayForLabelData(:,5:8),...
    gtArrayForLabelData(:,9:12),...
    'VariableNames',labelNames);

%% groundTruthオブジェクトの生成
gTruth = groundTruth(groundTruthDataSource(imds.Files),...
    labelDefs,labelData);
save('gTruth','gTruth','imds','labelData');
disp(gTruth);
disp(gTruth.DataSource);

%% Image Labelerで確認
% >> imageLabeler;
% 「ラベルをインポート」→「ワークスペースから」でgTruthを選択して開く

%% 可視化
index = 1;
% 色付け表示
cmaps = im2uint8(jet(width(gTruth.LabelData)));
Iout = imread(gTruth.DataSource.Source{index});
for k = 1: width(gTruth.LabelData)
    bboxes = table2array(gTruth.LabelData(index,k));
    if ~isempty(bboxes{1})
        Iout = insertObjectAnnotation(Iout,'rectangle',bboxes{1},gTruth.LabelDefinitions.Name{k},'Color',cmaps(k,:));
    end
end
figure, imshow(Iout);

%% バウンディングボックスの分布を可視化

% すべてのバウンディングボックスを連結
allBoxes = d.groundTruthPoses.bboxesImg;

% バウンディングボックスの面積とアスペクト比のプロット
aspectRatio = allBoxes(:,3) ./ allBoxes(:,4);
area = prod(allBoxes(:,3:4),2);

figure
scatter(area,aspectRatio)
xlabel("バウンディングボックスの面積")
ylabel("アスペクト比 (width/height)");
title("面積 vs. アスペクト比")

%% クラスタリング

% 4つのグループにクラスタリング
numAnchors = 4;

% K-Medoidsを使ってクラスタリング
[clusterAssignments, anchorBoxes, sumd] = kmedoids(allBoxes(:,3:4),numAnchors,'Distance',@iouDistanceMetric);

% クラスタリングした平均アンカーボックスのサイズ
disp(anchorBoxes);

% クラスタリング結果のプロット
figure
gscatter(area,aspectRatio,clusterAssignments);
title("K-Mediodsで"+numAnchors+"個クラスタリング")
xlabel("面積")
ylabel("アスペクト比 (width/height)");
grid

% 累積加算
counts = accumarray(clusterAssignments, ones(length(clusterAssignments),1),[],@(x)sum(x)-1);

% 平均IoUを計算
meanIoU = mean(1 - sumd./(counts));
disp("平均IoU : " + meanIoU);

%% アンカーボックスの数と平均IoUの関係
% アンカーボックスを増やすと平均IoUは改善するが計算量は増大
% グラフからもk=4がバランスがよさそう

% 1から15までアンカーボックスを増やしたときに平均IoUの改善がどうなるか
maxNumAnchors = 15;
for k = 1:maxNumAnchors
    
    % Estimate anchors using clustering.
    [clusterAssignments, ~, sumd] = kmedoids(allBoxes(:,3:4),k,'Distance',@iouDistanceMetric);
    
    % 平均IoUを計算
    counts = accumarray(clusterAssignments, ones(length(clusterAssignments),1),[],@(x)sum(x)-1);
    meanIoU(k) = mean(1 - sumd./(counts));
end

figure
plot(1:maxNumAnchors, meanIoU,'-o')
ylabel("平均IoU")
xlabel("アンカー数")
title("アンカー数 vs. 平均IoU")

%% 学習用のテーブルに変換

trainingDataset = objectDetectorTrainingData(gTruth);

% 再現性を確保するために乱数のシードを一定値にする
rng(0);

% ランダムに分割
shuffledIndices = randperm(height(trainingDataset));
idx = floor(0.9 * length(shuffledIndices) );
trainingData = trainingDataset(shuffledIndices(1:idx),:);
testData = trainingDataset(shuffledIndices(idx+1:end),:);

%% 学習の準備(YOLO v2)

% 前段の特徴抽出に使う学習済みモデル
network = resnet50();

% 特徴抽出として使うレイヤーを指定
featureLayer = 'activation_40_relu';

% 入力画像サイズ
imageSize = network.Layers(1).InputSize;

% YOLO v2物体検出ネットワークを定義
lgraph = yolov2Layers(imageSize, numClasses, round(anchorBoxes), ...
    network, featureLayer);

% ネットワーク可視化
analyzeNetwork(lgraph)

options = trainingOptions('sgdm', ...
    'InitialLearnRate', 0.001, ...
    'Verbose', true, 'MiniBatchSize', 16, 'MaxEpochs', 50,...
    'Shuffle', 'every-epoch', 'VerboseFrequency', 1);

%% 学習
rng('default');
tic;
[detector,info] = trainYOLOv2ObjectDetector(trainingData,lgraph,options);
toc
figure
plot(info.TrainingLoss)
grid on
xlabel('繰り返し回数')
ylabel('損失関数の値')
save('trainedYOLOv2Detector','detector','info');

%% 推論

I = imread(trainingData.imageFilename{251});

% 検出器を実行
[bboxes, scores, labels] = detect(detector, I);
[~,ind] = ismember(labels,gTruth.LabelDefinitions.Name);

% 結果の可視化
detectedImg = insertObjectAnnotation(I, 'Rectangle', bboxes, cellstr(labels),...
    'Color',cmaps(ind,:));

figure
imshow(detectedImg)

%% 性能評価 
numImages = height(testData);
results = table('Size',[numImages 3],...
    'VariableTypes',{'cell','cell','cell'},...
    'VariableNames',{'Boxes','Scores','Labels'});

% 検出器を全テストデータに対して実行
for i = 1:numImages
    
    % 画像読み込み
    I = imread(testData.imageFilename{i});
    
    % 検出器を実行
    [bboxes,scores,labels] = detect(detector,I);
   
    % 結果の格納
    results.Boxes{i} = bboxes;
    results.Scores{i} = scores;
    results.Labels{i} = labels;
end

% テストデータから真値のバウンディングボックスを取り出し
expectedResults = testData(:, 2:end);

% 平均適合率、再現率、適合率を評価
[ap, recall, precision] = evaluateDetectionPrecision(results, expectedResults);

% 適合率/再現率(PR) 曲線をプロット
figure
hold on;
for k = 1:numel(recall)
    plot(recall{k},precision{k});
end
xlabel('再現率')
ylabel('適合率')
grid on
legend(labelNames,'Interpreter','none');
title(sprintf('平均適合率 = %.2f ', mean(ap)))

%% サポート関数
function dist = iouDistanceMetric(boxWidthHeight,allBoxWidthHeight)
% Return the IoU distance metric. The bboxOverlapRatio function
% is used to produce the IoU scores. The output distance is equal
% to 1 - IoU.

% Add x and y coordinates to box widths and heights so that
% bboxOverlapRatio can be used to compute IoU.
boxWidthHeight = prefixXYCoordinates(boxWidthHeight);
allBoxWidthHeight = prefixXYCoordinates(allBoxWidthHeight);

% Compute IoU distance metric.
dist = 1 - bboxOverlapRatio(allBoxWidthHeight, boxWidthHeight);
end

function boxWidthHeight = prefixXYCoordinates(boxWidthHeight)
% Add x and y coordinates to boxes.
n = size(boxWidthHeight,1);
boxWidthHeight = [ones(n,2) boxWidthHeight];
end

%% 終了
% Copyright 2019 The MathWorks, Inc.
