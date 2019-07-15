%% 深層学習を使用した音声コマンド認識

%% 音声コマンド データセットの読み込み
% データセットは
% https://storage.cloud.google.com/download.tensorflow.org/data/speech_commands_v0.02.tar.gz
% からダウンロード→解凍。
tempdir = pwd;
datafolder = fullfile(tempdir,'data_speech_commands_v0.02');
ads = audioDatastore(datafolder, ...
    'IncludeSubfolders',true, ...
    'FileExtensions','.wav', ...
    'LabelSource','foldernames')
ads0 = copy(ads);

%% 認識する単語の選択
% モデルにコマンドとして認識させる単語を指定。
% commands = categorical(["yes","no","up","down","left","right","on","off","stop","go"]);
commands = categorical(["forward","left","right","stop","go"]);

% 既知の単語と未知の単語の分割。
isCommand = ismember(ads.Labels,commands);
isUnknown = ~ismember(ads.Labels,[commands,"_background_noise_"]);

includeFraction = 0.2;
mask = rand(numel(ads.Labels),1) < includeFraction;
isUnknown = isUnknown & mask;
ads.Labels(isUnknown) = categorical("unknown");

% ファイルとラベルのみを含むデータストアを作成。
ads = subset(ads,isCommand|isUnknown);
countEachLabel(ads)

%% 学習セット、検証セット、およびテスト セットへのデータの分割
% データストアを学習セット、検証セット、およびテスト セットに分割。
[adsTrain,adsValidation,adsTest] = splitData(ads,datafolder);

%% 音声スペクトログラムの計算
% スペクトログラムの計算に使用するパラメーターを定義。
segmentDuration = 1;
frameDuration = 0.025;
hopDuration = 0.010;
numBands = 40;

% 滑らかな分布のデータを得るために、小オフセットepsilでスペクトログラムの対数を取る。
epsil = 1e-6;

XTrain = speechSpectrograms(adsTrain,segmentDuration,frameDuration,hopDuration,numBands);
XTrain = log10(XTrain + epsil);

XValidation = speechSpectrograms(adsValidation,segmentDuration,frameDuration,hopDuration,numBands);
XValidation = log10(XValidation + epsil);

XTest = speechSpectrograms(adsTest,segmentDuration,frameDuration,hopDuration,numBands);
XTest = log10(XTest + epsil);

YTrain = adsTrain.Labels;
YValidation = adsValidation.Labels;
YTest = adsTest.Labels;

%% データの可視化
% いくつかの学習例について波形とスペクトログラムをプロット。
% specMin = min(XTrain(:));
% specMax = max(XTrain(:));
% idx = randperm(size(XTrain,4),3);
% figure('Units','normalized','Position',[0.2 0.2 0.6 0.6]);
% for i = 1:3
%     [x,fs] = audioread(adsTrain.Files{idx(i)});
%     subplot(2,3,i)
%     plot(x)
%     axis tight
%     title(string(adsTrain.Labels(idx(i))))
% 
%     subplot(2,3,i+3)
%     spect = XTrain(:,:,1,idx(i));
%     pcolor(spect)
%     caxis([specMin+2 specMax])
%     shading flat
% 
%     sound(x,fs)
%     pause(2)
% end

% 学習データのピクセル値のヒストグラムをプロット。
% figure
% histogram(XTrain,'EdgeColor','none','Normalization','pdf')
% axis tight
% ax = gca;
% ax.YScale = 'log';
% xlabel("Input Pixel Value")
% ylabel("Probability Density")

%% バックグラウンド ノイズ データの追加
adsBkg = subset(ads0, ads0.Labels=="_background_noise_");
numBkgClips = 4000;
volumeRange = [1e-4,1];

XBkg = backgroundSpectrograms(adsBkg,numBkgClips,volumeRange,segmentDuration,frameDuration,hopDuration,numBands);
XBkg = log10(XBkg + epsil);

% 学習セット、検証セット、テスト セットに分割。
numTrainBkg = floor(0.8*numBkgClips);
numValidationBkg = floor(0.1*numBkgClips);
numTestBkg = floor(0.1*numBkgClips);

XTrain(:,:,:,end+1:end+numTrainBkg) = XBkg(:,:,:,1:numTrainBkg);
XBkg(:,:,:,1:numTrainBkg) = [];
YTrain(end+1:end+numTrainBkg) = "background";

XValidation(:,:,:,end+1:end+numValidationBkg) = XBkg(:,:,:,1:numValidationBkg);
XBkg(:,:,:,1:numValidationBkg) = [];
YValidation(end+1:end+numValidationBkg) = "background";

XTest(:,:,:,end+1:end+numTestBkg) = XBkg(:,:,:,1: numTestBkg);
clear XBkg;
YTest(end+1:end+numTestBkg) = "background";

YTrain = removecats(YTrain);
YValidation = removecats(YValidation);
YTest = removecats(YTest);

% 学習セットと検証セット内のさまざまなクラス ラベルの分布をプロット。
figure('Units','normalized','Position',[0.2 0.2 0.5 0.5]);
subplot(2,1,1)
histogram(YTrain)
title("Training Label Distribution")
subplot(2,1,2)
histogram(YValidation)
title("Validation Label Distribution")

%% データ拡張の追加
sz = size(XTrain);
specSize = sz(1:2);
imageSize = [specSize 1];
augmenter = imageDataAugmenter( ...
    'RandXTranslation',[-10 10], ...
    'RandXScale',[0.8 1.2], ...
    'FillValue',log10(epsil));
augimdsTrain = augmentedImageDatastore(imageSize,XTrain,YTrain, ...
    'DataAugmentation',augmenter);

%% ニューラル ネットワーク アーキテクチャの定義
% シンプルなネットワーク アーキテクチャを層の配列として作成。
% 畳み込み層とバッチ正規化層を使用。(ReLU)
% 最大プーリング層を追加。
% 最後の全結合層への入力に少量のドロップアウトを追加。
% 重み付き交差エントロピー分類損失を使用。

classWeights = 1./countcats(YTrain);
classWeights = classWeights'/mean(classWeights);
numClasses = numel(categories(YTrain));

dropoutProb = 0.2;
numF = 12;
layers = [
    imageInputLayer(imageSize)

    convolution2dLayer(3,numF,'Padding','same')
    batchNormalizationLayer
    reluLayer

    maxPooling2dLayer(3,'Stride',2,'Padding','same')

    convolution2dLayer(3,2*numF,'Padding','same')
    batchNormalizationLayer
    reluLayer

    maxPooling2dLayer(3,'Stride',2,'Padding','same')

    convolution2dLayer(3,4*numF,'Padding','same')
    batchNormalizationLayer
    reluLayer

    maxPooling2dLayer(3,'Stride',2,'Padding','same')

    convolution2dLayer(3,4*numF,'Padding','same')
    batchNormalizationLayer
    reluLayer
    convolution2dLayer(3,4*numF,'Padding','same')
    batchNormalizationLayer
    reluLayer

    maxPooling2dLayer([1 13])

    dropoutLayer(dropoutProb)
    fullyConnectedLayer(numClasses)
    softmaxLayer
    weightedClassificationLayer(classWeights)];

%% ネットワークの学習
% Adam オプティマイザーを使用。
% 学習は 25 エポック行い、20 エポック後に学習率を 10 分の 1 に下げる。
miniBatchSize = 128;
validationFrequency = floor(numel(YTrain)/miniBatchSize);
options = trainingOptions('adam', ...
    'InitialLearnRate',3e-4, ...
    'MaxEpochs',25, ...
    'MiniBatchSize',miniBatchSize, ...
    'Shuffle','every-epoch', ...
    'Plots','training-progress', ...
    'Verbose',false, ...
    'ValidationData',{XValidation,YValidation}, ...
    'ValidationFrequency',validationFrequency, ...
    'LearnRateSchedule','piecewise', ...
    'LearnRateDropFactor',0.1, ...
    'LearnRateDropPeriod',20);

doTraining = false;
if doTraining
    trainedNet = trainNetwork(augimdsTrain,layers,options);
    save('trainedNet.mat','trainedNet');
else
    load('trainedNet.mat','trainedNet');
end


%% 学習済みネットワークの評価
YValPred = classify(trainedNet,XValidation);
validationError = mean(YValPred ~= YValidation);
YTrainPred = classify(trainedNet,XTrain);
trainError = mean(YTrainPred ~= YTrain);
disp("Training error: " + trainError*100 + "%")
disp("Validation error: " + validationError*100 + "%")

% 混同行列をプロット。
figure('Units','normalized','Position',[0.2 0.2 0.5 0.5]);
cm = confusionchart(YValidation,YValPred);
cm.Title = 'Confusion Matrix for Validation Data';
cm.ColumnSummary = 'column-normalized';
cm.RowSummary = 'row-normalized';
sortClasses(cm, [commands,"unknown","background"])

% 利用可能なメモリおよび計算リソースの制限を考慮。
% info = whos('trainedNet');
% disp("Network size: " + info.bytes/1024 + " kB")
% 
% for i=1:100
%     x = randn(imageSize);
%     tic
%     [YPredicted,probs] = classify(trainedNet,x,"ExecutionEnvironment",'cpu');
%     time(i) = toc;
% end
% disp("Single-image prediction time on CPU: " + mean(time(11:end))*1000 + " ms")

%% 音声コマンド検出サンプルアプリ
% - 学習済みモデルとして trainedNet.mat を読込。
% 1. getCmdMain.m を実行
% 2. "forward","left","right","stop","go" を認識可能。それ以外は"unknown"。
% 3. "unknown" 以外の検出結果は波形グラフに出力。
% 4. Figure のクローズ or "stop" の認識によりbreak。
edit getCmdMain.m

%% 音声コマンド検出をROSノードとして実装
% 並列ワーカーに割り当てて非同期実行(マルチコアを効率的に利用するため)
rosshutdown;
ros_mater_ip = "127.0.0.1";
setenv('ROS_MASTER_URI',"")
setenv('ROS_IP',ros_mater_ip) %ホスト側のIPアドレス
rosinit;
delete(gcp('nocreate'))
p = gcp(); % Get the current parallel pool
f = parfeval(p,@speechRecognizerNode,0,ros_mater_ip,'trainedNet.mat','trainedNet');
wait(f,'running');
rostopic echo /speech_recognizer/speech_results
delete(gcp('nocreate'))
