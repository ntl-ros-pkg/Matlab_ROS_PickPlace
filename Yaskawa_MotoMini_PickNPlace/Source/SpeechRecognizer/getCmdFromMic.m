function [word] = getCmdFromMic(networkFile,natworkName,h)
    %% ネットワークの読み込み
    load(networkFile, natworkName);

    %% マイクからのストリーミング オーディオを使用したコマンドの検出
    % コマンド検出ネットワークをマイクからのストリーミングオーディオでテスト。

    % オーディオ デバイス リーダーの作成。
    fs = 16e3;
    classificationRate = 20;
    audioIn = audioDeviceReader('SampleRate',fs, ...
        'SamplesPerFrame',floor(fs/classificationRate));

    % ストリーミング スペクトログラムの計算用のパラメーターを指定。
    epsil = 1e-6;
    frameDuration = 0.025;
    hopDuration = 0.010;
    numBands = 40;
    
    frameLength = frameDuration*fs;
    hopLength = hopDuration*fs;
    waveBuffer = zeros([fs,1]);

    labels = trainedNet.Layers(end).Classes;
    YBuffer(1:classificationRate/2) = categorical("background");
    probBuffer = zeros([numel(labels),classificationRate/2]);

    specMin = -6;
    specMax = 3;

    breakFlg = false;
    word = 'unknown';
    
    % figure が存在する限り継続
    while ishandle(h)

        % オーディオデバイスからサンプル取得。
        x = audioIn();
        waveBuffer(1:end-numel(x)) = waveBuffer(numel(x)+1:end);
        waveBuffer(end-numel(x)+1:end) = x;

        % 最新のサンプルからスペクトラムを算出。
        spec = melSpectrogram(waveBuffer,fs, ...
            'WindowLength',frameLength, ...
            'OverlapLength',frameLength - hopLength, ...
            'FFTLength',512, ...
            'NumBands',numBands, ...
            'FrequencyRange',[50,7000]);

        spec = log10(spec + epsil);

        % 学習済みネットワークで分類。
        [YPredicted,probs] = classify(trainedNet,spec,'ExecutionEnvironment','cpu');
        YBuffer(1:end-1)= YBuffer(2:end);
        YBuffer(end) = YPredicted; % 過去10回分の認識結果を保存
        probBuffer(:,1:end-1) = probBuffer(:,2:end);
        probBuffer(:,end) = probs'; % 過去10回分の確率を保存

        % 現在の音声波形とスペクトラムを描画。
        subplot(2,1,1);
        plot(waveBuffer)
        axis tight
        ylim([-0.8,0.8])

        subplot(2,1,2)
        pcolor(spec)
        caxis([specMin+2 specMax])
        shading flat

        % 音声コマンド検出。
        % ノイズでもなく、過去10回中5回以上、かつ最大確率が0.7以上の場合に確定。
        [YMode,count] = mode(YBuffer);
        countThreshold = ceil(classificationRate*0.2);
        maxProb = max(probBuffer(labels == YMode,:));
        probThreshold = 0.7;
        subplot(2,1,1);
        if YMode == "background" || count<countThreshold || maxProb < probThreshold
            title(" ")
        else
            % 確定したら戻り値に認識結果を格納。
            word = string(YMode);
            title(word,'FontSize',20)
            breakFlg = true;
        end

        drawnow
        
        if breakFlg == true
            break;
        end
    end
end