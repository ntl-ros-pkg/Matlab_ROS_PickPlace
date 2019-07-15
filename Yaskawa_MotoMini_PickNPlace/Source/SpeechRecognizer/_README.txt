# 概要
- "forward","left","right","stop","go" のみを認識するネットワークを作成。
- 以下のコマンドを想定。
  - 把持対象パーツの指定 (MotoMINI から見て)
    - "forward": t_brace_part
    - "left": disk_part
    - "right": arm_part
  - 把持開始・停止の指定
    - "stop": 停止
    - "go": 開始
- 時系列のコマンド履歴を配列に格納。
  - "right", "go" が入っていれば、arm_part をピックアップ対象とする、といったコマンドを想定。

# 音声データ学習
- デモ用の学習済みデータは trainedNet.mat に格納。

1. データセットを
`https://storage.cloud.google.com/download.tensorflow.org/data/speech_commands_v0.02.tar.gz`
からダウンロード
2. 上記データセットを SpeechRecognizer フォルダ直下に解凍
3. TrainSpeechRecognitionNet.m を実行

# 音声コマンド検出サンプルアプリ
- 学習済みモデルとして trainedNet.mat を読込。
1. getCmdMain.m を実行
2. "forward","left","right","stop","go" を認識可能。それ以外は"unknown"。
3. "unknown" 以外の検出結果は波形グラフに出力。
4. Figure のクローズ or "stop" の認識によりbreak。

