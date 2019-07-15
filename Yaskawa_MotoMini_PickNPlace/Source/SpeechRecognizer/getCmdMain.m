%% 音声認識メインファイル
% "forward","left","right","stop","go" を認識可能。

% FFigure が存在する限りコマンドを検出。
h = figure('Units','normalized','Position',[0.2 0.1 0.6 0.8]);
cmd_hst = strings(0);

% Figure を閉じるか "stop" と入力するば break。
while ishandle(h)
  cmd_str = getCmdFromMic('trainedNet.mat','trainedNet',h);
  disp(cmd_str);
  
  if ~strcmp(cmd_str,'unknown')
    cmd_hst(end+1) = cmd_str;
  end
  
  if strcmp(cmd_str,'stop')
      break;
  end
end

% 音声コマンド履歴表示
disp('Command histories:');
disp(cmd_hst);
