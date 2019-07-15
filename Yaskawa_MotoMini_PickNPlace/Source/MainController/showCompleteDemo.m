%% 自律制御アームロボットによる物体検出とピック&プレイス

%% 初期化
clear all force; close all; clc; %#ok<CLALL>
delete(gcp('nocreate'))
parpool

%% roslaunchによるGazebo起動
% ROS Kinetic Kame + Gazebo v7が必要
%
%  $ roslaunch motoman_mathworks_apps motomini_picking_demo_gazebo_autorun.launch world:=motomini_with_table_parts load_grasp_fix:=true
% 
% セットアップについては下記を参照
% <https://github.com/ntl-ros-pkg/motoman_simulator>

%% 事前に定義したパラメータのロード
load('motominiSplineData.mat');         % load spline data for motion generation
load('params.mat');                   % Load detector data
homeToIdleSpline.Q = zeros(6,6);
homeToIdleSpline.dQ = zeros(6,6);
homeToIdleSpline.t = linspace(0,5,6);
idleToHomeSpline.Q = zeros(6,6);
idleToHomeSpline.dQ = zeros(6,6);
idleToHomeSpline.t = linspace(0,5,6);

%% ロボットパラメータとIPアドレスを設定
ros_master_ip = '192.168.93.128';
motomini = createMotomini();
MotominiParams = setMotominiParameters(ros_master_ip,motomini);
Q = homeToIdleSpline.Q;
numPoints = size(Q,2)-2; % -2 for start point and end point
numJoints = MotominiParams.numJoints;

%% ROSマスターに接続
rosshutdown;
setenv('ROS_MASTER_URI',"http://"+MotominiParams.ROS_IP+":11311")
setenv('ROS_IP','192.168.93.1') %ホスト側のIPアドレス
rosinit

%% Simulinkモデルを起動
open_system('mainController');

%% Simulinkモデルを実行
set_param('mainController','SimulationCommand','start');

%% 軌道計画ROSノードを起動(並列ワーカーに割り当て)
% managePlannerRequests(ros_master_ip,numPoints);
p = gcp(); % Get the current parallel pool
q = parallel.pool.DataQueue();
afterEach(q, @disp);
fp = parfeval(p,@managePlannerRequests,0,q,ros_master_ip,numPoints);
% 
% cancel(fp);

%% 音声コマンドを疑似的にROSで送信(デバッグ用)
% speechRecogNode = robotics.ros.Node('/matlab_speech_recognizer_dummy_node',ros_master_ip);
% speechResultsPub = robotics.ros.Publisher(speechRecogNode,'/speech_recognizer/speech_results',...
%     'std_msgs/String');
% speechResultsMsg = rosmessage(speechResultsPub);
% speechResultsMsg.Data = "right";
% %speechResultsMsg.Data = "left";
% %speechResultsMsg.Data = "forward";
% send(speechResultsPub,speechResultsMsg);
% pause(5);
% speechResultsMsg.Data = "go";
% send(speechResultsPub,speechResultsMsg);

%% 音声認識ROSノードを起動(並列プロセスとして)
p = gcp(); % Get the current parallel pool
fs = parfeval(p,@speechRecognizerNode,0,q,ros_master_ip,'trainedNet.mat','trainedNet');
% 
% cancel(fs);

%% 終了(並列プロセスを停止する)
% set_param('mainController','SimulationCommand','stop');
% cancel(fp);
% cancel(fs);

%% SimulinkモデルからCコード生成を使ってROSノード生成
% d = rosdevice(ros_master_ip,'user','password');

% if you encounter the error "virtual memory exhausted: Cannot allocate
% memory", you need to increate your virtual memory as follows:
% https://digitizor.com/create-swap-file-ubuntu-linux/

% Manually launch motoMINI ros node in virtual machine
% $ cd /home/user/catkin_ws
% $ devel/lib/maincontroller/maincontroller_node

%% ROS環境の設定
% ros_master_ip = MotominiParams.ROS_IP;
% username = 'user';
% password = 'password';
% d = rosdevice(ros_master_ip,username,password);
% d.CatkinWorkspace = '/home/user/catkin_ws_motomini';

% 
% [~,b] = fileparts(tempname);
% logFile = ['/tmp/roscore_' b '.log'];
% 
% % 最初にroslaunchのプロセスをすべて停止
% cmd = ['source ' d.ROSFolder '/setup.bash; pkill roslaunch'];
% d.system(cmd);
% 
% % roslaunchを実行
% cmd = ['source /home/user/.bashrc; ' ...
%     'export ROS_IP=' ros_master_ip ';' ...
%     ' export ROS_MASTER_URI=http://' ros_master_ip ':11311;' ...     % Export the ROS_MASTER_URI
%     ' export DISPLAY=:0;' ...
%     ' source ' d.CatkinWorkspace '/devel/setup.bash;' ...        % Source the setup.bash file we determined above
%     ' roslaunch motoman_mathworks_apps motomini_picking_demo_gazebo_autorun.launch&> ' logFile ...          % Run roscore and pipe output into log file
%     ' &'];
% d.system(cmd);
% pause(10);

% _Copyright 2019 The MathWorks, Inc._
