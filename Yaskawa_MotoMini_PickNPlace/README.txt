0. 実行前にすべてのウィルス対策ソフト、ファイアーウォールをオフにする

1. Ubuntu Linux 16.04上(VMwareなどでも可)でROS Kinetic Kame + Gazebo v7をインストール
  http://wiki.ros.org/kinetic/Installation/Ubuntu
  http://projectsfromtech.blogspot.com/2017/09/installing-ros-on-virtual-machine-for.html

2. catkin workspaceをホームディレクトリなどに作成する
$ mkdir ~/catkin_ws_motomini
$ mkdir ~/catkin_ws_motomini/src
$ cd ~/catkin_ws_motomini/src
$ catkin_init_workspace
$ git clone https://github.com/ntl-ros-pkg/motoman_apps.git
$ cd motoman_apps
$ sh install.sh
$ source ~/catkin_ws_motomini/devel/setup.bash

3. XACROを実行し、URDFを生成する (COLLADA(.dae)をSTLに変換)
$ cd ~/catkin_ws_motomini/src/motoman_pkgs/motoman_robot/motoman_description
$ rosrun xacro xacro --inorder motomini_with_gripper.urdf.xacro > ~/motomini_with_gripper.urdf
$ sudo apt install openctm-tools
$ find . -name '*.dae' | sed 's/.dae$//' | xargs -i ctmconv {}.dae {}.stl    
(ただし、一部のCOLLADAファイルが正しく変換されないので容量0のものはBlenderなどを使ってSTLに手動変換)  
$ sed -i 's/.dae/.stl/g' ~/motomini_with_gripper.urdf 

4. ROSFilesフォルダ以下の1つのファイルと1つのフォルダをコピー
(UbuntuからWindowsにファイルを持ってくる場合はWinSCPなどを使う)
~/motomini_with_gripper.urdf
~/catkin_ws_motomini/src/motoman_pkgs/motoman_robot/motoman_description

5. Motominiのシミュレーション環境を起動
$ source ~/catkin_ws_motomini/devel/setup.bash
$ roslaunch motoman_mathworks_apps motomini_picking_demo_gazebo_autorun.launch

6. MATLAB R2019a以降を起動する

7. 内蔵のIndigoのメッセージからKineticに置き換え
7-0. 「Robotics System Toolbox Interface for ROS Custom Messages」をアドオンエクスプローラからインストール。
アドオンエクスプローラは下記のコマンドで起動。
>>roboticsAddons

7-1. gazebo_msgsパッケージを下記からダウンロード
https://github.com/ros-simulation/gazebo_ros_pkgs
"kinetic-devel"ブランチを使うこと。

7-2.ROSFilesフォルダ以下のcustommsgに解凍
ROSFiles/custommsg/gazebo_msgs
ROSFiles/custommsg/gazebo_msgs/msg
ROSFiles/custommsg/gazebo_msgs/srv
ROSFiles/custommsg/gazebo_msgs/package.xml

7-3. rosgenmsgをMATLABで実行
>> rosgenmsg(fullfile(pwd,'ROSFiles','custommsg'))

7-4. コマンドラインに表示されるメッセージにしたがって javaclasspath.txtファイルを編集。
既存のJARファイルの代わりに新しいものを使用するため"before"トークンを行頭に入れる。
例：
<before>
c:\Yaskawa_MotoMini\ROSFiles\custommsg\matlab_gen\jar\gazebo_msgs-2.5.8.jar

下記のとおりMATLABパスにも追加
addpath(fullfile(pwd,'ROSFiles','custommsg','matlab_gen','msggen'))
savepath

7-5. MATLAB再起動

7-6. 前回生成されたファイルを削除し、再生成
rmdir(fullfile(pwd,'ROSFiles','custommsg','matlab_gen','msggen','+robotics','+ros','+custom','+msggen','+gazebo_msgs'), 's')
rosgenmsg(fullfile(pwd,'ROSFiles','custommsg'))

8. デモフォルダ(PickAndPlaceDemo)の中のstartupDemo.mを実行

9. ブラウザの「MATLAB/Simulink: ピック&プレイスアプリケーション」をクリック

10. showCompleteDemo.mが開いたら14行目のIPアドレスを

  Ubuntu 16.04がインストールされているマシンのIPアドレスに変更しておく

11. コマンドウィンドウで下記のmainControllerのSimulinkモデルを開いておく
  >> open_system('mainController');

12. showCompleteDemo.mを実行

13. mainControllerのSimulinkの実行ボタンで実行
   ピック＆プレイスがはじまる

必要なToolbox
MATLAB R2019a
Robotics System Toolbox
Image Acquisition Toolbox
Image Processing Toolbox
Computer Vision Toolbox
Statistics and Machine Learning Toolbox
Deep Learning Toolbox
Reinforcement Learning Toolbox
Simulink
Stateflow
Simscape
Simscape Multibody
Optimization Toolbox
Global Optimization Toolbox
Parallel Computing Toolbox
MATLAB Coder
GPU Coder
Simulink Coder
Embedded Coder
