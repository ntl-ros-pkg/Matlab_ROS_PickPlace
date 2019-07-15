%% 最適化計算による軌道計画
% MATLAB, Simulink, Robotics System
% Toolbox, Simscape Multibody, and Optimization Toolbox
% Author: Carlos Santacruz-Rosero, Ph.D. and Tohru Kikawada
% Copyright 2019 The MathWorks, Inc.

%% 0. 初期化
clear; close all; clc; bdclose all;

%% 1. MotoMiniをツリー構造ロボットとして表現
modelName = 'motominiPlanning';
motomini = createMotomini();
restConfig = motomini.homeConfiguration;
restConfig(7).JointPosition = 0.03;
restConfig(8).JointPosition = 0.03;
hFig = figure;
show(motomini,restConfig,'Frame','off');
axis tight
campos auto
EELinkName = 'gripper_EE_link';
motominiParams = setMotominiParameters('',motomini);

%% 2. ターゲットの設定
cupHeight = 0.05;
cupRadius = 0.02;
cupPosition = [0.3;0.3;cupHeight/2]; 

alpha 0.1;
hold on;
% Create points for visualizing a cup
[X,Y,Z] = cylinder(cupRadius*linspace(0,1,50).^0.125);
% Scale the Z coordinates
Z = cupHeight*Z - cupHeight/2;
% Translate to the specified position
X = X + cupPosition(1);
Y = Y + cupPosition(2);
Z = Z + cupPosition(3);
% Add the cup to the figure and configure lighting
s = patch(surf2patch(X,Y,Z));
s.FaceColor = 'red';
s.FaceLighting = 'gouraud';
s.EdgeAlpha = 0;
axis tight
campos auto
camtarget auto
camva auto
shg;

%% 最終姿勢の設定
% 初期姿勢
X0_tform = motomini.getTransform(restConfig,'gripper_EE_link');
X0.p = X0_tform(1:3,4); 
X0.ezyx = tform2eul(X0_tform)';

% 最終姿勢
Xf.p =  cupPosition; 
Xf.ezyx = [0; -pi/2; pi+pi/4];

% 可視化
rbtree = robotics.RigidBodyTree;
body1 = robotics.RigidBody('cup_reach_frame');
jnt1 = robotics.Joint('jnt1','fixed');
jnt1.setFixedTransform(trvec2tform(Xf.p')*eul2tform(Xf.ezyx'));
body1.Joint = jnt1;
rbtree.addBody(body1,'base');
h = show(rbtree);
shg;

%% 3. 始点と終点の逆運動学を解く

% 関節角計算
[q0, ~] = solveIK(motomini,EELinkName, X0.p, X0.ezyx, [restConfig.JointPosition]);
[qf, ~] = solveIK(motomini,EELinkName, Xf.p, Xf.ezyx, [restConfig.JointPosition]);
finalConfig = restConfig;
finalConfig = arrayfun(@(x,y) setfield(x, 'JointPosition', y), finalConfig, qf);
hold on;
show(motomini,finalConfig,'PreservePlot',true,'Frames','off');
shg;

%% 4. 線形補間で初期軌道を生成
doOptim = true;

% 実行までの時間
totalTime = 1;
% 障害物
obstacles = [];
% 経由点の数
numPoints = 4;
% 並列計算の有無
useParallel = false;
% 可視化
showMultibodyExplorer = true;

% 線形補間で初期軌道を生成
t0 = 0;
t = linspace(t0,totalTime,numPoints + 2);
Q0 = zeros(motominiParams.numJoints, numPoints + 2);
for i = 1:motominiParams.numJoints
    Q0(i,:) = linspace(q0(i), qf(i),numPoints + 2);
end
Qt = Q0;

% Robotics System Toolboxで軌道を可視化
% 軌道を関節座標系からEEの作業座標系に変換
Xt = zeros(size(Qt,2),3);
config = restConfig;
for k = 1:size(Qt,2)
    config = arrayfun(@(x,y) setfield(x, 'JointPosition', y), config, [Qt(:,k);0;0]');
    Xt(k,:) = tform2trvec(motomini.getTransform(config,'gripper_EE_link'));
end
XYZOr = calcXYZFromSplineMatrix(motomini,EELinkName,numPoints,Q0);
hXYZOr = plot3(XYZOr(:,1),XYZOr(:,2),XYZOr(:,3),'-o','LineWidth',3);
shg;

%% Simscape Multibodyで逆動力学シミュレーション(線形補間軌道)
% Simscape MultibodyのMechanics Explorer でアニメーション化
open_system(modelName);

set_param([modelName '/Draw Trajectory'],'Commented','on');
set_param(modelName,'SimMechanicsOpenEditorOnUpdate','on');
MotominiParams.jntMaxLim = motominiParams.jntMaxLim;
MotominiParams.jntMinLim = motominiParams.jntMinLim;
sim(modelName);

%% 4. 軌道計画を最適化問題として解く
% FMINCONを呼んで最適化を解く

[QOpt,Q0,t,Qout,dQout] = planTrajectory(doOptim,q0, qf, totalTime, ...
    numPoints, obstacles, motominiParams, modelName, useParallel,...
    showMultibodyExplorer);
% 時間がかかるので必要に応じて「停止」をクリックして最適化を止める

%% 5. 最適化計算結果をロードして表示
load('MotominiOptimResultsForDemoShowcase.mat')

%% 6. 3次スプライン補間による軌道生成と可視化
XYZOpt = calcXYZFromSplineMatrix(motomini,EELinkName,numPoints,QOpt);
figure(hFig);
plot3(XYZOpt(:,1),XYZOpt(:,2),XYZOpt(:,3),'-o','LineWidth',3);
shg;

%% 6. 3次スプライン補間による軌道生成のシミュレーション
% Show results of optimization in Multibody Explorer
q0 = Q0(:,1);
Qt = QOpt;
set_param(modelName,'SimMechanicsOpenEditorOnUpdate','on');
sim(modelName);

%%
% %% 8. 障害物の定義
% set_param([modelName '/Collision Model'],'Commented','off')
% colObject1.type = 'sphere';
% colObject1.radius = 0.13;
% colObject1.pos = [0.0428; 0.1358; 0.5536];
% obstacles = [colObject1];   % in the future this will be an array. 
% 
% 
% %% 9. 再度、最適化ルーチンを実行
% close all
% % PlanTrajectory calls FMINCON
% [QOpt,Q0,t,Qout,dQout] = planTrajectory(doOptim,q0, qf, totalTime, ...
%     numPoints, obstacles, motominiParams, modelName, useParallel,...
%     showMultibodyExplorer);
% % Stop the Optimization figure and load results in Next step
% 
% %% 10. 最適化計算結果のロード
% load('YBOptimResultsForDemoShowcase.mat')
% QOpt = QOptExampleObs;
% Q0 = Q0ExampleObs;
% openfig('QoptExampleObsYB.fig');
% 
% 
% %% 11. 3次スプライン補間による軌道生成と可視化、シミュレーション
% Qt = QOpt;
% q0 = Q0(:,1);
% set_param(modelName,'SimMechanicsOpenEditorOnUpdate','on');
% XYZOr = calcXYZFromSplineMatrix(youbot,EELinkName,numPoints,Q0);
% XYZOpt = calcXYZFromSplineMatrix(youbot,EELinkName,numPoints,QOpt);
% sim(modelName);




%% Dont' run this: Save results from optimization
% QOptExampleObs = QOpt;
% Q0ExampleObs = Q0;
% save('YBOptimResultsForDemoShowcase.mat','QOptExampleObs', 'Q0ExampleObs', '-append');


