% Author: Carlos Santacruz-Rosero, Ph.D. and Tohru Kikawada
% Copyright 2019 The MathWorks, Inc.

% Define the demo install directory
DemoDir = pwd;

% Add directories to papath:
% addpath(genpath(fullfile(pwd, 'Data')));
addpath(genpath(fullfile(pwd, 'html')));
addpath(genpath(fullfile(pwd, 'Images')));
% addpath(genpath(fullfile(pwd, 'MLFiles')));
% addpath(genpath(fullfile(pwd, 'SLFiles')));
% addpath(genpath(fullfile(pwd, 'URDF')));
% addpath(genpath(fullfile(pwd, 'SolidWorks')));
addpath(genpath(fullfile(pwd, 'Source')));
addpath(fullfile(pwd, 'ROSFiles'));
addpath(genpath(fullfile(pwd, 'ROSFiles', 'motoman_description')));
%addpath(genpath(fullfile(pwd, 'GeneratedROSNode', 'mainController_ert_rtw', 'html')));

% Open the html script
web('ManipulatorHTMLPaneGenerator.html');

% Move the user to the Scratch directory
cd('Scratch');

% Store the value of the demo install directory
save DemoDir DemoDir;
