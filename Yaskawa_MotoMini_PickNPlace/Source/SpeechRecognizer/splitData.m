% splitData(ads,datafolder) splits the data store ads for the Speech
% Commands Dataset into training, validation, and test datastores based on
% the list of validation and test files validation_list.txt and
% testing_list.txt in datafolder.

function [adsTrain,adsValidation,adsTest] = splitData(ads,datafolder)

% Read the list of validation files
c = fileread(fullfile(datafolder,'validation_list.txt'));
filesValidation = string(split(c));
filesValidation  = filesValidation(filesValidation ~= "");

% Read the list of test files
c = fileread(fullfile(datafolder,'testing_list.txt'));
filesTest = string(split(c));
filesTest  = filesTest(filesTest ~= "");

% Determine which files in the datastore should go to validation set and
% which should go to test set
files = ads.Files;
sf    = split(files,filesep);
isValidation = ismember(sf(:,end-1) + "/" + sf(:,end),filesValidation);
isTest       = ismember(sf(:,end-1) + "/" + sf(:,end),filesTest);

adsTest = subset(ads,isTest);
adsValidation = subset(ads,isValidation);
adsTrain = subset(ads,~isValidation & ~isTest);

end