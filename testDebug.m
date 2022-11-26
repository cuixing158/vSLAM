


parkingLotRoot = "C:\Users\6000022946\Desktop\bugImages\parkingLotImages";%"E:\AllDataAndModels\parkingLotImages";
imds = imageDatastore(parkingLotRoot);
% imds = subset(imds,1:10:length(imds.Files));

% setDir  = fullfile(toolboxdir('vision'),'visiondata','imageSets');
% imdsA = imageDatastore(setDir,'IncludeSubfolders',true);
% imds = imageDatastore(repmat(imdsA.Files,12,1));
bofData = bagOfFeatures(imds,CustomExtractor=@helperORBFeatureExtractorFunction,...
    TreeProperties=[5,10]);
