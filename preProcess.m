

%% 预先提取图片Orb特征,bagOfFeatures,用于回环检测
imgsDir = "E:\AllDataAndModels\underParkingLotImages20220606";
imds = imageDatastore(imgsDir);
imds = subset(imds,4:length(imds.Files));
bag = bagOfFeatures(imds,CustomExtractor=@helperORBFeatureExtractorFunction,...
    TreeProperties=[5,10]);

%% 把地下停车场视频打散成图片，看colmap库重建的表现
function decodeVideo(videoRootDir,saveImgs)
% Example 提取少量图像验证
% subImgs = "E:\AllDataAndModels\subparkingImgs";
% imds  = imageDatastore(saveImgs);
% subimds = subset(imds,1:5:length(imds.Files));
% writeall(subimds,subImgs)
arguments
    videoRootDir (1,1) string= "E:\long_horn_newTech";
    saveImgs (1,1) string= "E:\AllDataAndModels\parkingImgs";
end
videoFReader = vision.VideoFileReader(fullfile(videoRootDir,"NOR_20160101_010752.MP4"));
videoPlayer = vision.VideoPlayer;
numPoints = 1000;
numFrame = 1;
while ~isDone(videoFReader)
    if numFrame>1
        if ~videoPlayer.isOpen()
            break;
        end
    end
    videoFrame = videoFReader();
    if mod(numFrame,5)==0
        saveFile = fullfile(saveImgs,[num2str(numFrame),'.jpg']);
        imwrite(videoFrame,saveFile);
    end
    videoPlayer(videoFrame);
    numFrame = numFrame+1;
end
end




