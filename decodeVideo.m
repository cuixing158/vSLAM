
%% 把地下停车场视频打散成图片，看colmap库重建的表现
videoRootDir = "E:\long_horn_newTech";
saveImgs = "E:\AllDataAndModels\parkingImgs";
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

%% 提取少量图像验证
subImgs = "E:\AllDataAndModels\subparkingImgs";
imds  = imageDatastore(saveImgs);
subimds = subset(imds,1:5:length(imds.Files));
writeall(subimds,subImgs)


