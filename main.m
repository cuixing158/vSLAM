%% 针对我们地下停车场视频测试特征点

videoRootDir = "E:\long_horn_newTech";

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
    videoFrame = im2gray(videoFReader());
    videoFrame = imresize(videoFrame,.3);

    t1 = tic;
    points1 = detectORBFeatures(videoFrame);
    t2 = toc(t1);

    % Select a subset of features, uniformly distributed throughout the image
%     points1 = selectUniform(points1, numPoints, size(videoFrame, 1:2));

    % Extract features
    features = extractFeatures(videoFrame, points1);


    t3 = tic;
    points2 = detectFastFeatures(videoFrame);
    t4 = toc(t3);
    % Select a subset of features, uniformly distributed throughout the image
%     points2 = selectUniform(points2, numPoints, size(videoFrame, 1:2));

    % Extract features
    features = extractFeatures(videoFrame, points2);

    fprintf('orb: %.6f seconds,fast: %.6f seconds\n',t2,t4);

    videoFrame = insertMarker(videoFrame,points1.Location,"x-mark","Color","green");
    videoFrame = insertMarker(videoFrame,points2.Location,"circle","Color","red");
    videoPlayer(videoFrame);
    numFrame = numFrame+1;
end
release(videoPlayer);
release(videoFReader);
