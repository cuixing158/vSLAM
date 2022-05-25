%% 使用曾总的图像做vSLAM
parkingLotRoot = "E:\AllDataAndModels\underParkingLotImages20220524";%"H:\dataSets\vSLAM\parkingLotImages";%"E:\AllDataAndModels\parkingLotImages";%
validIndexStart = 4;
imds = imageDatastore(parkingLotRoot);
imds = subset(imds,validIndexStart:length(imds.Files));
gTruthData = readtable(fullfile(parkingLotRoot,"simUE_eular.csv"));
gTruthData = gTruthData(validIndexStart:end,:);
gTruth = helperGetSensorGroundTruth(gTruthData);

%% 我们摄像头停车场视频
% videoReader = VideoReader('E:\AllDataAndModels\videos20220519\WIN_20220519_11_16_10_Pro.mp4');
% videoPlayer = vision.VideoPlayer;
% while hasFrame(videoReader)
%    frame = readFrame(videoReader);
%    frame = imrotate(frame,180);
%    step(videoPlayer,frame);
% end
% videoPlayer.release()

%% Camera intrinsics
% focalLength    = [467.421, 467.3539];  % specified in units of pixels，自己标定的，暂时标定参数与实际采集视频不匹配，有问题再弄
% principalPoint = [ 320.7214,180.4589];  % in pixels [x, y]
% imageSize      = [1080, 1920]; % in pixels [mrows, ncols]

focalLength    = [1046, 1046];  % specified in units of pixels，曾总提供的
principalPoint = [ 1920/2,1080/2];  % in pixels [x, y]
imageSize      = [1080, 1920]; % in pixels [mrows, ncols]

% focalLength    = [700, 700];  % specified in units of pixels,demo 自带的
% principalPoint = [ 600,180];  % in pixels [x, y]
% imageSize      = [370, 1230]; % in pixels [mrows, ncols]
intrinsics     = cameraIntrinsics(focalLength, principalPoint, imageSize);

%% 主模块
% Set random seed for reproducibility
rng(0);

%% Map Initialization
currFrameIdx = 1;
currI        = readimage(imds, currFrameIdx);

% Detect and extract ORB features
scaleFactor  = 1.2;
numLevels    = 8;
[preFeatures, prePoints] = helperDetectAndExtractFeatures(currI, scaleFactor, ...
    numLevels); 

currFrameIdx = currFrameIdx + 1;
firstI       = currI; % Preserve the frame 

isMapInitialized  = false;

% Map initialization loop
while ~isMapInitialized && currFrameIdx < numel(imds.Files)
    currI = readimage(imds, currFrameIdx);

    [currFeatures, currPoints] = helperDetectAndExtractFeatures(currI, ...
        scaleFactor, numLevels); 
    
    currFrameIdx = currFrameIdx + 1;
    
    % Find putative feature matches
    indexPairs = matchFeatures(preFeatures, currFeatures, 'Unique', true, ...
        'MaxRatio', 0.7, 'MatchThreshold', 40);

    % If not enough matches are found, check the next frame
    minMatches = 10;
    if size(indexPairs, 1) < minMatches 
        continue
    end

    preMatchedPoints  = prePoints(indexPairs(:,1),:);
    currMatchedPoints = currPoints(indexPairs(:,2),:);
    
    % Compute fundamental matrix and evaluate reconstruction
    [tformF, ~, inliersIdxF] = helperComputeFundamentalMatrix(preMatchedPoints, currMatchedPoints);

    % Select the model based on a heuristic
    inlierTformIdx = inliersIdxF;
    tform          = tformF;


    % Computes the camera location up to scale. Use half of the 
    % points to reduce computation
    inlierPrePoints  = preMatchedPoints(inlierTformIdx);
    inlierCurrPoints = currMatchedPoints(inlierTformIdx);
    [relOrient, relLoc, validFraction] = relativeCameraPose(tform, intrinsics, ...
        inlierPrePoints(1:2:end), inlierCurrPoints(1:2:end));
    
    % If not enough inliers are found, move to the next frame
    if validFraction < 0.6 || numel(size(relOrient))==3
        continue
    end
    
    % Triangulate two views to obtain 3-D map points
    relPose = rigid3d(relOrient, relLoc);
    minParallax = 0.2;
    [isValid, xyzWorldPoints, inlierTriangulationIdx] = helperTriangulateTwoFrames(...
        rigid3d, relPose, inlierPrePoints, inlierCurrPoints, intrinsics, minParallax);
    
    if ~isValid
        continue
    end
    
    % Get the original index of features in the two key frames
    indexPairs = indexPairs(inlierTformIdx(inlierTriangulationIdx),:);
    
    isMapInitialized = true;
    
    disp(['Map initialized with frame 1 and frame ', num2str(currFrameIdx-1)])
end % End of map initialization loop

if isMapInitialized
    % Show matched features
    hfeature = figure;
    showMatchedFeatures(firstI, currI, prePoints(indexPairs(:,1)), ...
        currPoints(indexPairs(:, 2)), 'montage', 'Parent', gca(hfeature));
else
    error('Unable to initialize the map.')
end

%% Store Initial Key Frames and Map Points
% 到这里只有第一个姿态和最后的姿态地面特征点和车引擎盖特征点匹配，UE的图应当间隔较小再获取图像才对

% Create an empty imageviewset object to store key frames
vSetKeyFrames = imageviewset;

% Create an empty worldpointset object to store 3-D map points
mapPointSet   = worldpointset;

% Create a helperViewDirectionAndDepth object to store view direction and depth 
directionAndDepth = helperViewDirectionAndDepth(size(xyzWorldPoints, 1));

% Add the first key frame. Place the camera associated with the first 
% key frame at the origin, oriented along the Z-axis
preViewId     = 1;
vSetKeyFrames = addView(vSetKeyFrames, preViewId, rigid3d, 'Points', prePoints,...
    'Features', preFeatures.Features);

% Add the second key frame
currViewId    = 2;
vSetKeyFrames = addView(vSetKeyFrames, currViewId, relPose, 'Points', currPoints,...
    'Features', currFeatures.Features);

% Add connection between the first and the second key frame
vSetKeyFrames = addConnection(vSetKeyFrames, preViewId, currViewId, relPose, 'Matches', indexPairs);

% Add 3-D map points
[mapPointSet, newPointIdx] = addWorldPoints(mapPointSet, xyzWorldPoints);

% Add image points corresponding to the map points in the first key frame
mapPointSet   = addCorrespondences(mapPointSet, preViewId, newPointIdx, indexPairs(:,1));

% Add image points corresponding to the map points in the second key frame
mapPointSet   = addCorrespondences(mapPointSet, currViewId, newPointIdx, indexPairs(:,2));

%% Refine and Visualize the Initial Reconstruction

[vSetKeyFrames, mapPointSet, directionAndDepth] = helperGlobalBundleAdjustment(...
    vSetKeyFrames, mapPointSet, directionAndDepth, intrinsics, relPose);

% Visualize matched features in the current frame
featurePlot   = helperVisualizeMatchedFeatures(currI, currPoints(indexPairs(:,2)));

% Visualize initial map points and camera trajectory
mapPlot       = helperVisualizeSceneAndTrajectory(vSetKeyFrames, mapPointSet);

% Show legend
showLegend(mapPlot);


%% Tracking
% The tracking process is performed using every frame and determines when to 
% insert a new key frame. To simplify this example, we will terminate the tracking 
% process once a loop closure is found.

% ViewId of the current key frame
currKeyFrameId    = currViewId;

% ViewId of the last key frame
lastKeyFrameId    = currViewId;

% Index of the last key frame in the input image sequence
lastKeyFrameIdx   = currFrameIdx - 1; 

% Indices of all the key frames in the input image sequence
addedFramesIdx    = [1; lastKeyFrameIdx];

isLoopClosed      = false;

% Main loop
isLastFrameKeyFrame = true;
while ~isLoopClosed && currFrameIdx < numel(imds.Files)
    currI = readimage(imds, currFrameIdx);

    [currFeatures, currPoints] = helperDetectAndExtractFeatures(currI, ...
        scaleFactor, numLevels);

    % Track the last key frame
    % mapPointsIdx:   Indices of the map points observed in the current frame
    % featureIdx:     Indices of the corresponding feature points in the 
    %                 current frame
    [currPose, mapPointsIdx, featureIdx] = helperTrackLastKeyFrame(mapPointSet, ...
        vSetKeyFrames.Views, currFeatures, currPoints, lastKeyFrameId, ...
        intrinsics, scaleFactor);
    
    % Track the local map and check if the current frame is a key
    % frame，跟踪局部地图以获取更多的对应点
    numSkipFrames     = 15;
    numPointsKeyFrame = 120;
    [localKeyFrameIds, currPose, mapPointsIdx, featureIdx, isKeyFrame] = ...
        helperTrackLocalMap(mapPointSet, directionAndDepth, vSetKeyFrames, mapPointsIdx, ...
        featureIdx, currPose, currFeatures, currPoints, intrinsics, scaleFactor, numLevels, ...
        isLastFrameKeyFrame, lastKeyFrameIdx, currFrameIdx, numSkipFrames, numPointsKeyFrame);

    % Visualize matched features
    updatePlot(featurePlot, currI, currPoints(featureIdx));
    
    if ~isKeyFrame
        currFrameIdx = currFrameIdx + 1;
        isLastFrameKeyFrame = false;
        continue
    else
        isLastFrameKeyFrame = true;
    end
    
    % Update current key frame ID
    currKeyFrameId  = currKeyFrameId + 1;

    %% Local Mapping

    % Add the new key frame 
    [mapPointSet, vSetKeyFrames] = helperAddNewKeyFrame(mapPointSet, vSetKeyFrames, ...
        currPose, currFeatures, currPoints, mapPointsIdx, featureIdx, localKeyFrameIds);
    
    % Remove outlier map points that are observed in fewer than 3 key frames
    % mapPointsIdx更改后为从1开始的索引点序号
    [mapPointSet, directionAndDepth, mapPointsIdx] = helperCullRecentMapPoints( ...
        mapPointSet, directionAndDepth, mapPointsIdx, newPointIdx);
    
    % Create new map points by triangulation
    minNumMatches = 20;
    minParallax   = 0.35;
    [mapPointSet, vSetKeyFrames, newPointIdx] = helperCreateNewMapPoints( ...
        mapPointSet, vSetKeyFrames, currKeyFrameId, intrinsics, scaleFactor, ...
        minNumMatches, minParallax);
    
    % Update view direction and depth
    directionAndDepth = update(directionAndDepth, mapPointSet, ...
        vSetKeyFrames.Views, [mapPointsIdx; newPointIdx], true);
    
    % Local bundle adjustment
    [mapPointSet, directionAndDepth, vSetKeyFrames, newPointIdx] = ...
        helperLocalBundleAdjustment(mapPointSet, directionAndDepth, ...
        vSetKeyFrames, currKeyFrameId, intrinsics, newPointIdx);
    
    % Visualize 3D world points and camera trajectory
    updatePlot(mapPlot, vSetKeyFrames, mapPointSet);
    
    % Initialize the loop closure database
    % Update IDs and indices
    lastKeyFrameId  = currKeyFrameId;
    lastKeyFrameIdx = currFrameIdx;
    addedFramesIdx  = [addedFramesIdx; currFrameIdx]; %#ok<AGROW>
    currFrameIdx  = currFrameIdx + 1;
end % End of main loop
    optimizedPoses = vSetKeyFrames.Views;

    % Plot the camera ground truth trajectory
 scaledTrajectory = plotActualTrajectory(mapPlot, gTruth(addedFramesIdx), optimizedPoses);

% Show legend
showLegend(mapPlot);


%%
function [features, validPoints] = helperDetectAndExtractFeatures(Irgb, ...
    scaleFactor, numLevels, varargin)
%helperDetectAndExtractFeatures detect and extract features

numPoints   = 1500;

% In this example, the images are already undistorted. In a general
% workflow, uncomment the following code to undistort the images.
%
% if nargin > 3
%     intrinsics = varargin{1};
% end
% Irgb  = undistortImage(Irgb, intrinsics);

% Detect ORB features
Igray  = im2gray(Irgb);

points = detectORBFeatures(Igray, 'ScaleFactor', scaleFactor, 'NumLevels', numLevels);

% Select a subset of features, uniformly distributed throughout the image
points = selectUniform(points, numPoints, size(Igray, 1:2));

% Extract features
[features, validPoints] = extractFeatures(Igray, points);
end

%--------------------------------------------------------------------------
function [F, score, inliersIndex] = helperComputeFundamentalMatrix(matchedPoints1, matchedPoints2)
%helperComputeFundamentalMatrix compute fundamental matrix and evaluate reconstruction

[F, inliersLogicalIndex]   = estimateFundamentalMatrix( ...
    matchedPoints1, matchedPoints2, 'Method','RANSAC',...
    'NumTrials', 5e3, 'DistanceThreshold', 0.1);

inlierPoints1 = matchedPoints1(inliersLogicalIndex);
inlierPoints2 = matchedPoints2(inliersLogicalIndex);

inliersIndex  = find(inliersLogicalIndex);

locations1    = inlierPoints1.Location;
locations2    = inlierPoints2.Location;

% Distance from points to epipolar line
lineIn1   = epipolarLine(F', locations2);
error2in1 = (sum([locations1, ones(size(locations1, 1),1)].* lineIn1, 2)).^2 ...
    ./ sum(lineIn1(:,1:2).^2, 2);
lineIn2   = epipolarLine(F, locations1);
error1in2 = (sum([locations2, ones(size(locations2, 1),1)].* lineIn2, 2)).^2 ...
    ./ sum(lineIn2(:,1:2).^2, 2);

outlierThreshold = 4;

score = sum(max(outlierThreshold-error1in2, 0)) + ...
    sum(max(outlierThreshold-error2in1, 0));

end

%--------------------------------------------------------------------------
function [isValid, xyzPoints, inlierIdx] = helperTriangulateTwoFrames(...
    pose1, pose2, matchedPoints1, matchedPoints2, intrinsics, minParallax)
%helperTriangulateTwoFrames triangulate two frames to initialize the map

[R1, t1]   = cameraPoseToExtrinsics(pose1.Rotation, pose1.Translation);
camMatrix1 = cameraMatrix(intrinsics, R1, t1);

[R2, t2]   = cameraPoseToExtrinsics(pose2.Rotation, pose2.Translation);
camMatrix2 = cameraMatrix(intrinsics, R2, t2);

[xyzPoints, reprojectionErrors, isInFront] = triangulate(matchedPoints1, ...
    matchedPoints2, camMatrix1, camMatrix2);

% Filter points by view direction and reprojection error
minReprojError = 1;
inlierIdx  = isInFront & reprojectionErrors < minReprojError;
xyzPoints  = xyzPoints(inlierIdx ,:);

% A good two-view with significant parallax
ray1       = xyzPoints - pose1.Translation;
ray2       = xyzPoints - pose2.Translation;
cosAngle   = sum(ray1 .* ray2, 2) ./ (vecnorm(ray1, 2, 2) .* vecnorm(ray2, 2, 2));

% Check parallax
isValid = nnz(cosAngle < cosd(minParallax) & cosAngle>0) > 20;
end

%--------------------------------------------------------------------------
function [mapPointSet, directionAndDepth, mapPointsIdx] = helperCullRecentMapPoints(...
        mapPointSet, directionAndDepth, mapPointsIdx, newPointIdx)
%helperCullRecentMapPoints cull recently added map
%points；此函数功能应该是剔除非最近添加的map points之外的点，官方注释应当调整
outlierIdx    = setdiff(newPointIdx, mapPointsIdx);
if ~isempty(outlierIdx)
    mapPointSet   = removeWorldPoints(mapPointSet, outlierIdx);
    directionAndDepth = remove(directionAndDepth, outlierIdx);
    mapPointsIdx  = mapPointsIdx - arrayfun(@(x) nnz(x>outlierIdx), mapPointsIdx);% 2022.5.9不是很理解;2022.5.15,mapPointsIdx就是索引重新从1开始排序,本质就是序号平行移动
    % 可以替换以下代码
%     [~,idx] = sort(mapPointsIdx);
%     mapPointsIdx(idx)=1:length(mapPointsIdx);
end
end

%--------------------------------------------------------------------------
function [mapPointSet, directionAndDepth] = helperUpdateGlobalMap(...
    mapPointSet, directionAndDepth, vSetKeyFrames, vSetKeyFramesOptim, poseScales)
%helperUpdateGlobalMap update map points after pose graph optimization
posesOld     = vSetKeyFrames.Views.AbsolutePose;
posesNew     = vSetKeyFramesOptim.Views.AbsolutePose;
positionsOld = mapPointSet.WorldPoints;
positionsNew = positionsOld;
indices = 1:mapPointSet.Count;

% Update world location of each map point based on the new absolute pose of 
% the corresponding major view and the associated scales
for i = 1: mapPointSet.Count
    majorViewIds = directionAndDepth.MajorViewId(i);
    poseNew = posesNew(majorViewIds).T;
    poseNew(1:3, 1:3) = poseNew(1:3, 1:3)*poseScales(majorViewIds);
    tform = posesOld(majorViewIds).T \ poseNew;
    positionsNew(i, :) = positionsOld(i, :) * tform(1:3,1:3) + tform(4, 1:3);
end
mapPointSet = updateWorldPoints(mapPointSet, indices, positionsNew);
end

function gTruth = helperGetSensorGroundTruth(gTruthData)
gTruth = repmat(rigid3d, height(gTruthData), 1);
for i = 1:height(gTruthData) 
    currLocations = gTruthData{i,3:5};% 以车载摄像头位置为基准
    gTruth(i).Translation = currLocations;
    currEular = gTruthData{i,6:8};
    currEularDeg = rad2deg(currEular);
    normalRotationMat = rotx(currEularDeg(1))*roty(currEularDeg(2))*rotz(currEularDeg(3));
    postRotationMat = normalRotationMat';
    gTruth(i).Rotation = postRotationMat;
end
end

