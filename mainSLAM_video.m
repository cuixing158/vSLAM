%% 使用王家才的地下停车场视频数据集做vSLAM
videoPath = "your_video_folder";
fs = fileDatastore(videoPath,"ReadFcn",@VideoReader,...
    ReadMode="file",FileExtensions=".mp4");

%% Camera intrinsics
load fisheyeCamParas.mat % come from "undistortFishEye.m"
intrinsics = cameraIntrinsics(K(1,1),[K(1,3),K(2,3)],size(mapX));

%% 主模块
% Set random seed for reproducibility
rng(0);

%% Map Initialization
currFrameIdx = 1;
currI = readUndistortFisheyeImg(fs,currFrameIdx,mapX,mapY);

% Detect and extract ORB features
scaleFactor  = 1.2;
numLevels    = 8;
[preFeatures, prePoints] = helperDetectAndExtractFeatures(currI, scaleFactor, ...
    numLevels); 

currFrameIdx = currFrameIdx + 1;
firstI       = currI; % Preserve the frame 

isMapInitialized  = false;

% Map initialization loop
while ~isMapInitialized && hasdata(fs)
    currI = readUndistortFisheyeImg(fs,currFrameIdx,mapX,mapY);

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
%     hfeature = figure;
%     showMatchedFeatures(firstI, currI, prePoints(indexPairs(:,1)), ...
%         currPoints(indexPairs(:, 2)), 'montage', 'Parent', gca(hfeature));
else
    error('Unable to initialize the map.')
end

%% Store Initial Key Frames and Map Points
% 到这里只有第一个姿态和最后的姿态地面特征点和车引擎盖特征点匹配，UE的图应当间隔较小再获取图像才对

% Create an empty imageviewset object to store key frames
vSetKeyFrames = imageviewset;

% Create an empty worldpointset object to store 3-D map points
mapPointSet   = worldpointset;

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

%% Refine and Visualize the Initial Reconstruction,初始关键帧的3d map points归一化

[vSetKeyFrames, mapPointSet] = helperGlobalBundleAdjustment(...
    vSetKeyFrames, mapPointSet, intrinsics, relPose);

% Visualize matched features in the current frame
featurePlot   = helperVisualizeMatchedFeatures(currI, currPoints(indexPairs(:,2)));

% Visualize initial map points and camera trajectory
keyFrameIds = [1;currFrameIdx-1];
% currGTruths = gTruth(keyFrameIds);
mapPlot       = visualizeSceneAndTrajectory(vSetKeyFrames, mapPointSet);

% Show legend
showLegend(mapPlot);

% %% Initialize Place Recognition Database
% 
% % Load the bag of features data created offline
% placeRecDatabase = fullfile(parkingLotRoot,"bagOfFeaturesDataSLAM.mat");
% if isfile(placeRecDatabase)
%   load(placeRecDatabase);
% else
%     bofData = bagOfFeatures(imds,CustomExtractor=@helperORBFeatureExtractorFunction,...
%     TreeProperties=[5,10]);
%     save(placeRecDatabase,'bofData');
% end
% 
% % Initialize the place recognition database
% loopDatabase    = invertedImageIndex(bofData, "SaveFeatureLocations", false);
% 
% % Add features of the first two key frames to the database
% addImageFeatures(loopDatabase, preFeatures, preViewId);
% addImageFeatures(loopDatabase, currFeatures, currViewId);

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

isLoopClosed      = false;

% Main loop
isLastFrameKeyFrame = true;
while ~isLoopClosed && hasdata(fs)
    currI = readUndistortFisheyeImg(fs,currFrameIdx,mapX,mapY);

    [currFeatures, currPoints] = helperDetectAndExtractFeatures(currI, ...
        scaleFactor, numLevels);

    % Track the last key frame,这里只跟踪了上一个关键帧获得的对应点，多次匹配多次BA优化确定当前视图姿态
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
        helperTrackLocalMap(mapPointSet, vSetKeyFrames, mapPointsIdx, ...
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
    mapPointSet = helperCullRecentMapPoints(mapPointSet, mapPointsIdx, newPointIdx);
    
    % Create new map points by triangulation
    minNumMatches = 20;
    minParallax   = 0.35;
    [mapPointSet, vSetKeyFrames, newPointIdx] = helperCreateNewMapPoints( ...
        mapPointSet, vSetKeyFrames, currKeyFrameId, intrinsics, scaleFactor, ...
        minNumMatches, minParallax);
    
    % Local bundle adjustment
    [refinedViews, dist] = connectedViews(vSetKeyFrames, currKeyFrameId, MaxDistance=2);
    refinedKeyFrameIds = refinedViews.ViewId;
    fixedViewIds = refinedKeyFrameIds(dist==2);
    fixedViewIds = fixedViewIds(1:min(10, numel(fixedViewIds)));

    % Refine local key frames and map points
    [mapPointSet, vSetKeyFrames, mapPointIdx] = bundleAdjustment(...
        mapPointSet, vSetKeyFrames, [refinedKeyFrameIds; currKeyFrameId], intrinsics, ...
        FixedViewIDs=fixedViewIds, PointsUndistorted=true, AbsoluteTolerance=1e-7,...
        RelativeTolerance=1e-16, Solver="preconditioned-conjugate-gradient", ...
        MaxIteration=10);

    % Update view direction and depth
    mapPointSet = updateLimitsAndDirection(mapPointSet, mapPointIdx, vSetKeyFrames.Views);
    
    % Update representative view
    mapPointSet = updateRepresentativeView(mapPointSet, mapPointIdx, vSetKeyFrames.Views);

    
    % Visualize 3D world points and camera trajectory
%     keyFrameIds  = [keyFrameIds; currFrameIdx]; %#ok<AGROW>
%     currGTruths = gTruth(keyFrameIds);
    t1 = tic;
    updatePlot(mapPlot, vSetKeyFrames, mapPointSet);
    t2 = toc(t1);
    fprintf('currKeyFrameId:%03d,updatePlot take time:%.2f s.\n',...
        currFrameIdx,t2);

%     %% Loop Closure
%     % Initialize the loop closure database
% 
%     % Check loop closure after some key frames have been created    
%     if currKeyFrameId > 100
%         
%         % Detect possible loop closure key frame candidates
%         loopEdgeNumMatches = 40;
%         [isDetected, validLoopCandidates] = helperCheckLoopClosure(vSetKeyFrames, currKeyFrameId, ...
%             loopDatabase, currI, loopEdgeNumMatches);
%         
%         if isDetected 
%             % Add loop closure connections
%             isStereo = false;
%             [mapPointSet, vSetKeyFrames, loopClosureEdge] = helperAddLoopConnections(...
%                 mapPointSet, vSetKeyFrames, validLoopCandidates, ...
%                 currKeyFrameId, currFeatures, currPoints, loopEdgeNumMatches, isStereo);
%             isLoopClosed = ~isempty(loopClosureEdge);
%         end
%     end
    
%     % If no loop closure is detected, add current features into the database
%     if ~isLoopClosed
%         addImageFeatures(loopDatabase,  currFeatures, currKeyFrameId);
%     end

    %% update index

    % Initialize the loop closure database
    % Update IDs and indices
    lastKeyFrameId  = currKeyFrameId;
    lastKeyFrameIdx = currFrameIdx;
    currFrameIdx  = currFrameIdx + 1;
end % End of main loop

if ~isLoopClosed
    disp('Loop closure cannot be found');
    optimizedPoses = [];
    return
end

% Create a similarity pose graph from the key frames set
G = createPoseGraph(vSetKeyFrames);

% Remove weak edges and keep loop closure edges
EG = rmedge(G, find(G.Edges.Weight < minNumMatches & ...
    ~ismember(G.Edges.EndNodes, loopClosureEdge, 'rows')));

% Optimize the similarity pose graph
optimG = optimizePoseGraph(EG, 'g2o-levenberg-marquardt');
optimizedPoses = optimG.Nodes(:, 1:2);

% Update the view poses
vSetKeyFramesOptim = updateView(vSetKeyFrames, optimizedPoses);

% Update map points after optimizing the poses
mapPointSet = helperUpdateGlobalMap(mapPointSet, ...
    vSetKeyFrames, vSetKeyFramesOptim, optimG.Nodes.Scale);

keyFrameIds  = [keyFrameIds; currFrameIdx]; %#ok<AGROW>
currGTruths = gTruth(keyFrameIds);
updatePlot(mapPlot, vSetKeyFrames, mapPointSet,currGTruths);

% Plot the optimized camera trajectory
plotOptimizedTrajectory(mapPlot, optimizedPoses)

% Show legend
showLegend(mapPlot);


%% 一些支持子函数
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
function mapPointSet= helperCullRecentMapPoints(mapPointSet, mapPointsIdx, newPointIdx)
%helperCullRecentMapPoints cull recently added map points
%points；此函数功能应该是剔除非最近添加的map points之外的点，官方注释应当调整
outlierIdx    = setdiff(newPointIdx, mapPointsIdx);
if ~isempty(outlierIdx)
    mapPointSet   = removeWorldPoints(mapPointSet, outlierIdx);
end
end

%--------------------------------------------------------------------------
function mapPointSet = helperUpdateGlobalMap(...
    mapPointSet, vSetKeyFrames, vSetKeyFramesOptim, poseScales)
%helperUpdateGlobalMap update map points after pose graph optimization
posesOld     = vSetKeyFrames.Views.AbsolutePose;
posesNew     = vSetKeyFramesOptim.Views.AbsolutePose;
positionsOld = mapPointSet.WorldPoints;
positionsNew = positionsOld;
indices     = 1:mapPointSet.Count;

% Update world location of each map point based on the new absolute pose of 
% the corresponding major view
for i = indices
    majorViewIds = mapPointSet.RepresentativeViewId(i);
    poseNew = posesNew(majorViewIds).A;
    poseNew(1:3, 1:3) = poseNew(1:3, 1:3) * poseScales(majorViewIds);
    tform = affinetform3d(poseNew/posesOld(majorViewIds).A);
    positionsNew(i, :) = transformPointsForward(tform, positionsOld(i, :));
end
mapPointSet = updateWorldPoints(mapPointSet, indices, positionsNew);
end

function gTruth = helperGetSensorGroundTruth(gTruthData)
gTruth = repmat(rigidtform3d(), height(gTruthData), 1);
for i = 1:height(gTruthData) 
    currLocations = gTruthData{i,3:5};% 以车载摄像头位置为基准
    gTruth(i).Translation = currLocations;
    currEular = gTruthData{i,6:8}; % 曾总角度顺序依次XYZ 
    eularDegree = double(rad2deg(currEular));
    postRotationMat  = rotz(eularDegree(3))*roty(eularDegree(2))*rotx(eularDegree(1))*roty(90)*rotz(-90);% 相机默认开始朝向为Z轴正向
    gTruth(i).R = postRotationMat;
end
end

function frame = readUndistortFisheyeImg(fileDs,idx,mapX,mapY)
% 从多个畸变的视频流fileDs中读取无畸变的灰度图像，fileDs中视频数据集连续排列
persistent vi currIdx
if isempty(vi) % idx must less than numbers of the fist video frames
    currIdx = 1;
    vi = read(fileDs);
else
    if ~hasFrame(vi)
        currIdx = idx;
        vi = read(fileDs);
    end
end
currI = im2gray(im2double(read(vi,idx-currIdx+1)));
frame = interp2(currI,mapX,mapY,"linear",0);% 以2维插值代替像素索引映射操作
end
