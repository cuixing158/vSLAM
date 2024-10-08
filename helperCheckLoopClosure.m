function [isDetected, loopKeyFrameIds] = helperCheckLoopClosure(vSetKeyFrames, ...
    currKeyframeId, imageDatabase, currImg, loopEdgeNumMatches)
%helperCheckLoopClosure detect loop candidates key frames by retrieving
%   visually similar images from the feature database.
%
%   This is an example helper function that is subject to change or removal 
%   in future releases.

%   Copyright 2019-2021 The MathWorks, Inc.

% Retrieve all the visually similar key frames
[candidateViewIds, similarityscores] = retrieveImages(currImg, imageDatabase);

% Compute similarity between the current key frame and its strongly-connected 
% key frames. The minimum similarity score is used as a baseline to find 
% loop candidate key frames, which are visually similar to but not connected 
% to the current key frame
covisViews          = connectedViews(vSetKeyFrames, currKeyframeId);
covisViewsIds       = covisViews.ViewId;
strongCovisViews    = connectedViews(vSetKeyFrames, currKeyframeId, loopEdgeNumMatches);% 熟练掌握connectedView方法
strongCovisViewIds  = strongCovisViews.ViewId;

% Retrieve the top 10 similar connected key frames
[~,~,scores] = evaluateImageRetrieval(currImg, imageDatabase, strongCovisViewIds, 'NumResults', 10);
minScore     = min(scores);

% loopKeyFrameIds与candidateScores数组大小一致，一一对应，candidateScores按照分数从大到小排序，loopKeyFrameIds乱序
[loopKeyFrameIds,ia] = setdiff(candidateViewIds, covisViewsIds, 'stable');

% Scores of non-connected key frames
candidateScores  = similarityscores(ia); % Descending

if ~isempty(ia)
    bestScore       = candidateScores(1);
    
    % Score must be higher than the 75% of the best score 
    isValid         = candidateScores > max(bestScore*0.75, minScore);
    
    loopKeyFrameIds = loopKeyFrameIds(isValid);
else
    loopKeyFrameIds = [];
end

% Loop candidates need to be consecutively detected
minNumCandidates = 3; % At least 3 candidates are found
if size(loopKeyFrameIds,1) >= minNumCandidates
    groups = nchoosek(loopKeyFrameIds, minNumCandidates);
    consecutiveGroups = groups(max(groups,[],2) - min(groups,[],2) <= 4, :);% 很妙的代码，乱序的groups序号获得是否连续不大于4帧的序号
    if ~isempty(consecutiveGroups) % Consecutive candidates are found
        loopKeyFrameIds = consecutiveGroups(1,:);
        isDetected = true;
    else
        isDetected = false;
    end
else
    isDetected = false;
end
end