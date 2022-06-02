%% 测试fundamental matrix估计

load stereoPointPairs
[fLMedS,inliers] = estimateFundamentalMatrix(matchedPoints1,matchedPoints2,...
    NumTrials=2000,Method="MSAC");
fLMedS

% I1 = imread('viprectification_deskLeft.png');
% I2 = imread('viprectification_deskRight.png');
% figure;
% showMatchedFeatures(I1,I2,matchedPoints1,matchedPoints2,'montage','PlotOptions',{'ro','go','y--'});
% title('Putative Point Matches');
% figure;
% showMatchedFeatures(I1,I2,matchedPoints1(inliers,:),matchedPoints2(inliers,:),'montage','PlotOptions',{'ro','go','y--'});
% title('Point Matches After Outliers Are Removed');

x1 = [matchedPoints1,ones(size(matchedPoints1,1),1)]';
x2 = [matchedPoints2,ones(size(matchedPoints2,1),1)]';
F = det_F_normalized_8point(x1,x2)
I1 = imread('viprectification_deskLeft.png');
I2 = imread('viprectification_deskRight.png');
figure;
showMatchedFeatures(I1,I2,matchedPoints1,matchedPoints2,...
    'montage','PlotOptions',{'ro','go','y--'});

