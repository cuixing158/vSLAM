%% 特征点匹配
imgsDir = "./errcalibimg";
img1 = imread(fullfile(imgsDir,"m_stitchingimage_0.jpg"));
img2 = imread(fullfile(imgsDir,"m_stitchingimage_19.jpg"));

%% 参考主题 Find Point Tracks Across Sequence of Images
imageDir = fullfile(imgsDir);
imds = imageDatastore(imageDir);
I = im2gray(readimage(imds,1));
pointsPrev = detectSURFFeatures(I);
[featuresPrev,pointsPrev] = extractFeatures(I,pointsPrev);
% Create an image view set and add one view to the set.
vSet = imageviewset;
vSet = addView(vSet,1,'Features',featuresPrev,'Points',pointsPrev);
% Compute features and matches for the rest of the images.
img1 = I;
points1 = pointsPrev;
for i = 2:numel(imds.Files)
    img2 = readimage(imds, i);
    I = im2gray(img2);
    points = detectSURFFeatures(I);
    [features, points] = extractFeatures(I,points);
    vSet = addView(vSet,i,'Features',features,'Points',points);
    pairsIdx = matchFeatures(featuresPrev,features);

    points2 = points;
    matchedPoints1 = points1(pairsIdx(:,1),:);
    matchedPoints2 = points2(pairsIdx(:,2),:);
    showMatchedFeatures(readimage(imds,i-1),readimage(imds,i),matchedPoints1,...
        matchedPoints2,"montage")

    vSet = addConnection(vSet,i-1,i,'Matches',pairsIdx);
    featuresPrev = features;
    points1 = points2;
end
% Find point tracks across views in the image view set.
tracks = findTracks(vSet);

