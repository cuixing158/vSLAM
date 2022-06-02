classdef helperVisualizeMatchedFeatures < handle
%helperVisualizeMatchedFeatures show the matched features in a frame
%
%   This is an example helper class that is subject to change or removal 
%   in future releases.

%   Copyright 2019-2020 The MathWorks, Inc.

    properties (Access = private)
        Image
        
        Feature
    end
    
    methods (Access = public)
        
        function obj = helperVisualizeMatchedFeatures(I, featurePoints)
            locations= featurePoints.Location;
            
            % Plot image
            hFig  = figure;
            hAxes = newplot(hFig); 
            
            % Set figure visibility and position
            hFig.Visible = 'on';
            hFig.Position(1:2) = [200, 1000];
            
            % Show the image
            obj.Image = imshow(I, 'Parent', hAxes, 'Border', 'tight', ...
                'InitialMagnification', 70);
            title(hAxes, 'Matched Features in Current Frame');
            hold(hAxes, 'on');
            
            % Plot features
            plot(featurePoints, hAxes, 'ShowOrientation',false, ...
                'ShowScale',false);
            obj.Feature = findobj(hAxes.Parent,'Type','Line'); 
        end 
        
        function updatePlot(obj, I, featurePoints)
            locations = featurePoints.Location;
            obj.Image.CData   = I;
            obj.Feature.XData = locations(:,1);
            obj.Feature.YData = locations(:,2);
%             drawnow limitrate
        end
    end
end



