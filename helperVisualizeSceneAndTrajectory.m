classdef helperVisualizeSceneAndTrajectory < handle
% 此类修改为只提供2个外部接口，一个初始化，另一个更新显示地图

    properties
        XLim = [0,1] 
        
        YLim = [0,1]
        
        ZLim = [0,1]

        transformT % 初始关键帧到世界坐标系的转换

        Axes
    end
    
    properties (Access = private)
        MapPointsPlot
 
        EstimatedTrajectory
        
        OptimizedTrajectory
        
        ActualTrajectory

        CameraPlot
    end
    
    methods (Access = public)
        function obj = helperVisualizeSceneAndTrajectory(vSetKeyFrames, mapPoints,cumGTruth)
            arguments
                vSetKeyFrames imageviewset 
                mapPoints worldpointset
                cumGTruth (:,1) rigid3d = rigid3d()
            end
        
            [xyzPoints, camCurrPose, trajectory]  = retrievePlottedData(obj, vSetKeyFrames, mapPoints);
             
            obj.MapPointsPlot = pcplayer(obj.XLim, obj.YLim, obj.ZLim,...
                'VerticalAxis', 'y', 'VerticalAxisDir', 'down', 'MarkerSize', 5);
            if length(cumGTruth)>1
                obj.MapPointsPlot = pcplayer(obj.XLim, obj.YLim, obj.ZLim);
            end
            
            
            obj.Axes  = obj.MapPointsPlot.Axes;
            hold(obj.Axes,'on');% 防止背景颜色改变

            color = xyzPoints(:, 2);
            color = -min(0.1, max(-0.4, color));
            estiTrajectory = trajectory;
      
            % 尺度和坐标变换，转换到真值下
            obj.transformT = rigid3d();
            if length(cumGTruth)>1
                assert(length(cumGTruth)==vSetKeyFrames.NumViews)
                actualTrans = vertcat(cumGTruth.Translation);
                cumActualDist = cumsum(vecnorm(diff(actualTrans),2,2));
                cumEstimateDist = cumsum(vecnorm(diff(estiTrajectory),2,2));
                scale = median(cumActualDist./cumEstimateDist);
                xyzPoints = xyzPoints.*scale;
                camCurrPose.AbsolutePose = rigid3d(camCurrPose.AbsolutePose.Rotation,...
                    camCurrPose.AbsolutePose.Translation.*scale);
                estiTrajectory = estiTrajectory.*scale;
              % 转换变换
                 srcPose = rigid3d(eye(3),estiTrajectory(1,:));% 初始关键帧的第一个姿态
                 dstPose = cumGTruth(1); %如何没有输入cumGTruth,则默认就是rigid3d()，即与srcPose一样
%                  dstPose.Rotation = (eul2rotm([0,0,-pi],'XYZ')*roty(90)*rotz(-90))';
                 %                  initGTpose = plotCamera('AbsolutePose',dstPose, 'Parent', obj.Axes, 'Size', 0.2);
                 obj.transformT = rigid3d(srcPose.T*dstPose.T);% 摄像机物理坐标转换为世界坐标的变换

                 obj.ActualTrajectory = plot3(obj.Axes,actualTrans(:,1),actualTrans(:,2),...
                     actualTrans(:,3),'g','LineWidth',2,'DisplayName','Actual trajectory');
            end

            estiTrajectory = transformPointsForward(obj.transformT,estiTrajectory);
            camCurrPose.AbsolutePose = rigid3d(camCurrPose.AbsolutePose.T*obj.transformT.T);
            xyzPoints = transformPointsForward(obj.transformT,xyzPoints);
            
            % Plot camera trajectory
            obj.EstimatedTrajectory = plot3(obj.Axes, estiTrajectory(:,1), estiTrajectory(:,2), ...
                estiTrajectory(:,3), 'r', 'LineWidth', 2 , 'DisplayName', 'Estimated trajectory');
            
            obj.MapPointsPlot.view(xyzPoints, color); 
            h = findobj(obj.Axes,'type','Scatter');
            h.DisplayName = 'map points';
            % Plot the current cameras
            obj.CameraPlot = plotCamera(camCurrPose, 'Parent', obj.Axes, 'Size', 0.1);
        end
        
        function updatePlot(obj, vSetKeyFrames, mapPoints,cumGTruth)
            arguments
                obj
                vSetKeyFrames imageviewset 
                mapPoints worldpointset
                cumGTruth (:,1) rigid3d = rigid3d() % 长度应当与vSetKeyFrames的imageID一致
            end
            
            [xyzPoints, currPose, trajectory]  = retrievePlottedData(obj, vSetKeyFrames, mapPoints);
            
            % Update the point cloud
            color = xyzPoints(:, 2);
            color = -min(0.1, max(-0.4, color));
            estiTrajectory = trajectory;

            % Update the camera trajectory
             % 尺度和坐标变换，转换到真值下
             if length(cumGTruth)>1
                 assert(length(cumGTruth)==vSetKeyFrames.NumViews)
                 actualTrans = vertcat(cumGTruth.Translation);
                 cumActualDist = cumsum(vecnorm(diff(actualTrans),2,2));
                 cumEstimateDist = cumsum(vecnorm(diff(estiTrajectory),2,2));
                 scale = median(cumActualDist./cumEstimateDist);
                 fprintf('current scale:%.2f\n',scale);
                 xyzPoints = xyzPoints.*scale;
                 currPose.AbsolutePose = rigid3d(currPose.AbsolutePose.Rotation,...
                     currPose.AbsolutePose.Translation.*scale);
                 estiTrajectory = estiTrajectory.*scale;
                 set(obj.ActualTrajectory,'XData',actualTrans(:,1),'YData',actualTrans(:,2),...
                      'ZData',actualTrans(:,3))
%                  obj.transformT.Rotation = cumGTruth(end).Rotation;
              end
            % 待加入对齐操作
            xyzPoints = transformPointsForward(obj.transformT,xyzPoints);
            currPose.AbsolutePose = rigid3d(currPose.AbsolutePose.T*obj.transformT.T);
            estiTrajectory = transformPointsForward(obj.transformT,estiTrajectory);
            
            % 更新轨迹
            obj.MapPointsPlot.view(xyzPoints, color);
            set(obj.EstimatedTrajectory, 'XData', estiTrajectory(:,1), 'YData', ...
                estiTrajectory(:,2), 'ZData', estiTrajectory(:,3));
           
            % Update the current camera pose since the first camera is fixed
            obj.CameraPlot.AbsolutePose = currPose.AbsolutePose;
            obj.CameraPlot.Label        = num2str(currPose.ViewId);
            if mod(size(estiTrajectory,1),10)==1
                camSize = 0.1;
                if length(cumGTruth)>1
                    camSize = 1;
                end
                plotCamera('AbsolutePose',currPose.AbsolutePose,...
                    'Parent', obj.Axes, 'Size', camSize,...
                    'Color',[0.2,0.6,0.3],'Opacity',0.5,'AxesVisible',true,...
                    'Label',num2str(currPose.ViewId));
            end
            xmin = min(estiTrajectory(:,1));
            ymin = min(estiTrajectory(:,2));
            zmin = min(estiTrajectory(:,3));
            xmax = max(estiTrajectory(:,1));
            ymax = max(estiTrajectory(:,2));
            zmax = max(estiTrajectory(:,3));
            Xoffset = (xmax-xmin)*0.3;
            Yoffset = (ymax-ymin)*0.3;
            Zoffset = (zmax-zmin)*0.3;
            set(obj.Axes,'XLim',[xmin-Xoffset,xmax+Xoffset]);
            set(obj.Axes,'YLim',[ymin-Yoffset,ymax+Yoffset]);
            set(obj.Axes,'ZLim',[zmin-Zoffset,zmax+Zoffset]);
%             exportgraphics(obj.Axes,"vSLAM.gif","Append",true,"BackgroundColor","current")
        end
        
        function showLegend(obj)
            % Add a legend to the axes
            hLegend = legend(obj.Axes, 'Location',  'northwest', ...
                'TextColor', [1 1 1], 'FontWeight', 'bold');
        end
    end
    
    methods (Access = private)
        
        function [xyzPoints, currPose, trajectory]  = retrievePlottedData(obj, vSetKeyFrames, mapPoints)
            camPoses    = poses(vSetKeyFrames);
            currPose    = camPoses(end,:); % Contains both ViewId and Pose

            % Ensure the rotation matrix is a rigid transformation
            R = double(currPose.AbsolutePose.Rotation);
            t = double(currPose.AbsolutePose.Translation);
            [U, ~, V] = svd(R);
            currPose.AbsolutePose.T = eye(4);
            currPose.AbsolutePose.T(4, 1:3) = t;
            currPose.AbsolutePose.T(1:3, 1:3) = U * V';

            trajectory  = vertcat(camPoses.AbsolutePose.Translation);
            xyzPoints   = mapPoints.WorldPoints;%(mapPoints.UserData.Validity,:);
            
        end
    end
end