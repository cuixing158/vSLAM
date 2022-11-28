classdef visualizeSceneAndTrajectory < handle
% 此类修改为只提供2个外部接口，一个初始化，另一个更新显示地图

    properties
        XLim = [0,1] 
        
        YLim = [0,1]
        
        ZLim = [0,1]

        transformT % 初始关键帧到世界坐标系的转换

        Scale % 尺度

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
        function obj = visualizeSceneAndTrajectory(vSetKeyFrames, mapPoints,cumGTruth)
            arguments
                vSetKeyFrames imageviewset 
                mapPoints worldpointset
                cumGTruth (:,1) rigidtform3d = rigidtform3d()
            end
        
            [xyzPoints, camCurrPose, trajectory]  = retrievePlottedData(obj, vSetKeyFrames, mapPoints);
            if length(cumGTruth)>1
                obj.MapPointsPlot = pcplayer(obj.XLim, obj.YLim, obj.ZLim);
            else
                obj.MapPointsPlot = pcplayer(obj.XLim, obj.YLim, obj.ZLim,...
                    'VerticalAxis', 'y', 'VerticalAxisDir', 'down', 'MarkerSize', 5);
            end

            obj.Axes  = obj.MapPointsPlot.Axes;
            obj.Axes.XLabel.String = "X(m)";
            obj.Axes.YLabel.String = "Y(m)";
            obj.Axes.ZLabel.String = "Z(m)";
            hold(obj.Axes,'on');% 防止背景颜色改变

            color = xyzPoints(:, 2);
            color = -min(0.1, max(-0.4, color));
            estiTrajectory = trajectory;
      
            % 尺度和坐标变换，转换到真值下
            obj.transformT = rigidtform3d();
            if length(cumGTruth)>1
                assert(length(cumGTruth)==vSetKeyFrames.NumViews)
                actualTrans = vertcat(cumGTruth.Translation);
                cumActualDist = cumsum(vecnorm(diff(actualTrans),2,2));
                cumEstimateDist = cumsum(vecnorm(diff(estiTrajectory),2,2));
                obj.Scale = median(cumActualDist./cumEstimateDist);
                scale = obj.Scale;
                xyzPoints = xyzPoints.*scale;
                camCurrPose.AbsolutePose = rigidtform3d(camCurrPose.AbsolutePose.R,...
                    camCurrPose.AbsolutePose.Translation.*scale);
                estiTrajectory = estiTrajectory.*scale;
              % 转换变换
                 srcPose = rigidtform3d(eye(3),estiTrajectory(1,:));% 初始关键帧的第一个姿态
                 dstPose = cumGTruth(1); %如果没有输入cumGTruth,则默认就是rigidtform3d()，即与srcPose一样
                 obj.transformT = rigidtform3d(dstPose.A*srcPose.A);% 摄像机物理坐标转换为世界坐标的变换

                 obj.ActualTrajectory = plot3(obj.Axes,actualTrans(:,1),actualTrans(:,2),...
                     actualTrans(:,3),'g','LineWidth',2,'DisplayName','Actual trajectory');
            end

            estiTrajectory = transformPointsForward(obj.transformT,estiTrajectory);
            camCurrPose.AbsolutePose = rigidtform3d(obj.transformT.A*camCurrPose.AbsolutePose.A);
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
                cumGTruth (:,1) rigidtform3d = rigidtform3d() % 长度应当与vSetKeyFrames的imageID一致
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
                 obj.Scale = median(cumActualDist./cumEstimateDist);
                 scale = obj.Scale;
                 fprintf('current scale:%.2f\n',scale);
                 xyzPoints = xyzPoints.*scale;
                 currPose.AbsolutePose = rigidtform3d(currPose.AbsolutePose.R,...
                     currPose.AbsolutePose.Translation.*scale);
                 estiTrajectory = estiTrajectory.*scale;
                 % 1、绘制实际轨迹
                 set(obj.ActualTrajectory,'XData',actualTrans(:,1),'YData',actualTrans(:,2),...
                      'ZData',actualTrans(:,3))
%                  obj.transformT.Rotation = cumGTruth(end).Rotation;
              end
            % 加入对齐操作
            xyzPoints = transformPointsForward(obj.transformT,xyzPoints);
            currPose.AbsolutePose = rigidtform3d(obj.transformT.A*currPose.AbsolutePose.A);
            estiTrajectory = transformPointsForward(obj.transformT,estiTrajectory);
            
            % 2、更新估计特征点云
            obj.MapPointsPlot.view(xyzPoints, color);

            % 3、更新估计轨迹
            set(obj.EstimatedTrajectory, 'XData', estiTrajectory(:,1), 'YData', ...
                estiTrajectory(:,2), 'ZData', estiTrajectory(:,3));
           
            % 4、周期性绘制相机姿态
            obj.CameraPlot.AbsolutePose = currPose.AbsolutePose;
            obj.CameraPlot.Label        = num2str(currPose.ViewId);
            if mod(size(estiTrajectory,1),100)==1
                camSize = 0.1;
                if length(cumGTruth)>1
                    camSize = 1;
                end
                plotCamera(AbsolutePose=currPose.AbsolutePose,...
                    Parent=obj.Axes, Size=camSize,...
                    Color=[0.2,0.6,0.7],Opacity=0.9,AxesVisible=true,...
                    Label=num2str(currPose.ViewId));
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
        
        function plotOptimizedTrajectory(obj, poses)
            
            % Delete the camera plot
            delete(obj.CameraPlot);
            
            % Plot the optimized trajectory
            trans = vertcat(poses.AbsolutePose.Translation);
            trans = transformPointsForward(obj.transformT,obj.Scale*trans);

            obj.OptimizedTrajectory = plot3(obj.Axes, trans(:, 1), trans(:, 2), trans(:, 3), 'm', ...
                'LineWidth', 2, 'DisplayName', 'Optimized trajectory');
        end
        
        function showLegend(obj)
            % Add a legend to the axes
            hLegend = legend(obj.Axes, 'Location',  'northwest', ...
                'TextColor', [1 1 1], 'FontWeight', 'bold');
            set(obj.Axes,'ZLim',[0,6]);
        end
    end
    
    methods (Access = private)
        
        function [xyzPoints, currPose, trajectory]  = retrievePlottedData(obj, vSetKeyFrames, mapPoints)
            camPoses    = poses(vSetKeyFrames);
            currPose    = camPoses(end,:); % Contains both ViewId and Pose

            % Ensure the rotation matrix is a rigid transformation
            R = double(currPose.AbsolutePose.R);
            t = double(currPose.AbsolutePose.Translation);
            [U, ~, V] = svd(R);
            currPose.AbsolutePose.A = eye(4);
            currPose.AbsolutePose.A(1:3, 4) = t;
            currPose.AbsolutePose.A(1:3, 1:3) = U * V';

            trajectory  = vertcat(camPoses.AbsolutePose.Translation);
            xyzPoints   = mapPoints.WorldPoints;%(mapPoints.UserData.Validity,:);
            
        end
    end
end