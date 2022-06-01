classdef helperVisualizeSceneAndTrajectory < handle
% 此类修改为只提供2个外部接口，一个初始化，另一个更新显示地图

    properties
        XLim = [-25,30] % 根据定义的世界坐标系范围估算
        
        YLim = [-45,15]
        
        ZLim = [0,10]

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
        function obj = helperVisualizeSceneAndTrajectory(vSetKeyFrames, mapPoints,cumGTruth,initPose)
            arguments
                vSetKeyFrames imageviewset 
                mapPoints worldpointset
                cumGTruth (:,1) rigid3d = rigid3d()
                initPose (1,1) rigid3d = rigid3d() % 
            end
        
            [xyzPoints, camCurrPose, trajectory]  = retrievePlottedData(obj, vSetKeyFrames, mapPoints);
             
            obj.MapPointsPlot = pcplayer(obj.XLim, obj.YLim, obj.ZLim);
%                 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', 'MarkerSize', 5);
            
            obj.Axes  = obj.MapPointsPlot.Axes;
            hold(obj.Axes,'on');% 防止背景颜色改变
            
            % Set figure location
            obj.Axes.Parent.Position = [100 100 800 350];
            
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
%                  dstPose = rigid3d(roty(90),cumGTruth(1).Translation);
                 keyFramePose = cumGTruth(1);
                 firstInitPose = rigid3d()
                 dstPose = cumGTruth(1); %如何没有输入cumGTruth,则默认就是rigid3d()，即与srcPose一样
                 dstPose.Rotation = (eul2rotm([0,0,-pi],'XYZ')*roty(90)*rotz(-90))';
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
%             obj.Axes.Children.DisplayName = 'Map points';
            % Plot the current cameras
            obj.CameraPlot = plotCamera(camCurrPose, 'Parent', obj.Axes, 'Size', 0.2);
%             view(obj.Axes, [0 -1 0]);
%             camroll(obj.Axes, 90);
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
                plotCamera('AbsolutePose',currPose.AbsolutePose,...
                    'Parent', obj.Axes, 'Size', 1,...
                    'Color',[0.2,0.6,0.3],'Opacity',0.5,'AxesVisible',true,...
                    'Label',num2str(currPose.ViewId));
            end
            offset = 5;
            set(obj.Axes,'XLim',[min(estiTrajectory(:,1))-offset,max(estiTrajectory(:,1))+offset]);
            set(obj.Axes,'YLim',[min(estiTrajectory(:,2))-offset,max(estiTrajectory(:,2))+offset]);
            set(obj.Axes,'ZLim',[min(estiTrajectory(:,3))-offset,max(estiTrajectory(:,3))+offset]);
            exportgraphics(obj.Axes,"vSLAM.gif","Append",true,"BackgroundColor","current")
            %            drawnow limitrate
        end
        
        function plotOptimizedTrajectory(obj, poses)
            % poses是传入的优化后的姿态，这个函数应当摒弃掉，因为我们没有回环检测优化姿态，但可以设想在终点检索图像进行优化
            % Delete the camera plot
            delete(obj.CameraPlot);
            
            % Plot the optimized trajectory
            trans = vertcat(poses.AbsolutePose.Translation);
            obj.OptimizedTrajectory = plot3(obj.Axes, trans(:, 1), trans(:, 2), trans(:, 3), 'm', ...
                'LineWidth', 2, 'DisplayName', 'Optimized trajectory');
        end
        
        function scaledLocations = plotActualTrajectory(obj, gTruth, optimizedPoses)
            % scaledLocatiions为优化后的位置，应当摒弃掉
            estimatedCameraLoc = vertcat(optimizedPoses.AbsolutePose.Translation);
            actualCameraLoc    = vertcat(gTruth.Translation);
            scale = median(vecnorm(actualCameraLoc(2:5:end,:) - actualCameraLoc(1,:), 2, 2)./ ...
                vecnorm(estimatedCameraLoc(2:5:end,:) - estimatedCameraLoc(1,:), 2, 2));
            
            % Update the plot based on the ground truth
            updatePlotScale(obj, scale);
            
            % Transform to the global coordinate system,初始原点对齐，尺度上步已经完成
            gTruth(1) = rigid3d(eye(3),gTruth(1).Translation);% 真值不考虑旋转
            scaledLocations = transformCoodinates(obj, gTruth(1));
            
            % Plot the ground truth
            plot3(obj.Axes, actualCameraLoc(:,1), actualCameraLoc(:,2), actualCameraLoc(:,3), ...
                'g','LineWidth',2, 'DisplayName', 'Actual trajectory');
            view(obj.Axes, [0 0 1]);
            drawnow limitrate
        end
        
        function showLegend(obj)
            % Add a legend to the axes
            hLegend = legend(obj.Axes, 'Location',  'northwest', ...
                'TextColor', [1 1 1], 'FontWeight', 'bold');
        end
    end
    
    methods (Access = private)
        function scaledLocations = transformCoodinates(obj, initialPose)
            % Update the map points and camera trajectory based on the
            % initial pose of the sensor
            
            % imageToCamera转换为eul角度为[0,-pi/2,pi/2]
            %             imageToCamera = rigid3d([0 -1 0 0; 0 0 -1 0; 1 0 0 0; 0 0 0 1]);% 推测旋转矩阵为绕Z轴-pi/2角度
            angle = -90;
            rx = [1,0,0;
                0,cosd(angle),sind(angle);
                0,-sind(angle),cosd(angle)];
            tx = [0,0,0];
            rz = [cosd(-angle),sind(-angle),0;
                -sind(-angle),cosd(-angle),0;
                0,0,1];
            tz = [0,0,0];
            r = rx*rz;% first x,then z
            t = tx+tz;
            imageToCamera = rigid3d(r,t);% 点绕x轴旋转-90°，即XOZ平面转换为XOY平面的转换,再绕Z轴旋转-90°，得YOX
%             origin2init = invert(initialPose);
            tform = rigid3d(imageToCamera.T * initialPose.T);% 点云转换到gt的变换过程,先转换imageToCamera，这个时候已经是基于initialPose下的坐标，再转换initalPose到世界坐标系，注意顺序 
            view(obj.Axes,[0,0,1])

            preLim = [obj.Axes.XLim.', obj.Axes.YLim.', obj.Axes.ZLim.'];
            currLim = transformPointsForward(tform, preLim); 
            obj.Axes.XLim = [min(currLim(:,1)), max(currLim(:,1))];
            obj.Axes.YLim = [min(currLim(:,2)), max(currLim(:,2))];
            obj.Axes.ZLim = [min(currLim(:,3)), max(currLim(:,3))];
            
            % Map points
            [mapX, mapY, mapZ] = transformPointsForward(tform, ...
                obj.Axes.Children(end).XData, ...
                obj.Axes.Children(end).YData, ...
                obj.Axes.Children(end).ZData);
            obj.Axes.Children(end).XData = mapX;
            obj.Axes.Children(end).YData = mapY;
            obj.Axes.Children(end).ZData = mapZ;
            
            % Estimated and optimized Camera trajectory
            [estimatedTrajX, estimatedTrajY, estimatedTrajZ] = ...
                transformPointsForward(tform, ...
                obj.EstimatedTrajectory.XData, ...
                obj.EstimatedTrajectory.YData, ...
                obj.EstimatedTrajectory.ZData);
            obj.EstimatedTrajectory.XData = estimatedTrajX;
            obj.EstimatedTrajectory.YData = estimatedTrajY;
            obj.EstimatedTrajectory.ZData = estimatedTrajZ;

            %             [optimizedTrajX, optimizedTrajY, optimizedTrajZ] = ...
            %                 transformPointsForward(tform, ...
            %                 obj.OptimizedTrajectory.XData, ...
            %                 obj.OptimizedTrajectory.YData, ...
            %                 obj.OptimizedTrajectory.ZData);
            %             obj.OptimizedTrajectory.XData = optimizedTrajX;
            %             obj.OptimizedTrajectory.YData = optimizedTrajY;
            %             obj.OptimizedTrajectory.ZData = optimizedTrajZ;
            %
            %             scaledLocations = [optimizedTrajX.', optimizedTrajY.', optimizedTrajZ.'];
            scaledLocations = [estimatedTrajX',estimatedTrajY',estimatedTrajZ'];
        end
        
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
            
%             % Only plot the points within the limit
%             inPlotRange = xyzPoints(:, 1) > obj.XLim(1) & ...
%                 xyzPoints(:, 1) < obj.XLim(2) & xyzPoints(:, 2) > obj.YLim(1) & ...
%                 xyzPoints(:, 2) < obj.YLim(2) & xyzPoints(:, 3) > obj.ZLim(1) & ...
%                 xyzPoints(:, 3) < obj.ZLim(2);
%             xyzPoints   = xyzPoints(inPlotRange, :);
        end
        
        function updatePlotScale(obj, scale)
            % Update the map points and camera trajectory based on the
            % ground truth scale
            obj.Axes.XLim = obj.Axes.XLim * scale;
            obj.Axes.YLim = obj.Axes.YLim * scale;
            obj.Axes.ZLim = obj.Axes.ZLim * scale;
            
            % Map points
            obj.Axes.Children(end).XData = obj.Axes.Children(end).XData * scale;
            obj.Axes.Children(end).YData = obj.Axes.Children(end).YData * scale;
            obj.Axes.Children(end).ZData = obj.Axes.Children(end).ZData * scale;
            
            % Estiamted and optimized Camera trajectory
            obj.EstimatedTrajectory.XData =  obj.EstimatedTrajectory.XData * scale;
            obj.EstimatedTrajectory.YData =  obj.EstimatedTrajectory.YData * scale;
            obj.EstimatedTrajectory.ZData =  obj.EstimatedTrajectory.ZData * scale;
        end
    end
end