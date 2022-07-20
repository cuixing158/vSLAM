%% 使用openstreetmap 的图导入到matlab 中的drivingScenario进行cuboid simuliaion 和
% UE simulination
scenario = drivingScenario("SampleTime",0.01,"StopTime",3);
roadNetwork(scenario,"OpenStreetMap","my_openstreetmap_export.xml");
egovhicle = vehicle(scenario,ClassID=1);
egopath = [-133,-388,0;-98,-266,0];
egospeed = 30;
smoothTrajectory(egovhicle,egopath,egospeed);
chasePlot(egovhicle)

%%
while advance(scenario)
    pause(scenario.SampleTime)
end


