%% 使用openstreetmap 的图导入到matlab 中的drivingScenario进行cuboid simuliaion 和
% UE simulination
scenario = drivingScenario("SampleTime",0.01,"StopTime",inf);
roadNetwork(scenario,"OpenStreetMap","my_openstreetmap_export.xml");
egovhicle = vehicle(scenario,ClassID=1);
egopath = [-129.7,-390,0.5;
    17.69,210,0.5;
    34.7,250,0.5];
egospeed = 30;
smoothTrajectory(egovhicle,egopath,egospeed);
chasePlot(egovhicle)

%%
while advance(scenario)
    pause(scenario.SampleTime)
end


