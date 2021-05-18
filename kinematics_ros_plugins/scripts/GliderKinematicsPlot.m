clear;clc;
close all;


KINEMATICSLOGFILENAME = 'KinematicsLog.csv';
DRLOGFILENAME = 'DRLog.csv';


% Import KinematicsLog.csv
opts = delimitedTextImportOptions("NumVariables", 19);
opts.DataLines = [3, Inf]; opts.Delimiter = ",";
opts.VariableNames = ["T", "x", "y", "z", "p", "q", "r", "lat", "lon", ...
    "thrustPower", "pumpVol", "batPos", "thrustVel", "xBuoyancyVel", ...
    "zBuoyancyVel", "xVehicleVel", "yVehicleVel", "zVehicleVel"];
opts.VariableTypes = ["double", "double", "double", "double", "double", ...
    "double", "double", "double", "double", "double", "double", "double", ...
    "double", "double", "double", "double", "double", "string"];
opts.ExtraColumnsRule = "ignore"; opts.EmptyLineRule = "read";
opts = setvaropts(opts, "zVehicleVel", "WhitespaceRule", "preserve");
opts = setvaropts(opts, "zVehicleVel", "EmptyFieldRule", "auto");
tbl = readtable(KINEMATICSLOGFILENAME, opts);
T = tbl.T;x = tbl.x;y = tbl.y;z = tbl.z;p = tbl.p;q = tbl.q;r = tbl.r;
lat = tbl.lat;lon = tbl.lon;thrustPower = tbl.thrustPower;
pumpVol = tbl.pumpVol;batPos = tbl.batPos;thrustVel = tbl.thrustVel;
xBuoyancyVel = tbl.xBuoyancyVel;zBuoyancyVel = tbl.zBuoyancyVel;
xVehicleVel = tbl.xVehicleVel;yVehicleVel = tbl.yVehicleVel;
zVehicleVel = tbl.zVehicleVel;
clear opts tbl

% Import DRLog.csv
opts = delimitedTextImportOptions("NumVariables", 4);
opts.DataLines = [3, Inf];opts.Delimiter = ",";
opts.VariableNames = ["time", "depth1", "lat1", "lon1", "gpsBool"];
opts.VariableTypes = ["double", "double", "double", "double", "categorical"];
opts.ExtraColumnsRule = "ignore";opts.EmptyLineRule = "read";
opts = setvaropts(opts, "gpsBool", "EmptyFieldRule", "auto");
tbl = readtable(DRLOGFILENAME, opts);
time_dr = tbl.time;depth_dr = tbl.depth1;lat_dr = tbl.lat1;lon_dr = tbl.lon1;gpsBool = tbl.gpsBool;
clear opts tbl


figure;
hF = gcf;
hF.Position(3:4) = [1400 400];

subplot(1,3,1)
plot(T,-z,'k','LineWidth',2); hold on; grid on;
scatter(time_dr,depth_dr,'+r');
legend('Depth(Simulator)','Depth(Dead Reckoning)','Location','Best');

subplot(1,3,2)
plot(T,lat,'k','LineWidth',2); hold on; grid on;
scatter(time_dr,lat_dr,'+r');
legend('Latitude(Simulator)','Latitude(Dead Reckoning)','Location','Best');

subplot(1,3,3)
plot(T,lon,'k','LineWidth',2); hold on; grid on;
scatter(time_dr,lon_dr,'+r');
legend('Longitude(Simulator)','Longitude(Dead Reckoning)','Location','Best');

% subplot(1,5,4)
% plot(T,x,'k');
% legend('X','Location','Best');
% 
% subplot(1,5,5)
% plot(T,y,'k');
% legend('Y','Location','Best');
