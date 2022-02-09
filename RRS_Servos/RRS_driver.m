close all
%% RPS (w servos) PLATFORM GEOMETRY INPUT
geometry.a = .1; %m (12 inches)
geometry.s = .5; %m (21.5 inches)

%Smaller radii have better ranges of motion
geometry.base_rad = .508/2; %20 in diameter
geometry.plat_rad = .508/2; %geometry.base_rad;

%% Calculate connection positions
geometry = calculateRestOfRPSServoGeom(geometry);

%% RPS (w servo) PLATFORM INPUTS
plat_rot = [0 0 0]; %theta (y axis) 15% grade or 8.5308 deg (RPY)
plat_trans = [0 0 geometry.s+geometry.a-0.05]'; %desired translation of COM


alphas = findRPSServoAngles(geometry,plat_trans,plat_rot)
plotRPSServoPlatform(geometry,plat_trans,plat_rot,alphas)