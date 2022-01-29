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
plat_trans = [0 0 geometry.s+geometry.a/2]'; %desired translation of COM


findRPSServoAngles(geometry,plat_trans,[0 8.5 0])

%% Range of Motion parameters

angles = -45:0.1:45;
%Iterate through z values you want to check
min_pitch = 1000;
max_pitch = -1000;
min_roll = 1000;
max_roll = -1000;
for angle = angles
    %Try to achieve pitch with base height
    try
        findRPSServoAngles(geometry,plat_trans,[0 angle 0]);
        if angle < min_pitch
            min_pitch = angle;
        elseif angle > max_pitch
            max_pitch = angle;
        end
            
    catch ME
        lengths = [];
        if startsWith(ME.message,"Length")
        elseif startsWith(ME.message,"Angle")
        elseif startsWith(ME.message,"Plane")
        else
            disp(ME.message)
        end
    end
    
    try
        findRPSServoAngles(geometry,plat_trans,[angle 0 0]);
        if angle < min_roll
            min_roll = angle;
        elseif angle > max_roll
            max_roll = angle;
        end
            
    catch ME
        lengths = [];
        if startsWith(ME.message,"Length")
        elseif startsWith(ME.message,"Angle")
        elseif startsWith(ME.message,"Plane")
        else
            disp(ME.message)
        end
    end
end

disp("Vertical Range: " + (sqrt(geometry.s^2-geometry.a^2)) + ...
    " to " + (geometry.a+geometry.s) + "m ("+ ...
    (geometry.a+geometry.s-sqrt(geometry.s^2-geometry.a^2)) + "m total)")
disp("Max Pitch: " + max_pitch)
disp("Min Pitch: " + min_pitch)
alphas = findRPSServoAngles(geometry,plat_trans,[0 max_pitch 0]);
plotRPSServoPlatform(geometry,plat_trans,[0 max_pitch 0],alphas)
disp("Max Roll: " + max_roll)
disp("Min Roll: " + min_roll)
alphas = findRPSServoAngles(geometry,plat_trans,[max_roll 0 0]);
plotRPSServoPlatform(geometry,plat_trans,[max_roll 0 0],alphas)
alphas = findRPSServoAngles(geometry,plat_trans,[0 10.2 0]);


a = 2.69;
m = 25;
g = 9.81;
t= 0.25;
% v = sqrt(2*a*geometry.leg_extend/2)/.0254
v = a*t;
% F = 1/2*25/3*v^2/(geometry.leg_extend/2)
% 
% Ftot = (25/3)/9.81*v/(v/a)+25/3*9.81