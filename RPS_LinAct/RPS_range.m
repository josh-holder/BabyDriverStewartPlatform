close all
%% RPS (w Lin Actuator) PLATFORM GEOMETRY INPUT
%https://www.amazon.com/Progressive-Automations-Industrial-Linear-Actuator/dp/B006P58XUS/ref=asc_df_B006P58Y6G/?tag=hyprod-20&linkCode=df0&hvadid=312319107733&hvpos=&hvnetw=g&hvrand=12277548775097128658&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9027671&hvtargid=pla-571254056585&th=1
geometry.leg_extend = .3048; %m (12 inches)
geometry.base_leg_length = .51; %m (21.5 inches)

%Smaller radii have better ranges of motion
geometry.base_rad = .635/2; %25 in diameter
geometry.plat_rad = .635/2;%geometry.base_rad;

%% Calculate connection positions
geometry.linact_pos_b = [geometry.base_rad 0 0; %bottom two servos
                 -geometry.base_rad*cosd(60) geometry.base_rad*sind(60) 0;
                 -geometry.base_rad*cosd(60) -geometry.base_rad*sind(60) 0]';

geometry.plat_pos_p = [geometry.plat_rad 0 0; %bottom two servos
             -geometry.plat_rad*cosd(60) geometry.plat_rad*sind(60) 0;
             -geometry.plat_rad*cosd(60) -geometry.plat_rad*sind(60) 0]';

%% RPS (w Lin Actuator) PLATFORM INPUTS
plat_rot = [0 0 0]; %theta (y axis) 15% grade or 8.5308 deg (RPY)
plat_base = [0 0 geometry.base_leg_length + geometry.leg_extend/2]'; %desired translation of COM


%% Range of Motion parameters
angle_mag_range = 4;

zs = geometry.base_leg_length:geometry.leg_extend*.1:geometry.base_leg_length+geometry.leg_extend;
angles = -45:0.1:45;
%Iterate through z values you want to check
min_pitch = 1000;
max_pitch = -1000;
min_roll = 1000;
max_roll = -1000;
for angle = angles
    %Try to achieve pitch with base height
    try
        lengths = calcRPSLengths(geometry,plat_base,[0 angle 0],0);
        if angle < min_pitch
            min_pitch = angle;
        elseif angle > max_pitch
            max_pitch = angle;
        end
            
    catch ME
        lengths = [];
        if startsWith(ME.message,"Length")
        elseif startsWith(ME.message,"Angle")
        else
            disp(ME.message)
        end
    end
    
    try
        lengths = calcRPSLengths(geometry,plat_base,[angle 0 0],0);
        if angle < min_roll
            min_roll = angle;
        elseif angle > max_roll
            max_roll = angle;
        end
            
    catch ME
        lengths = [];
        if startsWith(ME.message,"Length")
        elseif startsWith(ME.message,"Angle")
        else
            disp(ME.message)
        end
    end
end

disp("Vertical Range: " + geometry.base_leg_length + " to " + (geometry.base_leg_length+geometry.leg_extend) + "m ("+ geometry.leg_extend + "m total)")
disp("Max Pitch: " + max_pitch)
disp("Min Pitch: " + min_pitch)
calcRPSLengths(geometry,plat_base,[0 max_pitch 0],1);
disp("Max Roll: " + max_roll)
disp("Min Roll: " + min_roll)
calcRPSLengths(geometry,plat_base,[max_roll 0 0],1);

a = 2.69;
m = 25;
g = 9.81;
t=0.25;
% v = sqrt(2*a*geometry.leg_extend/2)/.0254
v = a*t
v_in = v/.0254
F = m/3*(a+g)
distance = .5*a*t^2
% F = 1/2*25/3*v^2/(geometry.leg_extend/2)
% 
% Ftot = (25/3)/9.81*v/(v/a)+25/3*9.81