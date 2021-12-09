clear
close all
%% STEWART PLATFORM INPUTS
plat_rot = [0 8 0]; %theta (y axis) 15% grade or 8.5308 deg
plat_trans = [0 0 0.5]'; %desired translation of COM

%Calculate rotation matrix from platform to inertial coords (inertial = R*platform)
Rp_b = [cosd(plat_rot(3))*cosd(plat_rot(2)) cosd(plat_rot(3))*sind(plat_rot(2))*sind(plat_rot(1))-sind(plat_rot(3))*cosd(plat_rot(1)) cosd(plat_rot(3))*sind(plat_rot(2))*cosd(plat_rot(1))+sind(plat_rot(3))*sind(plat_rot(1));
       sind(plat_rot(3))*cosd(plat_rot(2)) sind(plat_rot(3))*sind(plat_rot(2))*sind(plat_rot(1))+cosd(plat_rot(3))*cosd(plat_rot(1)) sind(plat_rot(3))*sind(plat_rot(2))*cosd(plat_rot(1))-cosd(plat_rot(3))*sind(plat_rot(1));
       -sind(plat_rot(2)) cosd(plat_rot(2))*sind(plat_rot(1)) cosd(plat_rot(2))*cosd(plat_rot(1))];

%% DEFINE SERVO, PLATFORM CONNECT AND ARM GEOMETRY
%Geometry object will hold all geometric parameters of stewart platform
%Define parameters below, and then call calculateRestOfStewartGeom to
%calculate remaining geometric parameters
%Use this for math
%https://web.archive.org/web/20130506134518/http://www.wokinghamu3a.org.uk/Maths%20of%20the%20Stewart%20Platform%20v5.pdf

geometry.base_angle = 19; %deg
geometry.base_rad = .635/2; %25 in diameter

geometry.plat_angle = geometry.base_angle;
geometry.plat_rad = geometry.base_rad;

%arm lengths
geometry.a = 0.1; %m
geometry.s = 0.5; %m

%From these geometric parameters, calculate vectors to platform
%connections, etc.
geometry = calculateRestOfStewartGeom(geometry);

%% Calculations
alphas = findPlatformAngles(geometry,Rp_b,plat_trans);
plotPlatform(geometry,Rp_b,plat_trans,alphas)
% for trans = -0.06:.01:.06
%     plat_trans = [trans 0 0.5]'; %desired translation of COM
%     alphas = findPlatformAngles(geometry,Rp_b,plat_trans);
% 
%     plotPlatform(geometry,Rp_b,plat_trans,alphas)
% end

%Calculate home position
% h0 = sqrt(geometry.s^2+geometry.a^2-(geometry.plat_pos_to_end(1,1)-geometry.servo_pos_b(1,1))^2-(geometry.plat_pos_to_end(1,2)-geometry.servo_pos_b(1,2))^2)-geometry.plat_pos_to_end(1,3);
% geometry.l0 = (geometry.plat_pos_to_end(1,1)-geometry.servo_pos_b(1,1))^2+(geometry.plat_pos_to_end(1,2)-geometry.servo_pos_b(1,2))^2+(geometry.h0+geometry.plat_pos_to_end(1,3))^2;
% geometry.L0 = 2*geometry.a^2;
% geometry.M0 = 2*geometry.a*(cosd(geometry.servo_betas(1)) * (geometry.plat_pos_to_end(1,1)-geometry.servo_pos_b(1,1)) + sind(geometry.servo_betas(1)) * (geometry.plat_pos_to_end(1,2)-geometry.servo_pos_b(1,2)));
% geometry.N0 = 2*geometry.a*(geometry.h0+geometry.plat_pos_to_end(1,3));
% geometry.alpha0 = asind(geometry.L0/sqrt(geometry.M0^2+geometry.N0^2))-atand(geometry.M0/geometry.N0);
