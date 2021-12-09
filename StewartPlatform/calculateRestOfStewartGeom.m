function geometry = calculateRestOfStewartGeom(geometry)
% Using global constants which define Stewart Platform geometry, set up
% Stewart platform (calculate vector locations of platform connection
% points, as well as servo angles 
    geometry.servo_pos_b = [geometry.base_rad*cosd(geometry.base_angle) -geometry.base_rad*sind(geometry.base_angle) 0; %bottom two servos
                 geometry.base_rad*cosd(geometry.base_angle) geometry.base_rad*sind(geometry.base_angle) 0;
                 -geometry.base_rad*sind(30 - geometry.base_angle) geometry.base_rad*cosd(30 - geometry.base_angle) 0; %top right servos
                 -geometry.base_rad*sind(30 + geometry.base_angle) geometry.base_rad*cosd(30 + geometry.base_angle) 0;
                 -geometry.base_rad*cosd(60 - geometry.base_angle) -geometry.base_rad*sind(60 - geometry.base_angle) 0; %top left servos
                 -geometry.base_rad*cosd(60 + geometry.base_angle) -geometry.base_rad*sind(60 + geometry.base_angle) 0]';
    geometry.servo_betas = [-(90-geometry.base_angle);
                   (90-geometry.base_angle);
                   (30-geometry.base_angle);
                   (30+geometry.base_angle);
                   (30+geometry.base_angle);
                   (30-geometry.base_angle);]';

    geometry.plat_pos_p = [geometry.plat_rad*cosd(geometry.plat_angle) -geometry.plat_rad*sind(geometry.plat_angle) 0; %bottom two servos
                 geometry.plat_rad*cosd(geometry.plat_angle) geometry.plat_rad*sind(geometry.plat_angle) 0;
                 -geometry.plat_rad*sind(30 - geometry.plat_angle) geometry.plat_rad*cosd(30 - geometry.plat_angle) 0; %top right servos
                 -geometry.plat_rad*sind(30 + geometry.plat_angle) geometry.plat_rad*cosd(30 + geometry.plat_angle) 0;
                 -geometry.plat_rad*cosd(60 - geometry.plat_angle) -geometry.plat_rad*sind(60 - geometry.plat_angle) 0; %top left servos
                 -geometry.plat_rad*cosd(60 + geometry.plat_angle) -geometry.plat_rad*sind(60 + geometry.plat_angle) 0]';

    %gamma is rotation around z axis of the servo arm, for plotting
    geometry.servo_arm_end_gammas = [-geometry.base_angle;
                            geometry.base_angle;
                            120-geometry.base_angle;
                            120+geometry.base_angle;
                            240-geometry.base_angle;
                            240+geometry.base_angle]; 
    geometry.servo_arm_end_unrotated = [0; geometry.a; 0];
end