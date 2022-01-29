function geometry = calculateRestOfRPSServoGeom(geometry)
% U,sing global constants which define Stewart Platform geometry, set up
% Stewart platform (calculate vector locations of platform connection
% points, as well as servo angles
    geometry.servo_pos_b = [geometry.base_rad 0 0; %bottom two servos
                 -geometry.base_rad*cosd(60) geometry.base_rad*sind(60) 0;
                 -geometry.base_rad*cosd(60) -geometry.base_rad*sind(60) 0]';

    geometry.plat_pos_p = [geometry.plat_rad 0 0; %bottom two servos
             -geometry.plat_rad*cosd(60) geometry.plat_rad*sind(60) 0;
             -geometry.plat_rad*cosd(60) -geometry.plat_rad*sind(60) 0]';

    %gamma is rotation around z axis of the servo arm, for plotting
    geometry.servo_arm_end_gammas = [0;
                            120;
                            240];

    geometry.servo_arm_end_unrotated = [0; geometry.a; 0];
    %hi
end
