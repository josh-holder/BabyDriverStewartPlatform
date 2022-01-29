function alphas = findRPSServoAngles(geometry,plat_trans,plat_rot)  

    %Calculate rotation matrix from platform to inertial coords (inertial = R*platform)
    Rp_b = [cosd(plat_rot(3))*cosd(plat_rot(2)) cosd(plat_rot(3))*sind(plat_rot(2))*sind(plat_rot(1))-sind(plat_rot(3))*cosd(plat_rot(1)) cosd(plat_rot(3))*sind(plat_rot(2))*cosd(plat_rot(1))+sind(plat_rot(3))*sind(plat_rot(1));
           sind(plat_rot(3))*cosd(plat_rot(2)) sind(plat_rot(3))*sind(plat_rot(2))*sind(plat_rot(1))+cosd(plat_rot(3))*cosd(plat_rot(1)) sind(plat_rot(3))*sind(plat_rot(2))*cosd(plat_rot(1))-cosd(plat_rot(3))*sind(plat_rot(1));
           -sind(plat_rot(2)) cosd(plat_rot(2))*sind(plat_rot(1)) cosd(plat_rot(2))*cosd(plat_rot(1))];

    plat_pos_b = plat_trans+Rp_b*geometry.plat_pos_p;
               
    alphas = [];
    plane_vector = [0 0 1];
    for l_index = 1:size(geometry.servo_pos_b,2)
        %grab vectors
        curr_servo_pos_b = geometry.servo_pos_b(:,l_index); %servo pos vector
        curr_plat_pos_b = plat_pos_b(:,l_index);
        
        %length between top and bot
        length_b = curr_plat_pos_b - curr_servo_pos_b;
        %Use law of cosines to solve for alpha
        alpha = acosd((geometry.a^2+norm(length_b)^2-geometry.s^2)/(2*geometry.a*norm(length_b)));

        gamma = geometry.servo_arm_end_gammas(l_index);
        gamma_rot_matrix = [cosd(gamma) -sind(gamma) 0;
                            sind(gamma) cosd(gamma) 0;
                            0 0 1];
        alpha_rot_matrix = [1 0 0;
                            0 cosd(90-alpha) -sind(90-alpha);
                            0 sind(90-alpha) cosd(90-alpha)];
                        
                        
        servo_arm_end_b = gamma_rot_matrix*(alpha_rot_matrix*geometry.servo_arm_end_unrotated);
        %vector of longer link
        arm_length_b = curr_plat_pos_b - (curr_servo_pos_b + servo_arm_end_b);
                        
        %get vector normal to this plane - together with plane_vector,
        %long link must be in this plane
        plane_vector_2 = cross(plane_vector,curr_servo_pos_b);
        plane_normal = cross(plane_vector,plane_vector_2);
        
        dot_prod = dot(-arm_length_b,plane_normal)
        norm(arm_length_b);
        
        %Only add if the configurations satisfies all conditions
        if abs(norm(arm_length_b)-geometry.s) >= geometry.s*.001
            error("Length Error: requested configuration yields impossible leg length "+norm(arm_length_b))
        end
        if abs(dot_prod) > .001
            error("Plane Error: requested configuration has long leng out of plane")
        end
        if (abs(alpha) > 120)
            error("Angle Error: requested angle goes below horizontal, "+alpha)
        end
            
        alphas = [alphas alpha];
    end
end