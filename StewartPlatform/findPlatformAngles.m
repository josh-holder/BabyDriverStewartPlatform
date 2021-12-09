function alphas = findPlatformAngles(geometry,Rp_b,plat_trans)   
    plat_pos_b = plat_trans+Rp_b*geometry.plat_pos_p;
               
    alphas = [];
    for l_index = 1:size(geometry.servo_pos_b,2)
        curr_servo_pos_b = geometry.servo_pos_b(:,l_index); %servo pos vector
        curr_plat_pos_b = plat_pos_b(:,l_index);
        length_b = curr_plat_pos_b - curr_servo_pos_b;

        %Calculate servo angle from math pdf
        beta = geometry.servo_betas(l_index);
        L = norm(length_b)^2-(geometry.s^2-geometry.a^2);
        M = 2*geometry.a*(curr_plat_pos_b(3)-curr_servo_pos_b(3));
        N = 2*geometry.a*(cosd(beta)*(curr_plat_pos_b(1)-curr_servo_pos_b(1)) + ...
            sind(beta)*(curr_plat_pos_b(2)-curr_servo_pos_b(2)));

        alpha = asind(L/sqrt(M^2+N^2))-atand(N/M);

        gamma = geometry.servo_arm_end_gammas(l_index);
        gamma_rot_matrix = [cosd(gamma) -sind(gamma) 0;
                            sind(gamma) cosd(gamma) 0;
                            0 0 1];
        alpha_rot_matrix = [1 0 0;
                            0 cosd(alpha) -sind(alpha);
                            0 sind(alpha) cosd(alpha)];

        servo_arm_end_b = gamma_rot_matrix*(alpha_rot_matrix*geometry.servo_arm_end_unrotated);

        arm_length_b = curr_plat_pos_b - (curr_servo_pos_b + servo_arm_end_b);
        disp(arm_length_b)
        
        %Only add if the arm lengh is correct
        if abs(norm(arm_length_b)-geometry.s) >= geometry.s*.025
            disp("SOMETHING WENT WRONG - WRONG ARM LENGTH " +norm(arm_length_b))
            alphas = [];
            break
        else
            alphas = [alphas alpha];
        end
    end
end