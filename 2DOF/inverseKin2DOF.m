%Inverse kinematics for 2DOF custom platform
function angles = inverseKin2DOF(a,s,base_rad,roll,z,plot)
    Rp_b = [1 0 0;
            0 cosd(roll) -sind(roll);
            0 sind(roll) cosd(roll)];
    
    plat_trans = [0; 0; z];
        
    servo_pos_b = [0 base_rad 0;
                    0 -base_rad 0]';
    
    plat_pos_p = [0 base_rad 0;
                  0 -base_rad 0]';
    
    servo_betas = [90;270];
    
    servo_arm_end_gammas = [0; 180];
    
    servo_arm_end_unrotated = [0; a; 0];
              
    %convert platform positions to base frame
    plat_pos_b = plat_trans + Rp_b*plat_pos_p;
    
    angles = [];
    
    for l_index = 1:size(servo_pos_b,2)
        curr_servo_pos_b = servo_pos_b(:,l_index); %servo pos vector
        curr_plat_pos_b = plat_pos_b(:,l_index);
        length_b = curr_plat_pos_b - curr_servo_pos_b;

        %Calculate servo angle from math pdf
        beta = servo_betas(l_index);
        L = norm(length_b)^2-(s^2-a^2);
        M = 2*a*(curr_plat_pos_b(3)-curr_servo_pos_b(3));
        N = 2*a*(cosd(beta)*(curr_plat_pos_b(1)-curr_servo_pos_b(1)) + ...
            sind(beta)*(curr_plat_pos_b(2)-curr_servo_pos_b(2)));
        
        angle = asind(L/sqrt(M^2+N^2))-atand(N/M);
        angles = [angles angle];
    end

    %% Plot
    if plot
        close all
        figure()
        clf
        xlim([-0.5,0.5])
        ylim([-0.5,0.5])
        ylabel('y (m)')
        xlabel('x (m)')
        zlabel('z (m)')
        daspect([1 1 1])
        view(90,0)
        hold on
        for l_index = 1:size(servo_pos_b,2)
            curr_servo_pos_b = servo_pos_b(:,l_index); %servo pos vector
            curr_plat_pos_b = plat_pos_b(:,l_index);
            length_b = curr_plat_pos_b - curr_servo_pos_b;

            %Plot base servo positions in body frame
            plot3([0 curr_servo_pos_b(1)],[0 curr_servo_pos_b(2)],...
                [0 curr_servo_pos_b(3)],'Color','r')

            %Plot platform positions in body frame
            plot3([plat_trans(1) curr_plat_pos_b(1)],[plat_trans(2) curr_plat_pos_b(2)],...
                [plat_trans(3) curr_plat_pos_b(3)],'Color','b')

            %Plot lengths (start at servo, travel length)
            plot3([curr_servo_pos_b(1) curr_servo_pos_b(1)+length_b(1)],...
                [curr_servo_pos_b(2) curr_servo_pos_b(2)+length_b(2)],...
                [curr_servo_pos_b(3) curr_servo_pos_b(3)+length_b(3)],'--g')

            alpha = angles(l_index);

            gamma = servo_arm_end_gammas(l_index);
            gamma_rot_matrix = [cosd(gamma) -sind(gamma) 0;
                                sind(gamma) cosd(gamma) 0;
                                0 0 1];
            alpha_rot_matrix = [1 0 0;
                                0 cosd(alpha) -sind(alpha);
                                0 sind(alpha) cosd(alpha)];

            servo_arm_end_b = gamma_rot_matrix*(alpha_rot_matrix*servo_arm_end_unrotated);
            %disp servo arm
            plot3([curr_servo_pos_b(1) curr_servo_pos_b(1)+servo_arm_end_b(1)],...
                [curr_servo_pos_b(2) curr_servo_pos_b(2)+servo_arm_end_b(2)],...
                [curr_servo_pos_b(3) curr_servo_pos_b(3)+servo_arm_end_b(3)],'Color','g')

            arm_length_b = curr_plat_pos_b - (curr_servo_pos_b + servo_arm_end_b);

            plot3([curr_servo_pos_b(1)+servo_arm_end_b(1) curr_plat_pos_b(1)],...
                [curr_servo_pos_b(2)+servo_arm_end_b(2) curr_plat_pos_b(2)],...
                [curr_servo_pos_b(3)+servo_arm_end_b(3) curr_plat_pos_b(3)],'Color','g')
        end
    end
end