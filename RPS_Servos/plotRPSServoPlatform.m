function plotRPSServoPlatform(geometry,plat_trans,plat_rot,alphas)
    Rp_b = [cosd(plat_rot(3))*cosd(plat_rot(2)) cosd(plat_rot(3))*sind(plat_rot(2))*sind(plat_rot(1))-sind(plat_rot(3))*cosd(plat_rot(1)) cosd(plat_rot(3))*sind(plat_rot(2))*cosd(plat_rot(1))+sind(plat_rot(3))*sind(plat_rot(1));
           sind(plat_rot(3))*cosd(plat_rot(2)) sind(plat_rot(3))*sind(plat_rot(2))*sind(plat_rot(1))+cosd(plat_rot(3))*cosd(plat_rot(1)) sind(plat_rot(3))*sind(plat_rot(2))*cosd(plat_rot(1))-cosd(plat_rot(3))*sind(plat_rot(1));
           -sind(plat_rot(2)) cosd(plat_rot(2))*sind(plat_rot(1)) cosd(plat_rot(2))*cosd(plat_rot(1))];
       
    plat_pos_b_non_trans = Rp_b*geometry.plat_pos_p;
    plat_pos_b = plat_trans+plat_pos_b_non_trans;
    %% Plot
    figure()
    clf
    xlim([-0.5,0.5])
    ylim([-0.5,0.5])
    ylabel('y (m)')
    xlabel('x (m)')
    zlabel('z (m)')
    daspect([1 1 1])
    view(3)
    hold on
    for l_index = 1:size(geometry.servo_pos_b,2)
        curr_servo_pos_b = geometry.servo_pos_b(:,l_index); %servo pos vector
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

        alpha = alphas(l_index);

        gamma = geometry.servo_arm_end_gammas(l_index);
        gamma_rot_matrix = [cosd(gamma) -sind(gamma) 0;
                            sind(gamma) cosd(gamma) 0;
                            0 0 1];
        alpha_rot_matrix = [1 0 0;
                            0 cosd(90-alpha) -sind(90-alpha);
                            0 sind(90-alpha) cosd(90-alpha)];

        servo_arm_end_b = gamma_rot_matrix*(alpha_rot_matrix*geometry.servo_arm_end_unrotated);
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
