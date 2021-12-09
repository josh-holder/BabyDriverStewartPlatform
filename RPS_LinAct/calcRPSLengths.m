function lengths = calcRPSLengths(geometry,plat_trans,plat_rot,plot)
    if plot
        f = figure();
        clf
        xlim([-0.5,0.5])
        ylim([-0.5,0.5])
        ylabel('y (m)')
        xlabel('x (m)')
        zlabel('z (m)')
        daspect([1 1 1])
        view(3)
        hold on
    end

    %Calculate rotation matrix from platform to inertial coords (inertial = R*platform)
    Rp_b = [cosd(plat_rot(3))*cosd(plat_rot(2)) cosd(plat_rot(3))*sind(plat_rot(2))*sind(plat_rot(1))-sind(plat_rot(3))*cosd(plat_rot(1)) cosd(plat_rot(3))*sind(plat_rot(2))*cosd(plat_rot(1))+sind(plat_rot(3))*sind(plat_rot(1));
           sind(plat_rot(3))*cosd(plat_rot(2)) sind(plat_rot(3))*sind(plat_rot(2))*sind(plat_rot(1))+cosd(plat_rot(3))*cosd(plat_rot(1)) sind(plat_rot(3))*sind(plat_rot(2))*cosd(plat_rot(1))-cosd(plat_rot(3))*sind(plat_rot(1));
           -sind(plat_rot(2)) cosd(plat_rot(2))*sind(plat_rot(1)) cosd(plat_rot(2))*cosd(plat_rot(1))];
    
    max_length = geometry.base_leg_length + geometry.leg_extend;
       
    plat_pos_b = plat_trans+Rp_b*geometry.plat_pos_p;
    
    lengths = [];
    
    plane_vector = [0 0 1];
    for l_index = 1:size(geometry.linact_pos_b,2)
        curr_linact_pos_b = geometry.linact_pos_b(:,l_index); %servo pos vector
        
        plane_normal = cross(plane_vector,curr_linact_pos_b);
        
        curr_plat_pos_b = plat_pos_b(:,l_index);
        
        dot_prod = dot(curr_linact_pos_b-curr_plat_pos_b,plane_normal);
        
        if plot
            %Plot base servo positions in body frame
            plot3([0 curr_linact_pos_b(1)],[0 curr_linact_pos_b(2)],...
                [0 curr_linact_pos_b(3)],'Color','r')

            %Plot platform positions in body frame
            plot3([plat_trans(1) curr_plat_pos_b(1)],[plat_trans(2) curr_plat_pos_b(2)],...
                [plat_trans(3) curr_plat_pos_b(3)],'Color','b')

            plot3([curr_linact_pos_b(1) curr_plat_pos_b(1)],[curr_linact_pos_b(2) curr_plat_pos_b(2)],...
                [curr_linact_pos_b(3) curr_plat_pos_b(3)],'Color','g')
        end
        
        if abs(dot_prod) > .001
            if plot
                close(f)
            end
            error("Angle Error: requested configuration yields impossible angle")
        end
            
        length_b = curr_plat_pos_b - curr_linact_pos_b;
        length_mag = norm(length_b);
        if (length_mag <= max_length) && (length_mag >= geometry.base_leg_length)
            lengths = [lengths length_mag];
        else
            if plot
                close(f)
            end
            error("Length Error: requested configuration yields impossible leg length")
        end
    end
    if plot
        hold off
    end
end