function alphas = calcTilt(geometry,start_plat_trans,plat_rot)  
    %Calculate rotation matrix from platform to inertial coords (inertial = R*platform)
    Rp_b = [cosd(plat_rot(3))*cosd(plat_rot(2)) cosd(plat_rot(3))*sind(plat_rot(2))*sind(plat_rot(1))-sind(plat_rot(3))*cosd(plat_rot(1)) cosd(plat_rot(3))*sind(plat_rot(2))*cosd(plat_rot(1))+sind(plat_rot(3))*sind(plat_rot(1));
           sind(plat_rot(3))*cosd(plat_rot(2)) sind(plat_rot(3))*sind(plat_rot(2))*sind(plat_rot(1))+cosd(plat_rot(3))*cosd(plat_rot(1)) sind(plat_rot(3))*sind(plat_rot(2))*cosd(plat_rot(1))-cosd(plat_rot(3))*sind(plat_rot(1));
           -sind(plat_rot(2)) cosd(plat_rot(2))*sind(plat_rot(1)) cosd(plat_rot(2))*cosd(plat_rot(1))];
end