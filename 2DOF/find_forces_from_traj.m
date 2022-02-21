%Determine what torque and RPM we need to achieve our accelerations

%% Geometry input
a = 0.1;
s = 0.5;
base_rad = .508/2;

starting_zpos = sqrt(s^2-a^2);
starting_phi = atand(starting_zpos/a);

%% Accelerating in z direction from bottom
accel = 2.5;
t = .25;
tstep = .001;
zdots = accel*tstep:accel*tstep:accel*t;

pos_curr = starting_zpos;

thetas_curr = deg2rad([0 180-0]);
thetas = [];
theta_dots = [];
theta_ddots = [];

phis_curr = deg2rad([180-starting_phi starting_phi]);
phis = [];
phi_dots = [];
phi_ddots = [];

as_botl = [];
as_topl = [];
accels = [];
positions = [];

%iterate through platform vertical velocities
for i = 1:length(zdots)
    zdot = zdots(i);
    pos_curr = pos_curr + zdot*tstep;
    
    %save previous theta
    old_theta = thetas_curr;
    old_phi = phis_curr;
    [thetas_curr,phis_curr] = inverseKin2DOF(a,s,base_rad,0,pos_curr,0);
    %convert theta and phi to correct coord fram (theta2 and phi1 are
    %reversed)
    thetas_curr(2) = 180-thetas_curr(2);
    phis_curr(1) = 180-phis_curr(1);
    thetas_curr = deg2rad(thetas_curr);
    phis_curr = deg2rad(phis_curr);
    
    thetas = [thetas; thetas_curr];
    theta_dots = [theta_dots; (thetas_curr-old_theta)/tstep];
    
    phis = [phis; phis_curr];
    phi_dots = [phi_dots; (phis_curr-old_phi)/tstep];
    
    
    if size(theta_dots,1) > 1
        theta_ddots = [theta_ddots; (theta_dots(i,:)-theta_dots(i-1,:))/tstep];
        phi_ddots = [phi_ddots; (phi_dots(i,:)-phi_dots(i-1,:))/tstep];
    
        %calculate linear accelerations of COM bottom link
        botl_com_rads = [zeros(2,1) a/2*cos(thetas_curr)' a/2*sin(thetas_curr)'];

        botl_alphas = [theta_ddots(i-1,:)' zeros(2,2)];
        botl_omegas = [theta_dots(i,:)' zeros(2,2)];

        as_botl = [as_botl; cross(botl_alphas(1,:),botl_com_rads(1,:))+cross(botl_omegas(1,:),cross(botl_omegas(1,:),botl_com_rads(1,:))) ...
            cross(botl_alphas(2,:),botl_com_rads(2,:))+cross(botl_omegas(2,:),cross(botl_omegas(2,:),botl_com_rads(2,:)))];
        
        %calculate same way, just multiply radius by two to calculate at
        %joint, not link COM
        a_joint = [cross(botl_alphas(1,:),2*botl_com_rads(1,:))+cross(botl_omegas(1,:),cross(botl_omegas(1,:),2*botl_com_rads(1,:))) ...
            cross(botl_alphas(2,:),2*botl_com_rads(2,:))+cross(botl_omegas(2,:),cross(botl_omegas(2,:),2*botl_com_rads(2,:)))];

        topl_com_rads = [zeros(2,1) s/2*cos(phis_curr)' s/2*sin(phis_curr)'];
        topl_alphas = [phi_ddots(i-1,:)' zeros(2,2)];
        topl_omegas = [phi_dots(i,:)' zeros(2,2)];
        
        a_topl_without_joint = [cross(topl_alphas(1,:),topl_com_rads(1,:))+cross(topl_omegas(1,:),cross(topl_omegas(1,:),topl_com_rads(1,:))) ...
            cross(topl_alphas(2,:),topl_com_rads(2,:))+cross(topl_omegas(2,:),cross(topl_omegas(2,:),topl_com_rads(2,:)))];
        as_topl = [as_topl; a_topl_without_joint + a_joint];
        
    end
end

figure(1)
clf
hold on
plot(1:length(zdots)-1,as_botl(:,2),'DisplayName','bot')
plot(1:length(zdots)-1,as_botl(:,5),'DisplayName','top')
legend()
hold off

%% Accelerate in rotation
alpha = 1.5; %rad/s^2
t = 0.25; %s
omegas = alpha*tstep:alpha*tstep:alpha*t;

starting_z = .55;

plat_theta_curr = deg2rad(-8);
[thetas_curr, phis_curr] = inverseKin2DOF(a,s,base_rad,rad2deg(plat_theta_curr),starting_z,0);
thetas_curr(2) = 180-thetas_curr(2);
phis_curr(1) = 180-phis_curr(1);
thetas_curr = deg2rad(thetas_curr);
phis_curr = deg2rad(phis_curr);

thetas = [];
theta_dots = [];
theta_ddots = [];

phis = [];
phi_dots = [];
phi_ddots = [];

as_botl = [];
as_topl = [];

for i = 1:length(omegas)
    plat_omega = omegas(i);
    plat_theta_curr = plat_theta_curr + plat_omega*tstep
    
    %save previous theta
    old_theta = thetas_curr;
    old_phi = phis_curr;
    [thetas_curr,phis_curr] = inverseKin2DOF(a,s,base_rad,rad2deg(plat_theta_curr),starting_z,0);
    %convert theta and phi to correct coord frame (theta2 and phi1 are
    %reversed)
    thetas_curr(2) = 180-thetas_curr(2);
    phis_curr(1) = 180-phis_curr(1);
    thetas_curr = deg2rad(thetas_curr);
    phis_curr = deg2rad(phis_curr);
    
    thetas = [thetas; thetas_curr];
    theta_dots = [theta_dots; (thetas_curr-old_theta)/tstep];
    
    phis = [phis; phis_curr];
    phi_dots = [phi_dots; (phis_curr-old_phi)/tstep];
    
    
    if size(theta_dots,1) > 1
        theta_ddots = [theta_ddots; (theta_dots(i,:)-theta_dots(i-1,:))/tstep];
        phi_ddots = [phi_ddots; (phi_dots(i,:)-phi_dots(i-1,:))/tstep];
    
        %calculate linear accelerations of COM bottom link
        botl_com_rads = [zeros(2,1) a/2*cos(thetas_curr)' a/2*sin(thetas_curr)'];

        botl_alphas = [theta_ddots(i-1,:)' zeros(2,2)];
        botl_omegas = [theta_dots(i,:)' zeros(2,2)];

%         cross(botl_alphas(1,:),botl_com_rads(1,:))+cross(botl_omegas(1,:),cross(botl_omegas(1,:),botl_com_rads(1,:))) ...
%             + cross(botl_alphas(2,:),botl_com_rads(2,:))+cross(botl_omegas(2,:),cross(botl_omegas(2,:),botl_com_rads(2,:))
        
        as_botl = [as_botl; cross(botl_alphas(1,:),botl_com_rads(1,:))+cross(botl_omegas(1,:),cross(botl_omegas(1,:),botl_com_rads(1,:))) ...
            cross(botl_alphas(2,:),botl_com_rads(2,:))+cross(botl_omegas(2,:),cross(botl_omegas(2,:),botl_com_rads(2,:)))];
        
        %calculate same way, just multiply radius by two to calculate at
        %joint, not link COM
        a_joint = [cross(botl_alphas(1,:),2*botl_com_rads(1,:))+cross(botl_omegas(1,:),cross(botl_omegas(1,:),2*botl_com_rads(1,:))) ...
            cross(botl_alphas(2,:),2*botl_com_rads(2,:))+cross(botl_omegas(2,:),cross(botl_omegas(2,:),2*botl_com_rads(2,:)))];

        topl_com_rads = [zeros(2,1) s/2*cos(phis_curr)' s/2*sin(phis_curr)'];
        topl_alphas = [phi_ddots(i-1,:)' zeros(2,2)];
        topl_omegas = [phi_dots(i,:)' zeros(2,2)];
        
        a_topl_without_joint = [cross(topl_alphas(1,:),topl_com_rads(1,:))+cross(topl_omegas(1,:),cross(topl_omegas(1,:),topl_com_rads(1,:))) ...
            cross(topl_alphas(2,:),topl_com_rads(2,:))+cross(topl_omegas(2,:),cross(topl_omegas(2,:),topl_com_rads(2,:)))];
        as_topl = [as_topl; a_topl_without_joint + a_joint];
        
    end
end

figure(2)
clf
hold on
plot(1:length(omegas)-1,as_botl(:,3),'DisplayName','bot')
plot(1:length(omegas)-1,as_topl(:,3),'DisplayName','top')
legend()
hold off