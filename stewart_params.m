%% STEWART PLATFORM INPUTS
yaw = 0; %psi (z axis)
pitch = 0.1489; %theta (y axis) 15% grade
roll = 0; %phi (x axis)

T = [0 0 0.5]'; %desired translation of COM

%% DEFINE SERVO, PLATFORM CONNECT AND ARM GEOMETRY
base_angle = 19; %deg
base_rad = .635/2; %25 in diameter

plat_angle = base_angle;
plat_rad = base_rad;

%arm lengths
a = 0.1; %m
s = 0.5; %m

%% Calculations

%Rotation matrix from platform to inertial coords (inertial = R*platform)
Rp_b = [cos(yaw)*cos(pitch) cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll) cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll);
               sin(yaw)*cos(pitch) sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll) sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll);
               -sin(pitch) cos(pitch)*sin(roll) cos(pitch)*cos(roll)];
           
servo_pos_b = [base_rad*cosd(base_angle) -base_rad*sind(base_angle) 0; %bottom two servos
             base_rad*cosd(base_angle) base_rad*sind(base_angle) 0;
             -base_rad*sind(30 - base_angle) base_rad*cosd(30 - base_angle) 0; %top right servos
             -base_rad*sind(30 + base_angle) base_rad*cosd(30 + base_angle) 0;
             -base_rad*cosd(60 - base_angle) -base_rad*sind(60 - base_angle) 0; %top left servos
             -base_rad*cosd(60 + base_angle) -base_rad*sind(60 + base_angle) 0]';
servo_betas = [90-base_angle;
               -(90-base_angle);
               90-base_angle+120;
               -(90-base_angle)+120;
               90-base_angle+240;
               -(90-base_angle)+240]';
  
plat_pos_p = [plat_rad*cosd(plat_angle) -plat_rad*sind(plat_angle) 0; %bottom two servos
             plat_rad*cosd(plat_angle) plat_rad*sind(plat_angle) 0;
             -plat_rad*sind(30 - plat_angle) plat_rad*cosd(30 - plat_angle) 0; %top right servos
             -plat_rad*sind(30 + plat_angle) plat_rad*cosd(30 + plat_angle) 0;
             -plat_rad*cosd(60 - plat_angle) -plat_rad*sind(60 - plat_angle) 0; %top left servos
             -plat_rad*cosd(60 + plat_angle) -plat_rad*sind(60 + plat_angle) 0]';
plat_pos_b = T+Rp_b*plat_pos_p;
platform_betas = [90-plat_angle;
               -(90-plat_angle);
               90-plat_angle+120;
               -(90-plat_angle)+120;
               90-plat_angle+240;
               -(90-plat_angle)+240]';

%% Plot
figure(1)
clf
xlim([-0.5,0.5])
ylim([-0.5,0.5])
daspect([1 1 1])
hold on
for l_index = 1:size(servo_pos_b,2)
    curr_servo_pos_b = servo_pos_b(:,l_index); %servo pos vector
    curr_plat_pos_b = plat_pos_b(:,l_index);
    length_b = curr_plat_pos_b - curr_servo_pos_b;
    %Plot base servo positions in body frame
    plot3([0 curr_servo_pos_b(1)],[0 curr_servo_pos_b(2)],...
        [0 curr_servo_pos_b(3)],'Color','r')
    
    %Plot platform positions in body frame
    plot3([T(1) curr_plat_pos_b(1)],[T(2) curr_plat_pos_b(2)],...
        [T(3) curr_plat_pos_b(3)],'Color','b')
    
    %Plot lengths (start at servo, travel length)
    plot3([curr_servo_pos_b(1) curr_servo_pos_b(1)+length_b(1)],...
        [curr_servo_pos_b(2) curr_servo_pos_b(2)+length_b(2)],...
        [curr_servo_pos_b(3) curr_servo_pos_b(3)+length_b(3)],'Color','g')
    
    beta = servo_betas(l_index);
    L = norm(length_b)^2-(s^2-a^2);
    M = 2*a*(curr_plat_pos_b(3)-curr_servo_pos_b(3));
    N = 2*a*(cosd(beta)*(curr_plat_pos_b(1)-curr_servo_pos_b(1)) + ...
        sind(beta)*(curr_plat_pos_b(2)-curr_servo_pos_b(2)));

    alpha = asind(L/sqrt(M^2+N^2))-atand(N/M);
    
    
    
end