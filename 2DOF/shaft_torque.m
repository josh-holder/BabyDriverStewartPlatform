m = 25; %kg
g = 9.81; %m/s^2
a = 0.1; %m
s = 0.5; %m

max_T = 0;
theta_at_max = 0;
for theta = 0:0.1:90
    phi = 90-asind(a/s*sind(90-theta));
    T = m*g/4/tand(phi)*a*sind(theta) + m*g/4*a*cosd(theta);
    
    if T > max_T
        max_T = T;
        theta_at_max = theta;
    end
end

max_T
theta_at_max
