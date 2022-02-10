%Configuration analysis

%Question: If we set theta1 and theta2, is there a 
%unique combination of pz and beta that works for the configuration?

figure(1)
clf
a = 0.1;
s = 0.4;
r_base = 0.254;
theta1 = pi/8;
theta2 = -pi/8;

x1 = r_base+a*sin(theta1);
y1 = a*cos(theta1);

[xout,yout] = circcirc(0,a+s/2,r_base,x1,y1,s);

hold on
viscircles([0 a+s/2],r_base);
viscircles([x1 y1],s,'Color','g');
plot([r_base x1],[0 y1],'LineWidth',8);
hold off