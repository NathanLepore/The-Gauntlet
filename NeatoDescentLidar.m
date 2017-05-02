clf
%edit your ranges to display here.  important to not include the actual
%location of your object in this grid of points or it will give you
%infinities

step = .001;
x1 = 0;
y1 = 0;
d = 0.25;
% V = C;
 pub = rospublisher('/raw_vel');
 msg = rosmessage(pub);
sub = rossubscriber('/stable_scan');
BoBx = 111/intom;
BoBy = 55/intom;
heading1 = [1; 0];
run = 1;
lambda = 1;
totdist = 0;
V = [0;0];
while run == 1
pos = [x1; y1]
scan_message = receive(sub);
r_1 = scan_message.Ranges(1:end-1);
theta_1 = [0:359]';
[lidar_x, lidar_y] = ptocart(r_1, theta_1);
rottheta1 = theta;
Rmat1 = [cosd(rottheta1) -sind(rottheta1) 0; sind(rottheta1) cosd(rottheta1) 0; 0 0 1];
Tmat1 = [1 0 x1; 0 1 y1; 0 0 1];
positions = [lidar_x'; lidar_y'; ones(size(lidar_x))'];
Global = Rmat1*positions;
Global = Tmat1*Global;
plot(Global(1,:), Global(2,:), 'r*')
hold on
plot(pos(1), pos(2), 'b*')
quiver(pos(1), pos(2), Vfinal(1)/norm(Vfinal), Vfinal(2)/norm(Vfinal))
hold off
entries = size(Global);
for i = 1:entries(2)
   weight =  -1*gradcalc(pos(1), pos(2), Global(1, i), Global(2, i));
   V = V + weight;
end
Obj1 = gradcalc(pos(1), pos(2), BoBx, BoBy);
%Vfinal = V + 80*Obj1
Vfinal = V
costheta = dot(Vfinal,heading1)/(norm(Vfinal)*norm(heading1));
theta = acos(costheta);
heading1 = Vfinal
omega = (theta/lambda)*(d/2);
Vl = -omega;
Vr = omega;
msg.Data = [Vl, Vr];
send(pub, msg);
pause(lambda)
%Tell robot to drive at an increment
%distance = sqrt(xfinal.^2 + yfinal.^2)/10;
distance = 0.05;
velocity = distance/lambda;
Vl = 0.2;
Vr = 0.2;
msg.Data = [Vl, Vr];
send(pub, msg);
pause(distance/0.2)
BP = Vfinal - pos;
normBP = BP/norm(BP);
newpos = pos + normBP*distance
x1 = newpos(1);
y1 = newpos(2);
end
msg.Data = [0, 0];
send(pub, msg);