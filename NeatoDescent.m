clf
%edit your ranges to display here.  important to not include the actual
%location of your object in this grid of points or it will give you
%infinities

[px,py]=meshgrid(0:1:5,0:1:5);
[xlim,ylim] = size(px);
V = zeros(xlim, ylim);
% for i=1:xlim
%     for j=1:ylim
% %this is the equation and integral with ranges for a specific object:  you
% %should be able to figure out what this is and edit appropriately to get
% %what you want
% dV1 = @(x)  1./((px(i,j)-x).^2 + (py(i,j)-0)^2);
% dV2 = @(x)  1./((px(i,j)- 0).^2 + (py(i,j)-x).^2);
% V1(i,j) = integral(dV1,2,4);
% V2(i,j) = integral(dV2,2,4);
%     end
% end
intom = 39.37;
BoBx = 111/intom;
BoBy = 55/intom;
DoDx = 32/ intom;
DoDy = 32/intom;
SoSx = 78/intom;
SoSy = 6/intom;
FoFx = 62/intom; 
FoFy = 54/intom;
objects = [BoBx BoBy; DoDx DoDy; SoSx SoSy; FoFx FoFy];
% C = 3*(1./sqrt(((px - xfinal).^2+(py - yfinal).^2)));
% C1 =@(x,y) 3*(1./sqrt(((x - xfinal).^2+(y - yfinal).^2)));
step = .001;
x1 = 0;
y1 = 0;
d = 0.25;
% V = C;
 pub = rospublisher('/raw_vel');
 msg = rosmessage(pub);
heading1 = [1; 0];
run = 1;
lambda = 1;
totdist = 0;
while run == 1
pos = [x1; y1]
Obj1 = gradcalc(pos(1), pos(2), BoBx, BoBy);
Obj2 = -gradcalc(pos(1), pos(2), SoSx, SoSy);
Obj3 = -gradcalc(pos(1), pos(2), FoFx, FoFy);
Obj4 = -gradcalc(pos(1), pos(2), DoDx, DoDy);

V = 50*Obj1 + Obj2 + Obj3 + Obj4
plot(objects(:,1), objects(:, 2), 'r*')
hold on
plot(pos(1), pos(2), 'b*')
axis equal
quiver(pos(1), pos(2),V(1)/norm(V), V(2)/norm(V))
% streamslice(px, py, V(1), V(2))


costheta = dot(V,heading1)/(norm(V)*norm(heading1));
theta = acos(costheta)
heading1 = V
omega = (theta/lambda)*(d/2);
Vl = -omega
Vr = omega
msg.Data = [Vl, Vr];
send(pub, msg);
pause(lambda)
%Tell robot to drive at an increment
%distance = sqrt(xfinal.^2 + yfinal.^2)/10;
distance = 0.1;
velocity = distance/lambda;
Vl = 0.2
Vr = 0.2
msg.Data = [Vl, Vr];
send(pub, msg);
pause(distance/0.2)
BP = V - pos
normBP = BP/norm(BP)
newpos = pos + normBP*distance
x1 = newpos(1);
y1 = newpos(2);
% totdist = totdist + distance;
% if totdist >= sqrt(xfinal.^2 + yfinal.^2)
%     run = 2;
% end
% hold on
% plot(x1, y1, 'r*')
end
msg.Data = [0, 0];
send(pub, msg);