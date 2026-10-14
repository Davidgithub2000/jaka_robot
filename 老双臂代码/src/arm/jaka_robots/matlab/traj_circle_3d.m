function traj = traj_circle_3d(x,y,z)
h = z; %5; % 高度
r = 0.2;  %半径
pos = [x,y]; % 圆心位置
% pos = [0,0]; % 圆心位置
t=0:0.1:(2*pi);  % 圆滑性设置
t=[t,0];
Circle_X = pos(1)+r*sin(t);
Circle_Y = pos(2)+r*cos(t);
Circle_Z = h*ones(size(t));

traj = [Circle_X; Circle_Y; Circle_Z];
% plot3(Circle_X, Circle_Y, Circle_Z);
% axis([0 100 0 100]);
% axis square
% set(gca,'XTick',0:10:100);
% set(gca,'YTick',0:10:100);