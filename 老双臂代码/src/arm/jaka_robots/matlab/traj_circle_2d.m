x = 30;     % 圆心
y = 40;     % 圆心
r = 15;     % 半径
theta=0:0.01:2*pi ; % 圆滑性设置
Circle_X=x+r*cos(theta);
Circle_Y=y+r*sin(theta);
plot(Circle_X,Circle_Y,'Color','k','linewidth',1);
axis([0 100 0 100])
axis square
set(gca,'XTick',0:10:100);
set(gca,'YTick',0:10:100);
