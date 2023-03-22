function output = plot_drone(x, y, dimension,Rmax)
%% Parameter
scale        = dimension;
L            = 0.35*scale; % vehicle length

%% Plot wheel and rear axle

set(gcf,'renderer','painters')
alpha = linspace(0, 2*pi);
th = 0:pi/50:2.1*pi;
% head = [L/6 * cos(th);L/6 * sin(th)]+[x;y];
% h = plot(head(1,:), head(2,:),'k');

r1 = [L/3 * cos(th);L/3 * sin(th)]+[x+L;y+L];
% h1 = plot(r1(1,:), r1(2,:),'k');
r2 = [L/3 * cos(th);L/3 * sin(th)]+[x-L;y-L];
% h2 = plot(r2(1,:), r2(2,:),'k');
r3 = [L/3 * cos(th);L/3 * sin(th)]+[x-L;y+L];
% h3 = plot(r3(1,:), r3(2,:),'k');
r4 = [L/3 * cos(th);L/3 * sin(th)]+[x+L;y-L];
% h4 = plot(r4(1,:), r4(2,:),'k');
r5 = [Rmax *cos(th); Rmax*sin(th)]+[x;y];
plot1 = line([x,x+L],[y,y+L],'Color',[1 0 0],'linewidth',2);
hold on
plot2 = line([x,x-L],[y,y-L],'Color',[1 0 0],'linewidth',2);
plot3 = line([x,x-L],[y,y+L],'Color',[1 0 0],'linewidth',2);
plot4 = line([x,x+L],[y,y-L],'Color',[1 0 0],'linewidth',2);
% p1= patch('Faces',[1:length(alpha)],'Vertices',[head(1,:)',head(2,:)'], 'FaceColor',DarkOrange,'EdgeColor',DarkOrange);
p2= patch('Faces',[1:length(alpha)],'Vertices',[r1(1,:)',r1(2,:)'], 'FaceColor',[1 0 0],'EdgeColor',[1 0 0]);
p3= patch('Faces',[1:length(alpha)],'Vertices',[r2(1,:)',r2(2,:)'], 'FaceColor',[1 0 0],'EdgeColor',[1 0 0]);
p4= patch('Faces',[1:length(alpha)],'Vertices',[r3(1,:)',r3(2,:)'], 'FaceColor',[1 0 0],'EdgeColor',[1 0 0]);
p5= patch('Faces',[1:length(alpha)],'Vertices',[r4(1,:)',r4(2,:)'], 'FaceColor',[1 0 0],'EdgeColor',[1 0 0]);
p6= patch('Faces',[1:length(alpha)],'Vertices',[r5(1,:)',r5(2,:)'], 'FaceColor',DarkOrange,'EdgeColor',DarkOrange,'FaceAlpha',.1);


% plot5 = line([x-L,x-L-L/4],[y-L,y-L+L*sqrt(3)/4],'Color','r','linewidth',2);
% plot6 = line([x-L,x-L+L/4],[y-L,y-L+L*sqrt(3)/4],'Color','r','linewidth',2);
% plot7 = line([x,x+L],[y,y-L],'Color',[1 0 0],'linewidth',2);
% plot8 = scatter(x+R(1,:)*[0;0],y+R(2,:)*[0.2;0],'ro','filled');
output = [plot1, plot2, plot3, plot4, p2, p3, p4, p5,p6];