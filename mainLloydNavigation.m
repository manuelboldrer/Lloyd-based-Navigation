%/*-----------------------------------------------------
%|      Manuel Boldrer, PhD                            |
%|      Department of Cognitive Robotics               |
%|      Delft University of Technology                 |
%|                                                     |
%|      email: m.boldrer@tudelft.nl                    |
%|      https://manuelboldrer.github.io/               |
%-----------------------------------------------------*/
%The algorithms implemented in this code were used to generate
%part of the simulation results in the following paper:

%[1] Boldrer, M., Palopoli, L., & Fontanelli, D. (2020, October).
% Lloyd-based Approach for Robots Navigation in Human-shared environments.
% In 2020 IEEE/RSJ International Conference on Intelligent Robots and
% Systems.


close all force
clearvars
% clc


%% Parameters
sizeA           = 1;          % robot size
dx              = 0.075;      % space discretization
tend            = 200 ;       % ending time of simulation
n_agents        = 2 ;         % number of robots
dt              = .033 ;      % time discretization
time            = 0:dt:tend;  % time vector
Rmax            = 1.5*sizeA ; % sensing radius
xlim            = [0 30] ;
ylim            = [0 30] ;
bbox            = [xlim(1),ylim(1);xlim(1),ylim(2);xlim(2),ylim(2);xlim(2),ylim(1)];
obs             = [];   % IDs of dynamic obstacles
n_obsD          = length(obs)  ; % number of dynamic obstacles
sizeO           = .5;            % dynamic obstacle dimension
R0              = 0.2 ;          % rho value (density function parameter)
b               = 0.2;
n_wp            = 1;             % number of waypoints

%% Initialize variables
x               = zeros(length(time),n_agents);
y               = zeros(length(time),n_agents);
theta           = zeros(length(time),n_agents);
v               = cell(length(time),1);
c               = cell(length(time),1);
cx              = zeros(length(time),n_agents);
cy              = zeros(length(time),n_agents);
index_prev_vec  = ones(n_agents,1);
XX1             = cell(length(time)-1,n_agents);
YY1             = cell(length(time)-1,n_agents);
x_obs_dyna      = zeros(length(time),n_obsD);
y_obs_dyna      = zeros(length(time),n_obsD);
VobsD           = zeros(length(time),n_obsD,2);
vel             = zeros(length(time)-1,n_agents);      % control v (linear velocity)
omega           = zeros(length(time)-1,n_agents);      % control omega (angular velocity)
x_dot           = zeros(length(time),n_agents);        % derivative of x
y_dot           = zeros(length(time),n_agents);        % derivative of y
theta_dot       = zeros(length(time),n_agents);        % derivative of theta

hh              = zeros(2,length(time),n_agents);      % heading of vehicle
hd              = zeros(2,length(time),n_agents);      % desired heading
goal            = zeros(n_agents,2,n_wp);
Wp              = cell(n_agents,1);

%% Flags



flag_obs        = 2; %change static obstacle configutation
video_flag      = 0; %to generate video
nonholo_flag    = 0;

manual_ics      =1 ; % initial conditions
manual_goal     =1 ; % waypoint
planning_flag   =1 ; % planner

%% Obstacle definition
switch flag_obs
    case 0
        x_obs        = [];
        y_obs        = [];
        obstacle_dim = [];
    case 1
        x_obs        = [10,20,0,6];
        y_obs        = [10,15,15,15];
        obstacle_dim = [10,10;10,5;4,1;4,1];
    case 2
        x_obs        = [0.1,10,10];
        y_obs        = [20,10,0];
        obstacle_dim = [20,5;20,5;3,5];
    case 3
        x_obs        = [2.5,12.5,22.5];
        y_obs        = [4.9,0.1,4.9];
        obstacle_dim = [5,25;5,25;5,25];
end

obstacle  = zeros(length(x_obs),4);
obstacleD = zeros(length(time),n_obsD,4);

for j = 1:length(x_obs)
    obstacle(j,:)  = [x_obs(j),y_obs(j),obstacle_dim(j,1),obstacle_dim(j,2)];
end
%% Define Initial Conditions
sec_numb = 1;
if manual_ics == 1
    fprintf('%d) Define initial conditions of the vehicles \n',sec_numb); sec_numb = sec_numb + 1;
    figure('Name', 'Povo','units','normalized','outerposition',[0 0 1 1]);
    hold on;
    title('Map','Interpreter','latex');
    grid on;
    xlabel('$x_{glob} [m] $','Interpreter','latex');
    ylabel('$y_{glob} [m] $','Interpreter','latex');
    axis equal;
    axis([-0 30 -0 30])
    obstacles = zeros(length(x_obs),4);
    for j = 1:length(x_obs)
        obstacles(j,:)  = [x_obs(j),y_obs(j),obstacle_dim(j,1),obstacle_dim(j,2)];
    end
    for j = 1:length(x_obs)
        rectangle('Position',obstacles(j,:),'FaceColor',[.8 .8 .8],'EdgeColor',[.7 .7 .7]);
    end

    for i = 1:n_agents
        display(['insert vehicle position of agent ', num2str(i)]);
        tmp    = ginput(1);
        x(1,i) = tmp(1);
        y(1,i) = tmp(2);
        vehicle_ref_point = plot(tmp(1), tmp(2), '.', 'markersize', 8, 'color', 'k');
        display(['insert second point for the heading of vehicle ', num2str(i)]);
        tmp2       = ginput(1);
        theta(1,i) = atan2(tmp2(2) - tmp(2), tmp2(1) - tmp(1));
        delete(vehicle_ref_point);
        plot_unicycle(x(1,i), y(1,i), theta(1,i), 'k',sizeA);
    end
    x_1     = x(1,:);
    y_1     = y(1,:);
    theta_1 = theta(1,:);
    save('x_1','x_1');
    save('y_1','y_1');
    save('theta_1','theta_1');
else
    load('x_1');
    load('y_1');
    load('theta_1');

    x(1,:)     = x_1;
    y(1,:)     = y_1;
    theta(1,:) = theta_1;
end
%% Define Final Conditions

if manual_goal == 1

    for i = 1:n_agents
        for w = 1:n_wp
            display(['insert goal for agent ', num2str(i)]);
            tmp         = ginput(1);
            goal(i,1,w) = tmp(1);
            goal(i,2,w) = tmp(2);
            plot(goal(i,1,w), goal(i,2,w) ,'x');

        end

    end

    save('goal','goal');
else
    load('goal');
end

%% Meshgrid

x_grid = xlim(1):dx:xlim(2);
y_grid = ylim(1):dx:ylim(2);
[X,Y]  = meshgrid(x_grid,y_grid);
XX     = reshape(X,[size(X,1)*size(X,2),1]);
YY     = reshape(Y,[size(Y,1)*size(Y,2),1]);

%% Definition of the density function
R    = zeros(length(time),n_agents);
R(1,:) = R0*ones(n_agents,1);
U_0 = 1;
wpp = ones(n_agents,1);
wp_input_vec        = zeros(2, length(time), n_agents);
wp_path             = cell(2,1);
index_wp_smart      = zeros(length(time), n_agents);
syms r_x r_y wpx wpy RR
r     = [r_x; r_y];
wp    = [wpx; wpy];
U     = U_0 * exp(-(norm(r-wp))/RR);
U     = matlabFunction(U, 'File','UUfun');

%%
load('ETHdata1');
obstacle1   = [obstacle];
epsi2       = sizeA*0.35/2;
obstacle2   = [obstacle1(:,1)-epsi2,obstacle1(:,2)-epsi2,obstacle1(:,3)+2*epsi2,obstacle1(:,4)+2*epsi2];
xi          = sizeA*0.35+0.1;
%% Iteratively Apply Lloyd's Algorithm
h_waitbar = waitbar(0, 'Simulating...');
for kk = 1:length(time)-1
    for j = 1:n_agents
        if planning_flag == 1
            if kk == 1  % is not necessary to compute every step
                Wp{j}    = RRT1(x(1,j),y(1,j),goal(j,1,1),goal(j,2,1),x_obs-xi,y_obs-xi,obstacle_dim+xi);
                Wp{j}    = [goal(j,:,1);Wp{j}];
                Wp{j}    = flip(Wp{j}); %waypoints computed from the RRTstar
                save('Wp','Wp');
            end
        else
            load('Wp');
        end

        %% Compute the waypoint path
        [wp_input_vec(:,kk,j), index_prev_vec(j), wp_path{j}, index_wp_smart(kk,j),~] = generate_wp_path(Wp{j}', x(kk,j), y(kk,j), index_prev_vec(j),kk,obstacle2);
        if kk >1
            R(kk,j) = R0;
        end
    end
    %% Dynamic obstacles
for p =1 : n_obsD
    x_obs_dyna(1,p)          = 30*rand;
    y_obs_dyna(1,p)          = 30*rand;
end

    for q = 1:n_obsD
        VobsD(kk,q,1) = VX(kk,q)/2;
        VobsD(kk,q,2) = VY(kk,q)/2;
        x_obs_dyna(kk+1,q) =  x_obs_dyna(kk,q) + dt*VobsD(kk,q,1);
        y_obs_dyna(kk+1,q) =  y_obs_dyna(kk,q) + dt*VobsD(kk,q,2);
    end
    for jj = 1:length(x_obs)
        obstacle(jj,:)  = [x_obs(jj),y_obs(jj),obstacle_dim(jj,1),obstacle_dim(jj,2)];
    end
    for qq = 1:n_obsD
        obstacleD(kk,qq,:) =  [x_obs_dyna(kk,qq),y_obs_dyna(kk,qq),sizeO,sizeO];
    end
    [v{kk},c{kk}] = VoronoiBounded([x(kk,:)';x_obs_dyna(kk,:)'],[y(kk,:)';y_obs_dyna(kk,:)'], bbox);

    %% Distributed Control 
    for j = 1:n_agents %calculate the centroid of each cell
        obstacle1 = [obstacle];
        obstacle2   = [obstacle1(:,1),obstacle1(:,2),obstacle1(:,3),obstacle1(:,4)];
        [Xvis,Yvis]  = visibilitypoints2(x(kk,:),y(kk,:),obstacle2,obstacleD(kk,:,:), VobsD(kk,:,:),1.5,xlim,ylim,Rmax,epsi2,j,sizeA*0.35,inf,theta(kk,:),b);
        [in]       = inpolygon(Xvis,Yvis,v{kk}(c{kk}{j},1) ,v{kk}(c{kk}{j},2));
        X1         = Xvis(in);
        Y1         = Yvis(in);
        k          = boundary(X1',Y1',1);
        [in1,on]   = inpolygon(XX,YY,X1(k),Y1(k));
        XX1{kk,j}  = XX(in1);
        YY1{kk,j}  = YY(in1);  % (XX1,YY1) are the points belonging to the voronoi visible set.
        mass     = sum( UUfun(R(kk,j),XX1{kk,j},YY1{kk,j},wp_input_vec(1,kk,j),wp_input_vec(2,kk,j)) );
        CX          = 0;
        CY          = 0;

        for p = 1:length(XX1{kk,j})
            CX = CX + XX1{kk,j}(p)*(UUfun(R(kk,j),XX1{kk,j}(p),YY1{kk,j}(p),wp_input_vec(1,kk,j),wp_input_vec(2,kk,j)));
            CY = CY + YY1{kk,j}(p)*(UUfun(R(kk,j),XX1{kk,j}(p),YY1{kk,j}(p),wp_input_vec(1,kk,j),wp_input_vec(2,kk,j)));
        end
        cx(kk,j)   = CX/(mass);
        cy(kk,j)   = CY/(mass);  %(cx,cy) is the centroid position

        if nonholo_flag ==0 %nonholonomic constraints not active
            x_dot(kk,j) = 3*(cx(kk,j)-x(kk,j));
            y_dot(kk,j) = 3*(cy(kk,j)-y(kk,j));
            x(kk+1,j)  = x(kk,j)     + x_dot(kk,j)*dt ;
            y(kk+1,j)  = y(kk,j)     + y_dot(kk,j)*dt ;
        else %active nonholonomic constraints
            %% Controller
            hd(:,kk,j)     = [cx(kk,j)-x(kk,j)' cy(kk,j)-y(kk,j)]/norm([cx(kk,j)-x(kk,j) cy(kk,j)-y(kk,j)]);
            hh(:,kk,j)     = [cos(theta(kk,j)) ; sin(theta(kk,j))];
            if kk >1
                [vel(kk,:),omega(kk,:)] = controller(theta(kk,:),  hd(:,kk,:), vel(kk-1,:), dt);
            end
            for j = 1:n_agents
                if norm([x(kk,j),y(kk,j)]-[goal(j,1),goal(j,2)]) < .2
                    omega(kk,j) = 0;
                end
                x_dot(kk,j)      = vel(kk,j)*cos(theta(kk,j));
                y_dot(kk,j)      = vel(kk,j)*sin(theta(kk,j));
                theta_dot(kk,j)  = omega(kk,j);
                x(kk+1,j)        = x(kk,j)     + vel(kk,j) * cos(theta(kk,j)) * dt;
                y(kk+1,j)        = y(kk,j)     + vel(kk,j) * sin(theta(kk,j)) * dt;
                theta(kk+1,j)    = theta(kk,j) + omega(kk,j) * dt;
            end
        end
    end
    h_waitbar = waitbar(kk/(length(time)-1));

end
close(h_waitbar);

%% Post-processing
video_flag=1;
obstacles            = zeros(length(x_obs),4);
drones               = gobjects(n_agents,9); % initialize array of plots
plot_obj             = gobjects(n_agents,7); % initialize array of plots
verCellHandle        = gobjects(length(time),n_agents);
cellColors           = cool(n_agents);
rect                 = gobjects(1,length(x_obs));
rectD                = gobjects(n_obsD,13);
txt                  = gobjects(n_obsD,1);
rand                 = gobjects(length(time),n_agents);
centr                = gobjects(length(time),n_agents);
circ                 = gobjects(length(time),n_agents);
circ1                 = gobjects(length(time),n_agents);
arrow                = gobjects(n_agents,n_obsD);
arrow1               = gobjects(n_agents,1);
delaunay             = gobjects(length(time),1);
cone_obj             = gobjects(1,4); % initialize array of plots
cir                  = gobjects(length(time),1);
figure('Name', 'Animation','units','normalized','outerposition',[0 0 1 1]);
axis([xlim ylim])
grid on
hold on
epsiX = -sizeA*0.35;
obstacleX   = [obstacle1(:,1)-epsiX,obstacle1(:,2)-epsiX,obstacle1(:,3)+2*epsiX,obstacle1(:,4)+2*epsiX];
for j = 1:length(x_obs)
    rect(j)                 = rectangle('Position',obstacleX(j,:),'FaceColor',[.8 .8 .8],'EdgeColor',[.7 .7 .7]);
end
for kk = 1:1:length(time)-1
    for w = 1:n_obsD
        rectD(w,:) = human(obstacleD(kk,w,1),obstacleD(kk,w,2),VobsD(kk,w,1),VobsD(kk,w,2),sizeO);
    end
    for j = [1:n_agents]
        if nonholo_flag == 0
            %drones(j,:)    = plot_drone(x(kk,j),y(kk,j),sizeA,Rmax);
        else
            plot_obj(j,:)  = plot_unicycle(x(kk,j), y(kk,j), theta(kk,j), 'k', sizeA);
        end

        bound = boundary(XX1{kk,j},YY1{kk,j},1);
        verCellHandle(kk,j)  = patch(x(kk,j),y(kk,j),[1 1 1],'FaceAlpha',.1,'EdgeColor',DodgerBlue); % use color i  -- no robot assigned yet
        set(verCellHandle(kk,j), 'XData',XX1{kk,j}(bound),'YData',YY1{kk,j}(bound));
        centr(kk,j)             = plot(cx(kk,j),cy(kk,j),'bx');
        circ1(j)     = circle(x(kk,j),y(kk,j),.35*sizeA,DarkOrange);
        wpplot(j)    = plot(wp_input_vec(1,kk,j),wp_input_vec(2,kk,j),'rx');
    end

    if video_flag == 1
        drawnow
        F(kk) = getframe(gcf); %#ok<SAGROW>
    else
        drawnow
    end
    if kk < length(time)-1
        delete(cone_obj)
        delete(drones)
        delete(verCellHandle)
        delete(centr)
        delete(rand)
        delete(arrow)
        delete(arrow1)
        delete(rectD)
        delete(plot_obj)
        delete(circ);
        delete(circ1);
        delete(wpplot);
        delete(delaunay);
        delete(txt);

    end
end

%% Create a file .avi of simulation results
if video_flag == 1
    video = VideoWriter('DATA_20ag.avi','Motion JPEG AVI');
    video.Quality = 20;
    video.FrameRate = 1/dt;
    open(video)
    writeVideo(video,F(1:864))
    close(video)
end


