%% Read trajectory from csv.
clear all; close all;
data = readtable('traj.csv');
time = data.t;
dt = time(2) - time(1);
Horizon = round(time(end) / dt) + 1;
x_traj = [data.x'; data.xdot'; data.theta'; data.thetadot'];

xo = [0; 0; 0; 0];
p_target = [1; 0; pi; 0];

l = 0.3;

%% Plot Convergence Information.
figure('Renderer', 'painters', 'Position', [10 10 1000 600], ...
       'NumberTitle', 'off', 'Name', 'Cart Pole Convergence')

subplot(2,2,1)
hold on
plot(time,x_traj(1,:),'linewidth',4);  
plot(time,p_target(1,1)*ones(1,Horizon),'red','linewidth',4)
title('$X$','fontsize',20, 'Interpreter', 'latex');
xlabel('$t$','fontsize',14, 'Interpreter', 'latex');
hold off;
grid;


subplot(2,2,2);
hold on;
plot(time,x_traj(2,:),'linewidth',4); 
plot(time,p_target(2,1)*ones(1,Horizon),'red','linewidth',4)
title('$\dot{X}$','fontsize',20, 'Interpreter', 'latex');
xlabel('$t$','fontsize',14, 'Interpreter', 'latex');
hold off;
grid;

subplot(2,2,3);
hold on;
plot(time,x_traj(3,:),'linewidth',4); 
plot(time,p_target(3,1)*ones(1,Horizon),'red','linewidth',4)
title('$\theta$','fontsize',20, 'Interpreter', 'latex');
xlabel('$t$','fontsize',14, 'Interpreter', 'latex');
hold off;
grid;

subplot(2,2,4);
hold on;
plot(time,x_traj(4,:),'linewidth',4); 
plot(time,p_target(4,1)*ones(1,Horizon),'red','linewidth',4)
title('$\dot{\theta}$','fontsize',20, 'Interpreter', 'latex');
xlabel('$t$','fontsize',14, 'Interpreter', 'latex');
hold off;
grid;

pos1 = get(gcf,'Position'); % get position of Figure(1) 

%% Animate Cart Pole Trajectory.
fh = figure('Renderer', 'painters', 'Position', [10 10 2560 1600], ...
       'NumberTitle', 'off', 'Name', 'Cart Pole Animation');

pos2 = get(gcf,'Position');  % get position of Figure(2) 
set(gcf,'Position', pos2 + [pos1(3)/2,0,0,0]) % Shift position of Figure(2)

cart_w = 0.2;
cart_h = 0.1;

min_x = min(x_traj(1, :));
max_x = max(x_traj(1, :));

lims = [min_x - 2 * l, max_x + 2 * l, - 1.2 * l, 1.5 * l];

h1 = rectangle('Position', [xo(1) - cart_w / 2, 0, cart_w, cart_h],...
               'Curvature', 0.2, 'FaceColor',[0 .5 .5]);

x = [xo(1), xo(1) + l * sin(xo(3))];
y = [cart_h / 2, cart_h / 2 - l * cos(xo(3))];

hold on;
h2 = plot(x, y, '-o', 'MarkerSize', 10, 'MarkerFaceColor', 'black',...
    'LineWidth', 2, 'Color', [0, 0, 0]); 
plot([lims(1), lims(2)], [0, 0], 'k');

rectangle('Position', [p_target(1) - cart_w / 2, 0, cart_w, cart_h],...
          'LineStyle', '--', 'Curvature', 0.2);
plot([p_target(1), p_target(1) + l * sin(p_target(3))],...
        [cart_h / 2, cart_h / 2 - l * cos(p_target(3))],...
        'k--', 'LineWidth', 4);    

hold off;

axis(lims);
axis square
axis equal

x_dot_max = max(abs(x_traj(2, :)));

% Save video.
save_video = false;

if (save_video)
    video_filepath = 'gtdynamics_cart_pole_to';
    myVideo = VideoWriter(video_filepath, 'MPEG-4');
    myVideo.FrameRate = round(1 / dt);
    myVideo.Quality = 99;
    open(myVideo)
end

for i = 1:length(x_traj)
  x = x_traj(1, i);
  x_dot_abs = abs(x_traj(2, i));
  theta = x_traj(3, i);
  
  gc = x_dot_abs / x_dot_max;
  bc = 1. - x_dot_abs / x_dot_max;
  rc = i / length(x_traj) ;
  
  h1.Position = [x - cart_w / 2, 0, cart_w, cart_h];
  hold on;
  h2.XData = [x, x + l * sin(theta)];
  h2.YData = [cart_h / 2, cart_h / 2 - l * cos(theta)];
  plot(h2.XData(2), h2.YData(2), '-mo', 'LineWidth',2,...
      'MarkerEdgeColor',[rc, gc, bc], 'MarkerSize', 3,...
      'MarkerFaceColor', [rc, gc, bc]);
  axis(lims);
   
  hold off;  
  
  drawnow()  
  
  if (save_video)
      frame = getframe(gcf); %get frame
      writeVideo(myVideo, frame);
  end
  
  pause(dt)
end

if (save_video)
    close(myVideo)
end