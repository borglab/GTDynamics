%% Read trajectory from csv.
data = readtable('traj.csv');
time = data.t;
dt = time(2) - time(1);
Horizon = round(time(end) / dt) + 1;
x_traj = [data.theta'; data.dtheta'];

xo = [x_traj(1,1), x_traj(1,2)];
p_target = [pi; 0]; % Desired final state.
l1 = 0.3;           % Pendulum length.

close all;

%% Plot trajectory convergence information.
figure(1);
subplot(2,1,1)
hold on
plot(time,x_traj(1,:),'linewidth',4);
plot(time,p_target(1,1)*ones(1,Horizon),'red','linewidth',4)
title('Theta','fontsize',20);
xlabel('Time in sec','fontsize',20)
hold off;
grid;
subplot(2,1,2);
hold on;
plot(time,x_traj(2,:),'linewidth',4);
plot(time,p_target(2,1)*ones(1,Horizon),'red','linewidth',4)
title('Theta dot','fontsize',20);
xlabel('Time in sec','fontsize',20)
hold off;
grid;

%% Animate inverted pendulum trajectory.
pos1 = get(gcf,'Position');
set(gcf,'Position', pos1 - [pos1(3)/2,0,0,0])
fh = figure();
pos2 = get(gcf,'Position');
set(gcf,'Position', pos2 + [pos1(3)/2,0,0,0])

x = [0, l1 * sin(xo(1))];
y = [0, -l1 * cos(xo(1))];

h1 = plot(x, y, '-o', 'MarkerSize', 10, 'MarkerFaceColor', 'black',...
    'LineWidth', 2, 'Color', [0, 0, 0]);

t = text(-1.5 * l1, 1.5 * l1, 'Time: 0\Theta: 0');
axis([-2 * l1, 2 * l1, -2 * l1, 2 * l1])
axis square
alpha scaled

theta_dot_max = max(abs(x_traj(2, :)));

% Save video.
save_video = false;

if (save_video)
    video_filepath = 'ip_ddp';
    % This codec is available on all major OSes.
    myVideo = VideoWriter(video_filepath, 'Motion JPEG AVI'); %open video file
    myVideo.FrameRate = round(1 / dt);  %can adjust this, 5 - 10 works well
    myVideo.Quality = 99;
    open(myVideo)
end

for i = 1:length(x_traj)
    theta = x_traj(1, i);
    theta_dot = abs(x_traj(2, i));
    
    gc = 1 - i / length(x_traj);
    bc = theta_dot / theta_dot_max;
    rc = i / length(x_traj);
    
    t.String = sprintf('Time: %.2fs\nTheta: %.2frad', time(i), theta);
    h1.XData = [0, l1 * sin(theta)];
    h1.YData = [0, -l1 * cos(theta)];
    
    hold on;
    plot(h1.XData(2), h1.YData(2), '-mo', 'LineWidth',2, 'MarkerEdgeColor',...
        [rc, gc, bc], 'MarkerSize', 3, 'MarkerFaceColor', [rc, gc, bc]);
    hold off;
    
    drawnow()
    
    if (save_video)
        % Get the frame for the pendulum animation
        frame = getframe(fh); %get frame
        writeVideo(myVideo, frame.cdata);
    end
    
    pause(dt)
end

if (save_video)
    close(myVideo)
end