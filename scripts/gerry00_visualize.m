%%  gerry00_visualize.m
%   Visualizes the json output of gerry00_ForwardDynamicsPrismatic.cpp

clear;
animated = 0;

%% import json
data = fileread('../build/scripts/gerry00_result.json');
data = jsondecode(data);

%% parse json
values = data{1};
graph = data{2};

%% get poses
allt = 0:.1:1.5;
allT = [];
allQ = [];
allV = [];
for value = values'
    vVals = sscanf(value.name, '%c%d_%d');
    if vVals(1) == 'p'
        T = parsePose(value.value);
        allT(vVals(2)+1, vVals(3)+1, 1:4, 1:4) = T;
    elseif vVals(1) == 'q'
        allQ(vVals(2), vVals(3)+1) = str2double(value.value);
    elseif vVals(1) == 'v'
        allV(vVals(2), vVals(3)+1) = str2double(value.value);
    end
end


%% sdf joint stuffs
% this is so hacky.  TODO: upgrade to matlab 2019b and use SDF importer
l0(:,:,1) = [0, 0, 0, 0;...
             0, 0, 0, 0;...
             -0.1, -0.3, -0.3, -0.3;...
             1, 1, 1, 1];
l0(:,:,2) = [0, 0, 0, 0;...
             0, 0, 0, 0;...
             0.1, 0.3, 0.3, 0.3;...
             1, 1, 1, 1];

%% plot
linkcolors = {'r-', 'm-', 'm-', 'b-'};
% figure 1
figure(1);clf;
set(gcf,'Position',[100 100 800 600])
imgs = plotRobot(allT, allQ, allV, l0, linkcolors, animated);

% figure 2
jointcolors = {'r-', 'm-.', 'b-'};
figure(2);clf;
set(gcf,'Position',[900 100 800 600])
ax(1) = subplot(2,1,1);
for joint = 1:3
    if joint == 2
        yyaxis right
    else
        yyaxis left
    end
    plot(allt, allQ(joint, :), jointcolors{joint}, ...
         'DisplayName', sprintf('Joint %d', joint));
    hold on
end
xlabel('t (s)');
yyaxis left; ylabel('q (rad)'); yyaxis right; ylabel('q (m)'); ylim([0, Inf])
legend show
grid on;
ax(2) = subplot(2,1,2);
for joint = 1:3
    if joint == 2
        yyaxis right
    else
        yyaxis left
    end
    plot(allt, allV(joint, :), jointcolors{joint}, ...
         'DisplayName', sprintf('Joint %d', joint));
    hold on
end
xlabel('t (s)');
yyaxis left; ylabel('v (rad/s)'); yyaxis right; ylabel('v (m/s)'); ylim([0, Inf])
legend show
grid on;
linkaxes(ax, 'x');
sgtitle('Joint Angles and Velocities', 'FontSize', 36);

%% save figs
if animated
    writeToGif(imgs, 'gerry00_rpr_viz.gif');
else
    figure(1); print -dpng gerry00_rpr_viz
end
figure(2); print -dpng gerry00_rpr_joints

%% utils
function [T, R, t] = parsePose(str)
    % str = `Translation: [x, y, z]' Rotation rpy: [r, p, y]`
    pVals = sscanf(str, ...
        'Translation: [%f, %f, %f]'' Rotation rpy: [%f, %f, %f]');
    t = pVals(1:3);
    R = eul2rotm(pVals(4:6)', 'XYZ'); % yaw, pitch, roll, TODO: test
    T = zeros(4,4);
    T(1:3, 1:3) = R;
    T(1:3, 4) = t;
    T(4, 4) = 1;
end

function [imgs] = plotRobot(allT, allQ, allV, l0, linkcolors, animated)
    imgs = uint8([]);
    ylim([0, 2.5]);
    axis equal
    xlim([-.5, 2.0]);
    ax = gca;
    ax.Units = 'pixels';
    pos = ax.Position;
    marg = 50;
    pos(3) = pos(4);
    rect = [-marg, -marg, pos(3)+2*marg, pos(4)+2*marg];

    for t = 1:size(allT, 2)
        for link = 1:4
            l1 = l0(:, link, :);
            if (link == 2)
                l1(3, 2) = l1(3, 2) + allQ(link, t);
            end
            p(:, 1) = squeeze(allT(link, t, :, :)) * l1(:, 1);
            p(:, 2) = squeeze(allT(link, t, :, :)) * l1(:, 2);
            plot(p(1, :), p(3, :), linkcolors{link}, 'LineWidth', 5);
            hold on;
            if link < 4
                plot(p(1, 2), p(3, 2), 'k.', 'MarkerSize', 40);
            end
        end
        grid on;
        ylim([0, 2.5]);
        axis equal
        xlim([-.5, 2.0]);
        xlabel('x (m)'); ylabel('z (m)');
        title('Simple RPR Robot Motion', 'FontSize', 36);
        text(0.5, 0.5, ...
                 {'$q_0 = [0.0; 0.0; 0.0]$',...
                  '$\dot{q}_0 = [0.3; 0.1; 0.0]$',...
                  '~$\tau = [0.0; 0.0; 0.0]$'}, ...
              'interpreter' ,'latex',...
              'FontSize', 24);
        if animated
            drawnow
            frame = getframe(gca, rect); 
            imgs(t, :, :, :) = frame2im(frame); 
            hold off
        end
    end
end

function [] = writeToGif(imgs, filename)
    dt = 0.1;
    for imnum = 1:size(imgs, 1)
        [imind,cm] = rgb2ind(squeeze(imgs(imnum, :, :, :)), 256); 
        % Write to the GIF File 
        if imnum == 1 
          imwrite(imind,cm,filename,'gif', 'Loopcount',inf, 'DelayTime', dt); 
        else 
          imwrite(imind,cm,filename,'gif','WriteMode','append', 'DelayTime', dt); 
        end 
    end
end