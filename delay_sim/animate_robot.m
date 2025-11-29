%% Simulation Results Animation
close
clear
clc

% Data Extraction
load results.mat

t = nominal.time;
q = nominal.signals(1).values;

% v = VideoWriter('nominal_control.avi', 'Motion JPEG AVI');
% v.FrameRate = 60;
% v.Quality = 95;
% open(v);

% Setting up the figure
figure('WindowState', 'maximized');
ax = show(robot, q(1,:), 'PreservePlot', false, 'Frames', 'off');
plot3(ref.x, ref.y, ref.z, '--', 'LineWidth', 3, 'Color', '#808080');

xlim([-0.5 0.5]);
ylim([-0.5 0.5]);
zlim([0 1]);

view(160, 40);
camzoom(0.9);
lighting gouraud;
camlight('headlight');

hold on;
grid on;

% Preallocating memory for the positions
eePosition = zeros(length(t), 3);

% Initializing the trajectory (empty)
eeTrajectory = plot3(NaN, NaN, NaN, 'LineWidth', 3, 'Color', '#0072BD');

for i = 1:length(t)
    % Showing the robot in the current frame
    show(robot, q(i,:), 'PreservePlot', false, 'FastUpdate', true,  ...
         'Frames', 'off', 'Parent', ax);
    
    % Computing the end effector position
    T = getTransform(robot, q(i,:), 'tool0');
    eePosition(i,:) = T(1:3,4)';
    
    % Update the end effector trajectory until the last point
    set(eeTrajectory, 'XData', eePosition(1:i,1), ...
                      'YData', eePosition(1:i,2), ...
                      'ZData', eePosition(1:i,3));

    % Updating the title
    title(ax, sprintf('Nominal Control [ t = %.2f s ]', t(i)), 'FontSize', 16);

    % Updating the figure
    drawnow
    
    % Capturing the frame
    % frame = getframe(gcf);
    % writeVideo(v, frame);
end

close(v);