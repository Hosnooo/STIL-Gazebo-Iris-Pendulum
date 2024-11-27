clc; clear; close all;

%% GCF Options
sz = [200, 100, 400, 600];
fsz = 12;
set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');

% Add default y-axis limit relaxation and tick increment options
set(0, 'DefaultAxesYLimMode', 'manual'); % Ensure y-limits are user-controlled
set(0, 'DefaultAxesYLim', [-5, 5]); % Initial relaxed limits
set(0, 'DefaultAxesYTickMode', 'manual'); % Auto-generate more y-ticks
set(0, 'DefaultAxesYTick', linspace(-6, 6, 9));

%% Load Data
dataGazebo = readCSVFileMatrix('Data\log_sls.csv');
dataPX4 = readCSVFileMatrix('Data\log_px4_sls.csv');
dataMATLAB1 = readCSVFileMatrix('Data\setpoint1_data.csv');
dataMATLAB2 = readCSVFileMatrix('Data\setpoint2_data.csv');
dataMATLAB3 = readCSVFileMatrix('Data\setpoint3_data.csv');
dataMATLABSetpoints = [dataMATLAB1; dataMATLAB2; dataMATLAB3];
dataMATLABEight = readCSVFileMatrix('Data\trajectory_data.csv');

sp = [1  -1 -2;
     -2  -1 -4;
      1   2 -3];

% Generate constant trajectories for the setpoints
dt = 0.02;
t_const = 0:dt:15; % Time vector for each constant setpoint

trajectory = []; % Initialize full trajectory

% Add constant trajectories for the first three setpoints
for i = 1:3
    prx = sp(i, 1) * ones(length(t_const), 1); % Constant X position
    pry = sp(i, 2) * ones(length(t_const), 1); % Constant Y position
    prz = sp(i, 3) * ones(length(t_const), 1); % Constant Z position
    setpointTrajectory = [t_const'+(15+dt)*(i-1), prx, pry, prz];
    trajectory = [trajectory; setpointTrajectory]; % Append to full trajectory
end

dataMATLABSetpoints = [trajectory dataMATLABSetpoints(:,2:end)];

%% Trim Trajectory of Interest

[idxMatrixGazebo, combinedIdxsGazebo] = getTrajectoryIndicesMatrix(dataGazebo, sp);
[idxMatrixPX4, combinedIdxsPX4] = getTrajectoryIndicesMatrix(dataPX4, sp);

[idxGazebo, ~] = getTrajectoryIndicesMatrix(dataGazebo, [0 0 -1.5], 'late');
[idxPX4, ~] = getTrajectoryIndicesMatrix(dataPX4, [0 0 -1.5], 'late');

idxMatrixGazebo = [idxMatrixGazebo; idxMatrixGazebo(3,2)+1 idxGazebo(1)];
idxMatrixPX4 = [idxMatrixPX4; idxMatrixPX4(3,2)+1 idxPX4(1)-1];

eightIdxsGazebo = idxMatrixGazebo(4,1):idxMatrixGazebo(4,2)-1;
eightIdxsPX4 = idxMatrixPX4(4,1):idxMatrixPX4(4,2)-1;

% eightIdxsGazebo = eightIdxsGazebo(1:floor(2*length(eightIdxsGazebo)/3));
% eightIdxsPX4 = eightIdxsPX4(1:floor(2*length(eightIdxsPX4)/3));

dataGazeboSetpoints = dataGazebo(combinedIdxsGazebo, :);
dataPX4Setpoints = dataPX4(combinedIdxsPX4, :);
dataGazeboEight = dataGazebo(eightIdxsGazebo, :);
dataPX4Eight = dataPX4(eightIdxsPX4, :);

tGazeboSetpoints = dataGazeboSetpoints(:, 1) - dataGazeboSetpoints(1, 1);
tPX4Setpoints = dataPX4Setpoints(:, 1) - dataPX4Setpoints(1, 1);
tMATLABSetpoints = dataMATLABSetpoints(:,1);
tGazeboEight = dataGazeboEight(:, 1) - dataGazeboEight(1, 1);
tPX4Eight = dataPX4Eight(:, 1) - dataPX4Eight(1, 1);
tMATLABEight = dataMATLABEight(:,1);

posdesGazebo = dataGazeboSetpoints(:, 2:4);
posdesPX4 = dataPX4Setpoints(:, 2:4);
posdesMATLAB = dataMATLABSetpoints(:,2:4);

alphaGazebo = dataGazeboSetpoints(:, 14); betaGazebo = dataGazeboSetpoints(:, 15);
alphaPX4 = dataPX4Setpoints(:, 11); betaPX4 = dataPX4Setpoints(:, 12);

posGazebo = dataGazeboSetpoints(:, 5:7) +...
    1.1*[sin(betaGazebo), -sin(alphaGazebo).*cos(betaGazebo), cos(alphaGazebo).*cos(betaGazebo)];
posPX4 = dataPX4Setpoints(:, 5:7) +...
    1*[sin(betaPX4), -sin(alphaPX4).*cos(betaPX4), cos(alphaPX4).*cos(betaPX4)];
posMATLAB = dataMATLABSetpoints(:,8:10);

eposGazebo = posdesGazebo - posGazebo;
eposPX4 = posdesPX4 - posPX4;
eposMATLAB = posdesMATLAB - posMATLAB;

%% Plotting Ref Setpoints
labelNames = {'$X_{ref}$', '$Y_{ref}$', '$Z_{ref}$'};
figHandle = figure;
set(gcf, 'Position', sz);

for i = 1:3
    subplot(3,1,i);
    plot(tGazeboSetpoints, posdesGazebo(:, i), '-', 'LineWidth', 1.5, 'DisplayName', 'Gazebo');
    hold on;
    plot(tPX4Setpoints, posdesPX4(:, i), ':', 'LineWidth', 2, 'DisplayName', 'PX4');
    plot(tMATLABSetpoints, posdesMATLAB(:, i), '--', 'LineWidth', 2, 'DisplayName', 'Matlab');
    ylabel(labelNames{i}, 'FontSize', fsz);
    grid on; box on;
    if i == 1
        legend('show', 'Location', 'north', 'Orientation', 'horizontal', 'FontSize', fsz-3);
    end
end
xlabel('t (s)', 'FontSize', fsz);
saveas(figHandle, 'Figures/reference_setpoints', 'svg');
saveFigureAsPDF(figHandle,'Figures/refernce_setpoints.pdf');

%% Plotting Trajectory Setpoints
labelNames = {'X', 'Y', 'Z'};
figHandle = figure;
set(gcf, 'Position', sz);

for i = 1:3
    subplot(3,1,i);
    plot(tGazeboSetpoints, posdesGazebo(:, i), '-', 'LineWidth', 1.5, 'DisplayName', 'Ref');
    hold on;
    plot(tGazeboSetpoints, posGazebo(:, i), '-.', 'LineWidth', 1.5, 'DisplayName', 'Gazebo');
    plot(tPX4Setpoints, posPX4(:, i), ':', 'LineWidth', 2, 'DisplayName', 'PX4');
    plot(tMATLABSetpoints, posMATLAB(:, i), '--', 'LineWidth', 2, 'DisplayName', 'Matlab');
    ylabel(labelNames{i}, 'FontSize', fsz);
    grid on; box on;
    if i == 1
        legend('show', 'Location', 'north', 'Orientation', 'horizontal', 'FontSize', fsz-3);
    end
end
xlabel('t (s)', 'FontSize', fsz);
saveas(figHandle, 'Figures/trajectory_setpoints', 'svg');
saveFigureAsPDF(figHandle, 'Figures/trajectory_setpoints.pdf');

%% Plotting Error Setpoints
figHandle = figure;
set(gcf, 'Position', sz);

for i = 1:3
    subplot(3,1,i);
    plot(tGazeboSetpoints, eposGazebo(:, i), '-.', 'LineWidth', 1.5, 'DisplayName', 'Gazebo');
    hold on;
    plot(tPX4Setpoints, eposPX4(:, i), ':', 'LineWidth', 2, 'DisplayName', 'PX4');
    plot(tMATLABSetpoints, eposMATLAB(:, i), '--', 'LineWidth', 2, 'DisplayName', 'Matlab');
    ylabel(labelNames{i}, 'FontSize', fsz);
    grid on; box on;
    if i == 1
        legend('show', 'Location', 'northwest', 'Orientation', 'horizontal', 'FontSize', fsz-3);
    end
end
xlabel('t (s)', 'FontSize', fsz);
saveas(figHandle, 'Figures/error_setpoints', 'svg');
saveFigureAsPDF(figHandle, 'Figures/error_setpoints.pdf');

%% 8 Trajectory

alphaGazebo = dataGazeboEight(:, 14); betaGazebo = dataGazeboEight(:, 15);
alphaPX4 = dataPX4Eight(:, 11); betaPX4 = dataPX4Eight(:, 12);

posdesGazebo = dataGazeboEight(:, 2:4);
posdesPX4 = dataPX4Eight(:, 2:4);
posdesMATLAB = dataMATLABEight(:, 2:4);

posGazebo = dataGazeboEight(:, 5:7) +...
    1.1*[sin(betaGazebo), -sin(alphaGazebo).*cos(betaGazebo), cos(alphaGazebo).*cos(betaGazebo)];
posPX4 = dataPX4Eight(:, 5:7) +...
    1*[sin(betaPX4), -sin(alphaPX4).*cos(betaPX4), cos(alphaPX4).*cos(betaPX4)];
posMATLAB = dataMATLABEight(:,8:10);

eposGazebo = posdesGazebo - posGazebo;
eposPX4 = posdesPX4 - posPX4;
eposMATLAB = posdesMATLAB - posMATLAB;

%% Plotting Ref 8
set(0, 'DefaultAxesYLim', [-4, 4]);
set(0, 'DefaultAxesYTick', linspace(-6, 6, 9));

labelNames = {'$X_{ref}$', '$Y_{ref}$', '$Z_{ref}$'};
figHandle = figure;
set(gcf, 'Position', sz);

for i = 1:3
    subplot(3,1,i);
    plot(tGazeboEight, posdesGazebo(:, i), '-', 'LineWidth', 1.5, 'DisplayName', 'Gazebo');
    hold on;
    plot(tPX4Eight, posdesPX4(:, i), ':', 'LineWidth', 2, 'DisplayName', 'PX4');
    plot(tMATLABEight, posdesMATLAB(:, i), '--', 'LineWidth', 2, 'DisplayName', 'Matlab');
    ylabel(labelNames{i}, 'FontSize', fsz);
    grid on; box on;
    if i == 1
        legend('show', 'Location', 'north', 'Orientation', 'horizontal', 'FontSize', fsz-3);
    end
end
xlabel('t (s)', 'FontSize', fsz);
saveas(figHandle, 'Figures/reference_eight', 'svg');
saveFigureAsPDF(figHandle, 'Figures/reference_eight.pdf');

%% Plotting Trajectory 8
labelNames = {'X', 'Y', 'Z'};
figHandle = figure;
set(gcf, 'Position', sz);

for i = 1:3
    subplot(3,1,i);
    plot(tGazeboEight, posdesGazebo(:, i), '-', 'LineWidth', 1.5, 'DisplayName', 'Ref');
    hold on;
    plot(tGazeboEight, posGazebo(:, i), '-.', 'LineWidth', 1.5, 'DisplayName', 'Gazebo');
    plot(tPX4Eight, posPX4(:, i), ':', 'LineWidth', 2, 'DisplayName', 'PX4');
    plot(tMATLABEight, posMATLAB(:, i), '--', 'LineWidth', 2, 'DisplayName', 'Matlab');
    ylabel(labelNames{i}, 'FontSize', fsz);
    grid on; box on;
    if i == 1
        legend('show', 'Location', 'north', 'Orientation', 'horizontal', 'FontSize', fsz-3);
    end
end
xlabel('t (s)', 'FontSize', fsz);
saveas(figHandle, 'Figures/trajectory_eight', 'svg');
saveFigureAsPDF(figHandle, 'Figures/trajectory_eight.pdf');

%% Plotting Error 8
figHandle = figure;
set(gcf, 'Position', sz);

for i = 1:3
    subplot(3,1,i);
    plot(tGazeboEight, eposGazebo(:, i), '-.', 'LineWidth', 1.5, 'DisplayName', 'Gazebo');
    hold on;
    plot(tPX4Eight, eposPX4(:, i), ':', 'LineWidth', 2, 'DisplayName', 'PX4');
    plot(tMATLABEight, eposMATLAB(:, i), '--', 'LineWidth', 2, 'DisplayName', 'Matlab');
    ylabel(labelNames{i}, 'FontSize', fsz);
    grid on; box on;
    if i == 1
        legend('show', 'Location', 'north', 'Orientation', 'horizontal', 'FontSize', fsz-3);
    end
end
xlabel('t (s)', 'FontSize', fsz);
saveas(figHandle, 'Figures/error_eight', 'svg');
saveFigureAsPDF(figHandle, 'Figures/error_eight.pdf');

%% Plotting 3D Trajectory
figHandle = figure;
plot3(posdesGazebo(:, 1), posdesGazebo(:, 2), posdesGazebo(:, 3), '-', 'LineWidth', 1.5, 'DisplayName', 'Ref');
hold on;
plot3(posGazebo(:, 1), posGazebo(:, 2), posGazebo(:, 3), '-.', 'LineWidth', 1.5, 'DisplayName', 'Gazebo');
plot3(posPX4(:, 1), posPX4(:, 2), posPX4(:, 3), ':', 'LineWidth', 2, 'DisplayName', 'PX4');
plot3(posMATLAB(:, 1), posMATLAB(:, 2), posMATLAB(:, 3), '--', 'LineWidth', 2, 'DisplayName', 'Matlab');
xlabel('$X$', 'FontSize', fsz);
ylabel('$Y$', 'FontSize', fsz);
zlabel('$Z$', 'FontSize', fsz);
legend('show', 'Location', 'northeast', 'Orientation', 'vertical', 'FontSize', fsz);
saveas(figHandle, 'Figures/3d_trajectory', 'svg');
saveFigureAsPDF(figHandle, 'Figures/3d_trajectory.pdf');

%% Functions for Matrix-Based Approach
function data = readCSVFileMatrix(filePath)
    if ~isfile(filePath)
        error('File does not exist: %s', filePath);
    end
    data = readmatrix(filePath);
    if isempty(data)
        error('Failed to load data from: %s', filePath);
    end
end

function [idxMatrix, combinedIdxs] = getTrajectoryIndicesMatrix(data, setpoints, mode)
    % getTrajectoryIndicesMatrix Computes trajectory indices for given setpoints
    %   [idxMatrix, combinedIdxs] = getTrajectoryIndicesMatrix(data, setpoints)
    %   takes the input data and a matrix of setpoints, and returns:
    %   - idxMatrix: A Nx2 matrix where N is the number of setpoints. Each row
    %     contains the start and end indices for a trajectory corresponding to
    %     a setpoint.
    %   - combinedIdxs: A combined array of all trajectory indices across
    %     the given setpoints.
    %   - mode: How to deal with trajectory separated repetitions
    %           'early'        --> keep the earliest
    %           'late'         --> keep the latest
    %           'lateandearly' --> keep intermediate

    if nargin < 3
        mode = 'Continuous';
    end

    % Number of setpoints
    numSetpoints = size(setpoints, 1);

    % Initialize the index matrix
    idxMatrix = zeros(numSetpoints, 2);

    % Initialize combined indices array
    combinedIdxs = [];

    % Loop through each setpoint
    for i = 1:numSetpoints
        sp = setpoints(i, :); % Current setpoint
        % Get the start and end indices for the current setpoint
        idxMatrix(i, :) = getTrajectoryLimitsMatrix(data, sp, mode);
        % Generate the range of indices for the current setpoint
        spIdxs = idxMatrix(i, 1):idxMatrix(i, 2);
        % Append the current indices to the combined indices array
        combinedIdxs = [combinedIdxs, spIdxs];
    end
end

function idxs = getTrajectoryLimitsMatrix(data, setpoint, mode)
    % getTrajectoryLimitsMatrix Identifies the start and end indices for a trajectory
    % based on a given setpoint in the data.
    %
    % Input:
    %   - data: Matrix containing the data, with desired positions in columns 2:4
    %   - setpoint: A 1x3 vector [x, y, z] specifying the desired position
    %               in 3D space.
    %   - mode: How to deal with trajectory separated repetitions
    %           'early'        --> keep the earliest
    %           'late'         --> keep the latest
    %           'lateandearly' --> keep intermediate
    %
    % Output:
    %   - idxs: A 1x2 vector containing the start (idxs(1)) and end (idxs(2))
    %           indices of the trajectory that matches the given setpoint.

    if nargin < 3
        mode = 'Continuous';
    end

    % Create a condition to check where the setpoint matches the desired position
    % in all three dimensions (x, y, z).
    cond = all(data(:, 2:4) == setpoint, 2);

    % Find the indices where the condition is true
    sp1Span = find(cond);

    if isempty(sp1Span)
        error('Setpoint not found in data.');
    end

    % Handle discontinuous segments based on the specified mode
    if ~strcmp(mode, 'Continuous')
        sp1Span = removeDiscontinuous(sp1Span, mode);
    end

    % Assign the first matching index as the start of the trajectory
    idxs(1) = sp1Span(1);

    % Assign the last matching index as the end of the trajectory
    idxs(2) = sp1Span(end);
end

function contTraj = removeDiscontinuous(traj, mode)
    % Find discontinuities
    discont = find(diff(traj) > 1);

    if isempty(discont)
        % No discontinuities, return the full trajectory
        contTraj = traj;
    else
        % Determine the trajectory based on the mode
        switch mode
            case 'early'
                % Extract the first continuous segment
                contTraj = traj(1:discont(1));
            case 'late'
                % Extract the last continuous segment
                contTraj = traj(discont(end)+1:end);
            case 'lateandearly'
                % Extract the intermediate segment if it exists
                if length(discont) > 1
                    contTraj = traj(discont(1)+1:discont(end));
                else
                    error('No intermediate region exists in the trajectory.');
                end
            otherwise
                error('Invalid mode. Choose "late", "early", or "lateandearly".');
        end
    end
end
