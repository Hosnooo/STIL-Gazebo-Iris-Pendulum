clc; clear; close all;

%% GCF Options
sz = [200, 100, 400, 600];
fsz = 12;
%% Load Data
dataGazebo = readCSVFile('log.csv');
columnsGazebo = dataGazebo.Properties.VariableNames;
dataPX4 = readCSVFile('log_px4.csv');
columnsPX4 = dataPX4.Properties.VariableNames;

%% Trim Trajectory of interest
% setpoint 1: x = 1,  y = -1, z = -2
% setpoint 2: x = -2, y = -1, z = -4
% setpoint 3: x = 1,  y = 2,  z = -3

sp = [ 1  -1 -2;
    -2  -1 -4;
    1   2 -3];

[idxMatrixGazebo, combinedIdxsGazebo] = getTrajectoryIndices(dataGazebo, sp);
[idxMatrixPX4, combinedIdxsPX4]       = getTrajectoryIndices(dataPX4, sp);

% 8-shaped Trajectory
% ending setpoint: x = 0;  y = 0,  z = -1.5
[idxGazebo, ~]     = getTrajectoryIndices(dataGazebo, [0 0 -1.5], 'late');
[idxPX4, ~]        = getTrajectoryIndices(dataPX4, [0 0 -1.5], 'late');
idxMatrixGazebo    = [idxMatrixGazebo; idxMatrixGazebo(3,2)+1 idxGazebo(1)];
idxMatrixPX4       = [idxMatrixPX4; idxMatrixPX4(3,2)+1 idxPX4(1)-1];
eightIdxsGazebo    = idxMatrixGazebo(4,1):idxMatrixGazebo(4,2)-1;
eightIdxsPX4       = idxMatrixPX4(4,1):idxMatrixPX4(4,2)-1;

eightIdxsGazebo    = eightIdxsGazebo(1:floor(2*length(eightIdxsGazebo)/3));
eightIdxsPX4       = eightIdxsPX4(1:floor(2*length(eightIdxsPX4)/3));

dataGazeboSetpoints  = dataGazebo(combinedIdxsGazebo,:);
dataPX4Setpoints     = dataPX4(combinedIdxsPX4,:);
dataGazeboEight      = dataGazebo(eightIdxsGazebo,:);
dataPX4Eight         = dataPX4(eightIdxsPX4,:);
tGazeboSetpoints     = dataGazeboSetpoints.Time - dataGazeboSetpoints.Time(1);
tPX4Setpoints        = dataPX4Setpoints.Time - dataPX4Setpoints.Time(1);
tGazeboEight         = dataGazeboEight.Time - dataGazeboEight.Time(1);
tPX4Eight            = dataPX4Eight.Time - dataPX4Eight.Time(1);

posdesGazebo       = dataGazeboSetpoints(:,2:4);
posdesPX4          = dataPX4Setpoints(:,2:4);
posGazebo          = dataGazeboSetpoints(:,5:7);
posPX4             = dataPX4Setpoints(:,5:7);

columns = {'X', 'Y', 'Z'};
posdesGazebo.Properties.VariableNames = columns;
posdesPX4.Properties.VariableNames    = columns;
posGazebo.Properties.VariableNames    = columns;
posPX4.Properties.VariableNames       = columns;
columns = posPX4.Properties.VariableNames;

eposGazebo        = posdesGazebo - posGazebo;
eposPX4           = posdesPX4 - posPX4;

%% setpoints plotting
labelNames = {'x_{ref}', 'y_{ref}', 'z_{ref}'};
figHandle = figure;
set(gcf, 'Position',sz);
for i = 1:3
    subplot(3,1,i)
    plot(tGazeboSetpoints, posdesGazebo.(columns{i}), '-',...
        'LineWidth',1.5, 'DisplayName','Gazebo')
    hold on;
    plot(tPX4Setpoints, posdesPX4.(columns{i}), ':',...
        'LineWidth',2, 'DisplayName','PX4')
    ylabel(labelNames{i},'FontSize',fsz);

    yticks(round(min(posdesGazebo.(columns{i})),2):1:max(posdesGazebo.(columns{i})))
    xlim([tGazeboSetpoints(1) tGazeboSetpoints(end)])

    newLimits = editPlotLimits(axis, 'x', 0.05);
    axis(newLimits)
    newLimits = editPlotLimits(axis, 'y+', 0.5);
    axis(newLimits)
    newLimits = editPlotLimits(axis, 'y-', 0.1);
    axis(newLimits)
    grid on; box on;
    if i == 1
        legend('show','Location','north','Orientation','horizontal','FontSize', fsz-3)
    end
end
xlabel('t (s)', 'FontSize',fsz);

saveFigureAsPDF(figHandle,'Figures/refernce setpoints.pdf');

labelNames = {'x', 'y', 'z'};
figHandle = figure;
set(gcf, 'Position', sz); % [x, y, width, height]
for i = 1:3
    subplot(3,1,i)
    plot(tGazeboSetpoints, posdesGazebo.(columns{i}), '-',...
        'LineWidth',1.5, 'DisplayName','Reference')
    hold on;
    plot(tGazeboSetpoints, posGazebo.(columns{i}), '-.',...
        'LineWidth',1.5, 'DisplayName','Gazebo')
    plot(tPX4Setpoints, posPX4.(columns{i}), ':',...
        'LineWidth',2, 'DisplayName','PX4')
    ylabel(labelNames{i},'FontSize',fsz);

    yticks(round(min(posPX4.(columns{i})),2):1:max(posPX4.(columns{i})))
    xlim([tGazeboSetpoints(1) tGazeboSetpoints(end)])

    newLimits = editPlotLimits(axis, 'x', 0.05);
    axis(newLimits)
    newLimits = editPlotLimits(axis, 'y+', 0.4);
    axis(newLimits)
    newLimits = editPlotLimits(axis, 'y-', 0.1);
    axis(newLimits)
    grid on; box on;
    if i == 1
        legend('show','Location','north','Orientation','horizontal','FontSize', fsz-3)
    end
end
xlabel('t (s)', 'FontSize',fsz);

saveFigureAsPDF(figHandle,'Figures/trajectory setpoints.pdf');

labelNames = {'x', 'y', 'z'};
figHandle = figure;
set(gcf, 'Position', sz); % [x, y, width, height]
for i = 1:3
    subplot(3,1,i)
    % plot(tGazeboSetpoints, posdes, '-', 'LineWidth',1.5, 'DisplayName','Reference')
    hold on;
    plot(tGazeboSetpoints, eposGazebo.(columns{i}), '-.',...
        'LineWidth',1.5, 'DisplayName','Gazebo')
    plot(tPX4Setpoints, eposPX4.(columns{i}), ':',...
        'LineWidth',2, 'DisplayName','PX4')
    ylabel(labelNames{i},'FontSize',fsz);

    yticks(round(min(eposPX4.(columns{i})),2):1:max(eposPX4.(columns{i})))
    xlim([tGazeboSetpoints(1) tGazeboSetpoints(end)])

    newLimits = editPlotLimits(axis, 'x', 0.05);
    axis(newLimits)
    newLimits = editPlotLimits(axis, 'y+', 0.3);
    axis(newLimits)
    newLimits = editPlotLimits(axis, 'y-', 0.05);
    axis(newLimits)
    grid on; box on;
    if i == 1
        legend('show','Location','north','Orientation','horizontal','FontSize', fsz-3)
    end
end
xlabel('t (s)', 'FontSize',fsz);

saveFigureAsPDF(figHandle,'Figures/error setpoints.pdf');

%% 8-shaped trajectory

posdesGazebo       = dataGazeboEight(:,2:4);
posdesPX4          = dataPX4Eight(:,2:4);
posGazebo          = dataGazeboEight(:,5:7);
posPX4             = dataPX4Eight(:,5:7);

columns = {'X', 'Y', 'Z'};
posdesGazebo.Properties.VariableNames = columns;
posdesPX4.Properties.VariableNames    = columns;
posGazebo.Properties.VariableNames    = columns;
posPX4.Properties.VariableNames       = columns;
columns = posPX4.Properties.VariableNames;

eposGazebo        = posdesGazebo - posGazebo;
eposPX4           = posdesPX4 - posPX4;

labelNames = {'x_{ref}', 'y_{ref}', 'z_{ref}'};
figHandle = figure;
set(gcf, 'Position',sz);
for i = 1:3
    subplot(3,1,i)
    plot(tGazeboEight, posdesGazebo.(columns{i}), '-',...
        'LineWidth',1.5, 'DisplayName','Gazebo')
    hold on;
    plot(tPX4Eight, posdesPX4.(columns{i}), ':',...
        'LineWidth',2, 'DisplayName','PX4')
    ylabel(labelNames{i},'FontSize',fsz);

    yticks(round(min(posdesGazebo.(columns{i})),2):1:max(posdesGazebo.(columns{i})))
    xlim([tGazeboEight(1) tGazeboEight(end)])

    newLimits = editPlotLimits(axis, 'x', 0.05);
    axis(newLimits)
    newLimits = editPlotLimits(axis, 'y+', 0.5);
    axis(newLimits)
    newLimits = editPlotLimits(axis, 'y-', 0.1);
    axis(newLimits)
    grid on; box on;
    if i == 1
        legend('show','Location','north','Orientation','horizontal','FontSize', fsz-3)
    end
end
xlabel('t (s)', 'FontSize',fsz);

saveFigureAsPDF(figHandle,'Figures/refernce eight.pdf');

labelNames = {'x', 'y', 'z'};
figHandle = figure;
set(gcf, 'Position', sz); % [x, y, width, height]
for i = 1:3
    subplot(3,1,i)
    plot(tGazeboEight, posdesGazebo.(columns{i}), '-',...
        'LineWidth',1.5, 'DisplayName','Reference')
    hold on;
    plot(tGazeboEight, posGazebo.(columns{i}), '-.',...
        'LineWidth',1.5, 'DisplayName','Gazebo')
    plot(tPX4Eight, posPX4.(columns{i}), ':',...
        'LineWidth',2, 'DisplayName','PX4')
    ylabel(labelNames{i},'FontSize',fsz);

    yticks(round(min(posPX4.(columns{i})),2):1:max(posPX4.(columns{i})))
    xlim([tGazeboEight(1) tGazeboEight(end)])

    newLimits = editPlotLimits(axis, 'x', 0.05);
    axis(newLimits)
    newLimits = editPlotLimits(axis, 'y+', 0.4);
    axis(newLimits)
    newLimits = editPlotLimits(axis, 'y-', 0.1);
    axis(newLimits)
    grid on; box on;
    if i == 1
        legend('show','Location','north','Orientation','horizontal','FontSize', fsz-3)
    end
end
xlabel('t (s)', 'FontSize',fsz);

saveFigureAsPDF(figHandle,'Figures/trajectory eight.pdf');

labelNames = {'x', 'y', 'z'};
figHandle = figure;
set(gcf, 'Position', sz); % [x, y, width, height]
for i = 1:3
    subplot(3,1,i)
    % plot(tGazeboSetpoints, posdes, '-', 'LineWidth',1.5, 'DisplayName','Reference')
    hold on;
    plot(tGazeboEight, eposGazebo.(columns{i}), '-.',...
        'LineWidth',1.5, 'DisplayName','Gazebo')
    plot(tPX4Eight, eposPX4.(columns{i}), ':',...
        'LineWidth',2, 'DisplayName','PX4')
    ylabel(labelNames{i},'FontSize',fsz);

    yticks(round(min(eposPX4.(columns{i})),2):1:max(eposPX4.(columns{i})))
    xlim([tGazeboEight(1) tGazeboEight(end)])

    newLimits = editPlotLimits(axis, 'x', 0.05);
    axis(newLimits)
    newLimits = editPlotLimits(axis, 'y+', 0.3);
    axis(newLimits)
    newLimits = editPlotLimits(axis, 'y-', 0.05);
    axis(newLimits)
    grid on; box on;
    if i == 1
        legend('show','Location','north','Orientation','horizontal','FontSize', fsz-3)
    end
end
xlabel('t (s)', 'FontSize',fsz);

saveFigureAsPDF(figHandle,'Figures/error eight.pdf');

figHandle = figure;
plot3(posdesGazebo.X, posdesGazebo.Y, posdesGazebo.Z,...
      '-', 'LineWidth', 1.5, 'DisplayName','Reference');
hold on;
plot3(posGazebo.X, posGazebo.Y, posGazebo.Z,...
      '-.', 'LineWidth', 1.5, 'DisplayName','Gazebo');
plot3(posPX4.X, posPX4.Y, posPX4.Z,...
      ':', 'LineWidth', 2, 'DisplayName','PX4');

xlabel(labelNames{1},'FontSize',fsz);
ylabel(labelNames{2},'FontSize',fsz);
zlabel(labelNames{3},'FontSize',fsz);

grid on; box on;
legend('show','Location','northeast','Orientation','vertical','FontSize', fsz)

saveFigureAsPDF(figHandle,'Figures/3d eight.pdf');

%% Functions & Utilities
function data = readCSVFile(filePath)
% readCSVFile Reads and processes a CSV file
%   data = readCSVFile(filePath) reads the CSV file located at filePath
%   and returns the data as a table. It also demonstrates basic operations
%   on the data.

% Check if file exists
if ~isfile(filePath)
    error('File does not exist: %s', filePath);
end

% Read the CSV file into a table
try
    data = readtable(filePath);
    fprintf('Successfully loaded file: %s\n', filePath);
catch ME
    error('Failed to read the file: %s\nError: %s', filePath, ME.message);
end

% Display basic information about the data
fprintf('The data contains %d rows and %d columns.\n', size(data, 1), size(data, 2));
% disp('First few rows of the data:');
% disp(head(data));

end


function [idxMatrix, combinedIdxs] = getTrajectoryIndices(data, setpoints, mode)
% getTrajectoryIndices Computes trajectory indices for given setpoints
%   [idxMatrix, combinedIdxs] = getTrajectoryIndices(data, setpoints)
%   takes the input data and a matrix of setpoints, and returns:
%   - idxMatrix: A Nx2 matrix where N is the number of setpoints. Each row
%     contains the start and end indices for a trajectory corresponding to
%     a setpoint.
%   - combinedIdxs: A combined array of all trajectory indices across
%     the given setpoints.
%   - mode: How to deal with trajectory separated repitions
%           'early'        --> keep the earliest
%           'late'         --> keep the latest
%           'lateandearly' --> keep intermediant
%
%   Inputs:
%     - data: The dataset containing desired position fields
%             (Des_Pos_X, Des_Pos_y, Des_Pos_z).
%     - setpoints: An Nx3 matrix, where each row represents a setpoint
%                  [x, y, z].
%
%   Outputs:
%     - idxMatrix: Nx2 matrix containing start and end indices for each
%                  setpoint.
%     - combinedIdxs: 1xM array of all indices combined for all setpoints.

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
    idxMatrix(i, :) = getTrajectoryLimits(data, sp, mode);
    % Generate the range of indices for the current setpoint
    spIdxs = idxMatrix(i, 1):idxMatrix(i, 2);
    % Append the current indices to the combined indices array
    combinedIdxs = [combinedIdxs, spIdxs];
end
end


function idxs = getTrajectoryLimits(data,setpoint, mode)
% getTrajectoryLimits Identifies the start and end indices for a trajectory
% based on a given setpoint in the data.
%
% Input:
%   - data: Table containing the data
%   - setpoint: A 1x3 vector [x, y, z] specifying the desired position
%               in 3D space.
%   - mode: How to deal with trajectory separated repitions
%           'early'        --> keep the earliest
%           'late'         --> keep the latest
%           'lateandearly' --> keep intermediant
%
% Output:
%   - idxs: A 1x2 vector containing the start (idxs(1)) and end (idxs(2))
%           indices of the trajectory that matches the given setpoint.

if nargin < 3
   mode = 'Continuous';
end

% Initialize the output vector with zeros (default values)
idxs = zeros(1, 2);

% Create a condition to check where the setpoint matches the desired position
% in all three dimensions (x, y, z).
cond = data.Des_Pos_X == setpoint(1) & ...
    data.Des_Pos_Y == setpoint(2) & ...
    data.Des_Pos_Z == setpoint(3);

% Find the indices where the condition is true
sp1Span = find(cond);

% Handle repetitions
if ~strcmp(mode,'Continuous')
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

