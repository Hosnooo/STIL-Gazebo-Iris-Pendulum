function newLimits = editPlotLimits(axis, ax, limitIncreaseFactor)
%% Enhanced Plot Limits Adjustment Function: Flexible Axis Expansion
% Author: Mohssen Elshaar / g202309590@kfupm.edu.sa
% Created on: 15/3/2024
% Last Modified: 07/10/2024
% This function adjusts the axis limits of a plot to improve visualization by
% expanding the range of the specified axis or axes (`x`, `y`, or both `xy`).
% Users can control the expansion direction (`+`, `-`, or both) and specify the
% amount of expansion using a `limitIncreaseFactor`. It is particularly useful
% for enhancing plots with tightly packed data or for aesthetic purposes.
%
% Input:
%   - axis: A numeric array specifying the current axis limits in the format
%           [xmin, xmax, ymin, ymax].
%   - ax: A string indicating the axis and direction to expand. Possible values:
%         * 'xy': Expand both x and y axes in both directions.
%         * 'xy+': Expand both x and y axes in the positive direction only.
%         * 'xy-': Expand both x and y axes in the negative direction only.
%         * 'x': Expand only the x-axis in both directions.
%         * 'x+': Expand only the x-axis in the positive direction.
%         * 'x-': Expand only the x-axis in the negative direction.
%         * 'y': Expand only the y-axis in both directions.
%         * 'y+': Expand only the y-axis in the positive direction.
%         * 'y-': Expand only the y-axis in the negative direction.
%   - limitIncreaseFactor: A numeric value specifying the fraction of the axis
%                          range by which to expand the limits.
%
% Output:
%   - newLimits: A numeric array containing the modified axis limits in the format
%                [xmin, xmax, ymin, ymax].
%
% Example Usage:
%   currentLimits = [0, 10, -5, 5];
%   expansionFactor = 0.2; % Increase limits by 20% of the current range
%   newLimits = editPlotLimits(currentLimits, 'xy+', expansionFactor);
%   disp('New axis limits:');
%   disp(newLimits);


% Validate inputs
if nargin < 3
    error('Not enough input arguments. Both axis and limitIncreaseFactor are required.');
end

if ~isnumeric(axis) || ~isnumeric(limitIncreaseFactor)
    error('Both axis and limitIncreaseFactor must be numeric.');
end

if isempty(axis) || numel(axis) ~= 4
    error('Axis must be a numeric array with 4 elements [xmin, xmax, ymin, ymax].');
end

% Extract current limits
currentLimits = axis;
xmin = currentLimits(1);
xmax = currentLimits(2);
ymin = currentLimits(3);
ymax = currentLimits(4);

% Calculate new limits with the given factor
xRange = xmax - xmin;
yRange = ymax - ymin;

% Handle zero ranges to prevent no change in limits
if xRange == 0
    xRange = 1;
end
if yRange == 0
    yRange = 1;
end

switch ax
    case 'xy'
        newXmin = xmin - limitIncreaseFactor * xRange;
        newXmax = xmax + limitIncreaseFactor * xRange;
        newYmin = ymin - limitIncreaseFactor * yRange;
        newYmax = ymax + limitIncreaseFactor * yRange;
    case 'xy+'
        newXmin = xmin;
        newXmax = xmax + limitIncreaseFactor * xRange;
        newYmin = ymin;
        newYmax = ymax + limitIncreaseFactor * yRange;
    case 'xy-'
        newXmin = xmin - limitIncreaseFactor * xRange;
        newXmax = xmax;
        newYmin = ymin - limitIncreaseFactor * yRange;
        newYmax = ymax;
    case 'x'
        newXmin = xmin - limitIncreaseFactor * xRange;
        newXmax = xmax + limitIncreaseFactor * xRange;
        newYmin = ymin;
        newYmax = ymax;
    case 'x+'
        newXmin = xmin;
        newXmax = xmax + limitIncreaseFactor * xRange;
        newYmin = ymin;
        newYmax = ymax;
    case 'x-'
        newXmin = xmin - limitIncreaseFactor * xRange;
        newXmax = xmax;
        newYmin = ymin;
        newYmax = ymax;
    case 'y'
        newXmin = xmin;
        newXmax = xmax;
        newYmin = ymin - limitIncreaseFactor * yRange;
        newYmax = ymax + limitIncreaseFactor * yRange;
    case 'y+'
        newXmin = xmin;
        newXmax = xmax;
        newYmin = ymin;
        newYmax = ymax + limitIncreaseFactor * yRange;
    case 'y-'
        newXmin = xmin;
        newXmax = xmax;
        newYmin = ymin - limitIncreaseFactor * yRange;
        newYmax = ymax;
end

% Combine new limits
newLimits = [newXmin, newXmax, newYmin, newYmax];
end