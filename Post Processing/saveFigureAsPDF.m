function saveFigureAsPDF(figHandle, fileName, width, height, dpi)
% Save a given figure as a PDF with specified dimensions and resolution.
% Author: Mohssen Elshaar / g202309590@kfupm.edu.sa
% Created on: 15/3/2024
% Last Modified: 07/10/2024
% Parameters:
% figHandle - Handle of the figure to be saved.
% fileName  - Name of the output PDF file (with path if needed).
% width     - Width of the figure in inches (default: 6).
% height    - Height of the figure in inches (default: 4).
% dpi       - Resolution in dots per inch (default: 300).

% Set default values if not provided
if nargin < 3, width = 6; end
if nargin < 4, height = 4; end
if nargin < 5, dpi = 300; end

% Set figure properties for exporting
set(figHandle, 'Units', 'Inches');
% set(figHandle, 'Position', [1, 1, width, height]); % Adjust figure size
set(figHandle, 'PaperUnits', 'Inches');
% set(figHandle, 'PaperPosition', [0, 0, width, height]); % Adjust paper size
% set(figHandle, 'PaperSize', [width, height]); % Set paper size for PDF output

% Save the figure as a PDF
print(figHandle, fileName, '-dpdf', ['-r', num2str(dpi)]);

% Display confirmation
disp(['Figure saved as PDF: ', fileName]);
end