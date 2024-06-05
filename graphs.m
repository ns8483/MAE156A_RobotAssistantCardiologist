clear all;
clc;
%%
extensionTheo = [0, 4.8, 9.5, 14.3, 19, 23.8, 28.6, 33.3, 38.1, 42.9, 47.7, 52.4, 57.2, 61.9, 66.6, 71.3, 76, 80.8, 85.6];
extensionActual = [1, 6, 10, 14, 19, 24, 28, 32, 37, 41, 46, 50, 55, 59, 64, 68, 73, 77, 82];

% Fit a line (1st degree polynomial) to the data
p = polyfit(extensionTheo, extensionActual,  1);  % 1 indicates fitting a line (1st degree polynomial)

% Generate points for the line of best fit
xFit = linspace(min(extensionTheo), max(extensionTheo), length(extensionTheo));
yFit = polyval(p, xFit);

% Fit a line (1st degree polynomial) to the data
p2 = polyfit(extensionTheo, extensionTheo, 1);  % 1 indicates fitting a line (1st degree polynomial)

% Generate points for the line of best fit
xFit2 = linspace(min(extensionTheo), max(extensionTheo), length(extensionTheo));
yFit2 = polyval(p2, xFit2);

% Step 1: Calculate the differences
differences = extensionActual - extensionTheo

% Step 2: Square the differences
squared_differences = differences.^ 2;  % Element-wise square

% Step 3: Calculate the mean of squared differences
mean_squared_difference = mean(squared_differences);

% Step 4: Take the square root to get RMSE
rmse = sqrt(mean_squared_difference);

fprintf('Root Mean Squared Error (RMSE): %.2f\n', rmse);

figure(1)
hold on;
plot(extensionTheo,extensionTheo,'k*','LineWidth', 1)
plot(extensionTheo,extensionActual,'r*','LineWidth', 1)
plot(xFit,yFit,'r','LineWidth', 1)
plot(xFit2,yFit2,'k','LineWidth', 1)
xlabel("Input linear translation (mm)")
ylabel("Output linear translation (mm)")
legend("Theoretical", "Experimental")
% Add the RMSE value as text at the bottom left
text('Units', 'normalized', 'Position', [0.05, 0.05], 'String', sprintf('RMSE: %.2f mm', rmse), 'FontSize', 12, 'Color', 'k', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');
grid on;
grid minor;

%%
rotTheo = -1*[-90, -85, -80, -75, -70, -65, -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0];
rotHand = -1*[-91, -86, -81, -76, -70, -65, -59, -54, -51, -48, -44, -38, -34, -30, -26, -19, -13, -7, -2];
rotTip = -1*[-91, -86, -81, -76, -71, -66, -62, -57, -52, -47, -42, -38, -33, -28, -23, -18, -12, -8, -2];

% Step 1: Calculate the differences
differences = rotHand - rotTheo

% Step 2: Square the differences
squared_differences = differences.^ 2;  % Element-wise square

% Step 3: Calculate the mean of squared differences
mean_squared_difference = mean(squared_differences);

% Step 4: Take the square root to get RMSE
rmseHand = sqrt(mean_squared_difference);

% Step 1: Calculate the differences
differences = rotTip - rotTheo

% Step 2: Square the differences
squared_differences = differences.^ 2;  % Element-wise square

% Step 3: Calculate the mean of squared differences
mean_squared_difference = mean(squared_differences);

% Step 4: Take the square root to get RMSE
rmseTip = sqrt(mean_squared_difference);

figure(2)
hold on;
plot(rotTheo,rotTheo,'k-*','LineWidth', 1)
plot(rotTheo,rotHand,'r-*','LineWidth', 1)
plot(rotTheo,rotTip,'b-*','LineWidth', 1)
xlabel("Input rotation (°)")
ylabel("Output rotation (°)")
legend("Theoretical", "Experimental Probe Handle", "Expeimental Tip Control")
% Add the RMSE value as text at the bottom left
text('Units', 'normalized', 'Position', [0.35, 0.15], 'String', sprintf('Probe Handle RMSE: %.2f°', rmseHand), 'FontSize', 12, 'Color', 'k', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');
text('Units', 'normalized', 'Position', [0.35, 0.05], 'String', sprintf('Tip Control RMSE: %.2f°', rmseTip), 'FontSize', 12, 'Color', 'k', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');
grid on;
grid minor;