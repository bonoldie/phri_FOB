function plot_multiple_ts_subplots(tsList, varNames, tsLabels, holdPlots, FOB_lower_times, FOB_upper_times)
% plot_multiple_ts_subplots - Plot same channels from multiple timeseries together
%
% Inputs:
%   tsList    - Cell array of timeseries objects, each with Nx6 or Nx7 data
%   varNames  - (Optional) 1xD cell array of variable (channel) names
%   tsLabels  - (Optional) 1xN cell array of labels for each timeseries
%   holdPlots - (Optional) Boolean. If true, add plots to current figure.
%               If false (default), open a new figure.
%
% Example:
%   plot_multiple_ts_subplots({ts1, ts2});
%   plot_multiple_ts_subplots({ts1, ts2}, {'AccelX','AccelY',...});
%   plot_multiple_ts_subplots({ts1, ts2}, [], {'Trial 1','Trial 2'});
%   plot_multiple_ts_subplots({ts1, ts2}, [], [], true); % overlay on existing plot

    numTS = numel(tsList);
    assert(numTS > 0, 'tsList must contain at least one timeseries.');

    % Validate each timeseries
    for i = 1:numTS
        ts = tsList{i};
        assert(isa(ts, 'timeseries'), 'Each item in tsList must be a timeseries object.');
    end

    % Get number of variables (assume all same)
    D = size(tsList{1}.Data, 2);
    assert(D == 6 || D == 7, 'Expected each timeseries to have 6 or 7 variables.');

    % Default variable (channel) names
    if nargin < 2 || isempty(varNames)
        varNames = arrayfun(@(i) sprintf('Var%d', i), 1:D, 'UniformOutput', false);
    elseif numel(varNames) ~= D
        error('varNames must contain exactly %d names.', D);
    end

    % Default timeseries labels for legend
    if nargin < 3 || isempty(tsLabels)
        tsLabels = arrayfun(@(i) sprintf('TS%d', i), 1:numTS, 'UniformOutput', false);
    elseif numel(tsLabels) ~= numTS
        error('tsLabels must have one label per timeseries (%d).', numTS);
    end

    % Default holdPlots flag
    if nargin < 4 || isempty(holdPlots)
        holdPlots = false;
    end

    % Set up colors
    colors = lines(numTS);

    % New figure unless holding
    if ~holdPlots
        figure;
    end

    % Plot
    for varIdx = 1:D
        subplot(D, 1, varIdx); hold on;

        for tsIdx = 1:numTS
            ts = tsList{tsIdx};
            plot(ts.Time, ts.Data(:, varIdx), ...
                'DisplayName', tsLabels{tsIdx}, ...
                'Color', colors(tsIdx,:));
        end
        %---------------------------------------------------------
        if exist('FOB_lower_times','var')
            for t = FOB_lower_times'
                xline(t, '--r', 'FOB lower', 'Alpha', 0.6);
            end
        end
        
        if exist('FOB_upper_times','var')
            for t = FOB_upper_times'
                xline(t, '--b', 'FOB upper', 'Alpha', 0.6);
            end
        end
        %---------------------------------------------------------

        ylabel(varNames{varIdx}, 'Interpreter', 'none');
        if varIdx == D
            xlabel('Time');
        else
            set(gca, 'XTickLabel', []);
        end
        legend('show');
    end

    if ~holdPlots
        sgtitle(sprintf('Multiple Timeseries - Channel-wise Comparison (%d Vars)', D));
    end
end
