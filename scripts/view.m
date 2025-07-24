clear all;
close all;
clc;

DoF = 7;
joint_names = arrayfun(@(i) ['panda_joint_' num2str(i)], 0:(DoF-1), 'UniformOutput', false);

bag = rosbagreader('bags/FOB_PD_tracking.bag');

% Get list of topics in the bag
topics = bag.AvailableTopics.Properties.RowNames;

% Initialize a structure to store timeseries
timeseriesMap = containers.Map();

topics_to_parse = {'/FOB_controller/desired_trajectory', '/franka_state_controller/franka_states'};

for i=1:size(topics_to_parse, 2)
    topic = topics_to_parse{i};

    msgs = readMessages(select(bag, 'Topic', topic), 'DataFormat','struct');
    sel = select(bag, 'Topic', topic);
    timeStamps = sel.MessageList.Time;
    
    msgStructs = readMessages(sel, 'DataFormat', 'struct');
    msgFields = fieldnames(msgStructs{1});

    for j = 1:length(msgFields)
        fieldName = msgFields{j};
        try
            data = cellfun(@(m) getfield(m, fieldName), msgStructs, 'UniformOutput', false);
            data = reshape(cell2mat(data), DoF, []);
            ts = timeseries(data', timeStamps);
            key = sprintf('%s/%s', topic, fieldName);
            timeseriesMap(key) = ts;
        catch
            % Skip fields that can't be processed
            continue
        end
    end
    
    %data = cellfun(@(m) , msgStructs, 'UniformOutput', false);
    %data = reshape(cell2mat(data), 7, []);
    %ts = timeseries(data', timeStamps);
    %timeseriesMap(topic) = ts;
end

plot_multiple_ts_subplots({timeseriesMap('/franka_state_controller/franka_states/TauJ');timeseriesMap('/franka_state_controller/franka_states/TauJD')}, joint_names, {'TauJ'; 'TauJ_d'})


plot_multiple_ts_subplots({timeseriesMap('/franka_state_controller/franka_states/Q'); timeseriesMap('/FOB_controller/desired_trajectory/Data')}, joint_names, {'q'; 'q_d'})

% plot_ts_subplots(timeseriesMap('/FOB_controller/desired_trajectory/Data'), joint_names);
% plot_ts_subplots(timeseriesMap('/franka_state_controller/franka_states/Q'), joint_names);

% Iterate over each topic
% for i = 1:length(topics)
%     topic = topics{i};
% 
%     % Select messages from the topic
%     msgs = readMessages(select(bag, 'Topic', topic), 'DataFormat','struct');
% 
%     % Try to extract times and data
%     try
%         % Get message timestamps
%         sel = select(bag, 'Topic', topic);
%         timeStamps = sel.MessageList.Time;
% 
%         % Try to convert to time series
%         msgStructs = readMessages(sel, 'DataFormat', 'struct');
% 
%         % Flatten message structure
%         msgFields = fieldnames(msgStructs{1});
%         for j = 1:length(msgFields)
%             fieldName = msgFields{j};
%             try
%                 data = cellfun(@(m) getfield(m, fieldName), msgStructs);
%                 ts = timeseries(data, timeStamps);
%                 key = sprintf('%s/%s', topic, fieldName);
%                 timeseriesMap(key) = ts;
%             catch
%                 % Skip fields that can't be processed
%                 continue
%             end
%         end
%     catch
%         warning('Failed to convert topic %s to timeseries.', topic);
%         continue
%     end
% end



function plot_multiple_ts_subplots(tsList, varNames, tsLabels)
% plot_multiple_ts_subplots - Plot same channels from multiple timeseries together
%
% Inputs:
%   tsList    - Cell array of timeseries objects, each with Nx7 data
%   varNames  - (Optional) 1x7 cell array of variable (channel) names
%   tsLabels  - (Optional) 1xN cell array of labels for each timeseries
%
% Example:
%   plot_multiple_ts_subplots({ts1, ts2});
%   plot_multiple_ts_subplots({ts1, ts2}, {'AccelX','AccelY',...});
%   plot_multiple_ts_subplots({ts1, ts2}, [], {'Trial 1','Trial 2'});

    numTS = numel(tsList);
    assert(numTS > 0, 'tsList must contain at least one timeseries.');

    % Validate each timeseries
    for i = 1:numTS
        ts = tsList{i};
        assert(isa(ts, 'timeseries'), 'Each item in tsList must be a timeseries object.');
    end

    % Get number of variables (assume all same)
    D = size(tsList{1}.Data, 2);
    assert(D == 7, 'Expected each timeseries to have 7 variables.');

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

    % Set up colors
    colors = lines(numTS);

    % Plot
    figure;
    for varIdx = 1:D
        subplot(D, 1, varIdx); hold on;

        for tsIdx = 1:numTS
            ts = tsList{tsIdx};
            plot(ts.Time, ts.Data(:, varIdx), ...
                'DisplayName', tsLabels{tsIdx}, ...
                'Color', colors(tsIdx,:));
        end

        ylabel(varNames{varIdx}, 'Interpreter', 'none');
        if varIdx == D
            xlabel('Time');
        else
            set(gca, 'XTickLabel', []);
        end
        legend('show');
    end

    sgtitle('Multiple Timeseries - Channel-wise Comparison');
end