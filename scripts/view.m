clear all;
% close all;
clc;

DoF = 7;
joint_names = arrayfun(@(i) ['panda_joint_' num2str(i)], 0:(DoF-1), 'UniformOutput', false);

% bag = rosbagreader('bags/baseline_force_K_1_0_5.bag');
% bag = rosbagreader('bags/PD_tracking.bag');

bag = rosbagreader('bags/force_sensor.bag');

% Get list of topics in the bag
topics = bag.AvailableTopics.Properties.RowNames;

% Initialize a structure to store timeseries
timeseriesMap = containers.Map();


topics_to_parse = { '/franka_state_controller/franka_states', '/FOB_controller/tau_frc_hat', '/FOB_controller/desired_trajectory', '/dynamic_reconfigure_FOB_param_node/parameter_updates', '/netft/netft_data'};% '/FOB_controller/tau_ext_hat_filtered', '/FOB_controller/tau_frc_ref', '/FOB_controller/desired_trajectory',

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
            try 
                data = reshape(cell2mat(data), DoF, []);
                ts = timeseries(data', timeStamps);
                key = sprintf('%s/%s', topic, fieldName);
                timeseriesMap(key) = ts;
            catch
                try
                    data = reshape(cell2mat(data), 6, []);
                    ts = timeseries(data', timeStamps);
                    key = sprintf('%s/%s', topic, fieldName);
                    timeseriesMap(key) = ts;
                catch
                    continue
                end
            end
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

% Get all timeseries keys
keys = timeseriesMap.keys;

if isempty(keys)
    warning('No timeseries found to resample.');
else
    % Reference timeseries (first one)
    refTS = timeseriesMap(keys{1});
    t0 = refTS.Time(1);                 % shift start time to 0
    refTime = refTS.Time - t0;

    % Uniform time vector using average sampling interval
    dt = mean(diff(refTime));
    uniformTime = 0:dt:refTime(end);

    % Initialize new map for aligned/resampled timeseries
    alignedMap = containers.Map();

    for k = 1:length(keys)
        ts = timeseriesMap(keys{k});
        ts.Time = ts.Time - t0; % shift start time to 0
        ts_resampled = resample(ts, uniformTime, 'linear');
        alignedMap(keys{k}) = ts_resampled;
    end
end


%plot_multiple_ts_subplots({timeseriesMap('/franka_state_controller/franka_states/Q'); timeseriesMap('/FOB_controller/desired_trajectory/Data')}, joint_names, {'q'; 'q_d'})

% plot_multiple_ts_subplots({alignedMap('/franka_state_controller/franka_states/TauJ'),  alignedMap('/FOB_controller/desired_trajectory/Data')}, {'X'; 'Y'; 'Z'; 'phi'; 'theta'; 'psi'} , {'OFExtHatK', 'OFExtHatK_d'})
plot_multiple_ts_subplots({alignedMap('/netft/netft_data/Z'),  alignedMap('/FOB_controller/desired_trajectory/Data')},  joint_names, {'tauJ', 'tauJ_d'})
% plot_multiple_ts_subplots({timeseriesMap('/force_example_controller/desired_trajectory/Data')}, {'X'; 'Y'; 'Z'; 'phi'; 'theta'; 'psi'} , {'OFDesiredK'}, true)

% plot_multiple_ts_subplots({timeseriesMap('/franka_state_controller/franka_states/TauJ')})
% plot_multiple_ts_subplots({alignedMap('/franka_state_controller/franka_states/Q'); alignedMap('/FOB_controller/desired_trajectory/Data')}, joint_names, {'q'; 'q_d'})




% plot_multiple_ts_subplots({alignedMap('/FOB_controller/tau_frc_hat/Data')}, joint_names, {'tau_frc_hat'})

% time = timeseriesMap('/franka_state_controller/franka_states/Dq').Time;
% 
% ddq = diff(timeseriesMap('/franka_state_controller/franka_states/Dq').Data(:, 7));
% dt = mean(diff(time));
% 
% ddq = - ddq ./ dt;
% [b,a] = butter(1, 10/15);
% ddq = filter(b, a, ddq);
% tauJ = filter(b, a, timeseriesMap('/franka_state_controller/franka_states/TauJ').Data(:, 7));
% tauJ = tauJ / 0.05;
% 
% 
% figure()
% plot(time(1:end-1), tauJ(1:end-1));
% hold on;
% plot(time(1:end-1), ddq);
% legend('tau', 'ddq');

% plot_multiple_ts_subplots({timeseriesMap('/FOB_controller/tau_frc_ref/Data')}, joint_names, {'tau_frc_hat'})

% plot_multiple_ts_subplots({timeseriesMap('/FOB_controller/tau_frc_ref/Data')}, joint_names, {'estimated tau_{frc}'})

% plot_multiple_ts_subplots({timeseriesMap('/franka_state_controller/franka_states/TauJD')}, joint_names, {'tau_{Jd}'})

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

