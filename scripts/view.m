clear;
% close all;
clc;

DoF = 7;
joint_names = arrayfun(@(i) ['panda_joint_' num2str(i)], 0:(DoF-1), 'UniformOutput', false);

% bag = rosbagreader('bags/baseline_force_K_1_0_5.bag');
% bag = rosbagreader('bags/PD_tracking.bag');

bag = rosbagreader('bags0902/bags_DEFINITIVAH.bag');

% Get list of topics in the bag
topics = bag.AvailableTopics.Properties.RowNames;

% Initialize a structure to store timeseries
timeseriesMap = containers.Map();


topics_to_parse = { '/franka_state_controller/franka_states', '/FOB_controller/tau_frc_hat', '/FOB_controller/desired_trajectory', '/dynamic_reconfigure_FOB_param_node/parameter_updates', '/netft/netft_data','/franka_state_controller/F_ext'};% '/FOB_controller/tau_ext_hat_filtered', '/FOB_controller/tau_frc_ref', '/FOB_controller/desired_trajectory',

for i=1:size(topics_to_parse, 2)
    topic = topics_to_parse{i};

    msgs = readMessages(select(bag, 'Topic', topic), 'DataFormat','struct');
    sel = select(bag, 'Topic', topic);
    timeStamps = sel.MessageList.Time;
    
    msgStructs = readMessages(sel, 'DataFormat', 'struct');
    %-------------------------------------------------------------
    if strcmp(topic, '/dynamic_reconfigure_FOB_param_node/parameter_updates')

        N = numel(msgStructs);
        FOB_lower = nan(N,1);
        FOB_upper = nan(N,1);
    
        for k = 1:N
            if ~isfield(msgStructs{k}, 'Bools')
                continue
            end
            bools = msgStructs{k}.Bools;
            for b = 1:numel(bools)
                switch bools(b).Name
                    case 'FOB_lower'
                        FOB_lower(k) = bools(b).Value;
                    case 'FOB_upper'
                        FOB_upper(k) = bools(b).Value;
                end
            end
        end

        % dynamic_reconfigure publishes only on change
        FOB_lower = fillmissing(FOB_lower,'previous');
        FOB_upper = fillmissing(FOB_upper,'previous');
    
        % Rising edges
        FOB_lower_rise_idx = find(diff(FOB_lower) == 1) + 1;
        FOB_upper_rise_idx = find(diff(FOB_upper) == 1) + 1;
    
        % Store times (shift later like others)
        FOB_lower_times = timeStamps(FOB_lower_rise_idx);
        FOB_upper_times = timeStamps(FOB_upper_rise_idx);
    
    end
    %------------------------------------------------------------------

    if size(msgStructs, 1) < 1
        continue
    end

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
                    try
                        timeseriesMap = parseFextTorque(msgs, timeStamps, timeseriesMap, topic);
                    catch
                        continue
                    end
                end
            end
        catch
            % Skip fields that can't be processed
            continue
        end
    end
 
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
    FOB_lower_times = FOB_lower_times - t0;
    FOB_upper_times = FOB_upper_times - t0;

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


alignedMapGravity = alignedMap('/franka_state_controller/franka_states/Q');

for i=1:size(alignedMapGravity.Data,1)
    gravity=get_GravityVector(alignedMapGravity.Data(i,:));
    alignedMapGravity.Data(i,:)=gravity;
end

a=alignedMap('/franka_state_controller/franka_states/TauJ');

a.Data=a.Data-alignedMapGravity.Data;

tiralafuori=alignedMap('/franka_state_controller/franka_states/OFExtHatK');


ref=timeseries(alignedMap('/FOB_controller/desired_trajectory/Data').Data(:,3),...
    alignedMap('/franka_state_controller/franka_states/Q').Time);

forceMeasured=timeseries(alignedMap('/franka_state_controller/franka_states/OFExtHatK').Data(:,3)+2,...
     alignedMap('/franka_state_controller/franka_states/Q').Time);



plot_multiple_ts_subplots({alignedMap('/franka_state_controller/franka_states/Q'); alignedMap('/FOB_controller/desired_trajectory/Data')}, ...
    joint_names, {'q'; 'q_d'},1,FOB_lower_times,FOB_upper_times)
%plot_multiple_ts_subplots({ref,  forceMeasured}, joint_names , {'ref','measured'},1,FOB_lower_times,FOB_upper_times);
%plot_multiple_ts_subplots({a,  alignedMap('/franka_state_controller/franka_states/TauJD')}, joint_names , {'tau_j','tau_jD'},1,FOB_lower_times,FOB_upper_times)
%plot_multiple_ts_subplots({alignedMap('/netft/netft_data/Z'),  alignedMap('/FOB_controller/desired_trajectory/Data')},  joint_names, ...
    %{'tauJ', 'tauJ_d'},1,FOB_lower_times,FOB_upper_times)
% plot_multiple_ts_subplots({timeseriesMap('/force_example_controller/desired_trajectory/Data')}, {'X'; 'Y'; 'Z'; 'phi'; 'theta'; 'psi'} , {'OFDesiredK'}, true)

% plot_multiple_ts_subplots({timeseriesMap('/franka_state_controller/franka_states/TauJ')})
% plot_multiple_ts_subplots({alignedMap('/franka_state_controller/franka_states/Q'); alignedMap('/FOB_controller/desired_trajectory/Data')}, joint_names, {'q'; 'q_d'})
