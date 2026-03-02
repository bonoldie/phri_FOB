function timeseriesMap = parseFextTorque(msgs, times, timeseriesMap, key)
% Parses /franka_state_controller/F_ext messages
% Extracts Wrench.Torque [X Y Z] for ALL message elements

    n = numel(msgs);

    data = zeros(n, 3);
    tvec = zeros(n, 1);

    for i = 1:n
        torque = msgs{i}.Wrench.Torque;

        data(i, :) = [ ...
            torque.X, ...
            torque.Y, ...
            torque.Z ];

        tvec(i) = times(i);
    end

    if ~isKey(timeseriesMap, key)
        timeseriesMap(key) = timeseries(data, tvec);
        timeseriesMap(key).Name = key;
    else
        timeseriesMap(key) = addsample( ...
            timeseriesMap(key), ...
            'Time', tvec, ...
            'Data', data);
    end
end