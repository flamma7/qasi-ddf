function [measurements] = get_measurements(agent_ledger, index)

    meas_columns = ["type", "index", "start_x1", "start_x2", "data"];
    index_column = find(meas_columns == "index"); % 2

    meas_rows = find(agent_ledger(:,index_column) == index  );
    measurements = agent_ledger(meas_rows, :);
end