function [ledger] = add_meas(ledger, agent, type, index, start_x1, start_x2, data)

    % Create measurement row
    % Locate row in ledger & add

    % Measurement vector
    meas_columns = ["type", "index", "start_x1", "start_x2", "data"];

    types = ["modem_range", "modem_azimuth", "sonar_range", "sonar_azimuth"];
    type_num = find(types == type); % Returns the index of the measurement
    if length(type_num) < 1
        disp("Could not locate meas type: " + type);
        assert(0)
    end
    data_row = [type_num, index, start_x1, start_x2, data];

    % Add row to ledger
    % Find first nonzero row for a certain column
    agent_ledger = get_ledger(ledger, agent);
    zero_rows = find(agent_ledger(:,1)==0);
    first_zero_row = zero_rows(1,1);

    % Set the row
    inc = length(meas_columns);
    col = inc*(agent-1)+1 : inc*agent;
    ledger(first_zero_row, col) = data_row;
end