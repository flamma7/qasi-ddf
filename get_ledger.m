function [agent_ledger] = get_ledger(ledger, agent)
    meas_columns = ["type", "index", "start_x1", "start_x2", "data"];
    inc = length(meas_columns);
    agent_ledger = ledger(:, inc*(agent-1)+1:inc*agent);
end