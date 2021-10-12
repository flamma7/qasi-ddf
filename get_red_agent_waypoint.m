function [waypoint] = get_red_agent_waypoint(x_gt, agent_to_follow, offset_x, offset_y, STATES)

    blue_pos = x_gt(STATES*(agent_to_follow-1)+1:STATES*(agent_to_follow-1)+2, 1);
    blue_pos(1) = blue_pos(1) + offset_x;
    blue_pos(2) = blue_pos(2) + offset_y;
    waypoint = blue_pos;
end