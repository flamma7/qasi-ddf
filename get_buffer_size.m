function [explicit_cnt, implicit_cnt] = pull_buffer( share_buffer )
    % count only explicit measurements
    % do not count range

    meas_types = ["modem_range", "modem_azimuth", "sonar_range", "sonar_azimuth", "sonar_range_implicit", "sonar_azimuth_implicit"];
    meas_columns = ["type", "index", "start_x1", "start_x2", "data"];
    meas_type_col = find(meas_columns == "type");

    explicit_cnt = 0;
    implicit_cnt = 0;

    % Sonar Range
    ind = find(meas_types == "sonar_range");
    num = length( find(share_buffer(:,meas_type_col) == ind) );
    explicit_cnt = explicit_cnt + num;

    % Sonar Azimuth
    ind = find(meas_types == "sonar_azimuth");
    num = length( find(share_buffer(:,meas_type_col) == ind) );
    explicit_cnt = explicit_cnt + num;

    % Sonar Range Implicit
    ind = find(meas_types == "sonar_range_implicit");
    num = length( find(share_buffer(:,meas_type_col) == ind) );
    implicit_cnt = implicit_cnt + num;

    % Sonar Azimuth Implicit
    ind = find(meas_types == "sonar_range_implicit");
    num = length( find(share_buffer(:,meas_type_col) == ind) );
    implicit_cnt = implicit_cnt + num;
    
end
