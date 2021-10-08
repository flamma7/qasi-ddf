function [] = print_buffer_stats(share_buffer)

    meas_types = ["modem_range", "modem_azimuth", "sonar_range", "sonar_azimuth", "sonar_range_implicit", "sonar_azimuth_implicit"];
    meas_columns = ["type", "index", "start_x1", "start_x2", "data"];
    meas_type_col = find(meas_columns == "type");

    % Sonar Range
    ind = find(meas_types == "sonar_range");
    num = length( find(share_buffer(:,meas_type_col) == ind) );
    disp("Sonar Range: " + int2str(num))
    % Sonar Range Implicit
    ind = find(meas_types == "sonar_range_implicit");
    num = length( find(share_buffer(:,meas_type_col) == ind) );
    disp("Sonar Range Implicit: " + int2str(num))
    % Sonar Azimuth
    ind = find(meas_types == "sonar_azimuth");
    num = length( find(share_buffer(:,meas_type_col) == ind) );
    disp("Sonar Azimuth: " + int2str(num))
    % Sonar Azimuth Implicit
    ind = find(meas_types == "sonar_range_implicit");
    num = length( find(share_buffer(:,meas_type_col) == ind) );
    disp("Sonar Azimuth Implicit: " + int2str(num))
end