function [share_buffer] = quantize_buffer( share_buffer )
    % Quantize each explicit measurement using 1 byte per measurement (256 values)

    meas_types = ["modem_range", "modem_azimuth", "sonar_range", "sonar_azimuth", "sonar_range_implicit", "sonar_azimuth_implicit"];
    meas_columns = ["type", "index", "start_x1", "start_x2", "data"];
    meas_type_col = find(meas_columns == "type");
    index_col = find(meas_columns == "index");
    startx1_col = find(meas_columns == "start_x1");
    startx2_col = find(meas_columns == "start_x2");
    data_col = find(meas_columns == "data");

    BINS = 256;

    MAX_RANGE_MEAS = 10;
    
    precision_range = MAX_RANGE_MEAS / BINS;
    precision_azimuth = 2*pi / BINS;

    % ceil(meas / precision) * precision

    for i = 1:size(share_buffer,1)
        meas = share_buffer(i,:);
        if meas(1,meas_type_col) == 0
            break
        end
        meas_type = meas_types( meas(1,meas_type_col) );
        start_x1 = meas(1,startx1_col);
        start_x2 = meas(1,startx2_col);
        data = meas(data_col);

        if meas_type == "sonar_range"
            quantized_data = ceil(data / precision_range) * precision_range;
            share_buffer(i,data_col) = quantized_data;
        elseif meas_type == "sonar_azimuth"
            quantized_data = ceil(data / precision_azimuth) * precision_azimuth;
            share_buffer(i,data_col) = quantized_data;
        end
    end
end