function [] = plot_norm_error(error)
    
    norms = zeros(length(error));
    for i = 1:length(error)
        norms(i) = mean(error(:,i) .* error(:,i));
    end
    figure;
    plot(norms);
    title("MSE vs Time");
    ylabel("MSE");
    xlabel("Time");
    mean(norms, 'all')
end