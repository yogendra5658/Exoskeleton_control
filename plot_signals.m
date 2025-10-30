function plot_signals(time, signals, labels, colors, ylbl, ttl, marker_times, marker_labels)
    n = numel(signals);
    for i = 1:n
        subplot(n,1,i);
        plot(time, signals{i}, colors{i}, 'LineWidth', 1.2); hold on; grid on;
        xlabel('Time (ms)'); ylabel(ylbl{i}); title([ttl ' - ' labels{i}]);
        for j = 1:length(marker_times)
            xline(marker_times(j), '--k', marker_labels{j}, ...
                'LineWidth', 1.5, 'LabelOrientation','horizontal','Color','m');
        end
    end
end
