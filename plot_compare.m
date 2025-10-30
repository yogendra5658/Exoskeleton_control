% % ---- Step 3: Extract Signals ----
% %Euler angle raw data
% roll_raw  = data.roll_raw; pitch_raw = data.pitch_raw; yaw_raw   = data.yaw_raw;
% 
% %Euler angle calibrated data
% roll_cal  = data.roll_cal; pitch_cal = data.pitch_cal; yaw_cal   = data.yaw_cal;
% 
% %Quaternion
% qw = data.qw; qx = data.qx;qy = data.qy; qz = data.qz;
% 
% %acceleration raw data
% ax = data.ax; ay = data.ay; az = data.az;
% 
% %aceeleration calibrated data
% ax_cal = data.ax_cal; ay_cal = data.ay_cal; az_cal = data.az_cal;
% 
% %gyro data
% gx = data.gx; gy = data.gy; gz = data.gz;
% 
% gx_cal = data.gx_cal; gy_cal = data.gy_cal; gz_cal = data.gz_cal;
% 
% %emg data
% semg_raw      = data.semg_raw;
% semg_filtered = data.semg_filtered;
% semg_envelope = data.semg_envelope;
% 
% 
% % ---- Step 4: Extract Marker Info ----
% marker_rows = ~cellfun(@isempty, data.marker);   % rows where marker exists
% marker_times = time_ms(marker_rows);             % corresponding times
% %marker_labels = data.marker(marker_rows);        % marker labels (cell array)
% % Extract only first 3 characters of each marker
% marker_labels = cellfun(@(s) s(1:min(2,end)), data.marker(marker_rows), 'UniformOutput', false);
% 
% 
% % ---- Step 5: Plot ----
% figure;
% % --- Roll ---
% subplot(3,1,1);
% plot(time_ms, roll_raw, 'r', 'LineWidth', 1.2); hold on; grid on;
% xlabel('Time (ms)'); ylabel('Roll (°)');
% title('Roll vs Time');
% for i = 1:length(marker_times)
%     xline(marker_times(i), '--k', marker_labels{i}, ...
%         'LineWidth', 1.5, 'LabelOrientation','horizontal','Color','m');
% end
% % --- Pitch ---
% subplot(3,1,2);
% plot(time_ms, pitch_raw, 'g', 'LineWidth', 1.2); hold on; grid on;
% xlabel('Time (ms)'); ylabel('Pitch (°)');
% title('Pitch vs Time');
% for i = 1:length(marker_times)
%     xline(marker_times(i), '--k', marker_labels{i}, ...
%         'LineWidth', 1.5, 'LabelOrientation','horizontal','Color','m');
% end
% % --- Yaw ---
% subplot(3,1,3);
% plot(time_ms, yaw_raw, 'b', 'LineWidth', 1.2); hold on; grid on;
% xlabel('Time (ms)'); ylabel('Yaw (°)');
% title('Yaw vs Time');
% for i = 1:length(marker_times)
%     xline(marker_times(i), '--k', marker_labels{i}, ...
%         'LineWidth', 1.5, 'LabelOrientation','horizontal','Color','m');
% end
% 
% % ---- Step 5: Plot ----
% figure;
% 
% % --- Roll ---
% subplot(3,1,1);
% plot(time_ms, roll_raw, 'r', 'LineWidth', 1.2); grid on;
% xlabel('Time (ms)'); ylabel('Roll (°)'); title('Roll vs Time');
% yl = ylim;  % get axis limits for patch height
% shade_regions(gca, marker_times, marker_labels, yl);
% 
% % --- Pitch ---
% subplot(3,1,2);
% plot(time_ms, pitch_raw, 'g', 'LineWidth', 1.2); grid on;
% xlabel('Time (ms)'); ylabel('Pitch (°)'); title('Pitch vs Time');
% yl = ylim;
% shade_regions(gca, marker_times, marker_labels, yl);
% 
% % --- Yaw ---
% subplot(3,1,3);
% plot(time_ms, yaw_raw, 'b', 'LineWidth', 1.2); grid on;
% xlabel('Time (ms)'); ylabel('Yaw (°)'); title('Yaw vs Time');
% yl = ylim;
% shade_regions(gca, marker_times, marker_labels, yl);
clear; clc;

% ---- Load CSV ----
filename = '/home/yogendra/Documents/EMG_data_log/Marker_imu_emg/final_working_code/imu_semg_log_20251029_191839.csv';
%filename = '/home/yogendra/Documents/EMG_data_log/Marker_imu_emg/final_working_code/moukheet_10_times.csv';
opts = detectImportOptions(filename, 'CommentStyle', '#');
data = readtable(filename, opts);

% ---- Time vector ----
dt = datetime(data.pc_timestamp, 'InputFormat','yyyy-MM-dd HH:mm:ss.SSS');
time_ms = milliseconds(dt - dt(1));
time_sec = time_ms/1000;

% ---- Absolute-value version ----
numVars = varfun(@isnumeric, data, 'OutputFormat', 'uniform');
data_abs = data;
data_abs{:, numVars} = abs(data{:, numVars});

% ---- Marker info ----
marker_rows   = ~cellfun(@isempty, data.marker);
marker_times  = time_ms(marker_rows);
marker_labels = cellfun(@(s) s(1:min(3,end)), data.marker(marker_rows), 'UniformOutput', false);

%% -------- Plotting --------

% Euler raw
figure('Name','Euler Raw');
plot_signals(time_ms, {data.roll_raw, data.pitch_raw, data.yaw_raw}, ...
    {'Roll','Pitch','Yaw'}, {'r','g','b'}, ...
    {'(°)','(°)','(°)'}, 'Euler Raw', marker_times, marker_labels);

% Euler calibrated
figure('Name','Euler Calibrated');
plot_signals(time_ms, {data.roll_cal, data.pitch_cal, data.yaw_cal}, ...
    {'Roll','Pitch','Yaw'}, {'r','g','b'}, ...
    {'(°)','(°)','(°)'}, 'Euler Cal', marker_times, marker_labels);

% Quaternion
figure('Name','Quaternion');
plot_signals(time_ms, {data.qw, data.qx, data.qy, data.qz}, ...
    {'qw','qx','qy','qz'}, {'k','r','g','b'}, ...
    {'','', '', ''}, 'Quaternion', marker_times, marker_labels);

% Acc Raw
figure('Name','Acceleration Raw');
plot_signals(time_ms, {data.ax, data.ay, data.az}, ...
    {'ax','ay','az'}, {'r','g','b'}, ...
    {'(m/s^2)','(m/s^2)','(m/s^2)'}, 'Acc Raw', marker_times, marker_labels);

% Acc Cal
figure('Name','Acceleration Cal');
plot_signals(time_ms, {data.ax_cal, data.ay_cal, data.az_cal}, ...
    {'ax','ay','az'}, {'r','g','b'}, ...
    {'(m/s^2)','(m/s^2)','(m/s^2)'}, 'Acc Cal', marker_times, marker_labels);

% Gyro Raw
figure('Name','Gyro Raw');
plot_signals(time_ms, {data.gx, data.gy, data.gz}, ...
    {'gx','gy','gz'}, {'r','g','b'}, ...
    {'(°/s)','(°/s)','(°/s)'}, 'Gyro Raw', marker_times, marker_labels);

% Gyro Cal
figure('Name','Gyro Cal');
plot_signals(time_ms, {data.gx_cal, data.gy_cal, data.gz_cal}, ...
    {'gx','gy','gz'}, {'r','g','b'}, ...
    {'(°/s)','(°/s)','(°/s)'}, 'Gyro Cal', marker_times, marker_labels);

% EMG
figure('Name','sEMG');
plot_signals(time_ms, {data.semg_raw, data.semg_filtered, data.semg_envelope}, ...
    {'Raw','Filtered','Envelope'}, {'r','g','b'}, ...
    {'','', ''}, 'sEMG', marker_times, marker_labels);

%% -------- Absolute data plotting --------
figure('Name','Euler Raw Abs');
plot_signals(time_ms, {data_abs.roll_raw, data_abs.pitch_raw, data_abs.yaw_raw}, ...
    {'Roll','Pitch','Yaw'}, {'r','g','b'}, ...
    {'(°)','(°)','(°)'}, 'Euler Raw Abs', marker_times, marker_labels);


%% -------- Absolute data plotting --------
figure('Name','Euler Cal Abs');
plot_signals(time_ms, {data_abs.roll_cal, data_abs.pitch_cal, data_abs.yaw_cal}, ...
    {'Roll','Pitch','Yaw'}, {'r','g','b'}, ...
    {'(°)','(°)','(°)'}, 'Euler Raw Abs', marker_times, marker_labels);

% Euler calibrated
figure('Name','Euler Calibrated');
plot_signals(time_ms, {data.roll_cal, data.pitch_cal, data.yaw_cal, data.semg_raw}, ...
    {'Roll','Pitch','Yaw','sEMG'}, {'r','g','b','c'}, ...
    {'(°)','(°)','(°)','sEMG'}, 'Euler Cal', marker_times, marker_labels);