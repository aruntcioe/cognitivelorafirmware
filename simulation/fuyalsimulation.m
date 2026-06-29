% =========================================================================
% LORA TRUE PHYSICAL SIMULATOR & DATASET GENERATOR (BOTTOM-UP APPROACH)
% Target: 200 Packets -> 20 Windowed Feature Rows for AI Training
% Additions: 433MHz Band Channelization, Guard Bands, and Thermal Noise Fix
% Output: 'lora_dataset_physical.csv' 
% =========================================================================
clear; clc; close all;

%% 1. GLOBAL ENVIRONMENT, SPECTRUM & RF PHYSICS SETUP
fs = 1e6;                       % 1 MHz Sample Rate (Sufficient for 125kHz BW)
bw = 125e3;                     % 125 kHz Bandwidth
total_packets_to_simulate = 200;% 200 packets / 10 per window = 20 rows

% --- Channelization Math (433.05 MHz to 433.92 MHz) ---
band_start = 433.05e6;
band_end   = 433.92e6;
gap_hz     = 0.25 * bw;         % 25% gap = 31.25 kHz
channel_footprint = gap_hz + bw + gap_hz; % 187.5 kHz per channel block

channels_freq = [];
current_f = band_start + gap_hz + (bw/2); % Center of the first valid channel

% Calculate all valid channels that fit in the band
while (current_f + (bw/2) + gap_hz) <= band_end
    channels_freq = [channels_freq, current_f];
    current_f = current_f + channel_footprint;
end
num_channels = length(channels_freq);

fprintf('=================================================================\n');
fprintf('     LORA 433MHz SPECTRUM ALLOCATION (125kHz BW + 25%% GAPS)      \n');
fprintf('=================================================================\n');
for i = 1:num_channels
    fprintf(' [Channel %d] Center Freq: %.5f MHz\n', i, channels_freq(i)/1e6);
end
fprintf('=================================================================\n\n');

% --- Link Budget & Physical Constants ---
P_tx_dBm    = 14.0;             % Standard LoRa Transmit Power (14 dBm)
PL_0        = 25.0;             % Path loss at 1 meter (approx for 433MHz)
T_kelvin    = 290;              % System temperature
k_boltzmann = 1.38e-23;         % Boltzmann constant
NF_dB       = 6.0;              % Gateway Receiver Noise Figure

% CORRECTED: Thermal Noise Floor calculation (Result is exactly in dBm)
P_noise_dBm = 10*log10(k_boltzmann * T_kelvin * bw * 1000) + NF_dB; 
P_noise_W   = 10^((P_noise_dBm - 30)/10);

% Window Boundaries Selection
WINDOW_SIZE_PACKETS = 10;       
WINDOW_SIZE_TIME    = 5.0;      

% Sliding Window Memory Storage Queues
packet_counter = 0;             
global_clock   = 0.0;           
q_timestamps = []; q_rssi = []; q_snr = []; 
q_cfo = []; q_crc = []; q_toa = []; q_channel = [];            

master_dataset_table = table();

%% 2. DYNAMIC PACKET EMULATION & PHYSICS PROPAGATION LOOP
for p_id = 1:total_packets_to_simulate
    
    % --- PART A: SIMULATING FIRMWARE LINK ADAPTATION & HOPPING ---
    current_sf = randi([7, 12]);   
    current_cr = randi([1, 4]);   
    payload_bytes = randi([15, 50]); 
    
    % Randomly pick one of the calculated channels for this packet
    current_ch_id = randi([1, num_channels]);
    current_rf_freq = channels_freq(current_ch_id);
    
    phy = LoRaPHY(current_rf_freq, current_sf, bw, fs);
    phy.has_header = 1;          
    phy.cr = current_cr; 
    phy.crc = 1;          
    phy.preamble_len = 8;          
    
    packet_toa_ms = phy.time_on_air(payload_bytes);
    global_clock = global_clock + (packet_toa_ms / 1000) + 0.120;
    
    % --- PART B: DEFINING PHYSICAL SCENARIOS (THE "CLASSES") ---
    % DISTANCES TWEAKED to keep SNR inside the realistic LoRa decoding threshold
    if p_id <= 50
        % CLASS 0: Nominal Urban Environment
        current_env_label = 0;
        distance = 600 + randn()*50;       % MODIFIED: 600m
        path_loss_exp = 3.8;               
        shadowing = randn() * 2;           
        is_jamming = 0;
        cfo_drift_hz = 100 + randn()*10;   
        
    elseif p_id > 50 && p_id <= 100
        % CLASS 1: Active Jamming Attack
        current_env_label = 1;
        distance = 600 + randn()*50;       % MODIFIED: 600m
        path_loss_exp = 3.8;               
        shadowing = randn() * 2;
        is_jamming = 1;                    
        jammer_power_dBm = -85 + randn()*2;% Jammer injecting power
        cfo_drift_hz = 350 + randn()*50;   
        
    elseif p_id > 100 && p_id <= 150
        % CLASS 2: Severe Fading / Edge of Cell
        current_env_label = 2;
        distance = 1200 + randn()*100;     % MODIFIED: 1.2km (Pushes SNR down)
        path_loss_exp = 4.2;               
        shadowing = randn() * 6;           
        is_jamming = 0;
        cfo_drift_hz = 150 + randn()*20;
        
    else
        % CLASS 3: Excellent Link (Close Proximity / Line of Sight)
        current_env_label = 3;
        distance = 150 + randn()*20;       % 150m away
        path_loss_exp = 2.2;               
        shadowing = randn() * 0.5;         
        is_jamming = 0;
        cfo_drift_hz = 30 + randn()*5;     
    end
    
    % --- PART C: RF LINK BUDGET MATHEMATICS ---
    path_loss_dB = PL_0 + 10 * path_loss_exp * log10(distance) + shadowing;
    P_rx_dBm = P_tx_dBm - path_loss_dB;
    P_rx_W = 10^((P_rx_dBm - 30)/10);
    
    % --- PART D: HARDWARE SYNTHESIS & WAVEFORM CORRUPTION ---
    tx_payload = uint8(randi([0, 255], payload_bytes, 1));
    encoded_symbols = phy.encode(tx_payload);
    tx_signal = phy.modulate(encoded_symbols);
    
    tx_signal = tx_signal / rms(tx_signal); 
    tx_scaled = tx_signal * sqrt(P_rx_W);
    
    t_vec = (0:length(tx_scaled)-1)' / fs;
    tx_scaled = tx_scaled .* exp(1i * 2 * pi * cfo_drift_hz * t_vec);
    
    noise_wave = sqrt(P_noise_W/2) * (randn(size(tx_scaled)) + 1i*randn(size(tx_scaled)));
    
    P_jam_W = 0;
    jammer_wave = 0;
    if is_jamming
        P_jam_W = 10^((jammer_power_dBm - 30)/10);
        jammer_freq = 25e3; 
        jammer_wave = sqrt(P_jam_W) * exp(1i * 2 * pi * jammer_freq * t_vec);
    end
    
    rx_antenna = tx_scaled + noise_wave + jammer_wave;
    
    % --- PART E: GATEWAY HARDWARE METRICS MEASUREMENT ---
    P_total_W = P_rx_W + P_noise_W + P_jam_W;
    hardware_rssi = 10*log10(P_total_W) + 30; 
    hardware_snr = 10*log10(P_rx_W / (P_noise_W + P_jam_W));
    
    rx_baseband = rx_antenna / sqrt(P_total_W); 
    
    [decoded_symbols, estimated_cfo, ~] = phy.demodulate(rx_baseband);
    
    if isempty(decoded_symbols)
        packet_crc = 0; 
        final_cfo = 0;
        fprintf('ID:%3d | Lbl:%d | Ch:%d | Dist:%4.0fm | RSSI:%6.1f | SNR:%6.1f | Result: DROPPED\n', ...
            p_id, current_env_label, current_ch_id, distance, hardware_rssi, hardware_snr);
    else
        [~, checksum] = phy.decode(decoded_symbols);
        packet_crc = checksum;
        final_cfo = estimated_cfo; 
        
        status_str = 'PASS';
        if packet_crc == 0; status_str = 'FAIL'; end
        
        fprintf('ID:%3d | Lbl:%d | Ch:%d | Dist:%4.0fm | RSSI:%6.1f | SNR:%6.1f | Result: %s\n', ...
            p_id, current_env_label, current_ch_id, distance, hardware_rssi, hardware_snr, status_str);
    end
    
    % --- PART F: ENQUEUE CURRENT METRICS ---
    packet_counter = packet_counter + 1;
    q_timestamps   = [q_timestamps; global_clock];
    q_rssi         = [q_rssi; hardware_rssi];
    q_snr          = [q_snr; hardware_snr];
    q_cfo          = [q_cfo; final_cfo];
    q_crc          = [q_crc; packet_crc];
    q_toa          = [q_toa; packet_toa_ms];
    q_channel      = [q_channel; current_ch_id];
    
    active_window_time_span = q_timestamps(end) - q_timestamps(1);
        
    % --- PART G: EVALUATING SLIDING WINDOW BOUNDARIES ---
    if packet_counter >= WINDOW_SIZE_PACKETS || active_window_time_span >= WINDOW_SIZE_TIME
        
        mean_RSSI = mean(q_rssi);
        mean_SNR  = mean(q_snr);
        mean_CFO  = mean(q_cfo);
        mean_ToA  = mean(q_toa);
        
        % We take the MODE (most frequent) channel as the identifier for this window
        window_Channel_ID = mode(q_channel);
        
        var_RSSI = 0.0; var_SNR = 0.0;
        if length(q_rssi) > 1
            var_RSSI = var(q_rssi);
            var_SNR  = var(q_snr);
        end
        
        total_window_samples = length(q_crc);
        dropped_window_samples = sum(q_crc == 0);
        PLR = dropped_window_samples / total_window_samples;
        
        max_consecutive_crc_fails = 0; current_streak = 0;
        for i = 1:length(q_crc)
            if q_crc(i) == 0
                current_streak = current_streak + 1;
                max_consecutive_crc_fails = max(max_consecutive_crc_fails, current_streak);
            else
                current_streak = 0;
            end
        end
        
        % Added 'window_Channel_ID' to the feature row
        feature_row = table(mean_RSSI, var_RSSI, mean_SNR, var_SNR, mean_CFO, ...
                            PLR, max_consecutive_crc_fails, current_sf, current_cr, ...
                            mean_ToA, window_Channel_ID, current_env_label);
        
        master_dataset_table = [master_dataset_table; feature_row];
        
        packet_counter = 0;
        q_timestamps = []; q_rssi = []; q_snr = []; 
        q_cfo = []; q_crc = []; q_toa = []; q_channel = [];
    end
end

%% 3. EXPORT DATASET
fprintf('\n=================================================================\n');
fprintf('                 DATASET PIPELINE PROCESSING SUMMARY             \n');
fprintf('=================================================================\n');
output_filename = 'cha_inc_lora_dataset_physical.csv';
writetable(master_dataset_table, output_filename);
fprintf('SUCCESS: Saved %d Window Vectors as "%s"\n\n', size(master_dataset_table, 1), output_filename);
disp('First 5 Rows of Your Generated Dataset File:');
disp(master_dataset_table(1:min(5, size(master_dataset_table, 1)), :));