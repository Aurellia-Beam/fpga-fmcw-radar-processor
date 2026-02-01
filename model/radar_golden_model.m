clear; clc; close all;
%Radar Params
c = 3e8;
fc = 2.4e9;             
lambda = c / fc;
Tc = 1e-3;              
B = 100e6;              
slope = B / Tc;         
fs = 2e6;               
Ns = floor(fs * Tc);    
N_chirps = 64;          

%% 2. Target Definition
target_range = 850;     
target_vel = 15;        
t = (0:Ns-1) / fs;
mix_data = zeros(Ns, N_chirps);

for i = 1:N_chirps
    t_chirp = (i-1) * Tc;
    r_t = target_range + target_vel * t_chirp;
    td = 2 * r_t / c;
    phase_term = 2 * pi * (slope * td .* t + fc * td);
    noise = 0.001 * (randn(size(t)) + 1j*randn(size(t)));
    signal = exp(1j * phase_term) + noise;
    
    mix_data(:, i) = signal.';
end

% Range FFT (Windowed)
win_range = hamming(Ns);
range_fft = fft(mix_data .* win_range, Ns, 1);

% Doppler FFT (Windowed + Shift)
win_doppler = hamming(N_chirps).';
doppler_fft = fft(range_fft .* win_doppler, N_chirps, 2);
rd_map = fftshift(doppler_fft, 2);

% Plot
figure(1);
range_axis = (0:Ns-1) * (c * fs) / (2 * slope * Ns);
vel_axis = linspace(-N_chirps/2, N_chirps/2-1, N_chirps) * (lambda / (2 * N_chirps * Tc));

imagesc(vel_axis, range_axis, 20*log10(abs(rd_map)));
axis xy; colormap('jet'); colorbar;
title('Golden Model: Range-Doppler Map');
xlabel('Velocity (m/s)'); ylabel('Range (m)');
ylim([0 2000]); % Focus on relevant range

%% 5. Export for Vivado Testbench
%% 5. Export for Vivado Testbench
% We verify the FIRST CHIRP first.
chirp_1 = mix_data(:, 1);
scale = 2^14; % Scale up to fill 16-bit range roughly

% --- FIX: Robust File Path Handling ---
% This checks if "data" folder exists; if not, it creates it.
if ~exist('../data', 'dir')
    mkdir('../data');
end

filename = '../data/golden_input_chirp.txt';
fid = fopen(filename, 'w');

% Check if it actually worked this time
if fid == -1
    % Fallback: If ../data fails, just save in the current folder
    fprintf('Warning: Could not save to ../data. Saving in current folder instead.\n');
    fid = fopen('golden_input_chirp.txt', 'w');
end

for k = 1:length(chirp_1)
    re_val = floor(real(chirp_1(k)) * scale);
    im_val = floor(imag(chirp_1(k)) * scale);
    fprintf(fid, '%d %d\n', re_val, im_val);
end
fclose(fid);

fprintf('File saved\n');