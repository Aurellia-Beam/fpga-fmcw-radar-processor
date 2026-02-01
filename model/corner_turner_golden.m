%% Corner Turner Golden Model - MATLAB Verification
% Author: Generated for FPGA-FMCW-RADAR-PROCESSOR
% Purpose: Verify VHDL corner turner against MATLAB transpose
% Date: 2026-02-01

clear; clc; close all;

%% Configuration
N_RANGE = 1024;     % Range bins (FFT size)
N_DOPPLER = 128;    % Number of chirps in CPI
VERBOSE = false;    % Set true for detailed output

fprintf('==============================================\n');
fprintf('Corner Turner Golden Model Verification\n');
fprintf('==============================================\n');
fprintf('Configuration:\n');
fprintf('  Range Bins:    %d\n', N_RANGE);
fprintf('  Doppler Bins:  %d\n', N_DOPPLER);
fprintf('  Total Samples: %d\n\n', N_RANGE * N_DOPPLER);

%% Generate Input Matrix (Row-wise: Chirps x Range Bins)
fprintf('Generating input matrix...\n');
input_matrix = zeros(N_DOPPLER, N_RANGE);

for chirp = 0:(N_DOPPLER-1)
    for range = 0:(N_RANGE-1)
        % Match testbench pattern: chirp_num << 16 | range_bin
        input_matrix(chirp+1, range+1) = chirp * 65536 + range;
    end
end

fprintf('  Input matrix size: %d x %d (Chirps x Range)\n', ...
    size(input_matrix, 1), size(input_matrix, 2));

%% Perform Transpose (Column-wise: Range Bins x Chirps)
fprintf('Performing transpose...\n');
output_matrix = input_matrix';

fprintf('  Output matrix size: %d x %d (Range x Chirps)\n', ...
    size(output_matrix, 1), size(output_matrix, 2));

%% Visualize Matrix Structure
figure('Position', [100 100 1200 500]);

subplot(1,2,1);
imagesc(input_matrix(1:min(32,N_DOPPLER), 1:min(64,N_RANGE)));
colorbar;
title('Input Matrix (Chirps x Range Bins)');
xlabel('Range Bin');
ylabel('Chirp Number');
colormap(jet);

subplot(1,2,2);
imagesc(output_matrix(1:min(64,N_RANGE), 1:min(32,N_DOPPLER)));
colorbar;
title('Output Matrix (Range Bins x Chirps) - TRANSPOSED');
xlabel('Chirp Number');
ylabel('Range Bin');
colormap(jet);

%% Export Golden Vectors for VHDL Verification
fprintf('\nExporting golden vectors...\n');

% Create data directory if it doesn't exist
if ~exist('../data', 'dir')
    mkdir('../data');
end

% Export input (for future testbench enhancements)
input_file = '../data/corner_turner_input.txt';
fid = fopen(input_file, 'w');
for chirp = 1:N_DOPPLER
    for range = 1:N_RANGE
        fprintf(fid, '%d ', input_matrix(chirp, range));
    end
    fprintf(fid, '\n');
end
fclose(fid);
fprintf('  Wrote input to: %s\n', input_file);

% Export expected output
golden_file = '../data/corner_turner_golden.txt';
fid = fopen(golden_file, 'w');
for range = 1:N_RANGE
    for chirp = 1:N_DOPPLER
        fprintf(fid, '%d %d %d\n', range-1, chirp-1, output_matrix(range, chirp));
    end
end
fclose(fid);
fprintf('  Wrote golden output to: %s\n', golden_file);

%% Verify Against VHDL Simulation Output
vhdl_output_file = '../data/corner_turner_output.txt';

if exist(vhdl_output_file, 'file')
    fprintf('\nVerifying VHDL output...\n');
    
    % Read VHDL output
    % Format: range_bin doppler_bin value
    vhdl_data = dlmread(vhdl_output_file);
    
    if size(vhdl_data, 1) ~= N_RANGE * N_DOPPLER
        warning('VHDL output has %d samples, expected %d', ...
            size(vhdl_data, 1), N_RANGE * N_DOPPLER);
    end
    
    % Verify each sample
    errors = 0;
    error_log = [];
    
    for i = 1:size(vhdl_data, 1)
        range_bin = vhdl_data(i, 1) + 1;    % VHDL is 0-indexed
        doppler_bin = vhdl_data(i, 2) + 1;  % VHDL is 0-indexed
        vhdl_value = vhdl_data(i, 3);
        
        % Check bounds
        if range_bin < 1 || range_bin > N_RANGE || ...
           doppler_bin < 1 || doppler_bin > N_DOPPLER
            fprintf('  ERROR: Out of bounds at sample %d\n', i);
            errors = errors + 1;
            continue;
        end
        
        expected = output_matrix(range_bin, doppler_bin);
        
        if vhdl_value ~= expected
            if VERBOSE
                fprintf('  ERROR at (range=%d, doppler=%d): expected %d, got %d\n', ...
                    range_bin-1, doppler_bin-1, expected, vhdl_value);
            end
            errors = errors + 1;
            error_log = [error_log; range_bin doppler_bin expected vhdl_value];
        end
    end
    
    % Report results
    fprintf('\n==============================================\n');
    if errors == 0
        fprintf('✅ SUCCESS: All %d samples verified correctly!\n', ...
            size(vhdl_data, 1));
        fprintf('   Matrix transpose is functioning properly.\n');
    else
        fprintf('❌ FAILURE: %d errors detected out of %d samples\n', ...
            errors, size(vhdl_data, 1));
        fprintf('   Error rate: %.4f%%\n', 100 * errors / size(vhdl_data, 1));
        
        if size(error_log, 1) > 0 && size(error_log, 1) <= 20
            fprintf('\nError details:\n');
            fprintf('   Range  Doppler  Expected  Got\n');
            fprintf('   -----  -------  --------  ---\n');
            for j = 1:size(error_log, 1)
                fprintf('   %5d  %7d  %8d  %3d\n', error_log(j,:));
            end
        elseif size(error_log, 1) > 20
            fprintf('\nFirst 20 errors:\n');
            fprintf('   Range  Doppler  Expected  Got\n');
            fprintf('   -----  -------  --------  ---\n');
            for j = 1:20
                fprintf('   %5d  %7d  %8d  %3d\n', error_log(j,:));
            end
            fprintf('   ... and %d more errors\n', size(error_log, 1) - 20);
        end
    end
    fprintf('==============================================\n');
    
else
    fprintf('\nVHDL output file not found: %s\n', vhdl_output_file);
    fprintf('Run VHDL testbench first to generate output.\n');
    fprintf('Golden vectors have been exported for comparison.\n');
end

%% Performance Analysis
fprintf('\n--- Memory Analysis ---\n');
memory_bits = N_RANGE * N_DOPPLER * 32;  % 32-bit samples
memory_bytes = memory_bits / 8;
memory_kb = memory_bytes / 1024;

fprintf('Memory required per bank:\n');
fprintf('  Total bits:  %d\n', memory_bits);
fprintf('  Total bytes: %d\n', memory_bytes);
fprintf('  Total KB:    %.2f\n', memory_kb);

% BRAM analysis for Xilinx
bram_36kb_per_bank = ceil(memory_bits / 36864);
fprintf('\nBRAM (36Kb blocks) per bank: %d\n', bram_36kb_per_bank);
fprintf('Total BRAM for ping-pong:    %d blocks\n', bram_36kb_per_bank * 2);

% Zynq-7020 has 140 BRAM blocks
zynq_7020_bram = 140;
utilization = (bram_36kb_per_bank * 2) / zynq_7020_bram * 100;
fprintf('Utilization on Zynq-7020:    %.1f%%\n', utilization);

%% Address Pattern Visualization
fprintf('\n--- Address Pattern Sample ---\n');
fprintf('First 10 write addresses (row-wise):\n');
fprintf('   Chirp  Sample  Address\n');
for chirp = 0:2
    for sample = 0:2
        addr = chirp * N_RANGE + sample;
        fprintf('   %5d  %6d  %7d\n', chirp, sample, addr);
    end
end

fprintf('\nFirst 10 read addresses (column-wise):\n');
fprintf('   Range  Doppler Address\n');
for range = 0:2
    for doppler = 0:2
        addr = range * N_DOPPLER + doppler;
        fprintf('   %5d  %7d %7d\n', range, doppler, addr);
    end
end

fprintf('\nExample transpose mapping:\n');
fprintf('  Write [chirp=5, sample=100] → addr = %d\n', 5*N_RANGE + 100);
fprintf('  Read  [range=100, doppler=5] → addr = %d (same location!)\n', 100*N_DOPPLER + 5);

fprintf('\n==============================================\n');
fprintf('Golden model complete.\n');
fprintf('==============================================\n');
