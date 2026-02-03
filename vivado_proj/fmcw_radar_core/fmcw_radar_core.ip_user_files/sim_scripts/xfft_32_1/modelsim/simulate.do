onbreak {quit -f}
onerror {quit -f}

vsim -voptargs="+acc"  -L xil_defaultlib -L xpm -L xbip_utils_v3_0_15 -L axi_utils_v2_0_11 -L c_reg_fd_v12_0_11 -L xbip_dsp48_wrapper_v3_0_7 -L xbip_pipe_v3_0_11 -L c_addsub_v12_0_21 -L c_shift_ram_v12_0_20 -L mult_gen_v12_0_24 -L floating_point_v7_1_21 -L cmpy_v6_0_27 -L xfft_v9_1_15 -L unisims_ver -L unimacro_ver -L secureip -lib xil_defaultlib xil_defaultlib.xfft_32 xil_defaultlib.glbl

set NumericStdNoWarnings 1
set StdArithNoWarnings 1

do {wave.do}

view wave
view structure
view signals

do {xfft_32.udo}

run 1000ns

quit -force
