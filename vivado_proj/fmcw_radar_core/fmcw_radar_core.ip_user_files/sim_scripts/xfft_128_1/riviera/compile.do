transcript off
onbreak {quit -force}
onerror {quit -force}
transcript on

vlib work
vlib riviera/xpm
vlib riviera/xbip_utils_v3_0_15
vlib riviera/axi_utils_v2_0_11
vlib riviera/c_reg_fd_v12_0_11
vlib riviera/xbip_dsp48_wrapper_v3_0_7
vlib riviera/xbip_pipe_v3_0_11
vlib riviera/c_addsub_v12_0_21
vlib riviera/c_shift_ram_v12_0_20
vlib riviera/mult_gen_v12_0_24
vlib riviera/floating_point_v7_1_21
vlib riviera/cmpy_v6_0_27
vlib riviera/xfft_v9_1_15
vlib riviera/xil_defaultlib

vmap xpm riviera/xpm
vmap xbip_utils_v3_0_15 riviera/xbip_utils_v3_0_15
vmap axi_utils_v2_0_11 riviera/axi_utils_v2_0_11
vmap c_reg_fd_v12_0_11 riviera/c_reg_fd_v12_0_11
vmap xbip_dsp48_wrapper_v3_0_7 riviera/xbip_dsp48_wrapper_v3_0_7
vmap xbip_pipe_v3_0_11 riviera/xbip_pipe_v3_0_11
vmap c_addsub_v12_0_21 riviera/c_addsub_v12_0_21
vmap c_shift_ram_v12_0_20 riviera/c_shift_ram_v12_0_20
vmap mult_gen_v12_0_24 riviera/mult_gen_v12_0_24
vmap floating_point_v7_1_21 riviera/floating_point_v7_1_21
vmap cmpy_v6_0_27 riviera/cmpy_v6_0_27
vmap xfft_v9_1_15 riviera/xfft_v9_1_15
vmap xil_defaultlib riviera/xil_defaultlib

vlog -work xpm  -incr "+incdir+../../../../../../../../../../../AMDDesignTools/2025.2/Vivado/data/rsb/busdef" -l xpm -l xbip_utils_v3_0_15 -l axi_utils_v2_0_11 -l c_reg_fd_v12_0_11 -l xbip_dsp48_wrapper_v3_0_7 -l xbip_pipe_v3_0_11 -l c_addsub_v12_0_21 -l c_shift_ram_v12_0_20 -l mult_gen_v12_0_24 -l floating_point_v7_1_21 -l cmpy_v6_0_27 -l xfft_v9_1_15 -l xil_defaultlib \
"C:/AMDDesignTools/2025.2/Vivado/data/ip/xpm/xpm_memory/hdl/xpm_memory.sv" \

vcom -work xpm -93  -incr \
"C:/AMDDesignTools/2025.2/Vivado/data/ip/xpm/xpm_VCOMP.vhd" \

vcom -work xbip_utils_v3_0_15 -93  -incr \
"../../../ipstatic/hdl/xbip_utils_v3_0_vh_rfs.vhd" \

vcom -work axi_utils_v2_0_11 -93  -incr \
"../../../ipstatic/hdl/axi_utils_v2_0_vh_rfs.vhd" \

vcom -work c_reg_fd_v12_0_11 -93  -incr \
"../../../ipstatic/hdl/c_reg_fd_v12_0_vh_rfs.vhd" \

vcom -work xbip_dsp48_wrapper_v3_0_7 -93  -incr \
"../../../ipstatic/hdl/xbip_dsp48_wrapper_v3_0_vh_rfs.vhd" \

vcom -work xbip_pipe_v3_0_11 -93  -incr \
"../../../ipstatic/hdl/xbip_pipe_v3_0_vh_rfs.vhd" \

vcom -work c_addsub_v12_0_21 -93  -incr \
"../../../ipstatic/hdl/c_addsub_v12_0_vh_rfs.vhd" \

vcom -work c_shift_ram_v12_0_20 -93  -incr \
"../../../ipstatic/hdl/c_shift_ram_v12_0_vh_rfs.vhd" \

vcom -work mult_gen_v12_0_24 -93  -incr \
"../../../ipstatic/hdl/mult_gen_v12_0_vh_rfs.vhd" \

vcom -work floating_point_v7_1_21 -93  -incr \
"../../../ipstatic/hdl/floating_point_v7_1_vh_rfs.vhd" \

vcom -work cmpy_v6_0_27 -93  -incr \
"../../../ipstatic/hdl/cmpy_v6_0_vh_rfs.vhd" \

vcom -work xfft_v9_1_15 -2008  -incr \
"../../../ipstatic/hdl/xfft_v9_1_vh08_rfs.vhd" \

vcom -work xfft_v9_1_15 -93  -incr \
"../../../ipstatic/hdl/xfft_v9_1_vh_rfs.vhd" \

vcom -work xil_defaultlib -93  -incr \
"../../../../fmcw_radar_core.gen/sources_1/ip/xfft_128_1/sim/xfft_128.vhd" \


vlog -work xil_defaultlib \
"glbl.v"

