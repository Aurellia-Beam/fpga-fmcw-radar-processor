onbreak {quit -f}
onerror {quit -f}

vsim  -lib xil_defaultlib xfft_128_opt

set NumericStdNoWarnings 1
set StdArithNoWarnings 1

do {wave.do}

view wave
view structure
view signals

do {xfft_128.udo}

run 1000ns

quit -force
