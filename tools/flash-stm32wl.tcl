# Flash a Wio-E5 / LoRa-E5 (STM32WLE5) over SWD with a clone ST-Link that
# cannot drive NRST, against firmware that sleeps in STOP2 (debug clock gated).
#
# Recipe (proven on production silicon):
#   1. build:  cd firmware/stm32wl && pio run -e wio-e5-wmesh
#   2. start OpenOCD with NRST disabled and this script:
#        openocd -f interface/stlink.cfg -f target/stm32wlx.cfg \
#          -c "reset_config none separate" \
#          -c "set BIN .pio/build/wio-e5-wmesh/firmware.bin" \
#          -f ../../tools/flash-stm32wl.tcl
#   3. while the catcher loop runs, TAP THE RST BUTTON on the board a few
#      times — the loop catches the core in its brief awake window after reset
#   4. wait for ">>> verify" and ">>> run"; the board reboots into the new image
#
# Works at RDP level 0 only. If the catcher never halts (">>> NO HALT"),
# tap RST faster, or lower the SWD clock (adapter speed 240).

init
if {![info exists BIN]} {
  set BIN .pio/build/wio-e5-wmesh/firmware.bin
}
catch {stm32wlx.cpu0 cortex_m vector_catch reset} ve
echo "vector_catch: $ve"
set ok 0
for {set i 0} {$i < 600} {incr i} {
  catch {halt}
  if {![catch {set st [stm32wlx.cpu0 curstate]}] && ![string compare $st halted]} { set ok 1; break }
  sleep 40
}
if {!$ok} { echo ">>> NO HALT — tap RST while the loop runs, then retry"; exit }
echo ">>> caught, state=[stm32wlx.cpu0 curstate]"
# soft reset over SWD (no NRST wire needed), clean clocks for flashing
catch {stm32wlx.cpu0 cortex_m reset_config sysresetreq} e1
echo "reset_config: $e1"
catch {reset halt} e2
echo "reset halt: $e2  state=[stm32wlx.cpu0 curstate]"
flash probe 0
catch {flash write_image erase $BIN 0x08000000} w
echo ">>> write: $w"
catch {verify_image $BIN 0x08000000} v
echo ">>> verify: $v"
catch {stm32wlx.cpu0 cortex_m vector_catch none}
catch {reset run} r
echo ">>> run: $r"
exit
