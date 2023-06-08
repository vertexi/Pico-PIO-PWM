"openocd.exe" -c "gdb_port 50000" -c "tcl_port 50001" -c "telnet_port 50002" ^
    -s "%OPENOCD_PATH%\tcl" -f interface/picoprobe.cfg -f target/rp2040.cfg ^
    -c "adapter speed 5000" -c "program build/test.elf verify reset exit"
