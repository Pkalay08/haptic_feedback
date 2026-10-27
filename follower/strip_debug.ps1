$OBJCOPY = "C:\ncs\toolchains\dcbdc366a1\opt\zephyr-sdk\gnu\arm-zephyr-eabi\bin\arm-zephyr-eabi-objcopy.exe"
$HEX = "build\follower\zephyr\zephyr.elf"
$CLEAN = "clean.hex"
$UF2 = "nice.uf2"

& $OBJCOPY --strip-debug -O ihex $HEX $CLEAN
python uf2conv.py -c -f 0x239A00B3 -o $UF2 $CLEAN

Write-Host "Built $UF2. Double-tap reset and drag it onto the NICENANO drive."