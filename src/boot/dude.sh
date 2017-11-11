# Fuses: 1kw bootloader, no watchdog kthx, external xtal very fast
#avrdude -p m644p -c avrispmkII -U lfuse:w:0xcf:m -U hfuse:w:0xd4:m -U efuse:w:0xff:m

# Program the bootloader directly via avrispmkII
#avrdude -v -p m644p -c avrispmkII -U flash:w:stkload_m644_2k64k_18432000.hex

# Program the actual firmware using serial bootloader. Press reset, then run this
avrdude -c stk500v1 -P /dev/tty.usbserial-A6007JHq -p m644p -U flash:w:motori.hex:i -F
