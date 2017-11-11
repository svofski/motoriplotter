VERSION		   = 0.11
PRG            = motori
#OBJ            = motori.o usrat.o line.o arc.o shvars.o scale.o hpgl.o charset0.o font173.o htext.o
OBJ = motori.o motori_sim.o usrat_sim.o hpgl.o arc.o charset0.o font173.o line.o shvars.o htext.o scale.o path.o
OPTIMIZE       = -Os

DEFS           = -DSIM -DF_CPU=18432000L -DMCU_TARGET=$(MCU_TARGET) -DVERSION=\"$(VERSION)\" -DBUILDNUM=\"$(BUILDNUM)\"
LIBS           = `sdl2-config --libs`

override CFLAGS        = -g -Wall $(OPTIMIZE) $(DEFS) -I. -I.. `sdl2-config --cflags`
#override LDFLAGS       = -Wl,-Map,$(PRG).map -Wl,-u,vfprintf -Wl,-u,vfscanf -lprintf_flt -lscanf_flt

all: $(PRG)

doc:	doxygen

$(PRG):	$(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS) -lm

motori.o:	../motori.c
	$(CC) -c $(CFLAGS) -o $@ $<

hpgl.o:		../hpgl.c
	$(CC) -c $(CFLAGS) -o $@ $<

arc.o:		../arc.c
	$(CC) -c $(CFLAGS) -o $@ $<

charset0.o:		../charset0.c
	$(CC) -c $(CFLAGS) -o $@ $<

font173.o:		../font173.c
	$(CC) -c $(CFLAGS) -o $@ $<

line.o:		../line.c
	$(CC) -c $(CFLAGS) -o $@ $<

shvars.o:		../shvars.c
	$(CC) -c $(CFLAGS) -o $@ $<

htext.o:		../htext.c
	$(CC) -c $(CFLAGS) -o $@ $<

scale.o:		../scale.c
	$(CC) -c $(CFLAGS) -o $@ $<

path.o:		../path.c
	$(CC) -c $(CFLAGS) -o $@ $<

clean:
	rm -rf *.o $(PRG).elf *.eps *.png *.pdf *.bak 
	rm -rf *.lst *.map $(EXTRA_CLEAN_FILES)

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

# Rules for building the .text rom images

text: hex bin srec

hex:  $(PRG).hex
bin:  $(PRG).bin
srec: $(PRG).srec

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

# Rules for building the .eeprom rom images

eeprom: ehex ebin esrec

ehex:  $(PRG)_eeprom.hex
ebin:  $(PRG)_eeprom.bin
esrec: $(PRG)_eeprom.srec

%_eeprom.hex: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@

%_eeprom.srec: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O srec $< $@

%_eeprom.bin: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O binary $< $@

# Every thing below here is used by avr-libc's build system and can be ignored
# by the casual user.

FIG2DEV                 = fig2dev
EXTRA_CLEAN_FILES       = *.hex *.bin *.srec

dox: eps png pdf

eps: $(PRG).eps
png: $(PRG).png
pdf: $(PRG).pdf

doxygen:	doc/html
	$(DOXYGEN) Doxyfile

%.eps: %.fig
	$(FIG2DEV) -L eps $< $@

%.pdf: %.fig
	$(FIG2DEV) -L pdf $< $@

%.png: %.fig
	$(FIG2DEV) -L png $< $@

