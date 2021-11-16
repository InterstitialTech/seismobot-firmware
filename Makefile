OPENCM3_DIR = /home/joshua/Documents/seismobot-firmware/libopencm3
PROJECT = seismobot
CFILES += src/main.c
DEVICE = samd09c13a
OOCD_FILE = seismobot.cfg

# targets: all, clean
include $(OPENCM3_DIR)/mk/genlink-config.mk
include rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk

flash:
	openocd -f $(OOCD_FILE) -c "program $(PROJECT).elf verify reset exit"

debug:
	openocd -f $(OOCD_FILE)

