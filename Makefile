# Select the RTOS brand (FREERTOS, RTX, NONE)
export RTOS_BRAND=NONE

# The only task is to call the Makefile in the source directory
all:
	@echo Ra2 LPC54114 M4  && cd gcc && $(MAKE) -f Makefile_m4_ra2

clean:
	@echo Ra2 LPC54114 M4  && cd gcc && $(MAKE) -f Makefile_m4_ra2 clean

