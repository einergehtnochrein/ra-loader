# Select the RTOS brand (FREERTOS, RTX, NONE)
export RTOS_BRAND=NONE


# The only task is to call the Makefile in the source directory
all:
	@echo LPC54102 && cd gcc && $(MAKE) -f Makefile_m4

clean:
	@echo LPC54102 && cd gcc && $(MAKE) -f Makefile_m4 clean

