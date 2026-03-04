idf_component_register(
	SRCS
		"main.c"
		"renderer.c"
		"usb_device.c"
		"sdcard.c"
	PRIV_REQUIRES
		esp_lcd
		badge-bsp
		fatfs
		sdmmc
		esp_driver_sdmmc
		esp_driver_sdspi
		esp_driver_spi
		vfs
	INCLUDE_DIRS
		"."
)

idf_build_set_property(COMPILE_OPTIONS "-Wno-error=unused-variable" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=unused-const-variable" APPEND)
