from pathlib import Path

Import("env")


def patch_esp32p4_spi_hal():
    framework_dir = env.PioPlatform().get_package_dir("framework-arduinoespressif32")
    if not framework_dir:
        print("ESP32-P4 SPI patch: Arduino ESP32 framework package not found")
        return

    spi_hal = Path(framework_dir) / "cores" / "esp32" / "esp32-hal-spi.c"
    if not spi_hal.exists():
        print(f"ESP32-P4 SPI patch: {spi_hal} not found")
        return

    source = spi_hal.read_text(encoding="utf-8")
    patched = source

    clock_old = "spi_ll_set_clk_source(spi->dev, new_clk_src ? SPI_CLK_SRC_SPLL : SPI_CLK_SRC_XTAL);"
    clock_new = "spi_ll_set_clk_source((spi_dev_t *)spi->dev, new_clk_src ? SPI_CLK_SRC_SPLL : SPI_CLK_SRC_XTAL);"
    if clock_new not in patched:
        if clock_old in patched:
            patched = patched.replace(clock_old, clock_new, 1)
        else:
            print("ESP32-P4 SPI patch: clock source target line not found, skipping")

    ldo_old = """      sd_pwr_ctrl_ldo_config_t ldo_config;
      ldo_config.ldo_chan_id = BOARD_SDMMC_POWER_CHANNEL;
      sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;
      sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
      if (sd_pwr_ctrl_set_io_voltage(pwr_ctrl_handle, 3300)) {
        log_e("Unable to set power control to 3V3");
        return;
      }
"""
    ldo_new = """#ifdef BOARD_SDMMC_POWER_CHANNEL
      sd_pwr_ctrl_ldo_config_t ldo_config;
      ldo_config.ldo_chan_id = BOARD_SDMMC_POWER_CHANNEL;
      sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;
      sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
      if (sd_pwr_ctrl_set_io_voltage(pwr_ctrl_handle, 3300)) {
        log_e("Unable to set power control to 3V3");
        return;
      }
#endif
"""
    if ldo_new not in patched:
        if ldo_old in patched:
            patched = patched.replace(ldo_old, ldo_new, 1)
        else:
            print("ESP32-P4 SPI patch: LDO target block not found, skipping")

    if patched != source:
        spi_hal.write_text(patched, encoding="utf-8")
        print("ESP32-P4 SPI patch: applied Arduino ESP32-P4 compatibility fixes")


patch_esp32p4_spi_hal()
