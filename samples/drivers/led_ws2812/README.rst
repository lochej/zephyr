.. _led_ws2812_sample:

WS2812 Sample Application
#########################

Overview
********

This sample application demonstrates basic usage of the WS2812 LED
strip driver, for controlling LED strips using WS2812, WS2812b,
SK6812, and compatible driver chips.

Requirements
************

.. _NeoPixel Ring 12 from AdaFruit: https://www.adafruit.com/product/1643
.. _74AHCT125: https://cdn-shop.adafruit.com/datasheets/74AHC125.pdf

- LED strip using WS2812 or compatible, such as the `NeoPixel Ring 12
  from AdaFruit`_.

- Note that 5V communications may require a level translator, such as the
  `74AHCT125`_. LEDs can work at 3.3V but you could experience glitches.

- LED power strip supply. It's fine to power the LED strip off of your board's
  IO voltage level even if that's below 5V; the LEDs will simply be dimmer in
  this case.

Wiring
******

#. Ensure your Zephyr board, and the LED strip share a common ground.
#. Connect the LED strip control pin (either SPI MOSI or GPIO) from your board
   to the data input pin of the first WS2812 IC in the strip.
#. Power the LED strip at an I/O level compatible with the control pin signals.

Building and Running
********************

.. _blog post on WS2812 timing: https://wp.josh.com/2014/05/13/ws2812-neopixels-are-not-so-finicky-once-you-get-to-know-them/

This sample's source directory is :zephyr_file:`samples/drivers/led_ws2812/`.

To make sure the sample is set up properly for building, you must:

- select the correct WS2812 driver backend for your SoC. This can be
  :option:`CONFIG_WS2812_STRIP_SPI` for any core or
  :option:`CONFIG_WS2812_STRIP_GPIO` if you are using nRF51 SoC or STM32 familly.

- create a ``led-strip`` :ref:`devicetree alias <dt-alias-chosen>`, which
  refers to a node in your :ref:`devicetree <dt-guide>` with a
  ``worldsemi,ws2812-spi`` or ``worldsemi,ws2812-gpio`` compatible. The node
  must be properly configured for the driver backend (SPI or GPIO) and daisy
  chain length (number of WS2812 chips).

For example devicetree configurations for each compatible, see
:zephyr_file:`samples/drivers/led_ws2812/boards/nrf52dk_nrf52832.overlay`,
:zephyr_file:`samples/drivers/led_ws2812/boards/nrf51dk_nrf51422.overlay`,
:zephyr_file:`samples/drivers/led_ws2812/boards/nucleo_l073rz.overlay`,
:zephyr_file:`samples/drivers/led_ws2812/boards/nucleo_h743zi.overlay` and
:zephyr_file:`samples/drivers/led_ws2812/boards/nucleo_l432kc.overlay`.

Some boards are already supported out of the box; see the :file:`boards`
directory for this sample for details.

If you choose :option:`CONFIG_WS2812_STRIP_GPIO`, you need to provide your timings
so the build system adapts the software bitbang delays to your CPU clock frequency.

You need to provide 4 timings that you can tweak up or down to ajust the pulse length
on the data output pin.

Timings are configured in CPU cycles with the following options: 
- :option:`CONFIG_WS2812_STRIP_GPIO_CYCLES_T1H` 1 pulse high timing
- :option:`CONFIG_WS2812_STRIP_GPIO_CYCLES_T1L` 1 pulse low timing
- :option:`CONFIG_WS2812_STRIP_GPIO_CYCLES_T0H` 0 pulse high timing
- :option:`CONFIG_WS2812_STRIP_GPIO_CYCLES_T0L` 0 pulse low timing

For each timing, you can estimate the number of cycles with this equations:
- CYCLES = Tx * F_CPU / 1000
- CYCLES number of CPU cycles
- F_CPU in MHz
- Tx in nanoseconds

For example configuration files, see
:zephyr_file:`samples/drivers/led_ws2812/boards/nucleo_l073rz.conf`,
:zephyr_file:`samples/drivers/led_ws2812/boards/nucleo_h743zi.conf` and
:zephyr_file:`samples/drivers/led_ws2812/boards/nucleo_l432kc.conf`.

Then build and flash the application:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/led_ws2812
   :board: <board>
   :goals: flash
   :compact:

When you connect to your board's serial console, you should see the
following output:

.. code-block:: none

   ***** Booting Zephyr OS build v2.1.0-rc1-191-gd2466cdaf045 *****
   [00:00:00.005,920] <inf> main: Found LED strip device WS2812
   [00:00:00.005,950] <inf> main: Displaying pattern on strip

References
**********

- `RGB LED strips: an overview <http://nut-bolt.nl/2012/rgb-led-strips/>`_
- `74AHCT125 datasheet
  <https://cdn-shop.adafruit.com/datasheets/74AHC125.pdf>`_
- An excellent `blog post on WS2812 timing`_.
