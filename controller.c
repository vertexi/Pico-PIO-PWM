#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"

// for overclocking
#include "pico.h"
#include "hardware/vreg.h"

// for clock debug
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/clocks.h"

#include "hardware/dma.h"
// multicore support:
#include "pico/multicore.h"

#define POWER_SMPS 23

const uint LED_PIN = 25;

#define CPU_FREQ (240*KHZ) // 240k khz, 240 MHz cpu clock
#define UART_BAUD (115200) // UART baud rate

#include "hardware/pll.h" // don't forget to add hardware_pll to your Cmakelists.txt

void gset_sys_clock_pll(uint32_t vco_freq, uint post_div1, uint post_div2)
{
    if (!running_on_fpga())
    {
        clock_configure(clk_sys,
                        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                        48 * MHZ,
                        48 * MHZ);

        pll_init(pll_sys, 1, vco_freq, post_div1, post_div2);
        uint32_t freq = vco_freq / (post_div1 * post_div2);

        // Configure clocks
        // CLK_REF = XOSC (12MHz) / 1 = 12MHz
        clock_configure(clk_ref,
                        CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC,
                        0, // No aux mux
                        12 * MHZ,
                        12 * MHZ);

        // CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
        clock_configure(clk_sys,
                        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                        freq, freq);

       clock_configure(clk_peri,
                       0, // Only AUX mux on ADC
                       CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                       CPU_FREQ * KHZ,
                       CPU_FREQ * KHZ);
    }
}

static inline bool gset_sys_clock_khz(uint32_t freq_khz, bool required)
{
    uint vco, postdiv1, postdiv2;
    if (check_sys_clock_khz(freq_khz, &vco, &postdiv1, &postdiv2))
    {
        gset_sys_clock_pll(vco, postdiv1, postdiv2);
        return true;
    }
    else if (required)
    {
        panic("System clock of %u kHz cannot be exactly achieved", freq_khz);
    }
    return false;
}

void measure_freqs(void) {
    uint f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
    uint f_pll_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
    uint f_rosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
    uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    uint f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
    uint f_clk_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
    uint f_clk_adc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);
    uint f_clk_rtc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_RTC);

    printf("pll_sys = %dkHz\n", f_pll_sys);
    printf("pll_usb = %dkHz\n", f_pll_usb);
    printf("rosc = %dkHz\n", f_rosc);
    printf("clk_sys = %dkHz\n", f_clk_sys);
    printf("clk_peri = %dkHz\n", f_clk_peri);
    printf("clk_usb = %dkHz\n", f_clk_usb);
    printf("clk_adc = %dkHz\n", f_clk_adc);
    printf("clk_rtc = %dkHz\n", f_clk_rtc);

    // Can't measure clk_ref / xosc as it is the ref
}


uint8_t init_system()
{
    // set up clock
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    if (gset_sys_clock_khz(CPU_FREQ, true)) // set system clock to 240Mhz
    {
        stdio_uart_init_full(uart0, UART_BAUD, PICO_DEFAULT_UART_TX_PIN, PICO_DEFAULT_UART_RX_PIN);
        uart_set_format(uart0, 8, 1, UART_PARITY_EVEN);
    } else
    {
        return 0;
    }
    measure_freqs();

    // set up led indicator
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    // switch pico's power supply into SMPS mode to reduce voltage ripple
    gpio_init(POWER_SMPS);
    gpio_set_dir(POWER_SMPS, GPIO_OUT);
    gpio_put(POWER_SMPS, 1);

    return 1;
}

#define SPI_FREQ 20000000
#define PWM_SYNC 20

int  __time_critical_func(main)(void)
{
    if (!init_system())
    {
        return 0;
    }

    // use SPI0 around at 12MHz. due to overclock clkperi stuck at 48Mhz
    spi_init(spi_default, SPI_FREQ);
    spi_set_format(spi_default,
                   16,          // number of bits per transfer
                   SPI_CPOL_1, // polarity CPOL
                   SPI_CPHA_0, // phase CPHA
                   SPI_MSB_FIRST);
    printf("spi freq: %u\n", spi_get_baudrate(spi_default));
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI);

    gpio_init(PWM_SYNC);
    gpio_set_dir(PWM_SYNC, GPIO_OUT);
    gpio_put(PWM_SYNC, 1);

    uint32_t spi_txf = 0;
    while (1)
    {
        spi_txf += 0xff0000U;
        if (spi_txf > 0x10000000U)
        {
            spi_txf = 0U;
        }
        spi_write16_blocking(spi_default, (uint16_t *)(&spi_txf), 2);
        sleep_ms(1000);
    }
}