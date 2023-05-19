// sin_fix_interp 10 times precise than sin_fix
// sin_fix_interp 22.13 times fast than gcc sin

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"

// for overclocking
#include "pico.h"
#include "hardware/vreg.h"

// for clock debug
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/clocks.h"

// For ADC input:
#include "hardware/adc.h"
#include "hardware/dma.h"
// multicore support:
#include "pico/multicore.h"
#include "pico/sync.h"
// pio support
#include "hardware/pio.h"
// pwm
#include "hardware/pwm.h"
// for sine wave lookup table
#include "hardware/interp.h"

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

#include "pwm.pio.h"

typedef struct pwm_pio {
    PIO pio;
    uint sm;
    uint pin;
    uint dma_chan;
    uint32_t duty_phase;
    uint32_t duty;
    uint32_t period;
    bool phase_change;
    uint32_t phase_period;
} pwm_pio_t;

pwm_pio_t pwm0 = {
    .pio = pio0,
    .sm = 0,
    .pin = 17,
    .dma_chan = 0,
    .duty_phase = 0x7FFFFFFEU,
    .duty = 0xFFFDU,
    .period = 0xFFFEU,
    .phase_change = false,
    .phase_period = 0U
};

pwm_pio_t pwm1 = {
    .pio = pio0,
    .sm = 1,
    .pin = 18,
    .dma_chan = 1,
    .duty_phase = 0x7FFFFFFEU,
    .duty = 0xFFFDU,
    .period = 0xFFFEU,
    .phase_change = false,
    .phase_period = 0U
};

static inline void pwm_pio_dma_handler(pwm_pio_t *pwm)
{
    if (dma_channel_get_irq0_status(pwm->dma_chan))
    {
        if (pwm->phase_change)
        {
            printf("T %d\n", pwm->phase_period);
            // update new duty_phase
            pwm->duty_phase = (pwm->duty << 16) + pwm->phase_period;
            pwm->phase_change = false;
        } else
        {
            // update new duty_phase
            pwm->duty_phase = (pwm->duty << 16) + pwm->period;
        }
        // Clear the interrupt request.
        dma_hw->ints0 = 1u << pwm->dma_chan;
        // re-trigger dma
        dma_channel_start(pwm->dma_chan);
    }
}

void dma_handler() {
    pwm_pio_dma_handler(&pwm0);
    pwm_pio_dma_handler(&pwm1);
}

void pwm_pio_dma_config(pwm_pio_t *pwm)
{
    // Configure a channel to write the same word (32 bits) repeatedly to PIO0
    // SM0's TX FIFO, paced by the data request signal from that peripheral.
    uint dma_chan = pwm->dma_chan;
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);

    uint dreq = pwm->pio == pio0 ? DREQ_PIO0_TX0 + pwm->sm : DREQ_PIO1_TX0 + pwm->sm;
    channel_config_set_dreq(&c, dreq);

    volatile void *pio_tx_buf = pwm->pio == pio0 ? &pio0_hw->txf[pwm->sm] : &pio1_hw->txf[pwm->sm];
    dma_channel_configure(
        dma_chan,
        &c,
        pio_tx_buf,       // Write address (only need to set this once)
        &pwm->duty_phase, // Don't provide a read address yet
        1,                // Write the same value many times, then halt and interrupt
        false             // Don't start yet
    );

    // Tell the DMA to raise IRQ line 0 when the channel finishes a block
    dma_channel_set_irq0_enabled(dma_chan, true);

    // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_channel_start(dma_chan);
}

int main()
{
    // initialize clocks, on-board LED, SMPS mode power supply
    if (!init_system())
    {
        return 0;
    }
    printf("Hello, PWM!\n");

    uint offset = pio_add_program(pio0, &pwm_program);

    pwm_program_init(pwm0.pio, pwm0.sm, offset, pwm0.pin);
    pwm_program_init(pwm1.pio, pwm1.sm, offset, pwm1.pin);

    pwm_pio_dma_config(&pwm0);
    pwm_pio_dma_config(&pwm1);

    pio_sm_set_enabled(pwm0.pio, pwm0.sm, true);
    pio_sm_set_enabled(pwm1.pio, pwm1.sm, true);

    pwm1.phase_period = 0x7FFFU;
    pwm1.phase_change = true;

    while(true)
    {
        tight_loop_contents();
    }
    return 0;
}
