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

#define REP0(X)
#define REP1(X) X
#define REP2(X) REP1(X) X
#define REP3(X) REP2(X) X
#define REP4(X) REP3(X) X
#define REP5(X) REP4(X) X
#define REP6(X) REP5(X) X
#define REP7(X) REP6(X) X
#define REP8(X) REP7(X) X
#define REP9(X) REP8(X) X
#define REP10(X) REP9(X) X

#define REP(HUNDREDS,TENS,ONES,X) REP_(HUNDREDS,TENS,ONES,X)
#define REP_(HUNDREDS,TENS,ONES,X) \
  REP##HUNDREDS(REP10(REP10(X))) \
  REP##TENS(REP10(X)) \
  REP##ONES(X)

#include "pwm.pio.h"
#include "clock.pio.h"

#define PWM_SYNC_IN_PIN 16
#define PWM_SYNC_OUT_PIN 17
#define CLOCK_OUT_PIN 21
#define CLOCK_OUT_DIV 20

typedef struct pwm_pio {
    volatile PIO pio;
    volatile uint sm;
    volatile uint pin;
    volatile uint dma_chan;
    volatile uint32_t duty_phase;
    volatile uint32_t duty;
    volatile uint32_t period;
    volatile bool phase_change;
    volatile uint32_t phase_period;
} pwm_pio_t;

volatile pwm_pio_t pwms[8];
#define NUM_OF_PWM_CHANNEL 8

#define PWM_DMA_HANDLER() \
do { \
    uint8_t i = 0; \
    REP(0, 0, NUM_OF_PWM_CHANNEL, pwm_pio_dma_handler(pwms + (i++));)\
} while (0)

static inline void __time_critical_func(pwm_pio_dma_handler)(volatile pwm_pio_t *pwm)
{
    if (dma_channel_get_irq0_status(pwm->dma_chan))
    {
        if (pwm->phase_change)
        {
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

void __time_critical_func(dma_handler)(void)
{
    PWM_DMA_HANDLER();
}

void pwm_pio_dma_config(volatile pwm_pio_t *pwm)
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

int  __time_critical_func(main)(void)
{
    // initialize clocks, on-board LED, SMPS mode power supply
    if (!init_system())
    {
        return 0;
    }

    // output pico clock, the freq is 240Mhz/20 = 12Mhz.
    // this clock will connect to another pico XIN
    clock_gpio_init(CLOCK_OUT_PIN, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, CLOCK_OUT_DIV);

    // init the pwm sync out pin to low, make all PIO state machines stall
    gpio_init(PWM_SYNC_OUT_PIN);
    gpio_set_dir(PWM_SYNC_OUT_PIN, true);
    gpio_put(PWM_SYNC_OUT_PIN, false);

    // welcome message indicates the pico works.
    printf("Hello, PWM!\n");

    uint offset = pio_add_program(pio0, &pwm_program);
    uint offset1 = pio_add_program(pio1, &pwm_program);
    // initialize pwms parameters
    for (int i = 0; i < NUM_OF_PWM_CHANNEL; i++)
    {
        if (i < 4)
        {
            pwms[i].pio = pio0;
            pwms[i].sm = i;
        } else{
            pwms[i].pio = pio1;
            pwms[i].sm = i - 4;
        }
        pwms[i].pin = 2 + i;
        pwms[i].dma_chan = i;
        pwms[i].duty_phase = 0x00001000U;
        pwms[i].duty = 0xEFFU;
        pwms[i].period = 0x1000U;
        pwms[i].phase_change = false;
        pwms[i].phase_period = 0U;
    }
    // config and init state machines and dma
    for (int i = 0; i < NUM_OF_PWM_CHANNEL; i++)
    {
        if (i < 4)
        {
            pwm_program_init(pwms[i].pio, pwms[i].sm, offset, pwms[i].pin, PWM_SYNC_IN_PIN);
        } else {
            pwm_program_init(pwms[i].pio, pwms[i].sm, offset1, pwms[i].pin, PWM_SYNC_IN_PIN);
        }
        pwm_pio_dma_config(pwms + i);
    }
    // simultaneously start state machines
    pio_enable_sm_mask_in_sync(pio0, 0b1111);
    if (NUM_OF_PWM_CHANNEL > 4)
    {
        pio_enable_sm_mask_in_sync(pio1, 0b1111);
    }

    // // output cpu clock use for debug.
    // offset = pio_add_program(pio1, &clock_program);
    // clock_program_init(pio1, 0, offset, 18);
    // pio_sm_set_enabled(pio1, 0, true);

    // phase control test
    (pwms + 1)->phase_period = 0x3FFU;
    (pwms + 1)->phase_change = true;

    (pwms + 2)->phase_period = 0x7FFU;
    (pwms + 2)->phase_change = true;

    (pwms + 3)->phase_period = 0xBFFU;
    (pwms + 3)->phase_change = true;

    // set pwm sync out pin to high, to activate all state machines
    sleep_ms(1000);
    gpio_put(PWM_SYNC_OUT_PIN, true);

    sleep_us(100);
    // phase control test
    (pwms + 1)->phase_period = 0x3FFU;
    (pwms + 1)->phase_change = true;

    (pwms + 2)->phase_period = 0x7FFU;
    (pwms + 2)->phase_change = true;

    (pwms + 3)->phase_period = 0xBFFU;
    (pwms + 3)->phase_change = true;

    (pwms + 5)->phase_period = 0x3FFU;
    (pwms + 5)->phase_change = true;

    (pwms + 6)->phase_period = 0x7FFU;
    (pwms + 6)->phase_change = true;

    (pwms + 7)->phase_period = 0xBFFU;
    (pwms + 7)->phase_change = true;

    while(true)
    {
        tight_loop_contents();
    }
    return 0;
}
