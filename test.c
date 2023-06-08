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

#define PWM_SYNC_IN_PIN 22
#define PWM_DATA_SYNC_IN_PIN 15
#define CLOCK_OUT_PIN 21
#define CLOCK_OUT_DIV 20
#define ADDR_0 26
#define ADDR_1 27
#define ADDR_2 28
#define SPI_FREQ 20000000

#define NUM_OF_PWM_CHANNEL 8
#define UP_DOWN 0
#define COMPLEMENTARY 0
#define PWM_PERIOD 0x1000U

uint8_t PWM_ADDR = 0U;

typedef struct pwm_pio {
    volatile PIO pio;
    volatile uint sm;
    volatile uint pin;
    volatile bool polarity;
    volatile uint dma_chan;
    volatile uint32_t duty_phase;
    volatile uint32_t duty;
    volatile bool duty_flip;
    volatile uint32_t period;
    volatile bool phase_change;
    #if (UP_DOWN == 1)
        volatile uint8_t phase_change_count;
    #endif
    volatile uint32_t phase_period;
    volatile uint32_t phase;
} pwm_pio_t;

volatile pwm_pio_t pwms[8];

#define DUTY_COMPLEMENT(DUTY) \
    ((DUTY < PWM_PERIOD) ? ((PWM_PERIOD - 1) - DUTY) : ((PWM_PERIOD * 2 + 1) - DUTY))

#define DUTY_GET(DUTY, FLIP_FLAG) \
    (FLIP_FLAG ? DUTY_COMPLEMENT(DUTY) : DUTY)

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
            #if (UP_DOWN == 1)
                pwm->duty_phase = (DUTY_GET(pwm->duty, pwm->duty_flip) << 16) + pwm->phase_period;
                pwm->duty_flip = !pwm->duty_flip;
                pwm->phase_change_count++;
                if (pwm->phase_change_count == 2) {
                    pwm->phase_change_count = 0;
                    pwm->phase_change = false;
                }
            #else
                pwm->duty_phase = (pwm->duty << 16) + pwm->phase_period;
                pwm->phase_change = false;
            #endif
        } else
        {
            // update new duty_phase
            #if (UP_DOWN == 1)
                pwm->duty_phase = (DUTY_GET(pwm->duty, pwm->duty_flip) << 16) + pwm->period;
                pwm->duty_flip = !pwm->duty_flip;
            #else
                pwm->duty_phase = (pwm->duty << 16) + pwm->period;
            #endif
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

void __time_critical_func(phase_change)(volatile pwm_pio_t *pwm, uint16_t phase)
{
    uint16_t new_phase;
    phase += 3;
    new_phase = phase;
    if (phase < (pwm->phase + 3))
    {
        phase += (3 + PWM_PERIOD);
    }
    pwm->phase_period = phase - (pwm->phase + 3);
    pwm->phase = new_phase;
    pwm->phase_change = true;
}

#define TOTALNUM_OF_PWM_CHANNEL 8
volatile uint8_t pwm_rxf[5*TOTALNUM_OF_PWM_CHANNEL*32];
uint8_t* pwm_rxf_ptr = pwm_rxf;

#define PWM_RX_BUF_SIZE 5*TOTALNUM_OF_PWM_CHANNEL

void __time_critical_func(core1_main)(void)
{
    gpio_init(PWM_DATA_SYNC_IN_PIN);
    gpio_set_dir(PWM_DATA_SYNC_IN_PIN, GPIO_IN);
    gpio_pull_up(PWM_DATA_SYNC_IN_PIN);
    // gpio_set_irq_enabled_with_callback(PWM_DATA_SYNC_IN_PIN, GPIO_IRQ_EDGE_FALL, true, &pwm_dma_trigger);

    // pwm_rxf_ptr = pwm_rxf;
    // while(true)
    // {
    //     spi_read_blocking(spi_default, 0, pwm_rxf_ptr, PWM_RX_BUF_SIZE);
    //     pwm_rxf_ptr += PWM_RX_BUF_SIZE;
    //     if (pwm_rxf_ptr == pwm_rxf + 5*NUM_OF_PWM_CHANNEL*31)
    //     {
    //         for (int j = 0; j < 31; j++)
    //         {
    //             for (int i = 0; i < 5*NUM_OF_PWM_CHANNEL; i++)
    //             {
    //                 printf("%d ", pwm_rxf[i + j * (5*NUM_OF_PWM_CHANNEL)]);
    //             }
    //             printf("\n");
    //         }
    //         break;
    //     }
    // }

    while(true) {
        spi_read_blocking(spi_default, 0, (uint8_t *)pwm_rxf, PWM_RX_BUF_SIZE);
        for (int i = 0; i < TOTALNUM_OF_PWM_CHANNEL; i++)
        {
            if (pwm_rxf[5*i + 0] == PWM_ADDR)
            {
                pwms[i].duty = (pwm_rxf[5*i + 1] << 8) + pwm_rxf[5*i + 2];
                phase_change(pwms + i, (pwm_rxf[5*i + 3] << 8) + pwm_rxf[5*i + 4]);
            }
        }
    }

    while(1)
    {
        tight_loop_contents();
    }
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

    // use SPI0 around at 12MHz. due to overclock clkperi stuck at 48Mhz
    spi_init(spi_default, SPI_FREQ);
    spi_set_slave(spi_default, true);
    spi_set_format(spi_default,
                   8,          // number of bits per transfer
                   SPI_CPOL_0, // polarity CPOL
                   SPI_CPHA_1, // phase CPHA
                   SPI_MSB_FIRST);
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI);

    gpio_init(ADDR_0);
    gpio_set_dir(ADDR_0, GPIO_IN);
    gpio_pull_down(ADDR_0);
    gpio_init(ADDR_1);
    gpio_set_dir(ADDR_1, GPIO_IN);
    gpio_pull_down(ADDR_1);
    gpio_init(ADDR_2);
    gpio_set_dir(ADDR_2, GPIO_IN);
    gpio_pull_down(ADDR_2);

    PWM_ADDR = gpio_get(ADDR_0) + (gpio_get(ADDR_1) << 1) + (gpio_get(ADDR_2) << 2);

    uint pio0_offset = 0;
    uint pio1_offset = 0;
    #if (UP_DOWN == 1)
        pio0_offset = pio_add_program(pio0, &pwm_up_down_program);
        if (NUM_OF_PWM_CHANNEL > 4) {
            pio1_offset = pio_add_program(pio1, &pwm_up_down_program);
        }
    #else
        pio0_offset = pio_add_program(pio0, &pwm_program);
        if (NUM_OF_PWM_CHANNEL > 4) {
            pio1_offset = pio_add_program(pio1, &pwm_program);
        }
    #endif
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
        #if (COMPLEMENTARY == 0)
            pwms[i].polarity = true;
        #else
            pwms[i].polarity = !(i % 2);
        #endif
        pwms[i].dma_chan = i;
        pwms[i].duty_phase = PWM_PERIOD;
        pwms[i].duty = 0x100U;
        pwms[i].duty_flip = true;
        pwms[i].period = PWM_PERIOD;
        pwms[i].phase_change = false;
        pwms[i].phase_period = 0U;
        pwms[i].phase = 0U;
        #if (UP_DOWN == 1)
            pwms[i].phase_change_count = 0U;
        #endif
    }
    // config and init state machines and dma
    for (int i = 0; i < NUM_OF_PWM_CHANNEL; i++)
    {
        if (i < 4)
        {
            pwm_program_init(pwms[i].pio, pwms[i].sm, pio0_offset, UP_DOWN, pwms[i].pin, pwms[i].polarity, PWM_SYNC_IN_PIN);
        } else {
            pwm_program_init(pwms[i].pio, pwms[i].sm, pio1_offset, UP_DOWN, pwms[i].pin, pwms[i].polarity, PWM_SYNC_IN_PIN);
        }
        pwm_pio_dma_config(pwms + i);
    }
    // simultaneously start state machines
    pio_enable_sm_mask_in_sync(pio0, 0b1111);
    if (NUM_OF_PWM_CHANNEL > 4)
    {
        pio_enable_sm_mask_in_sync(pio1, 0b1111);
    }

    // for (uint32_t i = 0; i < 100000; i++)
    // {
    //     phase_change(pwms+1, 0x800);
    //     pwms[1].duty += 0x100U;
    //     if (pwms[1].duty > PWM_PERIOD)
    //     {
    //         pwms[1].duty = 0x0U;
    //     }

    //     sleep_us(100);

    //     phase_change(pwms+1, PWM_PERIOD);
    //     pwms[1].duty += 0x100U;
    //     if (pwms[1].duty > PWM_PERIOD)
    //     {
    //         pwms[1].duty = 0x0U;
    //     }

    //     sleep_us(100);
    // }

    // pwms[1].duty = 0x100U;

    multicore_reset_core1();
    multicore_launch_core1(core1_main);

    while(true)
    {
        tight_loop_contents();
    }
    return 0;
}
