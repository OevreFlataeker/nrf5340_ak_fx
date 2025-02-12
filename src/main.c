/**
 * @file        main.c
 *
 * @brief       Audio DK HW_CODEC test using I2S loop and tone/noise generators.
 *
 * Original source by ace.johnny (https://github.com/ace-johnny/nrfadk-hello_codec)
 */

#include <zephyr/kernel.h>
#include <nrf.h>
#include <nrfx_clock.h>
#include <nrfx_i2s.h>
#include "cs47l63_comm.h"
#include <zephyr/drivers/i2s.h>
#include <zephyr/logging/log.h>

#define CONFIG_MAIN_LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);

#define CONFIG_LOGGING

////////////////////////////////////////////////////////////////////////////////
// NRFX_CLOCKS

#define HFCLKAUDIO_12_288_MHZ 0x9BA6

// Claim the I2S0 peripheral
static const nrfx_i2s_t i2s_instance = NRFX_I2S_INSTANCE(0);

#undef ENABLE_LINEIN
#undef ENABLE_LINEIN_PASSTHROUGH
#define ENABLE_MIC
#undef ENABLE_MIC_PASSTHROUGH

#define ECHO

/**
 * @brief       Initialize the high-frequency clocks and wait for each to start.
 *
 * @details     HFCLK =         128,000,000 Hz
 *              HFCLKAUDIO =     12,288,000 Hz
 */
static int nrfadk_hfclocks_init(void)
{
	nrfx_err_t err;

	// HFCLK
	err = nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);
	if (err != NRFX_SUCCESS)
		return (err - NRFX_ERROR_BASE_NUM);

	nrfx_clock_start(NRF_CLOCK_DOMAIN_HFCLK);
	while (!nrfx_clock_is_running(NRF_CLOCK_DOMAIN_HFCLK, NULL))
		k_msleep(1);

	// HFCLKAUDIO
	nrfx_clock_hfclkaudio_config_set(HFCLKAUDIO_12_288_MHZ);

	nrfx_clock_start(NRF_CLOCK_DOMAIN_HFCLKAUDIO);
	while (!nrfx_clock_is_running(NRF_CLOCK_DOMAIN_HFCLKAUDIO, NULL))
		k_msleep(1);

	return 0;
}

////////////////////////////////////////////////////////////////////////////////
// NRF_I2S

#define MCKFREQ_6_144_MHZ 0x66666000
#define I2S_BUFF_SIZE 8192 // Define an appropriate buffer size

static int16_t rx_buffer1[I2S_BUFF_SIZE] __attribute__((aligned(4)));
static int16_t rx_buffer2[I2S_BUFF_SIZE] __attribute__((aligned(4)));
static int16_t tx_buffer1[I2S_BUFF_SIZE] __attribute__((aligned(4)));
static int16_t tx_buffer2[I2S_BUFF_SIZE] __attribute__((aligned(4)));

static volatile bool i2s_transfer_done = false;
static volatile bool use_first_buffer = true;

static void i2s_handler(nrfx_i2s_buffers_t const *p_released, uint32_t status)
{
// Select what effect we want: echo or dry signal
#ifdef ECHO
#define DELAY 2048
	static int16_t delay_buffer[DELAY];
	static int delay_index = 0;
#endif
    if (status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED)
    {
		int16_t *rx_buffer = use_first_buffer ? &rx_buffer1[0] : &rx_buffer2[0];
		int16_t *tx_buffer = use_first_buffer ? &tx_buffer1[0] : &tx_buffer2[0];

		size_t end = I2S_BUFF_SIZE / sizeof(uint32_t);
		for (size_t i = 0; i < end; i++)
		{
#ifdef ECHO
			int16_t delayed_sample = delay_buffer[delay_index];
			tx_buffer[i] = rx_buffer[i] + delayed_sample * 0.5;
			
			delay_buffer[delay_index] = rx_buffer[i];
			delay_index = (delay_index + 1) % DELAY;
#else
			tx_buffer[i] = rx_buffer[i];		
#endif
		}
        
		// Toggle buffer pointer
		use_first_buffer != use_first_buffer; 

        // Provide the next buffers for the transfer
        nrfx_i2s_buffers_t next_buffers = {
            .p_rx_buffer = use_first_buffer ? (uint32_t *) &rx_buffer1[0] : (uint32_t *) &rx_buffer2[0], 
            .p_tx_buffer = use_first_buffer ? (uint32_t *) &tx_buffer1[0] : (uint32_t *) &tx_buffer2[0], 
			.buffer_size = I2S_BUFF_SIZE / sizeof(uint32_t),
        };
        nrfx_i2s_next_buffers_set(&i2s_instance, &next_buffers);
    }
    
    if (status & NRFX_I2S_STATUS_TRANSFER_STOPPED)
    {
        i2s_transfer_done = true;
    }
}

nrfx_err_t i2s_start(void)
{
    nrfx_i2s_buffers_t initial_buffers = {
        .p_rx_buffer = (uint32_t *) &rx_buffer1[0],
        .p_tx_buffer = (uint32_t *) &tx_buffer1[0],
		.buffer_size = I2S_BUFF_SIZE / sizeof(uint32_t),
    };
    
	use_first_buffer = true;
    return nrfx_i2s_start(&i2s_instance, &initial_buffers, 0);
}	
	

nrfx_err_t i2s_init(void)
{   
	// Config for nRF5340 Audio DK

    nrfx_i2s_config_t config = {
		.clksrc = I2S_CONFIG_CLKCONFIG_CLKSRC_ACLK,		 
        .sck_pin = 14,   
        .lrck_pin = 16,  
        .mck_pin = 12,
        .sdout_pin = 13, 
        .sdin_pin = 15,  
        .mck_setup = MCKFREQ_6_144_MHZ,  // Adjust based on your setup
		.enable_bypass = false,
        .ratio = NRF_I2S_RATIO_128X, // 48 kHz
        .mode = NRF_I2S_MODE_MASTER,
        .format = NRF_I2S_FORMAT_I2S,
        .alignment = NRF_I2S_ALIGN_LEFT,
        .channels = NRF_I2S_CHANNELS_STEREO,
        .sample_width = NRF_I2S_SWIDTH_16BIT
    };

	return nrfx_i2s_init(&i2s_instance, &config, i2s_handler);
}

////////////////////////////////////////////////////////////////////////////////
// HW_CODEC

/** CS47L63 driver state handle. */
static cs47l63_t cs47l63_driver;

/** CS47L63 subsystems configuration. */
static const uint32_t cs47l63_cfg[][2] =
	{
		// Audio Serial Port 1 (I2S slave, 48kHz 16bit, stereo, RX/TX)
		{
			CS47L63_ASP1_CONTROL2,						// 0x101000200 (matches above)
			(0x10 << CS47L63_ASP1_RX_WIDTH_SHIFT) |		// 16bit (default is 0x18 = 24 bit)
				(0x10 << CS47L63_ASP1_TX_WIDTH_SHIFT) | // 16bit (default is 0x18 = 24 bit)
				(0b010 << CS47L63_ASP1_FMT_SHIFT)		// I2S (is default)
		},
		{
			CS47L63_ASP1_CONTROL3,
			(0b00 << CS47L63_ASP1_DOUT_HIZ_CTRL_SHIFT) // Always 0 (matches above)
		},
		// Enable the various channels for RX and TX
		{
			CS47L63_ASP1_ENABLES1,				   // 0x10001 (no match)
			(1 << CS47L63_ASP1_RX2_EN_SHIFT) |	   // Disabled
				(1 << CS47L63_ASP1_RX1_EN_SHIFT) | // Enabled
				(1 << CS47L63_ASP1_TX2_EN_SHIFT) | // Disabled
				(1 << CS47L63_ASP1_TX1_EN_SHIFT)   // Enabled
		},

#ifdef ENABLE_MIC
		// Enable digital MIC
		/* Set MICBIASes */
		{CS47L63_LDO2_CTRL1, 0x0005},	 // p165, 0b00000101 -> Bit 0, LDO2_EN=1, Bit 2, LDO2_DISCH=1, Voltage = 0x0 (default) 2.4V
		{CS47L63_MICBIAS_CTRL1, 0x00EC}, // p166, 0b11101100 -> Bit 2, MICB1_DISCH=1, discharge when disable, (default = 1)
										 //                     Bit 3, MICB1_RATE=1, 1 = Pop-free start-up/shutdown (default = 0)
										 //                     Bit 5-7 = 0x7 = 2.2V (default)
										 // We only need MICB1B related settings.
										 //{ CS47L63_MICBIAS_CTRL5, 0x0272 },  // p166, 0b0010 0111 0010 -> Bit 1 MICB1A_DISCH=1, discharge when disable (default)
										 //                        -> Bit 4 MICB1B_EN=1
										 //                        -> Bit 5 MICB1B_DISCH=1, discharge when disable (default)
										 //                        -> Bit 6 MICB1B_SRC=1 = VDD_A (default 0, MICBIAS regulator)
										 //                        -> Bit 9 MICB1C_DISCH=1, discharge when disable (default)

		{CS47L63_MICBIAS_CTRL5, 0x0070}, // Only set Bit 4,5,6 fo MICB1B = PDM Mic
		/* Enable IN1L */
		//{ CS47L63_INPUT_CONTROL, 0x000F },  // p29, 0b00001111 -> Bit 0, IN1R_EN,
		//                    Bit 1, IN1L_EN,
		//                    Bit 2, IN2R_EN,
		//                    Bit 3, IN2L_EN

		{CS47L63_INPUT_CONTROL, 0x0002}, // The PDM only needs IN1L_EN -> Bit 1 set

		/* Enable PDM mic as digital input */
		{CS47L63_INPUT1_CONTROL1, 0x50021}, // p31, 0b0101 00000000 0010 0001 -> Bit 0, Input Path 1 Mode=Digital input
											//                                -> Bit 5, fixed at according to p181? (see also p30 "Note")
											//                                -> Bit 16-18 Input Path 1 Oversample control, p27
											//                                   = 0b101 = 3.072 MHz, controls IN1_PDMCLK freq

		/* Un-mute and set gain to 0dB */
		{CS47L63_IN1L_CONTROL2, 0x800080}, // p31 & p34, p181 0b1000 0000 0000 0000 1000 0000
										   // Bit 1-7 = 0x40 = 0dB Input Path 1L PGA Volume (analog only)
										   // Bit 16-23 = 0x80 = default 0dB Input Path 1L Digial volume
										   // Bit 28 = 0 (default 1), unmute

		// We don´t actually need the settings for IN1R for PDM MIC
		//{ CS47L63_IN1R_CONTROL2, 0x800080 },  // p31 & p34, p181 0b1000 0000 0000 0000 1000 0000
		// Bit 1-7 = 0x40 = 0dB Input Path 1R PGA Volume (analog only)
		// Bit 16-23 = 0x80 = default 0dB Input Path 1R Digial volume
		// Bit 28 = 0 (default 1), unmute

		/* Volume Update */
		{CS47L63_INPUT_CONTROL3, 0x20000000}, // p181, set IN_VU to 1

		/* Send PDM MIC to I2S Tx */
		{CS47L63_ASP1TX1_INPUT1, 0x800010},
		// We don´t actually need IN1R but nvm
		{CS47L63_ASP1TX2_INPUT1, 0x800011},
#endif

		// Output 1 left/right (reduced MIX_VOLs to prevent clipping summed signals)
		{
			CS47L63_OUT1L_INPUT1,
			(0x40 << CS47L63_OUT1LMIX_VOL1_SHIFT) | // 0x2b = -21dB, 0x40 = 0dB, 0x2e=-18dB, 0x28=-24dB
				(0x020 << CS47L63_OUT1L_SRC1_SHIFT) // ASP1_RX1 // from MCU (currently a sine wave only)
		},

		{
			CS47L63_OUT1L_INPUT2,
			(0x40 << CS47L63_OUT1LMIX_VOL2_SHIFT) | // 0x2b = -21dB, 0x40 = 0dB, 0x2e=-18dB, 0x28=-24dB
				(0x021 << CS47L63_OUT1L_SRC2_SHIFT) // ASP1_RX2
		},

		{
			CS47L63_OUTPUT_ENABLE_1,
			(1 << CS47L63_OUT1L_EN_SHIFT) // Enabled
		},
};

/**
 * @brief       Write a configuration array to multiple CS47L63 registers.
 *
 * @param[in]   config: Array of address/data pairs.
 * @param[in]   length: Number of registers to write.
 *
 * @retval      `CS47L63_STATUS_OK`     The operation was successful.
 * @retval      `CS47L63_STATUS_FAIL`   Writing to the control port failed.
 */
static int nrfadk_hwcodec_config(const uint32_t config[][2], uint32_t length)
{
	int ret;
	uint32_t addr;
	uint32_t data;

	for (int i = 0; i < length; i++)
	{
		addr = config[i][0];
		data = config[i][1];

		ret = cs47l63_write_reg(&cs47l63_driver, addr, data);
		if (ret)
			return ret;
	}

	return CS47L63_STATUS_OK;
}

// Buggy configuration - leaves ASP2 unconfigured in default GPIO mode
const uint32_t GPIO_configuration[][2] = {
	{CS47L63_GPIO6_CTRL1, 0x61000001}, // ASP2
	{CS47L63_GPIO7_CTRL1, 0x61000001}, // ASP2
	{CS47L63_GPIO8_CTRL1, 0x61000001}, // ASP2

	/* Enable CODEC LED */
	{CS47L63_GPIO10_CTRL1, 0x41008001},
};

/**
 * @brief       Initialize the CS47L63, start clocks, and configure subsystems.
 *
 * @details     MCLK1 =   6,144,000 Hz  (I2S MCK = CONFIG.MCKFREQ)
 *              FSYNC =      48,000 Hz  (I2S LRCK = MCK / CONFIG.RATIO)
 *              BCLK =    1,536,000 Hz  (I2S SCK = LRCK * CONFIG.SWIDTH * 2)
 *              FLL1 =   49,152,000 Hz  (MCLK1 * 8)
 *              SYSCLK = 98,304,000 Hz  (FLL1 * 2)
 *
 * @retval      `CS47L63_STATUS_OK`     The operation was successful.
 * @retval      `CS47L63_STATUS_FAIL`   Initializing the CS47L63 failed.
 *
 * @note        I2S MCK must already be running before calling this function.
 */
static int nrfadk_hwcodec_init(void)
{
	int ret = CS47L63_STATUS_OK;

	// Initialize driver
	ret += cs47l63_comm_init(&cs47l63_driver);

	// Start FLL1 and SYSCLK
	ret += cs47l63_fll_config(&cs47l63_driver, CS47L63_FLL1,
							  CS47L63_FLL_SRC_MCLK1, 6144000, 49152000);

	ret += cs47l63_fll_enable(&cs47l63_driver, CS47L63_FLL1);

	ret += cs47l63_fll_wait_for_lock(&cs47l63_driver, CS47L63_FLL1);

	ret += cs47l63_update_reg(&cs47l63_driver, CS47L63_SYSTEM_CLOCK1,
							  CS47L63_SYSCLK_EN_MASK, CS47L63_SYSCLK_EN);

	// nrfadk_hwcodec_config(clock_configuration, ARRAY_SIZE(clock_configuration));
	nrfadk_hwcodec_config(GPIO_configuration, ARRAY_SIZE(GPIO_configuration));

	// Configure subsystems
	ret += nrfadk_hwcodec_config(cs47l63_cfg, ARRAY_SIZE(cs47l63_cfg));

	return ret;
}

////////////////////////////////////////////////////////////////////////////////
// MAIN

int main(void)
{
	// Initialize Audio DK
	if (nrfadk_hfclocks_init())
	{
		printk("HF Clock init error");
		return -1;
	}
	
	if (i2s_init() != NRFX_SUCCESS) 
	{
		printk("i2s init error");
		return -1;
	} 

	if (i2s_start() != NRFX_SUCCESS)
	{
		printk("i2s start error");
		return -1;
	}
	
	if (nrfadk_hwcodec_init())
	{		
		printk("\nError initializing Audio DK\n");
		return -1;
	}
		
	printk("\nAudio DK initialized\n");
	k_msleep(1250);
	
	// Unmute OUT1L playback
	cs47l63_update_reg(&cs47l63_driver, CS47L63_OUT1L_VOLUME_1,
					   CS47L63_OUT_VU_MASK | CS47L63_OUT1L_MUTE_MASK,
					   CS47L63_OUT_VU | 0);
	
	while(true)
	{
		k_yield();
	};

	// We never return!

	cs47l63_update_reg(&cs47l63_driver, CS47L63_OUT1L_VOLUME_1,
					   CS47L63_OUT_VU_MASK | CS47L63_OUT1L_MUTE_MASK,
					   CS47L63_OUT_VU | CS47L63_OUT1L_MUTE);

	printk("OUT1L muted\n");
	k_msleep(1250);

	// Shutdown Audio DK (reverse order initialized)

	cs47l63_update_reg(&cs47l63_driver, CS47L63_OUTPUT_ENABLE_1,
					   CS47L63_OUT1L_EN_MASK, 0);
	printk("\nOUT1L disabled\n");
	k_msleep(250);

	cs47l63_update_reg(&cs47l63_driver, CS47L63_SYSTEM_CLOCK1,
					   CS47L63_SYSCLK_EN_MASK, 0);
	printk("SYSCLK disabled\n");
	k_msleep(250);

	cs47l63_fll_disable(&cs47l63_driver, CS47L63_FLL1);
	printk("FLL1 disabled\n");
	k_msleep(250);

	nrfx_i2s_stop(&i2s_instance);

	//NRF_I2S0->TASKS_STOP = I2S_TASKS_STOP_TASKS_STOP_Trigger;
	//NRF_I2S0->ENABLE = I2S_ENABLE_ENABLE_Disabled;
	printk("I2S disabled\n");
	k_msleep(250);

	nrfx_clock_stop(NRF_CLOCK_DOMAIN_HFCLKAUDIO);
	while (nrfx_clock_is_running(NRF_CLOCK_DOMAIN_HFCLKAUDIO, NULL))
		k_msleep(1);
	printk("HFCLKAUDIO stopped\n");
	k_msleep(250);

	nrfx_clock_stop(NRF_CLOCK_DOMAIN_HFCLK);
	while (nrfx_clock_is_running(NRF_CLOCK_DOMAIN_HFCLK, NULL))
		k_msleep(1);
	printk("HFCLK stopped\n");
	k_msleep(250);

	printk("\nAudio DK shutdown\n\n");
	k_msleep(100);

	return 0;
}
