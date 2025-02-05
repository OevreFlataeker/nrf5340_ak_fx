/**
 * @file        main.c
 * 
 * @brief       Audio DK HW_CODEC test using I2S loop and tone/noise generators.
 */

#include <zephyr/kernel.h>
#include <nrf.h>
#include <nrfx_clock.h>

#include "cs47l63_comm.h"



////////////////////////////////////////////////////////////////////////////////
// NRFX_CLOCKS

#define HFCLKAUDIO_12_288_MHZ 0x9BA6
#undef ENABLE_LINEIN
#undef ENABLE_LINEIN_PASSTHROUGH
#define ENABLE_MIC
#undef ENABLE_MIC_PASSTHROUGH


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
	if (err != NRFX_SUCCESS) return (err - NRFX_ERROR_BASE_NUM);

	nrfx_clock_start(NRF_CLOCK_DOMAIN_HFCLK);
	while (!nrfx_clock_is_running(NRF_CLOCK_DOMAIN_HFCLK, NULL)) k_msleep(1);


	// HFCLKAUDIO
	nrfx_clock_hfclkaudio_config_set(HFCLKAUDIO_12_288_MHZ);

	nrfx_clock_start(NRF_CLOCK_DOMAIN_HFCLKAUDIO);
	while (!nrfx_clock_is_running(NRF_CLOCK_DOMAIN_HFCLKAUDIO, NULL)) k_msleep(1);


	return 0;
}



////////////////////////////////////////////////////////////////////////////////
// NRF_I2S

#define MCKFREQ_6_144_MHZ 0x66666000


/** I2S TX buffer. */
static int16_t sine750_48k_16b_1c[64] =
{
	  3211,   6392,   9511,  12539,  15446,  18204,  20787,  23169,
	 25329,  27244,  28897,  30272,  31356,  32137,  32609,  32767,
	 32609,  32137,  31356,  30272,  28897,  27244,  25329,  23169,
	 20787,  18204,  15446,  12539,   9511,   6392,   3211,      0,
	 -3212,  -6393,  -9512, -12540, -15447, -18205, -20788, -23170,
	-25330, -27245, -28898, -30273, -31357, -32138, -32610, -32767,
	-32610, -32138, -31357, -30273, -28898, -27245, -25330, -23170,
	-20788, -18205, -15447, -12540,  -9512,  -6393,  -3212,     -1,
};

static int16_t zerowave[64] = 
{
	0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,
};

#define I2S_BUFF_SIZE 8192  // Define an appropriate buffer size

static int16_t rx_buffer[I2S_BUFF_SIZE];  // Buffer for received data
static int16_t tx_buffer[I2S_BUFF_SIZE]; 

void i2s_callback(void)
{
    // Process received data (example: simple passthrough)
    for (int i = 0; i < I2S_BUFF_SIZE; i++) {
        tx_buffer[i] = rx_buffer[i];  // Modify as needed
    }

    // Restart TX with modified data
    NRF_I2S0->TXD.PTR = (uint32_t)tx_buffer;
    NRF_I2S0->RXTXD.MAXCNT = I2S_BUFF_SIZE / sizeof(uint32_t);

    // Restart I2S transmission and reception
    NRF_I2S0->TASKS_START = I2S_TASKS_START_TASKS_START_Trigger;
}

#define DELAY 2048
void process_audio(int16_t *rx, int16_t *tx, size_t size)
{
	static int16_t delay_buffer[DELAY];
	static int delay_index = 0;

	for (int i=0; i<size; ++i) {
		int16_t delayed_sample = delay_buffer[delay_index];
		tx[i] = rx[i] + delayed_sample * 0.5;
		//printk("%d %d\n", tx[i], rx[i]);
		delay_buffer[delay_index] = rx[i];
		delay_index = (delay_index + 1) % DELAY;
	}
    
	/*
	
	for (size_t i = 0; i < size; i++) {
		tx[i] = rx[i];  // Simple passthrough (modify as needed)
        //tx[i] = (int16_t)(rx[i]*0.1);  // Volume control
		//tx[size-1-i] = rx[i];  // Reverse - doesn't work... -> only outputs silence...?

		//printk("0x%02x ", rx[i]);
    }
	*/
}

/**
 * @brief       Initialize and start the I2S peripheral using NRF registers.
 * 
 * @details     I2S master, 48kHz 16bit, Left mono, TX only.
 */
static int nrfadk_i2s_reg_init(void)
{
	// Configure and enable
	NRF_I2S0->CONFIG.CLKCONFIG =    I2S_CONFIG_CLKCONFIG_CLKSRC_ACLK;
	NRF_I2S0->CONFIG.MCKFREQ =      MCKFREQ_6_144_MHZ;
	NRF_I2S0->CONFIG.RATIO =        I2S_CONFIG_RATIO_RATIO_128X;
	NRF_I2S0->CONFIG.CHANNELS =     I2S_CONFIG_CHANNELS_CHANNELS_Stereo; // if I2S_CONFIG_CHANNELS_CHANNELS_Left the left and right channel will be the same	
	NRF_I2S0->CONFIG.TXEN =         I2S_CONFIG_TXEN_TXEN_Enabled;
	NRF_I2S0->CONFIG.RXEN = 		I2S_CONFIG_RXEN_RXEN_Enabled;  // Enable RX (new)

	NRF_I2S0->ENABLE =              I2S_ENABLE_ENABLE_Enabled;

#define ECHO
#ifdef ECHO
	// Start TX buffer
	NRF_I2S0->RXD.PTR = 			(uint32_t)rx_buffer;
	NRF_I2S0->TXD.PTR = 			(uint32_t)tx_buffer;
    NRF_I2S0->RXTXD.MAXCNT = 		I2S_BUFF_SIZE / sizeof(uint32_t);	
#endif
	
#ifdef GENERATE_TONE
	NRF_I2S0->TXD.PTR =             (uint32_t)&sine750_48k_16b_1c[0];
	
	//NRF_I2S0->RXTXD.MAXCNT =        (sizeof(zerowave)) / (sizeof(uint32_t));//(sizeof(sine750_48k_16b_1c)) / (sizeof(uint32_t));
	NRF_I2S0->RXTXD.MAXCNT =        (sizeof(sine750_48k_16b_1c)) / (sizeof(uint32_t));
#endif
	
	 // Clear pending events
    NRF_I2S0->EVENTS_RXPTRUPD = 0;
    NRF_I2S0->EVENTS_TXPTRUPD = 0;

	// Enable interrupt for I2S event
    //NRF_I2S0->INTENSET = I2S_INTENSET_RXPTRUPD_Msk;  // Trigger on RX update
	
	
	NRF_I2S0->TASKS_START =         I2S_TASKS_START_TASKS_START_Trigger;
	//NVIC_ClearPendingIRQ(I2S0_IRQn);

	//NVIC_SetPriority(I2S0_IRQn, 3);
    //NVIC_EnableIRQ(I2S0_IRQn);
	return 0;
}

void i2s_polling_loop(void)
{
    while (1)
    {
        // Wait for new RX data
        while (NRF_I2S0->EVENTS_RXPTRUPD == 0);
		
		// printk("\nEvents: %d\n", NRF_I2S0->EVENTS_RXPTRUPD);
        // Clear RX event
        NRF_I2S0->EVENTS_RXPTRUPD = 0;

        // Process audio data
        process_audio(rx_buffer, tx_buffer, I2S_BUFF_SIZE);

        // Set TX buffer and restart TX
        //NRF_I2S0->TXD.PTR = (uint32_t)&sine750_48k_16b_1c[0];//(uint32_t)tx_buffer;
		//NRF_I2S0->RXTXD.MAXCNT =        (sizeof(sine750_48k_16b_1c)) / (sizeof(uint32_t));
        // Ensure we restart I2S for continuous operation
        // NRF_I2S0->TASKS_START = I2S_TASKS_START_TASKS_START_Trigger;
		while (NRF_I2S0->EVENTS_TXPTRUPD == 0);
    }
}

// ISR for I2S
void I2S0_IRQHandler(void)
{
    if (NRF_I2S0->EVENTS_RXPTRUPD)
    {
		printk("\nin ISR");
        NRF_I2S0->EVENTS_RXPTRUPD = 0;  // Clear event flag
        i2s_callback();
    }
}

////////////////////////////////////////////////////////////////////////////////
// HW_CODEC

/** CS47L63 driver state handle. */
static cs47l63_t cs47l63_driver;


/** CS47L63 subsystems configuration. */
static const uint32_t cs47l63_cfg[][2] =
{

#undef i2s_echo_ASP1
#ifdef i2s_echo_ASP1
	// Uses sample rate of 48kHz instead of 16kHz like below!
	/* Enable ASP1 GPIOs */
	{ CS47L63_GPIO1_CTRL1, 0xE1000000 }, // ASP1_DOUT, Sets the default config of GPIO1 (ASP1 interface) to Non-GPIO mode GPn_FN = 0, rest ignored, page 153
	{ CS47L63_GPIO2_CTRL1, 0xE1000000 }, // ASP1_DIN, Sets the default config of GPIO2 (ASP1 interface) to Non-GPIO mode GPn_FN = 0, rest ignored, 
	{ CS47L63_GPIO3_CTRL1, 0xE1000000 }, // ASP1_BCLK, Sets the default config of GPIO3 (ASP1 interface) to Non-GPIO mode GPn_FN = 0, rest ignored, 
	{ CS47L63_GPIO4_CTRL1, 0xE1000000 }, // ASP1_FSYNC, Sets the default config of GPIO4 (ASP1 interface) to Non-GPIO mode GPn_FN = 0, rest ignored, 

    /* Set correct sample rate */
	{ CS47L63_SAMPLE_RATE1, 0x000000012 }, // p125, 4.10.2., p132, 0x12 == 16 kHz -> p125, 4.10.3.1, requires SYSCLK to be 98.304MHz, SYSCLK_FREQ=100, SYSCLK_FRAC=0
	                                       // default is 0x03 == 48 kHz

	/* Disable unused sample rates */
	{ CS47L63_SAMPLE_RATE2, 0 }, // p132, None (default is 0)
	{ CS47L63_SAMPLE_RATE3, 0 }, // p132, None (default is 0)
	{ CS47L63_SAMPLE_RATE4, 0 }, // p132, None (default is 0)

	/* Set ASP1 in slave mode and 16 bit per channel */
	{ CS47L63_ASP1_CONTROL2, 0x10100200 }, // 00010000 00010000 00000010 00000000. p115
										   // Bit 0 (ASP1_FSYNC_MSTR) = ASP1_FSYNC Slave Mode
										   // Bit 1 (ASP1_FSYNC_FRC) = Normal
										   // Bit 2 (ASP1_FSYNC_INV) = ASP1_FSYNC not inverted
										   // Bit 4 (ASP1_BCLK_MSTR) = ASP1_BCLK Slave Mode
										   // Bit 5 (ASP1_BCLK_FRC) = Normal
										   // Bit 6 (ASP1_BCLK_INV)  = ASP1_BCLK not inverted
										   // Bit 8-10 = 0b010 = I2S Mode (p118) (is default)
										   // Bit 16-23 = 0b10000 = 0x10 = 16 = ASP1 TX Slot Width (Number of BCLK cycles per slot), default is 0x18 = 24 bit
										   // Bit 24-31 = 0b10000 = 0x10 = 16 = ASP1 RX Slot Width (Number of BCLK cycles per slot), default is 0x18 = 24 bit
	
	{ CS47L63_ASP1_CONTROL3, 0x0000 }, // Always 0, p120, ASP1_DOUT Tristate Control, 00 = Logic 0 during unused time slots, Logic 0 if all transmit channels are disabled
	{ CS47L63_ASP1_DATA_CONTROL1, 0x0020 }, // p118, 0x20 = 32, ASP1 TX Data Width (Number of valid data bits per slot), is default
	{ CS47L63_ASP1_DATA_CONTROL5, 0x0020 }, // p118, 0x20 = 32, ASP1 RX Data Width (Number of valid data bits per slot), is default
	{ CS47L63_ASP1_ENABLES1, 0x30003 },	  // 0011 00000000 00000011, p116
										  // Bit 0 = 1 = ASP1_TX1_EN
										  // Bit 1 = 1 = ASP1_TX2_EN
										  // Bit 16 = 1 = ASP1_RX1_EN
										  // Bit 17 = 1 = ASP1_RX2_EN
	// What is different?
	// (No change from default: Additional configuration of ASP1 RX/TX Data Width in CS47L63_ASP1_DATA_CONTROL1, CS47L63_ASP1_DATA_CONTROL5 to 0x20 = 32, default is 0x20)
	// Additional activation of ASP1_TX2 and ASP1_RX2 in CS47L63_ASP1_ENABLES1
	// Explicit configuration of GPIO ports for alternate services (default is GPIO and not alternate services!)
	// !! Explicit configuration of sample rate in CS47L63_SAMPLE_RATE1,2,3,4 to 16kHz (0x12), default is 48kHz (0x0x3) !!
#else
	// Audio Serial Port 1 (I2S slave, 48kHz 16bit, Left mono, RX)
	{ CS47L63_ASP1_CONTROL2,							// 0x101000200 (matches above)
		(0x10  << CS47L63_ASP1_RX_WIDTH_SHIFT) |        // 16bit (default is 0x18 = 24 bit)
		(0x10  << CS47L63_ASP1_TX_WIDTH_SHIFT) |        // 16bit (default is 0x18 = 24 bit)
		(0b010 << CS47L63_ASP1_FMT_SHIFT)               // I2S (is default)
	},
	{ CS47L63_ASP1_CONTROL3,
		(0b00 << CS47L63_ASP1_DOUT_HIZ_CTRL_SHIFT)      // Always 0 (matches above)
	},
	// Enable the various channels for RX and TX
	{ CS47L63_ASP1_ENABLES1,							// 0x10001 (no match)
		(1 << CS47L63_ASP1_RX2_EN_SHIFT) |              // Disabled
		(1 << CS47L63_ASP1_RX1_EN_SHIFT) |              // Enabled
		(1 << CS47L63_ASP1_TX2_EN_SHIFT) |              // Disabled
		//(0 << CS47L63_ASP1_TX1_EN_SHIFT)               // Disabled
		(1 << CS47L63_ASP1_TX1_EN_SHIFT)                // Enabled
	},
#endif

#ifdef ENABLE_MIC
	// Enable digital MIC
	/* Set MICBIASes */
	{ CS47L63_LDO2_CTRL1, 0x0005 },     // p165, 0b00000101 -> Bit 0, LDO2_EN=1, Bit 2, LDO2_DISCH=1, Voltage = 0x0 (default) 2.4V
	{ CS47L63_MICBIAS_CTRL1, 0x00EC },  // p166, 0b11101100 -> Bit 2, MICB1_DISCH=1, discharge when disable, (default = 1)
	                                    //                     Bit 3, MICB1_RATE=1, 1 = Pop-free start-up/shutdown (default = 0)
										//                     Bit 5-7 = 0x7 = 2.2V (default)
	// We only need MICB1B related settings.
	//{ CS47L63_MICBIAS_CTRL5, 0x0272 },  // p166, 0b0010 0111 0010 -> Bit 1 MICB1A_DISCH=1, discharge when disable (default)
   	                                      //                        -> Bit 4 MICB1B_EN=1
										  //                        -> Bit 5 MICB1B_DISCH=1, discharge when disable (default)
										  //                        -> Bit 6 MICB1B_SRC=1 = VDD_A (default 0, MICBIAS regulator)
										  //                        -> Bit 9 MICB1C_DISCH=1, discharge when disable (default)  
	
	{ CS47L63_MICBIAS_CTRL5, 0x0070 },    // Only set Bit 4,5,6 fo MICB1B = PDM Mic
	/* Enable IN1L */
	//{ CS47L63_INPUT_CONTROL, 0x000F },  // p29, 0b00001111 -> Bit 0, IN1R_EN, 
										  //                    Bit 1, IN1L_EN, 
										  //                    Bit 2, IN2R_EN, 
										  //                    Bit 3, IN2L_EN
    
	{ CS47L63_INPUT_CONTROL, 0x0002 },    // The PDM only needs IN1L_EN -> Bit 1 set
	
	/* Enable PDM mic as digital input */
	{ CS47L63_INPUT1_CONTROL1, 0x50021 }, // p31, 0b0101 00000000 0010 0001 -> Bit 0, Input Path 1 Mode=Digital input
	                                      //                                -> Bit 5, fixed at according to p181? (see also p30 "Note")
										  //                                -> Bit 16-18 Input Path 1 Oversample control, p27
										  //                                   = 0b101 = 3.072 MHz, controls IN1_PDMCLK freq

	/* Un-mute and set gain to 0dB */
	{ CS47L63_IN1L_CONTROL2, 0x800080 },  // p31 & p34, p181 0b1000 0000 0000 0000 1000 0000
	  									  // Bit 1-7 = 0x40 = 0dB Input Path 1L PGA Volume (analog only)
	                                      // Bit 16-23 = 0x80 = default 0dB Input Path 1L Digial volume
										  // Bit 28 = 0 (default 1), unmute

	// We don´t actually need the settings for IN1R for PDM MIC
	//{ CS47L63_IN1R_CONTROL2, 0x800080 },  // p31 & p34, p181 0b1000 0000 0000 0000 1000 0000
										  // Bit 1-7 = 0x40 = 0dB Input Path 1R PGA Volume (analog only)
	                                      // Bit 16-23 = 0x80 = default 0dB Input Path 1R Digial volume
										  // Bit 28 = 0 (default 1), unmute

	/* Volume Update */
	{ CS47L63_INPUT_CONTROL3, 0x20000000 }, // p181, set IN_VU to 1

	/* Send PDM MIC to I2S Tx */
	{ CS47L63_ASP1TX1_INPUT1, 0x800010 },
	// We don´t actually need IN1R but nvm
	{ CS47L63_ASP1TX2_INPUT1, 0x800011 },
#endif

#ifdef ENABLE_LINEIN
	// Enable line-in
	{ CS47L63_INPUT2_CONTROL1, 0x00050020 },/* MODE=analog */ 
	{ CS47L63_IN2L_CONTROL1, 0x10000000 },  /* SRC=IN2LP */
	{ CS47L63_IN2R_CONTROL1, 0x10000000 },  /* SRC=IN2RP */
	{ CS47L63_INPUT_CONTROL, 0x0000000C },  /* IN2_EN=1 */
	// Set volume for line-in
	{ CS47L63_IN2L_CONTROL2, 0x00800080 },  /* VOL=0dB, MUTE=0 */
	{ CS47L63_IN2R_CONTROL2, 0x00800080 },  /* VOL=0dB, MUTE=0 */
	{ CS47L63_INPUT_CONTROL3, 0x20000000 }, /* VU=1 */
	// Important
	/* Route IN2L and IN2R to I2S */
	{ CS47L63_ASP1TX1_INPUT1, 0x800012 },
	{ CS47L63_ASP1TX2_INPUT1, 0x800013 },
#endif
#ifdef COMMENTS
	// Noise Generator (increased GAIN from -114dB to 0dB)
	/*
	{ CS47L63_COMFORT_NOISE_GENERATOR,
		(0    << CS47L63_NOISE_GEN_EN_SHIFT) |          // Disabled
		(0x13 << CS47L63_NOISE_GEN_GAIN_SHIFT)          // 0dB
	},
	*/
#endif
	// Output 1 Left (reduced MIX_VOLs to prevent clipping summed signals)
	// this here is only there so we hear also that i2s data is sent 
	
	{ CS47L63_OUT1L_INPUT1,
		(0x40  << CS47L63_OUT1LMIX_VOL1_SHIFT) |        // 0x2b = -21dB, 0x40 = 0dB, 0x2e=-18dB, 0x28=-24dB
		(0x020 << CS47L63_OUT1L_SRC1_SHIFT)             // ASP1_RX1 // from MCU (currently a sine wave only)
	},

	{ CS47L63_OUT1L_INPUT2,
		(0x40  << CS47L63_OUT1LMIX_VOL2_SHIFT) |        // 0x2b = -21dB, 0x40 = 0dB, 0x2e=-18dB, 0x28=-24dB
		(0x021 << CS47L63_OUT1L_SRC2_SHIFT)             // ASP1_RX2
	},

#ifdef COMMENTS
	//{ CS47L63_OUT1L_INPUT3,
	//	(0x2B  << CS47L63_OUT1LMIX_VOL3_SHIFT) |        // -21dB
	//	(0x004 << CS47L63_OUT1L_SRC3_SHIFT)             // TONE1_GEN
	//},
#endif	

#ifdef ENABLE_LINEIN_PASSTHROUGH
	// We need both channels here, even if we only have one output channel
	// Iw we uncomment the next two {} we won´t get any line in pass-through, only i2s
	{
		CS47L63_OUT1L_INPUT3,
		(0x2B  << CS47L63_OUT1LMIX_VOL3_SHIFT) |
		(0x012 << CS47L63_OUT1L_SRC3_SHIFT)    // 0x12=IN2L
	},
	{
		CS47L63_OUT1L_INPUT4,
		(0x2B  << CS47L63_OUT1LMIX_VOL4_SHIFT) |
		(0x013 << CS47L63_OUT1L_SRC4_SHIFT)    // 0x13=IN2R
	},
#endif

#ifdef ENABLE_MIC_PASSTHROUGH
	// We need both channels here, even if we only have one output channel
	// Iw we uncomment the next two {} we won´t get any line in pass-through, only i2s
	{
		CS47L63_OUT1L_INPUT3,
		(0x2B  << CS47L63_OUT1LMIX_VOL3_SHIFT) |
		(0x010 << CS47L63_OUT1L_SRC3_SHIFT)    // 0x10=IN1L
	},
	{
		CS47L63_OUT1L_INPUT4,
		(0x2B  << CS47L63_OUT1LMIX_VOL4_SHIFT) |
		(0x011 << CS47L63_OUT1L_SRC4_SHIFT)    // 0x11=IN1R
	},
#endif

#ifdef COMMENTS
	//{ CS47L63_OUT1L_INPUT4,
	//	(0x28  << CS47L63_OUT1LMIX_VOL4_SHIFT) |        // -24dB
	//	(0x00C << CS47L63_OUT1L_SRC4_SHIFT)             // NOISE_GEN
	//},
#endif
	{ CS47L63_OUTPUT_ENABLE_1,
		(1 << CS47L63_OUT1L_EN_SHIFT)                   // Enabled
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
		if (ret) return ret;
	}

	return CS47L63_STATUS_OK;
}

// Just for testing and as reference!

// Unused in minimum example
// Used in i2s_echo
// clocks are different from our barebone example (see comments for nrfadk_hwcodec_init())
const uint32_t clock_configuration[][2] = {
	{ CS47L63_SAMPLE_RATE3, 0x0012 }, // p125, p132, 16kHz
	{ CS47L63_SAMPLE_RATE2, 0x0002 }, // p125, p132, 24kHz
	{ CS47L63_SAMPLE_RATE1, 0x0003 }, // p125, p132, 48kHz
	{ CS47L63_SYSTEM_CLOCK1, 0x034C }, // p131, 0b0011 01001100, Bit 8-10: 0b011 = 49.152 MHz, Bit 6 = SYSCLK_EN, Bit 0-4 = 0x00 = MCLK1
	{ CS47L63_ASYNC_CLOCK1, 0x034C }, // p132, 0b0011 01001100, Bit 8-10: 0b011 = 49.152 MHz, Bit 6 = ASYNCCLK_EN, Bit 0-4 = 0x00 = MCLK1
	{ CS47L63_FLL1_CONTROL2, 0x88200008 }, // p139, 0b10001000 00100000 00000000 00001000, Bit 0-9 = FLL1 Integer multiply for FREF, default 0x4, Bit 21 = FLL1 Reference Detect control, enabled (default), Bit 27 = FLL1_LOCKDET, enabled (default), Bit 28-31 = FLL1 Lock Detect threshold = 0x8 = default
	{ CS47L63_FLL1_CONTROL3, 0x10000 }, // p139, FLL1 Fractional multiply for FREF = 0x1, default is 0x0
	{ CS47L63_FLL1_GPIO_CLOCK, 0x0005 }, // p139, FLL1_GPCLK_DIV = 0x02, default
	{ CS47L63_FLL1_CONTROL1, 0x0001 }, // p139, FLL1 enable
};

// Buggy configuration - leaves ASP2 unconfigured in default GPIO mode
const uint32_t GPIO_configuration[][2] = {
	{ CS47L63_GPIO6_CTRL1, 0x61000001 }, // ASP2
	{ CS47L63_GPIO7_CTRL1, 0x61000001 }, // ASP2
	{ CS47L63_GPIO8_CTRL1, 0x61000001 }, // ASP2

	/* Enable CODEC LED */
	{ CS47L63_GPIO10_CTRL1, 0x41008001 },
};

const uint32_t FLL_toggle[][2] = {
	{ CS47L63_FLL1_CONTROL1, 0x0000 }, // p139, FLL1 disable
	{ 0x01, 1000 },                    // wait 1000ms
	{ CS47L63_FLL1_CONTROL1, 0x0001 }, // p139, FLL1 enable
};

const uint32_t soft_reset[][2] = {
	{ CS47L63_SFT_RESET, 0x5A000000 }, // p168, write 0x5A to bits 24-31 to perform softreset
	{ 0x01, 3000 },                    // wait 3000ms
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
	
	//nrfadk_hwcodec_config(clock_configuration, ARRAY_SIZE(clock_configuration));
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

	if (nrfadk_hfclocks_init() ||
	    nrfadk_i2s_reg_init()  ||
	    nrfadk_hwcodec_init())
	{
		printk("\nError initializing Audio DK\n");
		return -1;
	}

	printk("\nAudio DK initialized\n");
	k_msleep(1250);



	// Unmute OUT1L I2S playback and enable NOISE/TONE1 generators

	cs47l63_update_reg(&cs47l63_driver, CS47L63_OUT1L_VOLUME_1,
	                   CS47L63_OUT_VU_MASK | CS47L63_OUT1L_MUTE_MASK,
	                   CS47L63_OUT_VU | 0);

	printk("\nOUT1L unmuted for 5000ms\n");

	//k_msleep(5000);
	i2s_polling_loop();

#ifdef COMMENTS
	//cs47l63_update_reg(&cs47l63_driver, CS47L63_COMFORT_NOISE_GENERATOR,
	//                   CS47L63_NOISE_GEN_EN_MASK, CS47L63_NOISE_GEN_EN);

	//printk("NOISE enabled\n");
	//k_msleep(3000);


	//cs47l63_update_reg(&cs47l63_driver, CS47L63_TONE_GENERATOR1,
	//                   CS47L63_TONE1_EN_MASK, CS47L63_TONE1_EN);

	//printk("TONE1 enabled\n");
	//k_msleep(3000);



	// Disable NOISE/TONE1 generators and mute OUT1L I2S playback

	//cs47l63_update_reg(&cs47l63_driver, CS47L63_COMFORT_NOISE_GENERATOR,
	//                   CS47L63_NOISE_GEN_EN_MASK, 0);

	//printk("\nNOISE disabled\n");
	//k_msleep(3000);


	//cs47l63_update_reg(&cs47l63_driver, CS47L63_TONE_GENERATOR1,
	//                   CS47L63_TONE1_EN_MASK, 0);

	//printk("TONE1 disabled\n");
	//k_msleep(3000);
#endif

	while(1);
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

	NRF_I2S0->TASKS_STOP =  I2S_TASKS_STOP_TASKS_STOP_Trigger;
	NRF_I2S0->ENABLE =      I2S_ENABLE_ENABLE_Disabled;
	printk("I2S disabled\n");
	k_msleep(250);

	nrfx_clock_stop(NRF_CLOCK_DOMAIN_HFCLKAUDIO);
	while (nrfx_clock_is_running(NRF_CLOCK_DOMAIN_HFCLKAUDIO, NULL)) k_msleep(1);
	printk("HFCLKAUDIO stopped\n");
	k_msleep(250);

	nrfx_clock_stop(NRF_CLOCK_DOMAIN_HFCLK);
	while (nrfx_clock_is_running(NRF_CLOCK_DOMAIN_HFCLK, NULL)) k_msleep(1);
	printk("HFCLK stopped\n");
	k_msleep(250);


	printk("\nAudio DK shutdown\n\n");
	k_msleep(100);


	return 0;
}
