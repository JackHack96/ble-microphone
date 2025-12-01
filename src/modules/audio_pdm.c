/* src/modules/audio_pdm.c */
#include "audio_pdm.h"
#include "audio_sync_timer.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <nrfx_pdm.h>
#include <hal/nrf_pdm.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(audio_pdm, CONFIG_MODULE_AUDIO_USB_LOG_LEVEL);

#define PDM_NODE DT_NODELABEL(pdm0)

#if !DT_NODE_EXISTS(PDM_NODE)
#error "PDM node (pdm0) not found in device tree"
#endif

PINCTRL_DT_DEFINE(PDM_NODE);

/* Match the buffer size logic from audio_datapath.c / audio_i2s.c */
#define BLK_PERIOD_US 1000
/* Calculate samples per block. For 16kHz, 1ms = 16 samples. Stereo = 32 samples. */
#define PDM_BLOCK_SAMPLES ((CONFIG_AUDIO_SAMPLE_RATE_HZ * BLK_PERIOD_US) / 1000000)
/* PDM driver works with 16-bit samples. Buffer size is in int16_t units.
 * Multiply by 2 for Stereo.
 */
#define PDM_BUF_SIZE_SAMPLES (PDM_BLOCK_SAMPLES * 2)

/* PDM instance structure - static initialization */
static const nrfx_pdm_t pdm_inst = {
	.p_reg = (NRF_PDM_Type *)DT_REG_ADDR(PDM_NODE),
	.drv_inst_idx = 0,
};

static pdm_blk_comp_callback_t pdm_callback;
static bool pdm_started;
static uint32_t *current_buffer;
static uint32_t *next_buffer;

/* IRQ handler that dispatches to event handler */
static void pdm_irq_handler_wrapper(const void *arg)
{
	ARG_UNUSED(arg);
	
	NRF_PDM_Type *p_pdm = pdm_inst.p_reg;
	
	/* Check and handle STARTED event */
	if (nrf_pdm_event_check(p_pdm, NRF_PDM_EVENT_STARTED)) {
		nrf_pdm_event_clear(p_pdm, NRF_PDM_EVENT_STARTED);
		LOG_DBG("PDM started");
	}
	
	/* Check and handle STOPPED event */
	if (nrf_pdm_event_check(p_pdm, NRF_PDM_EVENT_STOPPED)) {
		nrf_pdm_event_clear(p_pdm, NRF_PDM_EVENT_STOPPED);
		LOG_DBG("PDM stopped");
	}
	
	/* Check and handle END event (buffer complete) */
	if (nrf_pdm_event_check(p_pdm, NRF_PDM_EVENT_END)) {
		nrf_pdm_event_clear(p_pdm, NRF_PDM_EVENT_END);
		
		/* The current_buffer has been filled, report it as completed */
		uint32_t *completed_buffer = current_buffer;
		
		/* Move next_buffer to current for the next cycle */
		current_buffer = next_buffer;
		next_buffer = NULL;
		
		/* Call the callback with the completed buffer */
		if (pdm_callback && completed_buffer) {
			pdm_callback(audio_sync_timer_frame_start_capture_get(),
				     completed_buffer,
				     NULL);
		}
	}
}

void audio_pdm_init(void)
{
	int ret;
	NRF_PDM_Type *p_pdm = pdm_inst.p_reg;
	
	LOG_INF("Initializing PDM interface");

	/* Configure PDM pins using pinctrl */
	ret = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(PDM_NODE), PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Failed to apply pinctrl state: %d", ret);
		return;
	}

	/* Disable PDM before configuration */
	nrf_pdm_disable(p_pdm);
	nrf_pdm_task_trigger(p_pdm, NRF_PDM_TASK_STOP);
	
	/* Configure PDM mode and edge together */
	nrf_pdm_mode_set(p_pdm, NRF_PDM_MODE_STEREO, NRF_PDM_EDGE_LEFTFALLING);
	
	/* Set clock frequency based on sample rate */
#if defined(CONFIG_AUDIO_SAMPLE_RATE_16000_HZ)
	nrf_pdm_clock_set(p_pdm, NRF_PDM_FREQ_1032K);
	/* Ratio 64. 1032k / 64 = 16.125 kHz */
	LOG_INF("PDM clock: 1032 kHz (for 16 kHz sample rate)");
#elif defined(CONFIG_AUDIO_SAMPLE_RATE_24000_HZ)
	nrf_pdm_clock_set(p_pdm, NRF_PDM_FREQ_1280K);
	/* 1280k / 64 = 20kHz, closest to 24kHz */
	LOG_INF("PDM clock: 1280 kHz (for ~20 kHz sample rate)");
#elif defined(CONFIG_AUDIO_SAMPLE_RATE_48000_HZ)
	/* For 48kHz: Use 1.280 MHz / 80 ratio = 16 kHz, or use default 1.067 MHz */
	/* nRF5340 PDM: 1.067 MHz (default) / 64 = 16.67 kHz (not ideal) */
	/* Best option: Use RATIO_80X with higher clock for better quality */
	/* However, RATIO_80X might not be available. Use DEFAULT (1.067 MHz) as fallback */
	nrf_pdm_clock_set(p_pdm, NRF_PDM_FREQ_DEFAULT);
	/* Default is typically 1.067 MHz. 1067k / 64 = ~16.67 kHz - STILL WRONG! */
	/* We need to decimate/upsample in software, or accept lower sample rate */
	LOG_WRN("PDM clock: DEFAULT (1.067 MHz) - produces ~16.67 kHz, not 48 kHz!");
	LOG_WRN("Audio will need software resampling or use lower BT profile");
#else
	nrf_pdm_clock_set(p_pdm, NRF_PDM_FREQ_DEFAULT);
	LOG_INF("PDM clock: DEFAULT");
#endif

	/* Set gain */
	nrf_pdm_gain_set(p_pdm, NRF_PDM_GAIN_DEFAULT, NRF_PDM_GAIN_DEFAULT);
	
	/* Configure ratio */
	#if defined(NRF_PDM_HAS_RATIO_CONFIG) && (NRF_PDM_HAS_RATIO_CONFIG == 1)
	nrf_pdm_ratio_set(p_pdm, NRF_PDM_RATIO_64X);
	#endif

	/* Connect and enable IRQ */
	IRQ_CONNECT(DT_IRQN(PDM_NODE), DT_IRQ(PDM_NODE, priority), 
		    pdm_irq_handler_wrapper, NULL, 0);
	irq_enable(DT_IRQN(PDM_NODE));
	
	/* Enable PDM interrupts */
	nrf_pdm_int_enable(p_pdm, NRF_PDM_INT_STARTED |
				   NRF_PDM_INT_STOPPED |
				   NRF_PDM_INT_END);

	/* Enable PDM peripheral */
	nrf_pdm_enable(p_pdm);

	LOG_INF("PDM initialized successfully (Stereo, ACLK)");
}

void audio_pdm_start(uint32_t *rx_buf)
{
	if (!pdm_started) {
		NRF_PDM_Type *p_pdm = pdm_inst.p_reg;
		
		/* Track the first buffer */
		current_buffer = rx_buf;
		next_buffer = NULL;
		
		/* Set the sample buffer */
		nrf_pdm_buffer_set(p_pdm, rx_buf, PDM_BUF_SIZE_SAMPLES);
		
		/* Start PDM sampling */
		nrf_pdm_task_trigger(p_pdm, NRF_PDM_TASK_START);
		
		pdm_started = true;
		LOG_INF("PDM capture started");
	}
}

void audio_pdm_stop(void)
{
	if (pdm_started) {
		NRF_PDM_Type *p_pdm = pdm_inst.p_reg;
		
		/* Stop PDM sampling */
		nrf_pdm_task_trigger(p_pdm, NRF_PDM_TASK_STOP);
		
		pdm_started = false;
		LOG_INF("PDM capture stopped");
	}
}

void audio_pdm_set_next_buf(uint32_t *rx_buf)
{
	NRF_PDM_Type *p_pdm = pdm_inst.p_reg;
	
	/* Track the next buffer */
	next_buffer = rx_buf;
	
	/* Feed the next buffer to PDM peripheral */
	nrf_pdm_buffer_set(p_pdm, rx_buf, PDM_BUF_SIZE_SAMPLES);
}

void audio_pdm_blk_comp_cb_register(pdm_blk_comp_callback_t callback)
{
	pdm_callback = callback;
}