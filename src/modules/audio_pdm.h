/* src/modules/audio_pdm.h */
#ifndef _AUDIO_PDM_H_
#define _AUDIO_PDM_H_

#include <stdint.h>

/**
 * Calculation:
 * FREQ_VALUE = 2^16 * ((12 * f_out / 32M) - 4)
 * f_out == 12.288
 * 39845.888 = 2^16 * ((12 * 12.288 / 32M) - 4)
 * 39846 = 0x9BA6
 */
#define HFCLKAUDIO_12_288_MHZ 0x9BA6
#define HFCLKAUDIO_12_165_MHZ 0x8FD8
#define HFCLKAUDIO_12_411_MHZ 0xA774

typedef void (*pdm_blk_comp_callback_t)(uint32_t frame_start_ts_us, uint32_t *rx_buf,
					uint32_t *tx_buf);

void audio_pdm_init(void);
void audio_pdm_start(uint32_t *rx_buf);
void audio_pdm_stop(void);
void audio_pdm_set_next_buf(uint32_t *rx_buf);
void audio_pdm_blk_comp_cb_register(pdm_blk_comp_callback_t callback);

#endif /* _AUDIO_PDM_H_ */