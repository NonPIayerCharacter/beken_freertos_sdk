#ifndef __AUDIO_INTF_H__
#define __AUDIO_INTF_H__

extern UINT32 audio_intf_init(void);
extern void audio_intf_dac_set_volume(void);
extern void audio_intf_adc_play(void);
extern void audio_intf_dac_play(void);
extern void audio_intf_dac_pause(void);
extern void audio_intf_adc_pause(void);
extern void audio_intf_set_sample_rate(void);

#endif
