
UINT32 bk_timer_disable(UINT32 channel);
UINT32 bk_timer_get_int_status(UINT32 channel, UINT32 *status);
UINT32 bk_timer_get_end_time(UINT32 channel, UINT32 *count_ms);
UINT32 bk_timer_enable(UINT32 channel, UINT32 delay_ms);
UINT32 bk_timer_measure(UINT32 channel, UINT32 *count_ms);
UINT32 bk_timer_pre_measure(UINT32 channel);


