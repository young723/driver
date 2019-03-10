#ifndef INCLUDE_AKM_DATASORT_H
#define INCLUDE_AKM_DATASORT_H

void akm_data_sort_init(void);
void akm_set_buffer_len(int len, int algo);
int akm_data_sort(struct AKM_SENSOR_DATA * input);
int akm_fusion_data_sort(struct AKM_SENSOR_DATA * input);

#endif

