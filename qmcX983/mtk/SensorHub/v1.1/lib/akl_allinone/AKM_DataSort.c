#include "AKM_CustomerSpec.h"
#include "AKM_Common.h"
#include "AKM_DataSort.h"

typedef struct _mag_data_buffer {
    struct AKM_SENSOR_DATA data;
    struct _mag_data_buffer *next;
} DATA_BUFFER;

typedef struct data_buffer_store{
    DATA_BUFFER *node;
    int buffer_size;
    int buffer_count;
}BUFFER_STORE;
#define AKM_SORT_BUFFER_MAX_LEN 36
#define AKM_SORT_BUFFER_LEN 10
//static int akm_buffer_count = 0;
//static int akm_buffer_len = 10;

//DATA_BUFFER *node_head = NULL;

typedef enum{
    CALIBRATE_BUFFER,
    FUSION_BUFFER,
    DATA_SORT_BUFFER_MAX,
}AKM_ALGO_TYPE;
struct data_buffer_store data_sort_buffer[DATA_SORT_BUFFER_MAX];
int akm_insert_data(DATA_BUFFER * node_add, int algo)
{
        DATA_BUFFER *node_tmp1 = data_sort_buffer[algo].node;
        DATA_BUFFER *node_tmp2 = NULL;
        data_sort_buffer[algo].buffer_count++;
        if (data_sort_buffer[algo].node == NULL) {
                data_sort_buffer[algo].node = node_add;
        } else {
                while (node_tmp1->next != NULL) {
                        if ((node_add->data.time_stamp > node_tmp1->data.time_stamp)
                            && (node_add->data.time_stamp <= node_tmp1->next->data.time_stamp)) {
                                node_tmp2 = node_tmp1->next;
                                node_tmp1->next = node_add;
                                node_add->next = node_tmp2;
                                node_tmp2 = NULL;
                                return AKM_SUCCESS;
                        } else {
                                node_tmp1 = node_tmp1->next;
                        }
                }

                if (node_add->data.time_stamp > node_tmp1->data.time_stamp) {
                        node_tmp1->next = node_add;
                } else {
                        node_add->next = data_sort_buffer[algo].node;
                        data_sort_buffer[algo].node = node_add;
                }
        }
        return AKM_SUCCESS;
}

int akm_get_data(struct AKM_SENSOR_DATA * result, int algo)
{
        DATA_BUFFER *node_tmp;
        AKM_MEMCOPY(result, &data_sort_buffer[algo].node->data, sizeof(struct AKM_SENSOR_DATA));
        if(data_sort_buffer[algo].buffer_count == data_sort_buffer[algo].buffer_size) {
                node_tmp = data_sort_buffer[algo].node->next;
                AKM_FREE(data_sort_buffer[algo].node);
                data_sort_buffer[algo].node = node_tmp;
                data_sort_buffer[algo].buffer_count--;
        }
        return AKM_SUCCESS;
}
void akm_data_sort_init(void)
{
    data_sort_buffer[0].node = NULL;
    data_sort_buffer[0].buffer_size = AKM_SORT_BUFFER_LEN;
    data_sort_buffer[0].buffer_count = 0;
    data_sort_buffer[1].node = NULL;
    data_sort_buffer[1].buffer_size = AKM_SORT_BUFFER_LEN;
    data_sort_buffer[1].buffer_count = 0;
}
void akm_set_buffer_len(int len, int algo)
{
    if ((len > 0) && (len < AKM_SORT_BUFFER_MAX_LEN))
        data_sort_buffer[algo].buffer_size = len;
}

int akm_data_sort(struct AKM_SENSOR_DATA * input)
{
        int ret = 0;
        DATA_BUFFER *node = (DATA_BUFFER *)AKM_MALLOC(sizeof(DATA_BUFFER));
        AKM_MEMCOPY(&node->data, input,sizeof(struct AKM_SENSOR_DATA));
        node->next = NULL;
        ret = akm_insert_data(node,CALIBRATE_BUFFER);
        if (ret != AKM_SUCCESS) {
                return AKM_ERROR;
        }
        if (data_sort_buffer[CALIBRATE_BUFFER].buffer_count< data_sort_buffer[CALIBRATE_BUFFER].buffer_size)
        {
            return AKM_SUCCESS;
        }
        ret = akm_get_data(input,CALIBRATE_BUFFER);
        if (ret != AKM_SUCCESS) {
                return AKM_ERROR;
        }
        return AKM_SUCCESS;
}
int akm_fusion_data_sort(struct AKM_SENSOR_DATA * input)
{
        int ret = 0;
        DATA_BUFFER *node = (DATA_BUFFER *)AKM_MALLOC(sizeof(DATA_BUFFER));
        AKM_MEMCOPY(&node->data, input,sizeof(struct AKM_SENSOR_DATA));
        node->next = NULL;
        ret = akm_insert_data(node,FUSION_BUFFER);
        if (ret != AKM_SUCCESS) {
                return AKM_ERROR;
        }
        if (data_sort_buffer[FUSION_BUFFER].buffer_count < data_sort_buffer[FUSION_BUFFER].buffer_size)
        {
            return AKM_SUCCESS;
        }
        ret = akm_get_data(input,FUSION_BUFFER);
        if (ret != AKM_SUCCESS) {
                return AKM_ERROR;
        }
        return AKM_SUCCESS;
}

