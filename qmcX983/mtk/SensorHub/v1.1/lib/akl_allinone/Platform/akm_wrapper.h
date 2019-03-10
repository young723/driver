//#include <vendor_cust.h>
void AKM_SetDevice(uint8_t *wia);
int AKM_SetGyroFlag(int32_t hasGyro);

int AKM_Open(void);
int AKM_Close(void);
void AKM_ReloadContext(float *offset);

int akm_get_sensor_support(void);
int AKM_SetMagData(int32_t data_x, int32_t data_y, int32_t data_z, int64_t time_stamp);
int AKM_SetGyroData(double data_x, double data_y, double data_z, int64_t time_stamp);
int AKM_GetMagData(double *cali_data, double *offset, int16_t *accuracy);

int AKM_FusionSetAccData(double data_x, double data_y, double data_z, int64_t time_stamp);
int AKM_FusionSetMagData(double data_x, double data_y, double data_z, int64_t time_stamp);
int AKM_FusionSetGyroData(double data_x, double data_y, double data_z, int64_t time_stamp);
int AKM_GetOri(double *cali_data, int16_t *accuracy);
int AKM_GetGyroscope(double *cali_data, int16_t *accuracy);
int AKM_GetRotationVector(double *cali_data, int16_t *accuracy);
int AKM_GetLinearAccelerometer(double *cali_data, int16_t *accuracy);
int AKM_GetGravity(double *cali_data, int16_t *accuracy);

