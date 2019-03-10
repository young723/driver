#ifndef RUN_ALGORITHM_H
#define RUN_ALGORITHM_H

extern "C" {

// Internal Type Definitions
typedef unsigned char	UCHAR;
typedef char			INT8;
typedef unsigned char	UINT8;
typedef UINT8			uint8_t;
typedef short			INT16;
typedef INT16			int16_t;
typedef unsigned short	UINT16;
typedef UINT16			uint16_t;
typedef long			INT32;
typedef unsigned long	UINT32;
typedef float			REAL;

	
typedef struct {
	float dRawMag[3];
	float mag_bias[3];
	float data_cali[3];
	float pitch,
	      roll,
		  yaw;
	float acc[3];
}_QMC7983;

int64_t get_time_in_nanosec(void);
void mcal(long timestamp_ms);
int process(REAL* magData,long timestamp_ms);
int get_mag_bias(REAL* mag_bias);
int8_t get_mag_accuracy(void);
int push2mcal(_QMC7983 *data);

}
#endif
