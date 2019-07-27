#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <poll.h>
#include <dlfcn.h> 
#include <fcntl.h>
#include <dirent.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <sys/ipc.h>
#include <sys/ioctl.h>
#include <string.h>
#include <signal.h>
#include <stdarg.h>
#include <time.h>
#include <math.h>
#include <linux/input.h>
#include <cutils/log.h>
#include <hardware/sensors.h>
#include <sensors_io.h>

/*--------------------------------------------------global values-----------------------------------------------*/
#ifdef LOG_TAG 
#undef LOG_TAG 
#define LOG_TAG "QMD6988D"
#endif

#define DAEMON_POLLING_INTV		20			// ms

#if 0
#define  D(...)  ALOGD(__VA_ARGS__) //open debug log
#else
#define  D(...)  ((void)0)		   //close debug log
#endif

static float Conv_A_S[10][2] = {
{-6.30E-03,4.30E-04},
{-1.90E-11,1.20E-10},
{1.00E-01,9.10E-02},
{1.20E-08,1.20E-06},
{3.30E-02,1.90E-02},
{2.10E-07,1.40E-07},
{-6.30E-10,3.50E-10},
{2.90E-13,7.60E-13},
{2.10E-15,1.20E-14},
{1.30E-16,7.90E-17},
};

static float a0,b00;
static float a1,a2,bt1,bt2,bp1,b11,bp2,b12,b21,bp3;

int b_fd =-1;
/*------------------------------------for log file-----------------*/

static int qmcd_init(void)
{
	pid_t pid;
	/* parent exits , child continues */
	if ((pid = fork()) < 0) {
		return -1;
	} else if (pid != 0) {
		exit(0);
	}
	setsid();
	/* become session leader */

	umask(0); /* clear file mode creation mask */
	return 0;
}
static void qmcd_abort(void)
{
	D("qmcd abort\n");
}

static void qmcd_sigterm(int signo) 
{
	qmcd_abort();
	D("signal: %d\n", signo); /* catched signal */
	D("qmcd stopped\n");
	exit(0);
}
/*--------------------------------------------------global values-----------------------------------------------*/

static int mediatek_read_raw_temperature(int fd, signed int *temperature) 
{
	signed int Dt;

	if (ioctl(fd, BAROMETER_GET_TEMP_RAW_DATA, (void*)&Dt) < 0)
		return -1;

	D("kernel temperature raw data %d \n",Dt);

	*temperature = Dt;
	
	return 0;
}

static int mediatek_read_raw_pressure(int fd, signed int *pressure) 
{
	signed int Dp;

	if (ioctl(fd, BAROMETER_GET_PRESS_RAW_DATA, (void*)&Dp) < 0) {
		return -1;
	}

	D("kernel pressure raw data %d \n",Dp);

	*pressure = Dp;
	
	return 0;
}

int main() 
{
	int sensor_stat = 0;
	int value = 0;

	unsigned int delay_timer = DAEMON_POLLING_INTV;

	signed int pt_val[2] ={0};

	signed int Dp;
	signed int Dt;
	
	struct qmp6988_calibration_data calibration_dat;
	
	struct timeval tv_begin;
	struct timeval tv_now;

	unsigned int usec_elapse = 0; /* usec */
	
	double Tr = 0,T = 0;
	double Pr = 0;

#if 0
	if (qmcd_init() == -1)
	{
		D("can't fork self\n");
		exit(0);
	}

	D("qmcd started. \n");

	signal(SIGTERM, qmcd_sigterm); /* arrange to catch the signal */
	signal(SIGHUP, qmcd_sigterm); /* arrange to catch the signal */
	signal(SIGINT, qmcd_sigterm); /* arrange to catch the signal */
	signal(SIGQUIT, qmcd_sigterm); /* arrange to catch the signal */
	signal(SIGTRAP, qmcd_sigterm); /* arrange to catch the signal */
	signal(SIGABRT, qmcd_sigterm); /* arrange to catch the signal */
	signal(SIGKILL, qmcd_sigterm); /* arrange to catch the signal */
	signal(SIGPIPE, qmcd_sigterm); /* arrange to catch the signal */
	signal(SIGSTOP, qmcd_sigterm); /* arrange to catch the signal */
	signal(SIGTSTP, qmcd_sigterm); /* arrange to catch the signal */
	signal(SIGURG, qmcd_sigterm); /* arrange to catch the signal */
#endif

	int b_fd = open("/dev/barometer", O_RDWR);
	if (b_fd == -1) {
		D("Cannot open barometer!\n");
		return -1;
	}

	if (ioctl(b_fd, BAROMETER_GET_CALIBRATION_DATA, &calibration_dat) < 0)
	{
		D("QMP_IOCTL_GET_OPEN_STATUS failed\n");
	}
	
	D("<-----------calibration data-------------->\n");
	D("COE_a0[%d]	COE_a1[%d]	COE_a2[%d]	COE_b00[%d]\n",
			calibration_dat.COE_a0,calibration_dat.COE_a1,calibration_dat.COE_a2,calibration_dat.COE_b00);
	D("COE_bt1[%d]	COE_bt2[%d]	COE_bp1[%d]	COE_b11[%d]\n",
			calibration_dat.COE_bt1,calibration_dat.COE_bt2,calibration_dat.COE_bp1,calibration_dat.COE_b11);
	D("COE_bp2[%d]	COE_b12[%d]	COE_b21[%d]	COE_bp3[%d]\n",
			calibration_dat.COE_bp2,calibration_dat.COE_b12,calibration_dat.COE_b21,calibration_dat.COE_bp3);
	D("<-----------calibration data-------------->\n");
	
	a0 = calibration_dat.COE_a0 /16.0f;
	b00 = calibration_dat.COE_b00 /16.0f;

	a1 = Conv_A_S[0][0] + Conv_A_S[0][1] * calibration_dat.COE_a1 / 32767.0f;
	a2 = Conv_A_S[1][0] + Conv_A_S[1][1] * calibration_dat.COE_a2 / 32767.0f;
	bt1 = Conv_A_S[2][0] + Conv_A_S[2][1] * calibration_dat.COE_bt1 / 32767.0f;
	bt2 = Conv_A_S[3][0] + Conv_A_S[3][1] * calibration_dat.COE_bt2 / 32767.0f;
	bp1 = Conv_A_S[4][0] + Conv_A_S[4][1] * calibration_dat.COE_bp1 / 32767.0f;
	b11 = Conv_A_S[5][0] + Conv_A_S[5][1] * calibration_dat.COE_b11 / 32767.0f;
	bp2 = Conv_A_S[6][0] + Conv_A_S[6][1] * calibration_dat.COE_bp2 / 32767.0f;
	b12 = Conv_A_S[7][0] + Conv_A_S[7][1] * calibration_dat.COE_b12 / 32767.0f;
	b21 = Conv_A_S[8][0] + Conv_A_S[8][1] * calibration_dat.COE_b21 / 32767.0f;
	bp3 = Conv_A_S[9][0] + Conv_A_S[9][1] * calibration_dat.COE_bp3 / 32767.0f;
	
	D("<----------- float calibration data -------------->\n");
	D("a0[%f]	a1[%f]	a2[%f]	b00[%f]\n",a0,a1,a2,b00);
	D("bt1[%f]	bt2[%f]	bp1[%f]	b11[%f]\n",bt1,bt2,bp1,b11);
	D("bp2[%f]	b12[%f]	b21[%f]	bp3[%f]\n",bp2,b12,b21,bp3);
	D("<----------- float calibration data -------------->\n");

	
	while (1)
	{
		gettimeofday(&tv_begin, NULL);

		if (ioctl(b_fd, QMP_IOCTL_GET_OPEN_STATUS, &sensor_stat) < 0)
		{
			D("QMP_IOCTL_GET_OPEN_STATUS failed\n");
		}

		if (ioctl(b_fd, QMP_IOCTL_GET_DELAY, &value) < 0)
		{
			delay_timer = DAEMON_POLLING_INTV;
		}
		else if (value < 0 || value > 20)
		{
			delay_timer = DAEMON_POLLING_INTV;
		}
		else
		{
			delay_timer = value;
		}
		
		if (sensor_stat != 0)
		{		
			mediatek_read_raw_temperature(b_fd, &Dt);
			
			mediatek_read_raw_pressure(b_fd, &Dp);

			D("raw data Temperature[%d] Pressure[%d]\n",Dt,Dp);
			
			//compensation temperature
			Tr = a0 + a1*Dt + a2*Dt*Dt;
			//Unit centigrade
			T = Tr / 256.0f;  
			
			D("Tr [%f] T[%f]\n",Tr,T);
			//compensation pressure, Unit Pa
			Pr = b00 + bt1*Tr + bp1*Dp + b11*Tr*Dp + bt2*Tr*Tr + bp2*Dp*Dp + b12*Dp*Tr*Tr + b21*Dp*Dp*Tr + bp3*Dp*Dp*Dp;
			D("Pr [%f]\n",Pr);
			
			pt_val[0] = (int)(Pr * 100);
			pt_val[1] = (int)(T * 100);
			
			if (ioctl(b_fd, BAROMETER_SET_PT_DATA, pt_val) < 0) 
			{
				D("BAROMETER_SET_PT_DATA failed!\n");
			}

			gettimeofday (&tv_now, NULL);
			usec_elapse = (tv_now.tv_sec - tv_begin.tv_sec) * 1000000 + tv_now.tv_usec - tv_begin.tv_usec;

			if(usec_elapse < delay_timer * 1000)
			{
				usleep(delay_timer * 1000 - usec_elapse);
			}

		}
	}
	close(b_fd);
	return 0;
}
