#ifndef ICAL_H_
#define ICAL_H_

// Internal Type Definitions
typedef unsigned char	UCHAR;
typedef char			INT8;
//typedef INT8			int8_t;
typedef unsigned char	UINT8;
//typedef UINT8			uint8_t;
typedef short			INT16;
//typedef INT16			int16_t;
typedef unsigned short	UINT16;
//typedef UINT16			uint16_t;
typedef long			INT32;
//typedef INT32			int32_t;
typedef unsigned long	UINT32;
//typedef UINT32			uint32_t;
typedef float			REAL;
//typedef long long		int64_t;
typedef int64_t			INT64;
//typedef unsigned long long		uint64_t;
typedef uint64_t		UINT64;

typedef struct
{
	int16_t	Offset[3];		// Magnetic Hard Iron x, y, z offsets
	int16_t rr;
} TRANSFORM_T;

//#define QST_ICAL_CORE_DEBUG

#ifdef QST_ICAL_CORE_DEBUG
#define QST_PRINT(...) do{printf(__VA_ARGS__);}while(0)
#else
#define QST_PRINT(...) do{}while(0)
#endif

void qst_ical_init(void);

int convert_magnetic(REAL *raw, REAL *result,int8_t *accuracy);

#endif /* ICAL_H_ */
