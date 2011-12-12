#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 151 "/usr/lib/gcc-lib/msp430/3.2.3/include/stddef.h" 3
typedef int ptrdiff_t;
#line 213
typedef unsigned int size_t;
#line 325
typedef int wchar_t;
# 8 "/usr/lib/ncc/deputy_nodeputy.h"
struct __nesc_attr_nonnull {
}  ;
#line 9
struct __nesc_attr_bnd {
#line 9
  void *lo, *hi;
}  ;
#line 10
struct __nesc_attr_bnd_nok {
#line 10
  void *lo, *hi;
}  ;
#line 11
struct __nesc_attr_count {
#line 11
  int n;
}  ;
#line 12
struct __nesc_attr_count_nok {
#line 12
  int n;
}  ;
#line 13
struct __nesc_attr_one {
}  ;
#line 14
struct __nesc_attr_one_nok {
}  ;
#line 15
struct __nesc_attr_dmemset {
#line 15
  int a1, a2, a3;
}  ;
#line 16
struct __nesc_attr_dmemcpy {
#line 16
  int a1, a2, a3;
}  ;
#line 17
struct __nesc_attr_nts {
}  ;
# 38 "/usr/msp430/include/sys/inttypes.h" 3
typedef signed char int8_t;
typedef unsigned char uint8_t;

typedef int int16_t;
typedef unsigned int uint16_t;

typedef long int32_t;
typedef unsigned long uint32_t;

typedef long long int64_t;
typedef unsigned long long uint64_t;




typedef int16_t intptr_t;
typedef uint16_t uintptr_t;
# 235 "/usr/lib/ncc/nesc_nx.h"
static __inline uint8_t __nesc_ntoh_uint8(const void * source)  ;




static __inline uint8_t __nesc_hton_uint8(void * target, uint8_t value)  ;





static __inline uint8_t __nesc_ntoh_leuint8(const void * source)  ;




static __inline uint8_t __nesc_hton_leuint8(void * target, uint8_t value)  ;
#line 264
static __inline uint16_t __nesc_ntoh_uint16(const void * source)  ;




static __inline uint16_t __nesc_hton_uint16(void * target, uint16_t value)  ;






static __inline uint16_t __nesc_ntoh_leuint16(const void * source)  ;




static __inline uint16_t __nesc_hton_leuint16(void * target, uint16_t value)  ;
#line 385
typedef struct { unsigned char data[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char data[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 41 "/usr/msp430/include/sys/types.h" 3
typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;
typedef unsigned short ushort;
typedef unsigned int uint;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
typedef uint64_t u_int64_t;

typedef u_int64_t u_quad_t;
typedef int64_t quad_t;
typedef quad_t *qaddr_t;

typedef char *caddr_t;
typedef const char *c_caddr_t;
typedef volatile char *v_caddr_t;
typedef u_int32_t fixpt_t;
typedef u_int32_t gid_t;
typedef u_int32_t in_addr_t;
typedef u_int16_t in_port_t;
typedef u_int32_t ino_t;
typedef long key_t;
typedef u_int16_t mode_t;
typedef u_int16_t nlink_t;
typedef quad_t rlim_t;
typedef int32_t segsz_t;
typedef int32_t swblk_t;
typedef int32_t ufs_daddr_t;
typedef int32_t ufs_time_t;
typedef u_int32_t uid_t;
# 39 "/usr/msp430/include/string.h" 3
extern int memcmp(const void *arg_0x4029fa18, const void *arg_0x4029fbb0, size_t arg_0x4029fd48);
extern void *memcpy(void *arg_0x4029e200, const void *arg_0x4029e398, size_t arg_0x4029e530);

extern void *memset(void *arg_0x402a1220, int arg_0x402a1378, size_t arg_0x402a1510);
#line 63
extern void *memset(void *arg_0x402ab118, int arg_0x402ab270, size_t arg_0x402ab408);
# 59 "/usr/msp430/include/stdlib.h" 3
#line 56
typedef struct __nesc_unnamed4242 {
  int quot;
  int rem;
} div_t;







#line 64
typedef struct __nesc_unnamed4243 {
  long quot;
  long rem;
} ldiv_t;
# 122 "/usr/msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/usr/msp430/include/sys/_types.h" 3
typedef long _off_t;
typedef long _ssize_t;
# 28 "/usr/msp430/include/sys/reent.h" 3
typedef __uint32_t __ULong;


struct _glue {

  struct _glue *_next;
  int _niobs;
  struct __sFILE *_iobs;
};

struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _atexit {
  struct _atexit *_next;
  int _ind;
  void (*_fns[32])(void );
};








struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;
#line 116
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;


  void *_cookie;

  int (*_read)(void *_cookie, char *_buf, int _n);
  int (*_write)(void *_cookie, const char *_buf, int _n);

  _fpos_t (*_seek)(void *_cookie, _fpos_t _offset, int _whence);
  int (*_close)(void *_cookie);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;

  struct _reent *_data;
};
#line 174
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};









struct _reent {


  int _errno;




  struct __sFILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  const char *_current_locale;

  int __sdidinit;

  void (*__cleanup)(struct _reent *arg_0x402ce510);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4244 {

    struct __nesc_unnamed4245 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
    } _reent;



    struct __nesc_unnamed4246 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int arg_0x402ccb88);




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/usr/msp430/include/math.h" 3
union __dmath {

  __uint32_t i[2];
  double d;
};




union __dmath;
#line 208
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 261
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 23 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4247 {
#line 24
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;
uint16_t TOS_NODE_ID = 1;






struct __nesc_attr_atmostonce {
};
#line 35
struct __nesc_attr_atleastonce {
};
#line 36
struct __nesc_attr_exactlyonce {
};
# 40 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/types/TinyError.h"
enum __nesc_unnamed4248 {
  SUCCESS = 0, 
  FAIL = 1, 
  ESIZE = 2, 
  ECANCEL = 3, 
  EOFF = 4, 
  EBUSY = 5, 
  EINVAL = 6, 
  ERETRY = 7, 
  ERESERVE = 8, 
  EALREADY = 9, 
  ENOMEM = 10, 
  ENOACK = 11, 
  ELAST = 11
};

typedef uint8_t error_t  ;

static inline error_t ecombine(error_t r1, error_t r2)  ;
# 39 "/usr/msp430/include/msp430/iostructures.h" 3
#line 27
typedef union port {
  volatile unsigned char reg_p;
  volatile struct __nesc_unnamed4249 {
    unsigned char __p0 : 1, 
    __p1 : 1, 
    __p2 : 1, 
    __p3 : 1, 
    __p4 : 1, 
    __p5 : 1, 
    __p6 : 1, 
    __p7 : 1;
  } __pin;
} __attribute((packed))  ioregister_t;
#line 108
struct port_full_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t ifg;
  ioregister_t ies;
  ioregister_t ie;
  ioregister_t sel;
};









struct port_simple_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t sel;
};




struct port_full_t;



struct port_full_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;
# 116 "/usr/msp430/include/msp430/gpio.h" 3
volatile unsigned char P1OUT __asm ("0x0021");

volatile unsigned char P1DIR __asm ("0x0022");

volatile unsigned char P1IFG __asm ("0x0023");

volatile unsigned char P1IES __asm ("0x0024");

volatile unsigned char P1IE __asm ("0x0025");

volatile unsigned char P1SEL __asm ("0x0026");










volatile unsigned char P2OUT __asm ("0x0029");

volatile unsigned char P2DIR __asm ("0x002A");

volatile unsigned char P2IFG __asm ("0x002B");

volatile unsigned char P2IES __asm ("0x002C");

volatile unsigned char P2IE __asm ("0x002D");

volatile unsigned char P2SEL __asm ("0x002E");










volatile unsigned char P3OUT __asm ("0x0019");

volatile unsigned char P3DIR __asm ("0x001A");

volatile unsigned char P3SEL __asm ("0x001B");










volatile unsigned char P4OUT __asm ("0x001D");

volatile unsigned char P4DIR __asm ("0x001E");

volatile unsigned char P4SEL __asm ("0x001F");










volatile unsigned char P5OUT __asm ("0x0031");

volatile unsigned char P5DIR __asm ("0x0032");

volatile unsigned char P5SEL __asm ("0x0033");










volatile unsigned char P6OUT __asm ("0x0035");

volatile unsigned char P6DIR __asm ("0x0036");

volatile unsigned char P6SEL __asm ("0x0037");
# 92 "/usr/msp430/include/msp430/usart.h" 3
volatile unsigned char U0CTL __asm ("0x0070");

volatile unsigned char U0TCTL __asm ("0x0071");



volatile unsigned char U0MCTL __asm ("0x0073");

volatile unsigned char U0BR0 __asm ("0x0074");

volatile unsigned char U0BR1 __asm ("0x0075");

volatile unsigned char U0RXBUF __asm ("0x0076");
#line 275
volatile unsigned char U1CTL __asm ("0x0078");

volatile unsigned char U1TCTL __asm ("0x0079");



volatile unsigned char U1MCTL __asm ("0x007B");

volatile unsigned char U1BR0 __asm ("0x007C");

volatile unsigned char U1BR1 __asm ("0x007D");

volatile unsigned char U1RXBUF __asm ("0x007E");
# 27 "/usr/msp430/include/msp430/timera.h" 3
volatile unsigned int TA0CTL __asm ("0x0160");

volatile unsigned int TA0R __asm ("0x0170");


volatile unsigned int TA0CCTL0 __asm ("0x0162");

volatile unsigned int TA0CCTL1 __asm ("0x0164");
#line 70
volatile unsigned int TA0CCTL2 __asm ("0x0166");
#line 127
#line 118
typedef struct __nesc_unnamed4250 {
  volatile unsigned 
  taifg : 1, 
  taie : 1, 
  taclr : 1, 
  dummy : 1, 
  tamc : 2, 
  taid : 2, 
  tassel : 2;
} __attribute((packed))  tactl_t;
#line 143
#line 129
typedef struct __nesc_unnamed4251 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  dummy : 1, 
  scci : 1, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tacctl_t;


struct timera_t {
  tactl_t ctl;
  tacctl_t cctl0;
  tacctl_t cctl1;
  tacctl_t cctl2;
  volatile unsigned dummy[4];
  volatile unsigned tar;
  volatile unsigned taccr0;
  volatile unsigned taccr1;
  volatile unsigned taccr2;
};



struct timera_t;
# 26 "/usr/msp430/include/msp430/timerb.h" 3
volatile unsigned int TBR __asm ("0x0190");


volatile unsigned int TBCCTL0 __asm ("0x0182");





volatile unsigned int TBCCR0 __asm ("0x0192");
#line 76
#line 64
typedef struct __nesc_unnamed4252 {
  volatile unsigned 
  tbifg : 1, 
  tbie : 1, 
  tbclr : 1, 
  dummy1 : 1, 
  tbmc : 2, 
  tbid : 2, 
  tbssel : 2, 
  dummy2 : 1, 
  tbcntl : 2, 
  tbclgrp : 2;
} __attribute((packed))  tbctl_t;
#line 91
#line 78
typedef struct __nesc_unnamed4253 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  clld : 2, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tbcctl_t;


struct timerb_t {
  tbctl_t ctl;
  tbcctl_t cctl0;
  tbcctl_t cctl1;
  tbcctl_t cctl2;

  tbcctl_t cctl3;
  tbcctl_t cctl4;
  tbcctl_t cctl5;
  tbcctl_t cctl6;



  volatile unsigned tbr;
  volatile unsigned tbccr0;
  volatile unsigned tbccr1;
  volatile unsigned tbccr2;

  volatile unsigned tbccr3;
  volatile unsigned tbccr4;
  volatile unsigned tbccr5;
  volatile unsigned tbccr6;
};





struct timerb_t;
# 20 "/usr/msp430/include/msp430/basic_clock.h" 3
volatile unsigned char DCOCTL __asm ("0x0056");

volatile unsigned char BCSCTL1 __asm ("0x0057");

volatile unsigned char BCSCTL2 __asm ("0x0058");
# 18 "/usr/msp430/include/msp430/adc12.h" 3
volatile unsigned int ADC12CTL0 __asm ("0x01A0");

volatile unsigned int ADC12CTL1 __asm ("0x01A2");
#line 42
#line 30
typedef struct __nesc_unnamed4254 {
  volatile unsigned 
  adc12sc : 1, 
  enc : 1, 
  adc12tovie : 1, 
  adc12ovie : 1, 
  adc12on : 1, 
  refon : 1, 
  r2_5v : 1, 
  msc : 1, 
  sht0 : 4, 
  sht1 : 4;
} __attribute((packed))  adc12ctl0_t;
#line 54
#line 44
typedef struct __nesc_unnamed4255 {
  volatile unsigned 
  adc12busy : 1, 
  conseq : 2, 
  adc12ssel : 2, 
  adc12div : 3, 
  issh : 1, 
  shp : 1, 
  shs : 2, 
  cstartadd : 4;
} __attribute((packed))  adc12ctl1_t;
#line 74
#line 56
typedef struct __nesc_unnamed4256 {
  volatile unsigned 
  bit0 : 1, 
  bit1 : 1, 
  bit2 : 1, 
  bit3 : 1, 
  bit4 : 1, 
  bit5 : 1, 
  bit6 : 1, 
  bit7 : 1, 
  bit8 : 1, 
  bit9 : 1, 
  bit10 : 1, 
  bit11 : 1, 
  bit12 : 1, 
  bit13 : 1, 
  bit14 : 1, 
  bit15 : 1;
} __attribute((packed))  adc12xflg_t;


struct adc12_t {
  adc12ctl0_t ctl0;
  adc12ctl1_t ctl1;
  adc12xflg_t ifg;
  adc12xflg_t ie;
  adc12xflg_t iv;
};




struct adc12_t;
# 71 "/usr/msp430/include/msp430/common.h" 3
volatile unsigned int WDTCTL __asm ("0x0120");
# 83 "/usr/msp430/include/msp430x16x.h" 3
volatile unsigned char ME1 __asm ("0x0004");





volatile unsigned char ME2 __asm ("0x0005");
# 158 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/msp430hardware.h"
static volatile uint8_t U0CTLnr __asm ("0x0070");
static volatile uint8_t I2CTCTLnr __asm ("0x0071");
static volatile uint8_t I2CDCTLnr __asm ("0x0072");
#line 193
typedef uint8_t mcu_power_t  ;
static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)  ;


enum __nesc_unnamed4257 {
  MSP430_POWER_ACTIVE = 0, 
  MSP430_POWER_LPM0 = 1, 
  MSP430_POWER_LPM1 = 2, 
  MSP430_POWER_LPM2 = 3, 
  MSP430_POWER_LPM3 = 4, 
  MSP430_POWER_LPM4 = 5
};

static inline void __nesc_disable_interrupt(void )  ;





static inline void __nesc_enable_interrupt(void )  ;




typedef bool __nesc_atomic_t;
__nesc_atomic_t __nesc_atomic_start(void );
void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);






__nesc_atomic_t __nesc_atomic_start(void )   ;







void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)   ;
#line 248
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_float;typedef float __nesc_nxbase_nx_float  ;
# 8 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosb/hardware.h"
enum __nesc_unnamed4258 {
  TOS_SLEEP_NONE = MSP430_POWER_ACTIVE
};
#line 36
static inline void TOSH_SET_SIMO0_PIN()  ;
#line 36
static inline void TOSH_CLR_SIMO0_PIN()  ;
#line 36
static inline void TOSH_MAKE_SIMO0_OUTPUT()  ;
static inline void TOSH_SET_UCLK0_PIN()  ;
#line 37
static inline void TOSH_CLR_UCLK0_PIN()  ;
#line 37
static inline void TOSH_MAKE_UCLK0_OUTPUT()  ;
#line 79
enum __nesc_unnamed4259 {

  TOSH_HUMIDITY_ADDR = 5, 
  TOSH_HUMIDTEMP_ADDR = 3, 
  TOSH_HUMIDITY_RESET = 0x1E
};



static inline void TOSH_SET_FLASH_CS_PIN()  ;
#line 88
static inline void TOSH_CLR_FLASH_CS_PIN()  ;
#line 88
static inline void TOSH_MAKE_FLASH_CS_OUTPUT()  ;
static inline void TOSH_SET_FLASH_HOLD_PIN()  ;
#line 89
static inline void TOSH_MAKE_FLASH_HOLD_OUTPUT()  ;
# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.h"
enum __nesc_unnamed4260 {
  MSP430TIMER_CM_NONE = 0, 
  MSP430TIMER_CM_RISING = 1, 
  MSP430TIMER_CM_FALLING = 2, 
  MSP430TIMER_CM_BOTH = 3, 

  MSP430TIMER_STOP_MODE = 0, 
  MSP430TIMER_UP_MODE = 1, 
  MSP430TIMER_CONTINUOUS_MODE = 2, 
  MSP430TIMER_UPDOWN_MODE = 3, 

  MSP430TIMER_TACLK = 0, 
  MSP430TIMER_TBCLK = 0, 
  MSP430TIMER_ACLK = 1, 
  MSP430TIMER_SMCLK = 2, 
  MSP430TIMER_INCLK = 3, 

  MSP430TIMER_CLOCKDIV_1 = 0, 
  MSP430TIMER_CLOCKDIV_2 = 1, 
  MSP430TIMER_CLOCKDIV_4 = 2, 
  MSP430TIMER_CLOCKDIV_8 = 3
};
#line 64
#line 51
typedef struct __nesc_unnamed4261 {

  int ccifg : 1;
  int cov : 1;
  int out : 1;
  int cci : 1;
  int ccie : 1;
  int outmod : 3;
  int cap : 1;
  int clld : 2;
  int scs : 1;
  int ccis : 2;
  int cm : 2;
} msp430_compare_control_t;
#line 76
#line 66
typedef struct __nesc_unnamed4262 {

  int taifg : 1;
  int taie : 1;
  int taclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tassel : 2;
  int _unused1 : 6;
} msp430_timer_a_control_t;
#line 91
#line 78
typedef struct __nesc_unnamed4263 {

  int tbifg : 1;
  int tbie : 1;
  int tbclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tbssel : 2;
  int _unused1 : 1;
  int cntl : 2;
  int tbclgrp : 2;
  int _unused2 : 1;
} msp430_timer_b_control_t;
# 43 "/usr/lib/gcc-lib/msp430/3.2.3/include/stdarg.h" 3
typedef __builtin_va_list __gnuc_va_list;
#line 110
typedef __gnuc_va_list va_list;
# 52 "/usr/msp430/include/stdio.h" 3
int __attribute((format(printf, 1, 2))) printf(const char *string, ...);






int putchar(int c);
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/CC2420.h"
typedef uint8_t cc2420_status_t;
#line 16
#line 15
typedef nx_struct cc2420_header_t {
} __attribute__((packed)) cc2420_header_t;


#line 18
typedef nx_struct cc2420_footer_t {
} __attribute__((packed)) cc2420_footer_t;


#line 21
typedef nx_struct cc2420_metadata_t {
} __attribute__((packed)) cc2420_metadata_t;

enum cc2420_enums {
  CC2420_TIME_ACK_TURNAROUND = 7, 
  CC2420_TIME_VREN = 20, 
  CC2420_TIME_SYMBOL = 2, 
  CC2420_BACKOFF_PERIOD = 20 / CC2420_TIME_SYMBOL, 
  CC2420_MIN_BACKOFF = 20 / CC2420_TIME_SYMBOL, 
  CC2420_ACK_WAIT_DELAY = 256
};

enum cc2420_status_enums {
  CC2420_STATUS_RSSI_VALID = 1 << 1, 
  CC2420_STATUS_LOCK = 1 << 2, 
  CC2420_STATUS_TX_ACTIVE = 1 << 3, 
  CC2420_STATUS_ENC_BUSY = 1 << 4, 
  CC2420_STATUS_TX_UNDERFLOW = 1 << 5, 
  CC2420_STATUS_XOSC16M_STABLE = 1 << 6
};

enum cc2420_config_reg_enums {
  CC2420_SNOP = 0x00, 
  CC2420_SXOSCON = 0x01, 
  CC2420_STXCAL = 0x02, 
  CC2420_SRXON = 0x03, 
  CC2420_STXON = 0x04, 
  CC2420_STXONCCA = 0x05, 
  CC2420_SRFOFF = 0x06, 
  CC2420_SXOSCOFF = 0x07, 
  CC2420_SFLUSHRX = 0x08, 
  CC2420_SFLUSHTX = 0x09, 
  CC2420_SACK = 0x0a, 
  CC2420_SACKPEND = 0x0b, 
  CC2420_SRXDEC = 0x0c, 
  CC2420_STXENC = 0x0d, 
  CC2420_SAES = 0x0e, 
  CC2420_MAIN = 0x10, 
  CC2420_MDMCTRL0 = 0x11, 
  CC2420_MDMCTRL1 = 0x12, 
  CC2420_RSSI = 0x13, 
  CC2420_SYNCWORD = 0x14, 
  CC2420_TXCTRL = 0x15, 
  CC2420_RXCTRL0 = 0x16, 
  CC2420_RXCTRL1 = 0x17, 
  CC2420_FSCTRL = 0x18, 
  CC2420_SECCTRL0 = 0x19, 
  CC2420_SECCTRL1 = 0x1a, 
  CC2420_BATTMON = 0x1b, 
  CC2420_IOCFG0 = 0x1c, 
  CC2420_IOCFG1 = 0x1d, 
  CC2420_MANFIDL = 0x1e, 
  CC2420_MANFIDH = 0x1f, 
  CC2420_FSMTC = 0x20, 
  CC2420_MANAND = 0x21, 
  CC2420_MANOR = 0x22, 
  CC2420_AGCCTRL = 0x23, 
  CC2420_AGCTST0 = 0x24, 
  CC2420_AGCTST1 = 0x25, 
  CC2420_AGCTST2 = 0x26, 
  CC2420_FSTST0 = 0x27, 
  CC2420_FSTST1 = 0x28, 
  CC2420_FSTST2 = 0x29, 
  CC2420_FSTST3 = 0x2a, 
  CC2420_RXBPFTST = 0x2b, 
  CC2420_FMSTATE = 0x2c, 
  CC2420_ADCTST = 0x2d, 
  CC2420_DACTST = 0x2e, 
  CC2420_TOPTST = 0x2f, 
  CC2420_TXFIFO = 0x3e, 
  CC2420_RXFIFO = 0x3f
};

enum cc2420_ram_addr_enums {
  CC2420_RAM_TXFIFO = 0x000, 
  CC2420_RAM_RXFIFO = 0x080, 
  CC2420_RAM_KEY0 = 0x100, 
  CC2420_RAM_RXNONCE = 0x110, 
  CC2420_RAM_SABUF = 0x120, 
  CC2420_RAM_KEY1 = 0x130, 
  CC2420_RAM_TXNONCE = 0x140, 
  CC2420_RAM_CBCSTATE = 0x150, 
  CC2420_RAM_IEEEADR = 0x160, 
  CC2420_RAM_PANID = 0x168, 
  CC2420_RAM_SHORTADR = 0x16a
};

enum cc2420_nonce_enums {
  CC2420_NONCE_BLOCK_COUNTER = 0, 
  CC2420_NONCE_KEY_SEQ_COUNTER = 2, 
  CC2420_NONCE_FRAME_COUNTER = 3, 
  CC2420_NONCE_SOURCE_ADDRESS = 7, 
  CC2420_NONCE_FLAGS = 15
};

enum cc2420_main_enums {
  CC2420_MAIN_RESETn = 15, 
  CC2420_MAIN_ENC_RESETn = 14, 
  CC2420_MAIN_DEMOD_RESETn = 13, 
  CC2420_MAIN_MOD_RESETn = 12, 
  CC2420_MAIN_FS_RESETn = 11, 
  CC2420_MAIN_XOSC16M_BYPASS = 0
};

enum cc2420_mdmctrl0_enums {
  CC2420_MDMCTRL0_RESERVED_FRAME_MODE = 13, 
  CC2420_MDMCTRL0_PAN_COORDINATOR = 12, 
  CC2420_MDMCTRL0_ADR_DECODE = 11, 
  CC2420_MDMCTRL0_CCA_HYST = 8, 
  CC2420_MDMCTRL0_CCA_MOD = 6, 
  CC2420_MDMCTRL0_AUTOCRC = 5, 
  CC2420_MDMCTRL0_AUTOACK = 4, 
  CC2420_MDMCTRL0_PREAMBLE_LENGTH = 0
};

enum cc2420_mdmctrl1_enums {
  CC2420_MDMCTRL1_CORR_THR = 6, 
  CC2420_MDMCTRL1_DEMOD_AVG_MODE = 5, 
  CC2420_MDMCTRL1_MODULATION_MODE = 4, 
  CC2420_MDMCTRL1_TX_MODE = 2, 
  CC2420_MDMCTRL1_RX_MODE = 0
};

enum cc2420_rssi_enums {
  CC2420_RSSI_CCA_THR = 8, 
  CC2420_RSSI_RSSI_VAL = 0
};

enum cc2420_syncword_enums {
  CC2420_SYNCWORD_SYNCWORD = 0
};

enum cc2420_txctrl_enums {
  CC2420_TXCTRL_TXMIXBUF_CUR = 14, 
  CC2420_TXCTRL_TX_TURNAROUND = 13, 
  CC2420_TXCTRL_TXMIX_CAP_ARRAY = 11, 
  CC2420_TXCTRL_TXMIX_CURRENT = 9, 
  CC2420_TXCTRL_PA_CURRENT = 6, 
  CC2420_TXCTRL_RESERVED = 5, 
  CC2420_TXCTRL_PA_LEVEL = 0
};

enum cc2420_rxctrl0_enums {
  CC2420_RXCTRL0_RXMIXBUF_CUR = 12, 
  CC2420_RXCTRL0_HIGH_LNA_GAIN = 10, 
  CC2420_RXCTRL0_MED_LNA_GAIN = 8, 
  CC2420_RXCTRL0_LOW_LNA_GAIN = 6, 
  CC2420_RXCTRL0_HIGH_LNA_CURRENT = 4, 
  CC2420_RXCTRL0_MED_LNA_CURRENT = 2, 
  CC2420_RXCTRL0_LOW_LNA_CURRENT = 0
};

enum cc2420_rxctrl1_enums {
  CC2420_RXCTRL1_RXBPF_LOCUR = 13, 
  CC2420_RXCTRL1_RXBPF_MIDCUR = 12, 
  CC2420_RXCTRL1_LOW_LOWGAIN = 11, 
  CC2420_RXCTRL1_MED_LOWGAIN = 10, 
  CC2420_RXCTRL1_HIGH_HGM = 9, 
  CC2420_RXCTRL1_MED_HGM = 8, 
  CC2420_RXCTRL1_LNA_CAP_ARRAY = 6, 
  CC2420_RXCTRL1_RXMIX_TAIL = 4, 
  CC2420_RXCTRL1_RXMIX_VCM = 2, 
  CC2420_RXCTRL1_RXMIX_CURRENT = 0
};

enum cc2420_rsctrl_enums {
  CC2420_FSCTRL_LOCK_THR = 14, 
  CC2420_FSCTRL_CAL_DONE = 13, 
  CC2420_FSCTRL_CAL_RUNNING = 12, 
  CC2420_FSCTRL_LOCK_LENGTH = 11, 
  CC2420_FSCTRL_LOCK_STATUS = 10, 
  CC2420_FSCTRL_FREQ = 0
};

enum cc2420_secctrl0_enums {
  CC2420_SECCTRL0_RXFIFO_PROTECTION = 9, 
  CC2420_SECCTRL0_SEC_CBC_HEAD = 8, 
  CC2420_SECCTRL0_SEC_SAKEYSEL = 7, 
  CC2420_SECCTRL0_SEC_TXKEYSEL = 6, 
  CC2420_SECCTRL0_SEC_RXKEYSEL = 5, 
  CC2420_SECCTRL0_SEC_M = 2, 
  CC2420_SECCTRL0_SEC_MODE = 0
};

enum cc2420_secctrl1_enums {
  CC2420_SECCTRL1_SEC_TXL = 8, 
  CC2420_SECCTRL1_SEC_RXL = 0
};

enum cc2420_battmon_enums {
  CC2420_BATTMON_BATT_OK = 6, 
  CC2420_BATTMON_BATTMON_EN = 5, 
  CC2420_BATTMON_BATTMON_VOLTAGE = 0
};

enum cc2420_iocfg0_enums {
  CC2420_IOCFG0_BCN_ACCEPT = 11, 
  CC2420_IOCFG0_FIFO_POLARITY = 10, 
  CC2420_IOCFG0_FIFOP_POLARITY = 9, 
  CC2420_IOCFG0_SFD_POLARITY = 8, 
  CC2420_IOCFG0_CCA_POLARITY = 7, 
  CC2420_IOCFG0_FIFOP_THR = 0
};

enum cc2420_iocfg1_enums {
  CC2420_IOCFG1_HSSD_SRC = 10, 
  CC2420_IOCFG1_SFDMUX = 5, 
  CC2420_IOCFG1_CCAMUX = 0
};

enum cc2420_manfidl_enums {
  CC2420_MANFIDL_PARTNUM = 12, 
  CC2420_MANFIDL_MANFID = 0
};

enum cc2420_manfidh_enums {
  CC2420_MANFIDH_VERSION = 12, 
  CC2420_MANFIDH_PARTNUM = 0
};

enum cc2420_fsmtc_enums {
  CC2420_FSMTC_TC_RXCHAIN2RX = 13, 
  CC2420_FSMTC_TC_SWITCH2TX = 10, 
  CC2420_FSMTC_TC_PAON2TX = 6, 
  CC2420_FSMTC_TC_TXEND2SWITCH = 3, 
  CC2420_FSMTC_TC_TXEND2PAOFF = 0
};

enum cc2420_sfdmux_enums {
  CC2420_SFDMUX_SFD = 0, 
  CC2420_SFDMUX_XOSC16M_STABLE = 24
};
# 6 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/types/AM.h"
typedef nx_uint8_t nx_am_id_t;
typedef nx_uint8_t nx_am_group_t;
typedef nx_uint16_t nx_am_addr_t;

typedef uint8_t am_id_t;
typedef uint8_t am_group_t;
typedef uint16_t am_addr_t;

enum __nesc_unnamed4264 {
  AM_BROADCAST_ADDR = 0xffff
};









enum __nesc_unnamed4265 {
  TOS_AM_GROUP = 0x22, 
  TOS_AM_ADDRESS = 1
};
# 72 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/Serial.h"
typedef uint8_t uart_id_t;



enum __nesc_unnamed4266 {
  HDLC_FLAG_BYTE = 0x7e, 
  HDLC_CTLESC_BYTE = 0x7d
};



enum __nesc_unnamed4267 {
  TOS_SERIAL_ACTIVE_MESSAGE_ID = 0, 
  TOS_SERIAL_CC1000_ID = 1, 
  TOS_SERIAL_802_15_4_ID = 2, 
  TOS_SERIAL_UNKNOWN_ID = 255
};


enum __nesc_unnamed4268 {
  SERIAL_PROTO_ACK = 67, 
  SERIAL_PROTO_PACKET_ACK = 68, 
  SERIAL_PROTO_PACKET_NOACK = 69, 
  SERIAL_PROTO_PACKET_UNKNOWN = 255
};
#line 110
#line 98
typedef struct radio_stats {
  uint8_t version;
  uint8_t flags;
  uint8_t reserved;
  uint8_t platform;
  uint16_t MTU;
  uint16_t radio_crc_fail;
  uint16_t radio_queue_drops;
  uint16_t serial_crc_fail;
  uint16_t serial_tx_fail;
  uint16_t serial_short_packets;
  uint16_t serial_proto_drops;
} radio_stats_t;







#line 112
typedef nx_struct serial_header {
  nx_am_addr_t dest;
  nx_am_addr_t src;
  nx_uint8_t length;
  nx_am_group_t group;
  nx_am_id_t type;
} __attribute__((packed)) serial_header_t;




#line 120
typedef nx_struct serial_packet {
  serial_header_t header;
  nx_uint8_t data[];
} __attribute__((packed)) serial_packet_t;



#line 125
typedef nx_struct serial_metadata {
  nx_uint8_t ack;
} __attribute__((packed)) serial_metadata_t;
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosa/platform_message.h"
#line 45
typedef union message_header {
  cc2420_header_t cc2420;
  serial_header_t serial;
} message_header_t;



#line 50
typedef union TOSRadioFooter {
  cc2420_footer_t cc2420;
} message_footer_t;




#line 54
typedef union TOSRadioMetadata {
  cc2420_metadata_t cc2420;
  serial_metadata_t serial;
} message_metadata_t;
# 19 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/types/message.h"
#line 14
typedef nx_struct message_t {
  nx_uint8_t header[sizeof(message_header_t )];
  nx_uint8_t data[28];
  nx_uint8_t footer[sizeof(message_footer_t )];
  nx_uint8_t metadata[sizeof(message_metadata_t )];
} __attribute__((packed)) message_t;
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/printf/printf.h"
int printfflush();






#line 64
typedef nx_struct printf_msg {
  nx_uint8_t buffer[28];
} __attribute__((packed)) printf_msg_t;

enum __nesc_unnamed4269 {
  AM_PRINTF_MSG = 100
};
# 32 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/types/Leds.h"
enum __nesc_unnamed4270 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 80 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/crc.h"
static uint16_t crcByte(uint16_t crc, uint8_t b);
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/msp430usart.h"
#line 48
typedef enum __nesc_unnamed4271 {

  USART_NONE = 0, 
  USART_UART = 1, 
  USART_UART_TX = 2, 
  USART_UART_RX = 3, 
  USART_SPI = 4, 
  USART_I2C = 5
} msp430_usartmode_t;










#line 58
typedef struct __nesc_unnamed4272 {
  unsigned int swrst : 1;
  unsigned int mm : 1;
  unsigned int sync : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
} __attribute((packed))  msp430_uctl_t;









#line 69
typedef struct __nesc_unnamed4273 {
  unsigned int txept : 1;
  unsigned int stc : 1;
  unsigned int txwake : 1;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
} __attribute((packed))  msp430_utctl_t;










#line 79
typedef struct __nesc_unnamed4274 {
  unsigned int rxerr : 1;
  unsigned int rxwake : 1;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int brk : 1;
  unsigned int oe : 1;
  unsigned int pe : 1;
  unsigned int fe : 1;
} __attribute((packed))  msp430_urctl_t;
#line 116
#line 99
typedef struct __nesc_unnamed4275 {
  unsigned int ubr : 16;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int  : 3;

  unsigned int  : 1;
  unsigned int stc : 1;
  unsigned int  : 2;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
  unsigned int  : 0;
} msp430_spi_config_t;





#line 118
typedef struct __nesc_unnamed4276 {
  uint16_t ubr;
  uint8_t uctl;
  uint8_t utctl;
} msp430_spi_registers_t;




#line 124
typedef union __nesc_unnamed4277 {
  msp430_spi_config_t spiConfig;
  msp430_spi_registers_t spiRegisters;
} msp430_spi_union_config_t;

msp430_spi_union_config_t msp430_spi_default_config = { 
{ 
.ubr = 0x0002, 
.ssel = 0x02, 
.clen = 1, 
.listen = 0, 
.mm = 1, 
.ckph = 1, 
.ckpl = 0, 
.stc = 1 } };
#line 169
#line 150
typedef enum __nesc_unnamed4278 {

  UBR_32KHZ_1200 = 0x001B, UMCTL_32KHZ_1200 = 0x94, 
  UBR_32KHZ_1800 = 0x0012, UMCTL_32KHZ_1800 = 0x84, 
  UBR_32KHZ_2400 = 0x000D, UMCTL_32KHZ_2400 = 0x6D, 
  UBR_32KHZ_4800 = 0x0006, UMCTL_32KHZ_4800 = 0x77, 
  UBR_32KHZ_9600 = 0x0003, UMCTL_32KHZ_9600 = 0x29, 

  UBR_1MHZ_1200 = 0x0369, UMCTL_1MHZ_1200 = 0x7B, 
  UBR_1MHZ_1800 = 0x0246, UMCTL_1MHZ_1800 = 0x55, 
  UBR_1MHZ_2400 = 0x01B4, UMCTL_1MHZ_2400 = 0xDF, 
  UBR_1MHZ_4800 = 0x00DA, UMCTL_1MHZ_4800 = 0xAA, 
  UBR_1MHZ_9600 = 0x006D, UMCTL_1MHZ_9600 = 0x44, 
  UBR_1MHZ_19200 = 0x0036, UMCTL_1MHZ_19200 = 0xB5, 
  UBR_1MHZ_38400 = 0x001B, UMCTL_1MHZ_38400 = 0x94, 
  UBR_1MHZ_57600 = 0x0012, UMCTL_1MHZ_57600 = 0x84, 
  UBR_1MHZ_76800 = 0x000D, UMCTL_1MHZ_76800 = 0x6D, 
  UBR_1MHZ_115200 = 0x0009, UMCTL_1MHZ_115200 = 0x10, 
  UBR_1MHZ_230400 = 0x0004, UMCTL_1MHZ_230400 = 0x55
} msp430_uart_rate_t;
#line 200
#line 171
typedef struct __nesc_unnamed4279 {
  unsigned int ubr : 16;

  unsigned int umctl : 8;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
  unsigned int  : 0;

  unsigned int  : 3;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int  : 1;

  unsigned int  : 2;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int  : 4;
  unsigned int  : 0;

  unsigned int utxe : 1;
  unsigned int urxe : 1;
} msp430_uart_config_t;








#line 202
typedef struct __nesc_unnamed4280 {
  uint16_t ubr;
  uint8_t umctl;
  uint8_t uctl;
  uint8_t utctl;
  uint8_t urctl;
  uint8_t ume;
} msp430_uart_registers_t;




#line 211
typedef union __nesc_unnamed4281 {
  msp430_uart_config_t uartConfig;
  msp430_uart_registers_t uartRegisters;
} msp430_uart_union_config_t;

msp430_uart_union_config_t msp430_uart_default_config = { 
{ 
.utxe = 1, 
.urxe = 1, 
.ubr = UBR_1MHZ_57600, 
.umctl = UMCTL_1MHZ_57600, 
.ssel = 0x02, 
.pena = 0, 
.pev = 0, 
.spb = 0, 
.clen = 1, 
.listen = 0, 
.mm = 0, 
.ckpl = 0, 
.urxse = 0, 
.urxeie = 1, 
.urxwie = 0, 
.utxe = 1, 
.urxe = 1 } };
#line 248
#line 240
typedef struct __nesc_unnamed4282 {
  unsigned int i2cstt : 1;
  unsigned int i2cstp : 1;
  unsigned int i2cstb : 1;
  unsigned int i2cctrx : 1;
  unsigned int i2cssel : 2;
  unsigned int i2ccrm : 1;
  unsigned int i2cword : 1;
} __attribute((packed))  msp430_i2ctctl_t;
#line 276
#line 253
typedef struct __nesc_unnamed4283 {
  unsigned int  : 1;
  unsigned int mst : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int xa : 1;
  unsigned int  : 1;
  unsigned int txdmaen : 1;
  unsigned int rxdmaen : 1;

  unsigned int  : 4;
  unsigned int i2cssel : 2;
  unsigned int i2crm : 1;
  unsigned int i2cword : 1;

  unsigned int i2cpsc : 8;

  unsigned int i2csclh : 8;

  unsigned int i2cscll : 8;

  unsigned int i2coa : 10;
  unsigned int  : 6;
} msp430_i2c_config_t;








#line 278
typedef struct __nesc_unnamed4284 {
  uint8_t uctl;
  uint8_t i2ctctl;
  uint8_t i2cpsc;
  uint8_t i2csclh;
  uint8_t i2cscll;
  uint16_t i2coa;
} msp430_i2c_registers_t;




#line 287
typedef union __nesc_unnamed4285 {
  msp430_i2c_config_t i2cConfig;
  msp430_i2c_registers_t i2cRegisters;
} msp430_i2c_union_config_t;
#line 309
typedef uint8_t uart_speed_t;
typedef uint8_t uart_parity_t;
typedef uint8_t uart_duplex_t;

enum __nesc_unnamed4286 {
  TOS_UART_1200 = 0, 
  TOS_UART_1800 = 1, 
  TOS_UART_2400 = 2, 
  TOS_UART_4800 = 3, 
  TOS_UART_9600 = 4, 
  TOS_UART_19200 = 5, 
  TOS_UART_38400 = 6, 
  TOS_UART_57600 = 7, 
  TOS_UART_76800 = 8, 
  TOS_UART_115200 = 9, 
  TOS_UART_230400 = 10
};

enum __nesc_unnamed4287 {
  TOS_UART_OFF, 
  TOS_UART_RONLY, 
  TOS_UART_TONLY, 
  TOS_UART_DUPLEX
};

enum __nesc_unnamed4288 {
  TOS_UART_PARITY_NONE, 
  TOS_UART_PARITY_EVEN, 
  TOS_UART_PARITY_ODD
};
# 29 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4289 {
#line 29
  int notUsed;
} 
#line 29
TMilli;
typedef struct __nesc_unnamed4290 {
#line 30
  int notUsed;
} 
#line 30
T32khz;
typedef struct __nesc_unnamed4291 {
#line 31
  int notUsed;
} 
#line 31
TMicro;
# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
# 11 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/LATIN.h"
enum __nesc_unnamed4292 {
  ScanWait = 0, 
  ScanSend = 1, 
  AdvWait = 2, 
  ReqSend = 3, 
  AdvSend = 4, 
  Build = 5, 
  BuildDone = 6
};



enum __nesc_unnamed4293 {
  SCAN_MSG = 0x09, 
  ADV_MSG = 0x0a, 
  REQ_MSG = 0x0b, 
  CONF_MSG = 0x0c, 
  SEED_MSG = 0x0d
};

enum __nesc_unnamed4294 {
  BUILDING = 0, SCHEDULING = 1
};



enum __nesc_unnamed4295 {
  UPLINK = 0, DOWNLINK = 1
};



enum __nesc_unnamed4296 {
  HOPPING_ENABLED = TRUE, 
  MAXNUMNEIGHBORS = 10, 
  NUMCHANNELS = 4, 
  MAXSCANSAMPLE = 50, 
  MAXADVSAMPLE = 50, 
  TXRETRIES = 3, 
  DEFAULT_SEED = 255
};









enum __nesc_unnamed4297 {
  S_SYNCHRONIZING = 0, 
  S_TX_TXDATAPREPARE = 1, 
  S_TX_TXDATAREADY = 2, 
  S_TX_TXDATA = 3, 
  S_TX_RXACKPREPARE = 4, 
  S_TX_RXACKREADY = 5, 
  S_TX_RXACK = 6, 
  S_RX_RXDATAPREPARE = 7, 
  S_RX_RXDATAREADY = 8, 
  S_RX_RXDATA = 9, 
  S_RX_TXACKPREPARE = 10, 
  S_RX_TXACKREADY = 11, 
  S_RX_TXACK = 12, 
  S_SLEEP = 13
};



enum __nesc_unnamed4298 {
  TsTxOffset = 86, 
  TsRxOffset = 44, 
  TsRxWaitTime = 93, 
  TsTxAckDelay = 64, 
  TsRxAckDelay = 1, 
  SLOT_TIME = 320, 
  radio_delay = 17
};



enum __nesc_unnamed4299 {
  TX_POWER = 10, 
  INVALID_TIMESTAMP = 0x80000000L
};

enum __nesc_unnamed4300 {
  FRAME_BASED_RESYNC = TRUE, ACK_BASED_RESYNC = FALSE
};







enum __nesc_unnamed4301 {
  COMPONENT_NULL = 0, 

  COMPONENT_APPDATA = 35, 

  COMPONENT_RES = 22, 

  COMPONENT_MAC = 25, 

  COMPONENT_CC2420DRIVER = 26, 
  COMPONENT_CC2420TRANSMIT = 27, 
  COMPONENT_CC2420RECEIVE = 28, 
  COMPONENT_CC2420CONTROL = 29, 

  COMPONENT_IDMANAGER = 30, 
  COMPONENT_OPENQUEUE = 31, 
  COMPONENT_PACKETFUNCTIONS = 33
};

enum __nesc_unnamed4302 {
  QUEUELENGTH = 20
};

enum __nesc_unnamed4303 {
  ADDR_NONE = 0, 
  ADDR_16B = 1, 
  ADDR_64B = 2, 
  ADDR_128B = 3, 
  ADDR_PANID = 4, 
  ADDR_PREFIX = 5
};

enum __nesc_unnamed4304 {
  LITTLE_ENDIAN = TRUE, BIG_ENDIAN = FALSE
};
#line 154
#line 147
typedef nx_struct CMD_MSG_t {
  nxle_uint16_t timingInformation;
  nxle_uint8_t commandFrameId;
  nxle_uint8_t seedNumber;
  nxle_uint8_t numHops;
  nxle_uint8_t numChildren;
  nxle_uint8_t seedList[MAXNUMNEIGHBORS];
} __attribute__((packed)) CMD_MSG_t;










#line 156
typedef struct open_addr_t {
  uint8_t type;
  union  {
    uint8_t addr_16b[2];
    uint8_t addr_64b[8];
    uint8_t addr_128b[16];
    uint8_t panid[2];
    uint8_t prefix[8];
  } ;
} open_addr_t;
#line 193
#line 169
typedef struct OpenQueueEntry_t {

  uint8_t creator;
  uint8_t owner;
  void *payload;
  uint8_t length;

  open_addr_t l3_destinationORsource;

  open_addr_t l2_nextORpreviousHop;
  uint8_t l2_seedNumber;
  uint8_t l2_frameType;
  uint8_t l2_commandFrameId;
  uint8_t l2_retriesLeft;
  bool l2_transmitInFirstSlot;

  uint8_t l1_txPower;
  uint8_t l1_channel;
  int8_t l1_rssi;
  uint8_t l1_lqi;
  bool l1_crc;
  uint32_t l1_rxTimestamp;

  uint8_t packet[124];
} OpenQueueEntry_t;

typedef uint16_t asn_t;
typedef uint32_t timervalue_t;
#line 208
#line 198
typedef struct neighborEntry_t {
  bool used;
  bool isChild;
  uint8_t seedNumber;
  int8_t rssi;
  uint8_t numHops;
  open_addr_t addr_16b;
  open_addr_t addr_64b;
  open_addr_t addr_128b;
} 
neighborEntry_t;
#line 226
#line 210
typedef struct neighborList_t {
  uint8_t numNeighbors;
  uint8_t old_numNeighbors;
  uint8_t counterNoNewNeighbors;
  uint8_t numChildren;
  uint8_t numSibling;
  uint8_t old_numChildren;
  uint8_t counterNoNewChildren;
  uint8_t counterSeedMsgSent;
  uint8_t seedList[MAXNUMNEIGHBORS];
  uint8_t childrenIndex[MAXNUMNEIGHBORS];
  uint8_t siblingList[MAXNUMNEIGHBORS];
  uint8_t parent;
  uint8_t minHops;
  int8_t maxRssi;
  neighborEntry_t neighbors[MAXNUMNEIGHBORS];
} neighborList_t;
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/IEEE802154.h"
enum __nesc_unnamed4305 {
  TIME_LIMITED_RX = TRUE, 
  INFITE_RX = FALSE
};
#line 23
#line 11
typedef struct ieee802154_header_iht {
  uint8_t headerLength;
  uint8_t frameType;
  bool securityEnabled;
  bool framePending;
  bool ackRequested;
  bool panIDCompression;
  uint8_t dsn;

  open_addr_t panid;
  open_addr_t dest;
  open_addr_t src;
} ieee802154_header_iht;

enum IEEE802154_fcf_enums {
  IEEE154_FCF_FRAME_TYPE = 0, 
  IEEE154_FCF_SECURITY_ENABLED = 3, 
  IEEE154_FCF_FRAME_PENDING = 4, 
  IEEE154_FCF_ACK_REQ = 5, 
  IEEE154_FCF_INTRAPAN = 6, 
  IEEE154_FCF_DEST_ADDR_MODE = 2, 
  IEEE154_FCF_SRC_ADDR_MODE = 6
};

enum IEEE802154_fcf_type_enums {
  IEEE154_TYPE_BEACON = 0, 
  IEEE154_TYPE_DATA = 1, 
  IEEE154_TYPE_ACK = 2, 
  IEEE154_TYPE_CMD = 3, 
  IEEE154_TYPE_UNDEFINED = 5
};

enum IEEE802154_fcf_sec_enums {
  IEEE154_SEC_NO_SECURITY = 0, 
  IEEE154_SEC_YES_SECURITY = 1
};

enum IEEE802154_fcf_pending_enums {
  IEEE154_PENDING_NO_FRAMEPENDING = 0, 
  IEEE154_PENDING_YES_FRAMEPENDING = 1
};

enum IEEE802154_fcf_ack_enums {
  IEEE154_ACK_NO_ACK_REQ = 0, 
  IEEE154_ACK_YES_ACK_REQ = 1
};

enum IEEE802154_fcf_panid_enums {
  IEEE154_PANID_UNCOMPRESSED = 0, 
  IEEE154_PANID_COMPRESSED = 1
};

enum IEEE802154_fcf_addr_mode_enums {
  IEEE154_ADDR_NONE = 0, 
  IEEE154_ADDR_SHORT = 2, 
  IEEE154_ADDR_EXT = 3
};
# 11 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/idmanager/IDManager.h"
#line 4
typedef struct debugIDManagerEntry_t {
  bool isDAGroot;
  bool isBridge;
  open_addr_t my16bID;
  open_addr_t my64bID;
  open_addr_t myPANID;
  open_addr_t myPrefix;
} debugIDManagerEntry_t;
enum __nesc_unnamed4306 {
  PAN_ID = 0xAAAA
};
# 9 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/LatinMac/IEEE802154E.h"
#line 6
typedef nx_struct IEEE802154E_ACK_ht {
  nxle_uint8_t dhrAckNack;
  nxle_uint16_t timeCorrection;
} __attribute__((packed)) IEEE802154E_ACK_ht;

enum IEEE802154E_ACK_dhrAckNack_enums {
  IEEE154E_ACK_dhrAckNack_DEFAULT = 0x82
};
# 42 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosb/UserButton.h"
typedef enum __nesc_unnamed4307 {
#line 42
  BUTTON_RELEASED = 0, BUTTON_PRESSED = 1
} 
#line 42
button_state_t;
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430Dma.h"
enum __nesc_unnamed4308 {
  DMA_CHANNELS = 3
};

enum __nesc_unnamed4309 {
  DMA_CHANNEL0 = 0, 
  DMA_CHANNEL1 = 1, 
  DMA_CHANNEL2 = 2, 
  DMA_CHANNEL_UNKNOWN = 3
};

enum __nesc_unnamed4310 {
  DMA_CHANNEL_AVAILABLE = 0, 
  DMA_CHANNEL_IN_USE = 1
};



enum __nesc_unnamed4311 {
  DMA0TSEL_SHIFT = 0, 
  DMA1TSEL_SHIFT = 4, 
  DMA2TSEL_SHIFT = 8, 
  DMATSEL_MASK = (uint16_t )0xf, 
  DMA0TSEL_MASK = 0xf, 
  DMA1TSEL_MASK = 0xf0, 
  DMA2TSEL_MASK = 0xf00
};
#line 110
#line 93
typedef enum __nesc_unnamed4312 {
  DMA_TRIGGER_DMAREQ = 0x0, 
  DMA_TRIGGER_TACCR2 = 0x1, 
  DMA_TRIGGER_TBCCR2 = 0x2, 
  DMA_TRIGGER_URXIFG0 = 0x3, 
  DMA_TRIGGER_UTXIFG0 = 0x4, 
  DMA_TRIGGER_DAC12IFG = 0x5, 
  DMA_TRIGGER_ADC12IFGx = 0x6, 
  DMA_TRIGGER_TACCR0 = 0x7, 
  DMA_TRIGGER_TBCCR0 = 0x8, 
  DMA_TRIGGER_URXIFG1 = 0x9, 
  DMA_TRIGGER_UTXIFG1 = 0xa, 
  DMA_TRIGGER_MULT = 0xb, 
  DMA_TRIGGER_DMAxIFG = 0xe, 


  DMA_TRIGGER_DMAE0 = 0xf
} dma_trigger_t;




#line 112
typedef struct dma_channel_trigger_s {
  unsigned int trigger : 4;
  unsigned int reserved : 12;
} __attribute((packed))  dma_channel_trigger_t;



enum __nesc_unnamed4313 {
  DISABLE_NMI = 0, 
  ENABLE_NMI = 1
};

enum __nesc_unnamed4314 {
  NOT_ROUND_ROBIN = 0, 
  ROUND_ROBIN = 1
};

enum __nesc_unnamed4315 {
  NOT_ON_FETCH = 0, 
  ON_FETCH = 1
};






#line 134
typedef struct dma_state_s {
  unsigned int enableNMI : 1;
  unsigned int roundRobin : 1;
  unsigned int onFetch : 1;
  unsigned int reserved : 13;
} __attribute((packed))  dma_state_t;





enum __nesc_unnamed4316 {
  DMADT_SHIFT = 12, 
  DMADT_MASK = 0x7
};








#line 150
typedef enum __nesc_unnamed4317 {
  DMA_SINGLE_TRANSFER = 0x0, 
  DMA_BLOCK_TRANSFER = 0x1, 
  DMA_BURST_BLOCK_TRANSFER = 0x2, 
  DMA_REPEATED_SINGLE_TRANSFER = 0x4, 
  DMA_REPEATED_BLOCK_TRANSFER = 0x5, 
  DMA_REPEATED_BURST_BLOCK_TRANSFER = 0x7
} dma_transfer_mode_t;


enum __nesc_unnamed4318 {
  DMASRCINCR_SHIFT = 8, 
  DMADSTINCR_SHIFT = 10, 
  DMAINCR_MASK = 0x3
};





#line 166
typedef enum __nesc_unnamed4319 {
  DMA_ADDRESS_UNCHANGED = 0x0, 
  DMA_ADDRESS_DECREMENTED = 0x2, 
  DMA_ADDRESS_INCREMENTED = 0x3
} dma_incr_t;




#line 172
typedef enum __nesc_unnamed4320 {
  DMA_WORD = 0x0, 
  DMA_BYTE = 0x1
} dma_byte_t;





#line 178
typedef enum __nesc_unnamed4321 {
  DMA_EDGE_SENSITIVE = 0x0, 
  DMA_LEVEL_SENSITIVE = 0x1
} dma_level_t;
#line 196
#line 183
typedef struct dma_channel_state_s {
  unsigned int request : 1;
  unsigned int abort : 1;
  unsigned int interruptEnable : 1;
  unsigned int interruptFlag : 1;
  unsigned int enable : 1;
  unsigned int level : 1;
  unsigned int srcByte : 1;
  unsigned int dstByte : 1;
  unsigned int srcIncrement : 2;
  unsigned int dstIncrement : 2;
  unsigned int transferMode : 3;
  unsigned int reserved2 : 1;
} __attribute((packed))  dma_channel_state_t;
# 7 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/openqueue/OpenQueue.h"
#line 4
typedef struct debugOpenQueueEntry_t {
  uint8_t creator;
  uint8_t owner;
} debugOpenQueueEntry_t;
enum /*PlatformSerialC.UartC*/Msp430Uart1C__0____nesc_unnamed4322 {
  Msp430Uart1C__0__CLIENT_ID = 0U
};
typedef T32khz /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__precision_tag;
typedef uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__size_type;
typedef T32khz /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag;
typedef /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__precision_tag;
typedef uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type;
enum /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0____nesc_unnamed4323 {
  Msp430Usart1C__0__CLIENT_ID = 0U
};
enum SerialAMQueueP____nesc_unnamed4324 {
  SerialAMQueueP__NUM_CLIENTS = 1U
};
typedef uint8_t /*PrintfC.QueueC*/BigQueueC__0__queue_t;
typedef /*PrintfC.QueueC*/BigQueueC__0__queue_t /*PrintfC.QueueC*/BigQueueC__0__Queue__t;
typedef uint8_t PrintfP__Queue__t;
typedef button_state_t LatinResP__UserButton__val_t;
typedef TMilli LatinResP__timerAdvWait__precision_tag;
typedef bool /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__Notify__val_t;
typedef bool /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__Get__val_t;
typedef bool UserButtonP__GetLower__val_t;
typedef button_state_t UserButtonP__Notify__val_t;
typedef bool UserButtonP__NotifyLower__val_t;
typedef button_state_t UserButtonP__Get__val_t;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC__0____nesc_unnamed4325 {
  Msp430Timer32khzC__0__ALARM_ID = 0U
};
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__frequency_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__frequency_tag /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type;
typedef TMilli /*CounterMilli32C.Transform*/TransformCounterC__0__to_precision_tag;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type;
typedef T32khz /*CounterMilli32C.Transform*/TransformCounterC__0__from_precision_tag;
typedef uint16_t /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__from_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__to_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type;
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__LocalTime__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__precision_tag;
typedef uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__size_type;
typedef T32khz LatinMacP__SlotAlarm__precision_tag;
typedef uint32_t LatinMacP__SlotAlarm__size_type;
typedef T32khz LatinMacP__FastAlarm__precision_tag;
typedef uint32_t LatinMacP__FastAlarm__size_type;
enum /*LatinMacC.SlotAlarmC.AlarmC.Msp430Timer*/Msp430Timer32khzC__1____nesc_unnamed4326 {
  Msp430Timer32khzC__1__ALARM_ID = 1U
};
typedef T32khz /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__frequency_tag;
typedef /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__frequency_tag /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__precision_tag;
typedef uint16_t /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type;
typedef T32khz /*Counter32khz32C.Transform*/TransformCounterC__1__to_precision_tag;
typedef uint32_t /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type;
typedef T32khz /*Counter32khz32C.Transform*/TransformCounterC__1__from_precision_tag;
typedef uint16_t /*Counter32khz32C.Transform*/TransformCounterC__1__from_size_type;
typedef uint16_t /*Counter32khz32C.Transform*/TransformCounterC__1__upper_count_type;
typedef /*Counter32khz32C.Transform*/TransformCounterC__1__from_precision_tag /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__precision_tag;
typedef /*Counter32khz32C.Transform*/TransformCounterC__1__from_size_type /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__size_type;
typedef /*Counter32khz32C.Transform*/TransformCounterC__1__to_precision_tag /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__precision_tag;
typedef /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__size_type;
typedef T32khz /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__to_precision_tag;
typedef uint32_t /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__to_size_type;
typedef T32khz /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__from_precision_tag;
typedef uint16_t /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__from_size_type;
typedef /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__to_precision_tag /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Alarm__precision_tag;
typedef /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__to_size_type /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Alarm__size_type;
typedef /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__from_precision_tag /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__AlarmFrom__precision_tag;
typedef /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__from_size_type /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__AlarmFrom__size_type;
typedef /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__to_precision_tag /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Counter__precision_tag;
typedef /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__to_size_type /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Counter__size_type;
enum /*LatinMacC.FastAlarmC.AlarmC.Msp430Timer*/Msp430Timer32khzC__2____nesc_unnamed4327 {
  Msp430Timer32khzC__2__ALARM_ID = 2U
};
typedef T32khz /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__frequency_tag;
typedef /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__frequency_tag /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Alarm__precision_tag;
typedef uint16_t /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Alarm__size_type;
typedef T32khz /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__to_precision_tag;
typedef uint32_t /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__to_size_type;
typedef T32khz /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__from_precision_tag;
typedef uint16_t /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__from_size_type;
typedef /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__to_precision_tag /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Alarm__precision_tag;
typedef /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__to_size_type /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Alarm__size_type;
typedef /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__from_precision_tag /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__AlarmFrom__precision_tag;
typedef /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__from_size_type /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__AlarmFrom__size_type;
typedef /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__to_precision_tag /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Counter__precision_tag;
typedef /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__to_size_type /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Counter__size_type;
typedef uint16_t RandomMlcgC__SeedInit__parameter;
typedef T32khz CC2420DriversP__RxWaitAlarm__precision_tag;
typedef uint32_t CC2420DriversP__RxWaitAlarm__size_type;
typedef T32khz CC2420ControlP__StartupAlarm__precision_tag;
typedef uint32_t CC2420ControlP__StartupAlarm__size_type;
enum /*CC2420ControlC.CC2420SpiC*/CC2420SpiC__0____nesc_unnamed4328 {
  CC2420SpiC__0__CLIENT_ID = 0U
};
enum /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0____nesc_unnamed4329 {
  Msp430Spi0C__0__CLIENT_ID = 0U
};
enum Msp430SpiDma0P____nesc_unnamed4330 {
  Msp430SpiDma0P__DMA_CHANNELS = 3
};
enum Msp430SpiDma0P____nesc_unnamed4331 {
  Msp430SpiDma0P__DMA_CHANNEL0 = 0, Msp430SpiDma0P__DMA_CHANNEL1 = 1, Msp430SpiDma0P__DMA_CHANNEL2 = 2, Msp430SpiDma0P__DMA_CHANNEL_UNKNOWN = 3
};
enum Msp430SpiDma0P____nesc_unnamed4332 {
  Msp430SpiDma0P__DMA_CHANNEL_AVAILABLE = 0, Msp430SpiDma0P__DMA_CHANNEL_IN_USE = 1
};
enum Msp430SpiDma0P____nesc_unnamed4333 {
  Msp430SpiDma0P__DMA0TSEL_SHIFT = 0, Msp430SpiDma0P__DMA1TSEL_SHIFT = 4, Msp430SpiDma0P__DMA2TSEL_SHIFT = 8, Msp430SpiDma0P__DMATSEL_MASK = (uint16_t )0xf, Msp430SpiDma0P__DMA0TSEL_MASK = 0xf, Msp430SpiDma0P__DMA1TSEL_MASK = 0xf0, Msp430SpiDma0P__DMA2TSEL_MASK = 0xf00
};
typedef enum Msp430SpiDma0P____nesc_unnamed4334 {
  Msp430SpiDma0P__DMA_TRIGGER_DMAREQ = 0x0, Msp430SpiDma0P__DMA_TRIGGER_TACCR2 = 0x1, Msp430SpiDma0P__DMA_TRIGGER_TBCCR2 = 0x2, Msp430SpiDma0P__DMA_TRIGGER_URXIFG0 = 0x3, Msp430SpiDma0P__DMA_TRIGGER_UTXIFG0 = 0x4, Msp430SpiDma0P__DMA_TRIGGER_DAC12IFG = 0x5, Msp430SpiDma0P__DMA_TRIGGER_ADC12IFGx = 0x6, Msp430SpiDma0P__DMA_TRIGGER_TACCR0 = 0x7, Msp430SpiDma0P__DMA_TRIGGER_TBCCR0 = 0x8, Msp430SpiDma0P__DMA_TRIGGER_URXIFG1 = 0x9, Msp430SpiDma0P__DMA_TRIGGER_UTXIFG1 = 0xa, Msp430SpiDma0P__DMA_TRIGGER_MULT = 0xb, Msp430SpiDma0P__DMA_TRIGGER_DMAxIFG = 0xe, Msp430SpiDma0P__DMA_TRIGGER_DMAE0 = 0xf
} Msp430SpiDma0P__dma_trigger_t;
typedef struct Msp430SpiDma0P__dma_channel_trigger_s {
  unsigned int trigger : 4;
  unsigned int reserved : 12;
} __attribute((packed))  Msp430SpiDma0P__dma_channel_trigger_t;
enum Msp430SpiDma0P____nesc_unnamed4335 {
  Msp430SpiDma0P__DISABLE_NMI = 0, Msp430SpiDma0P__ENABLE_NMI = 1
};
enum Msp430SpiDma0P____nesc_unnamed4336 {
  Msp430SpiDma0P__NOT_ROUND_ROBIN = 0, Msp430SpiDma0P__ROUND_ROBIN = 1
};
enum Msp430SpiDma0P____nesc_unnamed4337 {
  Msp430SpiDma0P__NOT_ON_FETCH = 0, Msp430SpiDma0P__ON_FETCH = 1
};
typedef struct Msp430SpiDma0P__dma_state_s {
  unsigned int enableNMI : 1;
  unsigned int roundRobin : 1;
  unsigned int onFetch : 1;
  unsigned int reserved : 13;
} __attribute((packed))  Msp430SpiDma0P__dma_state_t;
enum Msp430SpiDma0P____nesc_unnamed4338 {
  Msp430SpiDma0P__DMADT_SHIFT = 12, Msp430SpiDma0P__DMADT_MASK = 0x7
};
typedef enum Msp430SpiDma0P____nesc_unnamed4339 {
  Msp430SpiDma0P__DMA_SINGLE_TRANSFER = 0x0, Msp430SpiDma0P__DMA_BLOCK_TRANSFER = 0x1, Msp430SpiDma0P__DMA_BURST_BLOCK_TRANSFER = 0x2, Msp430SpiDma0P__DMA_REPEATED_SINGLE_TRANSFER = 0x4, Msp430SpiDma0P__DMA_REPEATED_BLOCK_TRANSFER = 0x5, Msp430SpiDma0P__DMA_REPEATED_BURST_BLOCK_TRANSFER = 0x7
} Msp430SpiDma0P__dma_transfer_mode_t;
enum Msp430SpiDma0P____nesc_unnamed4340 {
  Msp430SpiDma0P__DMASRCINCR_SHIFT = 8, Msp430SpiDma0P__DMADSTINCR_SHIFT = 10, Msp430SpiDma0P__DMAINCR_MASK = 0x3
};
typedef enum Msp430SpiDma0P____nesc_unnamed4341 {
  Msp430SpiDma0P__DMA_ADDRESS_UNCHANGED = 0x0, Msp430SpiDma0P__DMA_ADDRESS_DECREMENTED = 0x2, Msp430SpiDma0P__DMA_ADDRESS_INCREMENTED = 0x3
} Msp430SpiDma0P__dma_incr_t;
typedef enum Msp430SpiDma0P____nesc_unnamed4342 {
  Msp430SpiDma0P__DMA_WORD = 0x0, Msp430SpiDma0P__DMA_BYTE = 0x1
} Msp430SpiDma0P__dma_byte_t;
typedef enum Msp430SpiDma0P____nesc_unnamed4343 {
  Msp430SpiDma0P__DMA_EDGE_SENSITIVE = 0x0, Msp430SpiDma0P__DMA_LEVEL_SENSITIVE = 0x1
} Msp430SpiDma0P__dma_level_t;
typedef struct Msp430SpiDma0P__dma_channel_state_s {
  unsigned int request : 1;
  unsigned int abort : 1;
  unsigned int interruptEnable : 1;
  unsigned int interruptFlag : 1;
  unsigned int enable : 1;
  unsigned int level : 1;
  unsigned int srcByte : 1;
  unsigned int dstByte : 1;
  unsigned int srcIncrement : 2;
  unsigned int dstIncrement : 2;
  unsigned int transferMode : 3;
  unsigned int reserved2 : 1;
} __attribute((packed))  Msp430SpiDma0P__dma_channel_state_t;
enum /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0____nesc_unnamed4344 {
  Msp430Usart0C__0__CLIENT_ID = 0U
};
enum /*CC2420ControlC.OscSpiC*/CC2420SpiC__1____nesc_unnamed4345 {
  CC2420SpiC__1__CLIENT_ID = 1U
};
enum /*CC2420ControlC.Spi*/CC2420SpiC__2____nesc_unnamed4346 {
  CC2420SpiC__2__CLIENT_ID = 2U
};
enum /*CC2420ControlC.SyncSpiC*/CC2420SpiC__3____nesc_unnamed4347 {
  CC2420SpiC__3__CLIENT_ID = 3U
};
enum /*CC2420ControlC.RxOnSpiC*/CC2420SpiC__4____nesc_unnamed4348 {
  CC2420SpiC__4__CLIENT_ID = 4U
};
enum /*CC2420ControlC.RfOffSpiC*/CC2420SpiC__5____nesc_unnamed4349 {
  CC2420SpiC__5__CLIENT_ID = 5U
};
enum /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Timer*/Msp430Timer32khzC__3____nesc_unnamed4350 {
  Msp430Timer32khzC__3__ALARM_ID = 3U
};
typedef T32khz /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__frequency_tag;
typedef /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__frequency_tag /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__precision_tag;
typedef uint16_t /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__size_type;
typedef T32khz /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__to_precision_tag;
typedef uint32_t /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__to_size_type;
typedef T32khz /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__from_precision_tag;
typedef uint16_t /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__from_size_type;
typedef /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__to_precision_tag /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Alarm__precision_tag;
typedef /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__to_size_type /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Alarm__size_type;
typedef /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__from_precision_tag /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__AlarmFrom__precision_tag;
typedef /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__from_size_type /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__AlarmFrom__size_type;
typedef /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__to_precision_tag /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Counter__precision_tag;
typedef /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__to_size_type /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Counter__size_type;
enum /*CC2420ReceiveC.CC2420SpiC*/CC2420SpiC__6____nesc_unnamed4351 {
  CC2420SpiC__6__CLIENT_ID = 6U
};
typedef T32khz CC2420TransmitP__AlarmWatchdog__precision_tag;
typedef uint32_t CC2420TransmitP__AlarmWatchdog__size_type;
enum /*CC2420TransmitC.CC2420SpiC*/CC2420SpiC__7____nesc_unnamed4352 {
  CC2420SpiC__7__CLIENT_ID = 7U
};
enum /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Timer*/Msp430Timer32khzC__4____nesc_unnamed4353 {
  Msp430Timer32khzC__4__ALARM_ID = 4U
};
typedef T32khz /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__frequency_tag;
typedef /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__frequency_tag /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__precision_tag;
typedef uint16_t /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__size_type;
typedef T32khz /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__to_precision_tag;
typedef uint32_t /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__to_size_type;
typedef T32khz /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__from_precision_tag;
typedef uint16_t /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__from_size_type;
typedef /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__to_precision_tag /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__precision_tag;
typedef /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__to_size_type /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__size_type;
typedef /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__from_precision_tag /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__AlarmFrom__precision_tag;
typedef /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__from_size_type /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__AlarmFrom__size_type;
typedef /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__to_precision_tag /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Counter__precision_tag;
typedef /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__to_size_type /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Counter__size_type;
enum /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Timer*/Msp430Timer32khzC__5____nesc_unnamed4354 {
  Msp430Timer32khzC__5__ALARM_ID = 5U
};
typedef T32khz /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__frequency_tag;
typedef /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__frequency_tag /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Alarm__precision_tag;
typedef uint16_t /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Alarm__size_type;
typedef T32khz /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__to_precision_tag;
typedef uint32_t /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__to_size_type;
typedef T32khz /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__from_precision_tag;
typedef uint16_t /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__from_size_type;
typedef /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__to_precision_tag /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__precision_tag;
typedef /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__to_size_type /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__size_type;
typedef /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__from_precision_tag /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__AlarmFrom__precision_tag;
typedef /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__from_size_type /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__AlarmFrom__size_type;
typedef /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__to_precision_tag /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Counter__precision_tag;
typedef /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__to_size_type /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Counter__size_type;
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
static error_t PlatformP__Init__init(void );
#line 51
static error_t MotePlatformC__Init__init(void );
# 35 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 32
static void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );



static void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 31
static void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );





static void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 34
static void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void );
#line 29
static void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__default__initClocks(void );
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
static error_t Msp430ClockP__Init__init(void );
# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(
# 40 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4062c4f0);
# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(
# 40 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4062c4f0);
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
static bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t time);
# 31 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t time);
# 31 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t time);
# 31 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t time);
# 31 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );
#line 36
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );
# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );
# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t delta);
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void );
# 44 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(uint8_t cm);
#line 31
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void );
# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t time);
# 31 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void );
# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );
# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t delta);
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t time);
# 31 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__clearPendingInterrupt(void );
# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );
# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEventFromNow(uint16_t delta);
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t time);
# 31 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__clearPendingInterrupt(void );
# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );
# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__setEventFromNow(uint16_t delta);
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t time);
# 31 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__clearPendingInterrupt(void );
# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );
# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__setEventFromNow(uint16_t delta);
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t time);
# 31 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__clearPendingInterrupt(void );
# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );
# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__setEventFromNow(uint16_t delta);
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/McuSleep.nc"
static void McuSleepC__McuSleep__sleep(void );
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x4059ab40);
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x4059ab40);
# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP__Scheduler__init(void );
#line 61
static void SchedulerBasicP__Scheduler__taskLoop(void );
#line 54
static bool SchedulerBasicP__Scheduler__runNextTask(void );
# 89 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Send.nc"
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(
#line 85
message_t * msg, 



error_t error);
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 69 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AMSend.nc"
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(
# 36 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x407205f8, 
# 69 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Packet.nc"
static uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__payloadLength(
#line 63
message_t * msg);
#line 115
static 
#line 112
void * 


/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__getPayload(
#line 110
message_t * msg, 




uint8_t len);
#line 95
static uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength(void );
#line 83
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__setPayloadLength(
#line 79
message_t * msg, 



uint8_t len);
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x4072f010, 
# 60 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AMPacket.nc"
static am_addr_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__destination(
#line 63
message_t * amsg);
#line 92
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 136
static am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(
#line 132
message_t * amsg);
#line 151
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
# 83 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/SplitControl.nc"
static error_t SerialP__SplitControl__start(void );
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void SerialP__stopDoneTask__runTask(void );
#line 64
static void SerialP__RunTx__runTask(void );
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
static error_t SerialP__Init__init(void );
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialFlush.nc"
static void SerialP__SerialFlush__flushDone(void );
#line 38
static void SerialP__SerialFlush__default__flush(void );
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void SerialP__startDoneTask__runTask(void );
# 83 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialFrameComm.nc"
static void SerialP__SerialFrameComm__dataReceived(uint8_t data);





static void SerialP__SerialFrameComm__putDone(void );
#line 74
static void SerialP__SerialFrameComm__delimiterReceived(void );
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void SerialP__defaultSerialFlushTask__runTask(void );
# 60 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SendBytePacket.nc"
static error_t SerialP__SendBytePacket__completeSend(void );
#line 51
static error_t SerialP__SendBytePacket__startSend(uint8_t first_byte);
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask(void );
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Send.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(
# 40 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x407d0840, 
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 89
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(
# 40 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x407d0840, 
# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask(void );
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(
# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x407d0200, 
# 60 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 31 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x407cf398, 
# 31 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x407cf398);
# 23 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x407cf398, 
# 23 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t upperLen);
# 70 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SendBytePacket.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte(void );









static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error_t error);
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/ReceiveBytePacket.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket(void );






static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(uint8_t data);










static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(error_t result);
# 79 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/UartStream.nc"
static void HdlcTranslateC__UartStream__receivedByte(uint8_t byte);
#line 99
static void HdlcTranslateC__UartStream__receiveDone(
#line 95
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void HdlcTranslateC__UartStream__sendDone(
#line 53
uint8_t * buf, 



uint16_t len, error_t error);
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialFrameComm.nc"
static error_t HdlcTranslateC__SerialFrameComm__putDelimiter(void );
#line 68
static void HdlcTranslateC__SerialFrameComm__resetReceive(void );
#line 54
static error_t HdlcTranslateC__SerialFrameComm__putData(uint8_t data);
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(
# 44 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40877010);
# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__configure(
# 44 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40877010);
# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartConfigure.nc"
static msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(
# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408746c0);
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/UartStream.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__send(
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40877770, 
# 44 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len);
#line 79
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40877770, 
# 79 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/UartStream.nc"
uint8_t byte);
#line 99
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40877770, 
# 95 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__sendDone(
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40877770, 
# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__overflow(void );
# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__release(
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40875bb0);
# 87 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__immediateRequest(
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40875bb0);
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__granted(
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40875bb0);
# 118 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__isOwner(
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40875bb0);
# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__release(
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408785a8);
# 87 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__immediateRequest(
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408785a8);
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__default__granted(
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408785a8);
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__rxDone(
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40899970, 
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__txDone(
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40899970);
# 143 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430Usart1P__Usart__enableUartRx(void );
#line 123
static void HplMsp430Usart1P__Usart__enableUart(void );
#line 97
static void HplMsp430Usart1P__Usart__resetUsart(bool reset);
#line 179
static void HplMsp430Usart1P__Usart__disableIntr(void );
#line 90
static void HplMsp430Usart1P__Usart__setUmctl(uint8_t umctl);
#line 133
static void HplMsp430Usart1P__Usart__enableUartTx(void );
#line 148
static void HplMsp430Usart1P__Usart__disableUartRx(void );
#line 182
static void HplMsp430Usart1P__Usart__enableIntr(void );
#line 207
static void HplMsp430Usart1P__Usart__clrIntr(void );
#line 80
static void HplMsp430Usart1P__Usart__setUbr(uint16_t ubr);
#line 224
static void HplMsp430Usart1P__Usart__tx(uint8_t data);
#line 128
static void HplMsp430Usart1P__Usart__disableUart(void );
#line 174
static void HplMsp430Usart1P__Usart__setModeUart(msp430_uart_union_config_t *config);
#line 158
static void HplMsp430Usart1P__Usart__disableSpi(void );
#line 138
static void HplMsp430Usart1P__Usart__disableUartTx(void );
# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AsyncStdControl.nc"
static error_t HplMsp430Usart1P__AsyncStdControl__start(void );









static error_t HplMsp430Usart1P__AsyncStdControl__stop(void );
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void );






static bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void );
#line 64
static void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void );
#line 64
static void /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__makeInput(void );
#line 59
static bool /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__getRaw(void );
#line 85
static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void );
#line 85
static void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void );
#line 85
static void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectModuleFunc(void );
#line 64
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void );
#line 59
static bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get(void );
#line 85
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw(void );
#line 78
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void );
#line 71
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set(void );




static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr(void );
#line 71
static void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void );




static void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr(void );
#line 71
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void );




static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void );
#line 85
static void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc(void );
#line 85
static void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc(void );
#line 85
static void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc(void );
#line 71
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );




static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__clr(void );
#line 71
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );
#line 71
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );




static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void );
#line 71
static void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__makeOutput(void );
#line 71
static void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__makeOutput(void );
#line 71
static void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__makeOutput(void );
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
static /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );






static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
static error_t LedsP__Init__init(void );
# 83 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Leds.nc"
static void LedsP__Leds__led2Off(void );
#line 45
static void LedsP__Leds__led0On(void );
#line 78
static void LedsP__Leds__led2On(void );
# 35 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
#line 29
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr(void );




static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
#line 29
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );





static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
#line 29
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void );
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(
# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40a89010, 
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(
# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40a89010);
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void );
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceQueue.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );
#line 60
static resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40aac948);
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(
# 60 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40aaacf8);
# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(
# 60 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40aaacf8);
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40a99e98);
# 87 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40a99e98);
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40a99e98);
# 118 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40a99e98);
# 80 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
# 52 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/power/PowerDownCleanup.nc"
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup(void );
# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted(void );
#line 81
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested(void );
# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartConfigure.nc"
static msp430_uart_union_config_t *TelosSerialP__Msp430UartConfigure__getConfig(void );
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static void TelosSerialP__Resource__granted(void );
# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/StdControl.nc"
static error_t TelosSerialP__StdControl__start(void );









static error_t TelosSerialP__StdControl__stop(void );
# 31 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t SerialPacketInfoActiveMessageP__Info__upperLength(message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t SerialPacketInfoActiveMessageP__Info__offset(void );







static uint8_t SerialPacketInfoActiveMessageP__Info__dataLinkLength(message_t *msg, uint8_t upperLen);
# 69 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AMSend.nc"
static error_t /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 89 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Send.nc"
static void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(
#line 85
message_t * msg, 



error_t error);
# 99 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AMSend.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(
# 40 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/AMQueueImplP.nc"
am_id_t arg_0x40b1e908, 
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Send.nc"
static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(
# 38 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/AMQueueImplP.nc"
uint8_t arg_0x40afbe70, 
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 89
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(
# 38 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/AMQueueImplP.nc"
uint8_t arg_0x40afbe70, 
# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void );
#line 64
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void );
# 73 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/BigQueue.nc"
static /*PrintfC.QueueC*/BigQueueC__0__Queue__t /*PrintfC.QueueC*/BigQueueC__0__Queue__head(void );
#line 90
static error_t /*PrintfC.QueueC*/BigQueueC__0__Queue__enqueue(/*PrintfC.QueueC*/BigQueueC__0__Queue__t newVal);
#line 65
static uint16_t /*PrintfC.QueueC*/BigQueueC__0__Queue__maxSize(void );
#line 81
static /*PrintfC.QueueC*/BigQueueC__0__Queue__t /*PrintfC.QueueC*/BigQueueC__0__Queue__dequeue(void );
#line 50
static bool /*PrintfC.QueueC*/BigQueueC__0__Queue__empty(void );







static uint16_t /*PrintfC.QueueC*/BigQueueC__0__Queue__size(void );
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/SplitControl.nc"
static void PrintfP__SerialControl__startDone(error_t error);
#line 117
static void PrintfP__SerialControl__stopDone(error_t error);
# 99 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AMSend.nc"
static void PrintfP__AMSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void PrintfP__retrySend__runTask(void );
# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Boot.nc"
static void PrintfP__MainBoot__booted(void );
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
static error_t appDATADevP__SoftwareInit__init(void );
# 5 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/OpenSend.nc"
static void appDATADevP__OpenSendToLower__sendDone(OpenQueueEntry_t *msg, error_t error);
# 13 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/PacketFunctions.nc"
static void PacketFunctionsP__PacketFunctions__readAddress(uint8_t *payload, uint8_t type, open_addr_t *writeToAddress, bool littleEndian);
#line 8
static void PacketFunctionsP__PacketFunctions__mac16bToMac64b(open_addr_t *mac16b, open_addr_t *mac64btoWrite);









static void PacketFunctionsP__PacketFunctions__reserveFooterSize(OpenQueueEntry_t *pkt, uint8_t header_length);
static void PacketFunctionsP__PacketFunctions__tossFooter(OpenQueueEntry_t *pkt, uint8_t header_length);
#line 6
static void PacketFunctionsP__PacketFunctions__mac64bToIp128b(open_addr_t *prefix64b, open_addr_t *mac64b, open_addr_t *ip128bToWrite);




static bool PacketFunctionsP__PacketFunctions__sameAddress(open_addr_t *address_1, open_addr_t *address_2);


static void PacketFunctionsP__PacketFunctions__writeAddress(OpenQueueEntry_t *msg, open_addr_t *address, bool littleEndian);
#line 7
static void PacketFunctionsP__PacketFunctions__mac64bToMac16b(open_addr_t *mac64b, open_addr_t *mac16btoWrite);









static void PacketFunctionsP__PacketFunctions__tossHeader(OpenQueueEntry_t *pkt, uint8_t header_length);
#line 5
static void PacketFunctionsP__PacketFunctions__ip128bToMac64b(open_addr_t *ip128b, open_addr_t *prefix64btoWrite, open_addr_t *mac64btoWrite);




static bool PacketFunctionsP__PacketFunctions__isBroadcastMulticast(open_addr_t *address);





static void PacketFunctionsP__PacketFunctions__reserveHeaderSize(OpenQueueEntry_t *pkt, uint8_t header_length);
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void IDManagerP__taskHandleRoot__runTask(void );
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
static error_t IDManagerP__SoftwareInit__init(void );
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void IDManagerP__taskPrint__runTask(void );
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/IDManager.nc"
static bool IDManagerP__IDManager__getIsDAGroot(void );



static open_addr_t *IDManagerP__IDManager__getMyID(uint8_t type);
#line 5
static error_t IDManagerP__IDManager__setIsDAGroot(bool newRole);
static bool IDManagerP__IDManager__getIsBridge(void );



static bool IDManagerP__IDManager__isMyAddress(open_addr_t *addr);
#line 7
static error_t IDManagerP__IDManager__setIsBridge(bool newRole);
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void IDManagerP__taskHandleBridge__runTask(void );
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/OpenReceive.nc"
static void LatinResP__OpenReceiveFromLower__receive(OpenQueueEntry_t *msg);
# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Notify.nc"
static void LatinResP__UserButton__notify(LatinResP__UserButton__val_t val);
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
static error_t LatinResP__SoftwareInit__init(void );
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void LatinResP__taskBuildAndSendReqMsg__runTask(void );
# 72 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Timer.nc"
static void LatinResP__timerAdvWait__fired(void );
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void LatinResP__taskBuildAndSendAdvMsg__runTask(void );
#line 64
static void LatinResP__taskBuildAndSendSeedMsgToChildren__runTask(void );
# 5 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/OpenSend.nc"
static void LatinResP__OpenSendToLower__sendDone(OpenQueueEntry_t *msg, error_t error);
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void LatinResP__taskBuildAndSendDataToChildren__runTask(void );
#line 64
static void LatinResP__taskBuildAndSendScanMsg__runTask(void );
# 6 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/LatinMatrix.nc"
static uint8_t LatinResP__LatinMatrix__getParentSeed(void );
#line 2
static bool LatinResP__LatinMatrix__childCanTransmit(uint8_t minLatinWindow, uint8_t *latinChannelOffset);
static bool LatinResP__LatinMatrix__siblingCanTransmit(uint8_t minLatinWindow, uint8_t *latinChannelOffset);

static bool LatinResP__LatinMatrix__canTransmit(uint8_t minLatinWindow, uint8_t *latinChannelOffset);
#line 4
static bool LatinResP__LatinMatrix__parentCanTransmit(uint8_t minLatinWindow, uint8_t *latinChannelOffset);
# 41 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void HplMsp430InterruptP__Port14__clear(void );
#line 36
static void HplMsp430InterruptP__Port14__disable(void );
#line 56
static void HplMsp430InterruptP__Port14__edge(bool low_to_high);
#line 31
static void HplMsp430InterruptP__Port14__enable(void );









static void HplMsp430InterruptP__Port26__clear(void );
#line 61
static void HplMsp430InterruptP__Port26__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port17__clear(void );
#line 61
static void HplMsp430InterruptP__Port17__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port21__clear(void );
#line 61
static void HplMsp430InterruptP__Port21__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port12__clear(void );
#line 61
static void HplMsp430InterruptP__Port12__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port24__clear(void );
#line 61
static void HplMsp430InterruptP__Port24__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port15__clear(void );
#line 61
static void HplMsp430InterruptP__Port15__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port27__clear(void );
#line 36
static void HplMsp430InterruptP__Port27__disable(void );
#line 56
static void HplMsp430InterruptP__Port27__edge(bool low_to_high);
#line 31
static void HplMsp430InterruptP__Port27__enable(void );









static void HplMsp430InterruptP__Port10__clear(void );
#line 36
static void HplMsp430InterruptP__Port10__disable(void );
#line 56
static void HplMsp430InterruptP__Port10__edge(bool low_to_high);
#line 31
static void HplMsp430InterruptP__Port10__enable(void );









static void HplMsp430InterruptP__Port22__clear(void );
#line 61
static void HplMsp430InterruptP__Port22__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port13__clear(void );
#line 61
static void HplMsp430InterruptP__Port13__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port25__clear(void );
#line 61
static void HplMsp430InterruptP__Port25__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port16__clear(void );
#line 61
static void HplMsp430InterruptP__Port16__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port20__clear(void );
#line 61
static void HplMsp430InterruptP__Port20__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port11__clear(void );
#line 61
static void HplMsp430InterruptP__Port11__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port23__clear(void );
#line 61
static void HplMsp430InterruptP__Port23__default__fired(void );
# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
static void /*HplUserButtonC.UserButtonC*/Msp430GpioC__3__GeneralIO__makeInput(void );
#line 32
static bool /*HplUserButtonC.UserButtonC*/Msp430GpioC__3__GeneralIO__get(void );
# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__HplInterrupt__fired(void );
# 50 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioInterrupt.nc"
static error_t /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__Interrupt__disable(void );
#line 43
static error_t /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__Interrupt__enableFallingEdge(void );
#line 42
static error_t /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void );
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__sendEvent__runTask(void );
# 57 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioInterrupt.nc"
static void /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__GpioInterrupt__fired(void );
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Notify.nc"
static error_t /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__Notify__enable(void );
#line 59
static error_t UserButtonP__Notify__enable(void );
#line 74
static void UserButtonP__NotifyLower__notify(UserButtonP__NotifyLower__val_t val);
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type dt);
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
static void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
#line 53
static /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
# 98 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type dt);
#line 105
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
# 125 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
#line 118
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
# 72 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );
#line 72
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x40e3f9f0);
# 81 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Timer.nc"
static bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x40e3f9f0);
# 62 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x40e3f9f0, 
# 62 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Timer.nc"
uint32_t dt);
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 2 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/MAC.nc"
static void LatinMacP__MAC__changeNetworkState(uint8_t tmp_networkState);
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void LatinMacP__taskReceive__runTask(void );
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
static error_t LatinMacP__SoftwareInit__init(void );
# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Boot.nc"
static void LatinMacP__Boot__booted(void );
# 2 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/GlobalTime.nc"
static timervalue_t LatinMacP__GlobalTime__getGlobalSlotOffset(void );

static asn_t LatinMacP__GlobalTime__getASN(void );
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void LatinMacP__SlotAlarm__fired(void );
# 12 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/RadioControl.nc"
static void LatinMacP__RadioControl__startDone(error_t error);








static void LatinMacP__RadioControl__prepareReceiveDone(error_t error);
#line 43
static void LatinMacP__RadioControl__stopDone(error_t error);
#line 30
static void LatinMacP__RadioControl__receivedNothing(void );
# 7 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/RadioReceive.nc"
static void LatinMacP__RadioReceive__receive(OpenQueueEntry_t *msg);
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void LatinMacP__taskSendDone__runTask(void );
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/OpenSend.nc"
static error_t LatinMacP__OpenSendFromUpper__send(OpenQueueEntry_t *msg);
# 12 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/RadioSend.nc"
static void LatinMacP__RadioSend__prepareSendDone(error_t error);







static void LatinMacP__RadioSend__sendNowDone(error_t error);
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void LatinMacP__FastAlarm__fired(void );
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(/*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type t0, /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type dt);
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
static void /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void );
#line 53
static /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__size_type /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__get(void );
# 98 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Alarm__size_type /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Alarm__getNow(void );
#line 92
static void /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Alarm__startAt(/*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Alarm__size_type t0, /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Alarm__size_type dt);
#line 105
static /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Alarm__size_type /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Alarm__getAlarm(void );
#line 67
static void /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__AlarmFrom__fired(void );
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
static void /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Counter__overflow(void );
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired(void );
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow(void );
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Alarm__startAt(/*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Alarm__size_type t0, /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Alarm__size_type dt);





static /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Alarm__size_type /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Alarm__getNow(void );
#line 92
static void /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Alarm__startAt(/*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Alarm__size_type t0, /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Alarm__size_type dt);
#line 55
static void /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Alarm__start(/*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Alarm__size_type dt);
#line 67
static void /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__AlarmFrom__fired(void );
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
static void /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Counter__overflow(void );
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
static error_t PinDebugP__Reset__init(void );
# 35 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
static void /*PinDebugC.PinADC0*/Msp430GpioC__4__GeneralIO__makeOutput(void );
#line 35
static void /*PinDebugC.PinADC1*/Msp430GpioC__5__GeneralIO__makeOutput(void );
#line 35
static void /*PinDebugC.PinADC2*/Msp430GpioC__6__GeneralIO__makeOutput(void );
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
static error_t RandomMlcgC__Init__init(void );
# 9 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Config.nc"
static void CC2420DriversP__CC2420Config__setChannelDone(error_t error);
#line 5
static void CC2420DriversP__CC2420Config__startOscillatorDone(void );

static void CC2420DriversP__CC2420Config__writeIdDone(void );
#line 3
static void CC2420DriversP__CC2420Config__startVRegDone(void );
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void CC2420DriversP__RxWaitAlarm__fired(void );
# 8 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420sfd.nc"
static void CC2420DriversP__CC2420sfd__sfd(uint32_t time);
# 26 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/RadioControl.nc"
static error_t CC2420DriversP__RadioControl__receiveNow(bool timeLimitedRx, uint16_t rxWaitTime);








static error_t CC2420DriversP__RadioControl__rfOff(void );
#line 17
static error_t CC2420DriversP__RadioControl__prepareReceive(uint8_t frequencyChannel);
#line 7
static error_t CC2420DriversP__RadioControl__start(void );
# 3 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Transmit.nc"
static void CC2420DriversP__CC2420Transmit__loadPacketDone(error_t error);

static void CC2420DriversP__CC2420Transmit__sendNowDone(error_t error);
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void CC2420DriversP__taskSendDone__runTask(void );
#line 64
static void CC2420DriversP__taskStartOscillatorDone__runTask(void );
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420ReceivedNothing.nc"
static void CC2420DriversP__CC2420ReceivedNothing__receivedNothing(void );
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void CC2420DriversP__taskStopDone__runTask(void );
# 8 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/RadioSend.nc"
static error_t CC2420DriversP__RadioSend__prepareSend(OpenQueueEntry_t *msg);







static error_t CC2420DriversP__RadioSend__sendNow(void );
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void CC2420DriversP__taskStartDone__runTask(void );
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Config.nc"
static error_t CC2420ControlP__CC2420Config__startOscillator(void );






static error_t CC2420ControlP__CC2420Config__rfOff(void );
#line 8
static error_t CC2420ControlP__CC2420Config__setChannel(uint8_t channel);
#line 6
static error_t CC2420ControlP__CC2420Config__writeId(void );



static error_t CC2420ControlP__CC2420Config__rxOn(void );
#line 2
static error_t CC2420ControlP__CC2420Config__startVReg(void );










static error_t CC2420ControlP__CC2420Config__stopVReg(void );
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static void CC2420ControlP__OscResource__granted(void );
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
static error_t CC2420ControlP__Init__init(void );
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static void CC2420ControlP__RxOnResource__granted(void );
#line 92
static void CC2420ControlP__RfOffResource__granted(void );
#line 92
static void CC2420ControlP__SpiResource__granted(void );
#line 92
static void CC2420ControlP__SyncResource__granted(void );
# 57 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioInterrupt.nc"
static void CC2420ControlP__InterruptCCA__fired(void );
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void CC2420ControlP__StartupAlarm__fired(void );
# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__7__GeneralIO__makeInput(void );

static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__GeneralIO__makeOutput(void );
#line 29
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__GeneralIO__set(void );
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__GeneralIO__clr(void );

static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__9__GeneralIO__get(void );
#line 32
static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__10__GeneralIO__get(void );


static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__11__GeneralIO__makeOutput(void );
#line 29
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__11__GeneralIO__set(void );
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__11__GeneralIO__clr(void );


static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__12__GeneralIO__makeInput(void );
#line 32
static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__12__GeneralIO__get(void );


static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__13__GeneralIO__makeOutput(void );
#line 29
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__13__GeneralIO__set(void );
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__13__GeneralIO__clr(void );
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/SpiPacket.nc"
static void CC2420SpiP__SpiPacket__sendDone(
#line 64
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 62 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Fifo.nc"
static error_t CC2420SpiP__Fifo__continueRead(
# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104a950, 
# 62 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 91
static void CC2420SpiP__Fifo__default__writeDone(
# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104a950, 
# 91 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
#line 82
static cc2420_status_t CC2420SpiP__Fifo__write(
# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104a950, 
# 82 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 51
static cc2420_status_t CC2420SpiP__Fifo__beginRead(
# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104a950, 
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 71
static void CC2420SpiP__Fifo__default__readDone(
# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104a950, 
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static void CC2420SpiP__SpiResource__granted(void );
# 63 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420SpiP__Ram__write(
# 47 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
uint16_t arg_0x410483f0, 
# 63 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Ram.nc"
uint8_t offset, uint8_t * data, uint8_t length);
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420SpiP__Reg__write(
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x41048b98, 
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Register.nc"
uint16_t data);
# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__release(
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104def0);
# 87 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__immediateRequest(
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104def0);
# 78 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__request(
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104def0);
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static void CC2420SpiP__Resource__default__granted(
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104def0);
# 118 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static bool CC2420SpiP__Resource__isOwner(
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104def0);
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void CC2420SpiP__grant__runTask(void );
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420SpiP__Strobe__strobe(
# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x41047420);
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
static error_t StateImplP__Init__init(void );
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/State.nc"
static void StateImplP__State__toIdle(
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/StateImplP.nc"
uint8_t arg_0x4107ab60);
# 66 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/State.nc"
static bool StateImplP__State__isState(
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/StateImplP.nc"
uint8_t arg_0x4107ab60, 
# 66 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/State.nc"
uint8_t myState);
#line 61
static bool StateImplP__State__isIdle(
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/StateImplP.nc"
uint8_t arg_0x4107ab60);
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/State.nc"
static error_t StateImplP__State__requestState(
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/StateImplP.nc"
uint8_t arg_0x4107ab60, 
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/State.nc"
uint8_t reqState);
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__ResourceConfigure__unconfigure(
# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x410c55d0);
# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__ResourceConfigure__configure(
# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x410c55d0);
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/SpiPacket.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiPacket__send(
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x410c30c0, 
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
#line 71
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiPacket__default__sendDone(
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x410c30c0, 
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Msp430SpiConfigure__default__getConfig(
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x410c1418);
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/SpiByte.nc"
static uint8_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiByte__write(uint8_t tx);
# 96 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannel.nc"
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel1__transferDone(error_t success);
# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__release(
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x410c29a0);
# 87 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__immediateRequest(
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x410c29a0);
# 78 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__request(
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x410c29a0);
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__granted(
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x410c29a0);
# 118 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static bool /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__isOwner(
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x410c29a0);
# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__release(
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x410c6b00);
# 87 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__immediateRequest(
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x410c6b00);
# 78 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__request(
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x410c6b00);
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__default__granted(
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x410c6b00);
# 118 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static bool /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__isOwner(
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x410c6b00);
# 96 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannel.nc"
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel2__transferDone(error_t success);
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartInterrupts__txDone(void );
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone_task__runTask(void );
# 197 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430Usart0P__Usart__clrRxIntr(void );
#line 97
static void HplMsp430Usart0P__Usart__resetUsart(bool reset);
#line 179
static void HplMsp430Usart0P__Usart__disableIntr(void );
#line 90
static void HplMsp430Usart0P__Usart__setUmctl(uint8_t umctl);
#line 207
static void HplMsp430Usart0P__Usart__clrIntr(void );
#line 80
static void HplMsp430Usart0P__Usart__setUbr(uint16_t ubr);
#line 224
static void HplMsp430Usart0P__Usart__tx(uint8_t data);
#line 128
static void HplMsp430Usart0P__Usart__disableUart(void );
#line 153
static void HplMsp430Usart0P__Usart__enableSpi(void );
#line 168
static void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 231
static uint8_t HplMsp430Usart0P__Usart__rx(void );
#line 192
static bool HplMsp430Usart0P__Usart__isRxIntrPending(void );
#line 158
static void HplMsp430Usart0P__Usart__disableSpi(void );
# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaInterrupt.nc"
static void /*HplMsp430DmaC.Dma0*/HplMsp430DmaXP__0__Interrupt__fired(void );
# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
static error_t /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setTrigger(dma_trigger_t trigger);
#line 65
static void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setStateRaw(uint16_t state, uint16_t trigger, void *src, void *dest, uint16_t size);
#line 49
static void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__enableDMA(void );
#line 64
static void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setState(dma_channel_state_t s, dma_channel_trigger_t t, void *src, void *dest, uint16_t size);
# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaInterrupt.nc"
static void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__Interrupt__fired(void );
# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
static error_t /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setTrigger(dma_trigger_t trigger);
#line 65
static void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setStateRaw(uint16_t state, uint16_t trigger, void *src, void *dest, uint16_t size);
#line 49
static void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__enableDMA(void );
#line 64
static void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setState(dma_channel_state_t s, dma_channel_trigger_t t, void *src, void *dest, uint16_t size);
# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaInterrupt.nc"
static void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__Interrupt__fired(void );
# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
static void /*Msp430DmaC.Channel0P*/Msp430DmaChannelP__0__HplChannel__transferDone(error_t success);
# 96 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannel.nc"
static void /*Msp430DmaC.Channel0P*/Msp430DmaChannelP__0__Channel__default__transferDone(error_t success);
# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
static void /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__HplChannel__transferDone(error_t success);
# 38 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannel.nc"
static error_t /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__Channel__setupTransfer(dma_transfer_mode_t transfer_mode, 
dma_trigger_t trigger, 
dma_level_t level, 
void *src_addr, 
void *dst_addr, 
uint16_t size, 
dma_byte_t src_byte, 
dma_byte_t dst_byte, 
dma_incr_t src_incr, 
dma_incr_t dst_incr);
#line 73
static error_t /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__Channel__startTransfer(void );
# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
static void /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__HplChannel__transferDone(error_t success);
# 38 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannel.nc"
static error_t /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__Channel__setupTransfer(dma_transfer_mode_t transfer_mode, 
dma_trigger_t trigger, 
dma_level_t level, 
void *src_addr, 
void *dst_addr, 
uint16_t size, 
dma_byte_t src_byte, 
dma_byte_t dst_byte, 
dma_incr_t src_incr, 
dma_incr_t dst_incr);
#line 73
static error_t /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__Channel__startTransfer(void );
# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
static void Msp430DmaControlP__HplChannel1__transferDone(error_t success);
#line 74
static void Msp430DmaControlP__HplChannel2__transferDone(error_t success);
#line 74
static void Msp430DmaControlP__HplChannel0__transferDone(error_t success);
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__rxDone(
# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40a89010, 
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__txDone(
# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40a89010);
# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawI2CInterrupts__fired(void );
#line 39
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__I2CInterrupts__default__fired(
# 40 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40a88010);
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__txDone(void );
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void );
# 69 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(resource_client_id_t id);
#line 43
static bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void );








static bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void );
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40aac948);
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40aac948);
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(
# 60 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40aaacf8);
# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(
# 60 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40aaacf8);
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void );
#line 73
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__requested(void );
#line 46
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__granted(void );
#line 81
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__immediateRequested(void );
# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40a99e98);
# 87 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40a99e98);
# 78 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__request(
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40a99e98);
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40a99e98);
# 118 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40a99e98);
# 80 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId(void );
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask(void );
# 7 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430I2C.nc"
static void HplMsp430I2C0P__HplI2C__clearModeI2C(void );
#line 6
static bool HplMsp430I2C0P__HplI2C__isI2C(void );
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__fired(void );
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__overflow(void );
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__startAt(/*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__size_type t0, /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__size_type dt);





static /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Alarm__size_type /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Alarm__getNow(void );
#line 92
static void /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Alarm__startAt(/*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Alarm__size_type t0, /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Alarm__size_type dt);
#line 55
static void /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Alarm__start(/*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Alarm__size_type dt);
#line 67
static void /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__AlarmFrom__fired(void );
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
static void /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Counter__overflow(void );
# 8 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420sfd.nc"
static void CC2420ReceiveP__CC2420sfd__sfd(uint32_t time);
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void CC2420ReceiveP__taskReceiveDone__runTask(void );
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
static error_t CC2420ReceiveP__Init__init(void );
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static void CC2420ReceiveP__SpiResource__granted(void );
# 91 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420ReceiveP__RXFIFO__writeDone(uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420ReceiveP__RXFIFO__readDone(uint8_t * data, uint8_t length, error_t error);
# 3 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/StdAsyncControl.nc"
static error_t CC2420ReceiveP__StdAsyncControl__start(void );
static error_t CC2420ReceiveP__StdAsyncControl__stop(void );
# 57 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioInterrupt.nc"
static void CC2420ReceiveP__InterruptFIFOP__fired(void );
# 50 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioCapture.nc"
static void CC2420TransmitP__CaptureSFD__captured(uint16_t time);
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Transmit.nc"
static error_t CC2420TransmitP__CC2420Transmit__sendNow(bool useCca);
#line 2
static error_t CC2420TransmitP__CC2420Transmit__loadPacket(OpenQueueEntry_t *msg);
# 24 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420TransmitP__ChipSpiResource__releasing(void );
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
static error_t CC2420TransmitP__Init__init(void );
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static void CC2420TransmitP__SpiResource__granted(void );
# 3 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/StdAsyncControl.nc"
static error_t CC2420TransmitP__StdAsyncControl__start(void );
static error_t CC2420TransmitP__StdAsyncControl__stop(void );
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void CC2420TransmitP__AlarmWatchdog__fired(void );
# 91 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420TransmitP__TXFIFO__writeDone(uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420TransmitP__TXFIFO__readDone(uint8_t * data, uint8_t length, error_t error);
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Compare__fired(void );
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Timer__overflow(void );
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__startAt(/*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__size_type t0, /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__size_type dt);
#line 62
static void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__stop(void );
#line 98
static /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__size_type /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__getNow(void );
#line 92
static void /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__startAt(/*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__size_type t0, /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__size_type dt);
#line 55
static void /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__start(/*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__size_type dt);






static void /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__stop(void );




static void /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__AlarmFrom__fired(void );
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
static void /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Counter__overflow(void );
# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time);
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioCapture.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge(void );
#line 55
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable(void );
#line 42
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge(void );
# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__HplInterrupt__fired(void );
# 50 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioInterrupt.nc"
static error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__Interrupt__disable(void );
#line 42
static error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__Interrupt__enableRisingEdge(void );
# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__HplInterrupt__fired(void );
# 50 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioInterrupt.nc"
static error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__Interrupt__disable(void );
#line 43
static error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__Interrupt__enableFallingEdge(void );
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430Compare__fired(void );
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430Timer__overflow(void );
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Alarm__startAt(/*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Alarm__size_type t0, /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Alarm__size_type dt);
#line 62
static void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Alarm__stop(void );
#line 98
static /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__size_type /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__getNow(void );
#line 92
static void /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__startAt(/*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__size_type t0, /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__size_type dt);
#line 55
static void /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__start(/*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__size_type dt);






static void /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__stop(void );




static void /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__AlarmFrom__fired(void );
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
static void /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Counter__overflow(void );
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
static error_t OpenQueueP__SoftwareInit__init(void );
# 5 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/Malloc.nc"
static error_t OpenQueueP__Malloc__freePacketBuffer(OpenQueueEntry_t *pkt);
#line 4
static OpenQueueEntry_t *OpenQueueP__Malloc__getFreePacketBuffer(void );
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void OpenQueueP__taskDebugPrint__runTask(void );
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/OpenQueue.nc"
static OpenQueueEntry_t *OpenQueueP__OpenQueue__inQueueBySeed(uint8_t seedNumber);
static OpenQueueEntry_t *OpenQueueP__OpenQueue__inQueueToChild(uint8_t seedNumber);
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
static error_t PlatformP__MoteInit__init(void );
#line 51
static error_t PlatformP__MoteClockInit__init(void );
#line 51
static error_t PlatformP__LedsInit__init(void );
# 10 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void );
# 6 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__uwait(uint16_t u);




static __inline void MotePlatformC__TOSH_wait(void );




static void MotePlatformC__TOSH_FLASH_M25P_DP_bit(bool set);










static inline void MotePlatformC__TOSH_FLASH_M25P_DP(void );
#line 56
static inline error_t MotePlatformC__Init__init(void );
# 32 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__initTimerB(void );
#line 31
static void Msp430ClockP__Msp430ClockInit__initTimerA(void );
#line 29
static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__initClocks(void );
# 40 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430ClockP.nc"
static volatile uint8_t Msp430ClockP__IE1 __asm ("0x0000");
static volatile uint16_t Msp430ClockP__TA0CTL __asm ("0x0160");
static volatile uint16_t Msp430ClockP__TA0IV __asm ("0x012E");
static volatile uint16_t Msp430ClockP__TBCTL __asm ("0x0180");
static volatile uint16_t Msp430ClockP__TBIV __asm ("0x011E");

enum Msp430ClockP____nesc_unnamed4355 {

  Msp430ClockP__ACLK_CALIB_PERIOD = 8, 
  Msp430ClockP__TARGET_DCO_DELTA = 4096 / 32 * Msp430ClockP__ACLK_CALIB_PERIOD
};

static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );



static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void );
#line 68
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 89
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 104
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 119
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );





static inline void Msp430ClockP__startTimerA(void );
#line 152
static inline void Msp430ClockP__startTimerB(void );
#line 164
static void Msp430ClockP__set_dco_calib(int calib);





static inline uint16_t Msp430ClockP__test_calib_busywait_delta(int calib);
#line 193
static inline void Msp430ClockP__busyCalibrateDco(void );
#line 218
static inline error_t Msp430ClockP__Init__init(void );
# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(
# 40 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4062c4f0);
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void );
# 115 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n);
# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(
# 40 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4062c4f0);
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void );
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
#line 70
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
#line 115
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );








static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n);
# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time);
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void );
# 44 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t;


static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time);
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void );
# 44 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t;


static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time);
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void );
# 44 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t;


static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time);
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void );
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void );
# 44 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void );
#line 74
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );
#line 119
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x);
#line 169
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time);
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void );
# 44 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)  ;
#line 61
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__captureControl(uint8_t l_cm);
#line 74
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void );
#line 99
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(uint8_t cm);
#line 119
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 164
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void );




static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
#line 181
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time);
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void );
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get(void );
# 44 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t;


static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void );
#line 119
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t x);
#line 169
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time);
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void );
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__get(void );
# 44 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t;


static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__clearPendingInterrupt(void );
#line 119
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEventFromNow(uint16_t x);
#line 169
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time);
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void );
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__get(void );
# 44 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t;


static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__clearPendingInterrupt(void );
#line 119
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__setEventFromNow(uint16_t x);
#line 169
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time);
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void );
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__get(void );
# 44 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t;


static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__clearPendingInterrupt(void );
#line 119
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__setEventFromNow(uint16_t x);
#line 169
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time);
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void );
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__get(void );
# 44 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t;


static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__clearPendingInterrupt(void );
#line 119
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__setEventFromNow(uint16_t x);
#line 169
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void Msp430TimerCommonP__VectorTimerB1__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerA0__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerA1__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerB0__fired(void );
# 11 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
void sig_TIMERA0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(12)))  ;
void sig_TIMERA1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(10)))  ;
void sig_TIMERB0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(26)))  ;
void sig_TIMERB1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(24)))  ;
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void );
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/McuSleepC.nc"
bool McuSleepC__dirty = TRUE;
mcu_power_t McuSleepC__powerState = MSP430_POWER_ACTIVE;




const uint16_t McuSleepC__msp430PowerBits[MSP430_POWER_LPM4 + 1] = { 
0, 
0x0010, 
0x0040 + 0x0010, 
0x0080 + 0x0010, 
0x0080 + 0x0040 + 0x0010, 
0x0080 + 0x0040 + 0x0020 + 0x0010 };


static inline mcu_power_t McuSleepC__getPowerState(void );
#line 104
static inline void McuSleepC__computePowerState(void );




static inline void McuSleepC__McuSleep__sleep(void );
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
static error_t RealMainP__SoftwareInit__init(void );
# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Boot.nc"
static void RealMainP__Boot__booted(void );
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
static error_t RealMainP__PlatformInit__init(void );
# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Scheduler.nc"
static void RealMainP__Scheduler__init(void );
#line 61
static void RealMainP__Scheduler__taskLoop(void );
#line 54
static bool RealMainP__Scheduler__runNextTask(void );
# 52 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/RealMainP.nc"
int main(void )   ;
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x4059ab40);
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP__McuSleep__sleep(void );
# 50 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4356 {

  SchedulerBasicP__NUM_TASKS = 32U, 
  SchedulerBasicP__NO_TASK = 255
};

uint8_t SchedulerBasicP__m_head;
uint8_t SchedulerBasicP__m_tail;
uint8_t SchedulerBasicP__m_next[SchedulerBasicP__NUM_TASKS];








static __inline uint8_t SchedulerBasicP__popTask(void );
#line 86
static inline bool SchedulerBasicP__isWaiting(uint8_t id);




static inline bool SchedulerBasicP__pushTask(uint8_t id);
#line 113
static inline void SchedulerBasicP__Scheduler__init(void );









static bool SchedulerBasicP__Scheduler__runNextTask(void );
#line 138
static inline void SchedulerBasicP__Scheduler__taskLoop(void );
#line 159
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id);




static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id);
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Send.nc"
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__send(
#line 56
message_t * msg, 







uint8_t len);
# 99 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AMSend.nc"
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__sendDone(
# 36 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x407205f8, 
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__receive(
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x4072f010, 
# 60 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialActiveMessageP.nc"
static inline serial_header_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(message_t * msg);







static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(am_id_t id, am_addr_t dest, 
message_t *msg, 
uint8_t len);
#line 90
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(message_t *msg, error_t result);







static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(uint8_t id, message_t *msg, void *payload, uint8_t len);



static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(message_t *msg, void *payload, uint8_t len);








static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__payloadLength(message_t *msg);




static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__setPayloadLength(message_t *msg, uint8_t len);



static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength(void );



static void */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__getPayload(message_t *msg, uint8_t len);
#line 137
static am_addr_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__destination(message_t *amsg);









static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setDestination(message_t *amsg, am_addr_t addr);
#line 161
static am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(message_t *amsg);




static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setType(message_t *amsg, am_id_t type);
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/SplitControl.nc"
static void SerialP__SplitControl__startDone(error_t error);
#line 117
static void SerialP__SplitControl__stopDone(error_t error);
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t SerialP__stopDoneTask__postTask(void );
# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/StdControl.nc"
static error_t SerialP__SerialControl__start(void );









static error_t SerialP__SerialControl__stop(void );
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t SerialP__RunTx__postTask(void );
# 38 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialFlush.nc"
static void SerialP__SerialFlush__flush(void );
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t SerialP__startDoneTask__postTask(void );
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialFrameComm.nc"
static error_t SerialP__SerialFrameComm__putDelimiter(void );
#line 68
static void SerialP__SerialFrameComm__resetReceive(void );
#line 54
static error_t SerialP__SerialFrameComm__putData(uint8_t data);
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t SerialP__defaultSerialFlushTask__postTask(void );
# 70 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SendBytePacket.nc"
static uint8_t SerialP__SendBytePacket__nextByte(void );









static void SerialP__SendBytePacket__sendCompleted(error_t error);
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/ReceiveBytePacket.nc"
static error_t SerialP__ReceiveBytePacket__startPacket(void );






static void SerialP__ReceiveBytePacket__byteReceived(uint8_t data);










static void SerialP__ReceiveBytePacket__endPacket(error_t result);
# 189 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialP.nc"
enum SerialP____nesc_unnamed4357 {
#line 189
  SerialP__RunTx = 0U
};
#line 189
typedef int SerialP____nesc_sillytask_RunTx[SerialP__RunTx];
#line 320
enum SerialP____nesc_unnamed4358 {
#line 320
  SerialP__startDoneTask = 1U
};
#line 320
typedef int SerialP____nesc_sillytask_startDoneTask[SerialP__startDoneTask];





enum SerialP____nesc_unnamed4359 {
#line 326
  SerialP__stopDoneTask = 2U
};
#line 326
typedef int SerialP____nesc_sillytask_stopDoneTask[SerialP__stopDoneTask];








enum SerialP____nesc_unnamed4360 {
#line 335
  SerialP__defaultSerialFlushTask = 3U
};
#line 335
typedef int SerialP____nesc_sillytask_defaultSerialFlushTask[SerialP__defaultSerialFlushTask];
#line 79
enum SerialP____nesc_unnamed4361 {
  SerialP__RX_DATA_BUFFER_SIZE = 2, 
  SerialP__TX_DATA_BUFFER_SIZE = 4, 
  SerialP__SERIAL_MTU = 255, 
  SerialP__SERIAL_VERSION = 1, 
  SerialP__ACK_QUEUE_SIZE = 5
};

enum SerialP____nesc_unnamed4362 {
  SerialP__RXSTATE_NOSYNC, 
  SerialP__RXSTATE_PROTO, 
  SerialP__RXSTATE_TOKEN, 
  SerialP__RXSTATE_INFO, 
  SerialP__RXSTATE_INACTIVE
};

enum SerialP____nesc_unnamed4363 {
  SerialP__TXSTATE_IDLE, 
  SerialP__TXSTATE_PROTO, 
  SerialP__TXSTATE_SEQNO, 
  SerialP__TXSTATE_INFO, 
  SerialP__TXSTATE_FCS1, 
  SerialP__TXSTATE_FCS2, 
  SerialP__TXSTATE_ENDFLAG, 
  SerialP__TXSTATE_ENDWAIT, 
  SerialP__TXSTATE_FINISH, 
  SerialP__TXSTATE_ERROR, 
  SerialP__TXSTATE_INACTIVE
};





#line 109
typedef enum SerialP____nesc_unnamed4364 {
  SerialP__BUFFER_AVAILABLE, 
  SerialP__BUFFER_FILLING, 
  SerialP__BUFFER_COMPLETE
} SerialP__tx_data_buffer_states_t;

enum SerialP____nesc_unnamed4365 {
  SerialP__TX_ACK_INDEX = 0, 
  SerialP__TX_DATA_INDEX = 1, 
  SerialP__TX_BUFFER_COUNT = 2
};






#line 122
typedef struct SerialP____nesc_unnamed4366 {
  uint8_t writePtr;
  uint8_t readPtr;
  uint8_t buf[SerialP__RX_DATA_BUFFER_SIZE + 1];
} SerialP__rx_buf_t;




#line 128
typedef struct SerialP____nesc_unnamed4367 {
  uint8_t state;
  uint8_t buf;
} SerialP__tx_buf_t;





#line 133
typedef struct SerialP____nesc_unnamed4368 {
  uint8_t writePtr;
  uint8_t readPtr;
  uint8_t buf[SerialP__ACK_QUEUE_SIZE + 1];
} SerialP__ack_queue_t;



SerialP__rx_buf_t SerialP__rxBuf;
SerialP__tx_buf_t SerialP__txBuf[SerialP__TX_BUFFER_COUNT];



uint8_t SerialP__rxState;
uint8_t SerialP__rxByteCnt;
uint8_t SerialP__rxProto;
uint8_t SerialP__rxSeqno;
uint16_t SerialP__rxCRC;



uint8_t SerialP__txState;
uint8_t SerialP__txByteCnt;
uint8_t SerialP__txProto;
uint8_t SerialP__txSeqno;
uint16_t SerialP__txCRC;
uint8_t SerialP__txPending;
uint8_t SerialP__txIndex;


SerialP__ack_queue_t SerialP__ackQ;

bool SerialP__offPending = FALSE;



static __inline void SerialP__txInit(void );
static __inline void SerialP__rxInit(void );
static __inline void SerialP__ackInit(void );

static __inline bool SerialP__ack_queue_is_full(void );
static __inline bool SerialP__ack_queue_is_empty(void );
static __inline void SerialP__ack_queue_push(uint8_t token);
static __inline uint8_t SerialP__ack_queue_top(void );
static inline uint8_t SerialP__ack_queue_pop(void );




static __inline void SerialP__rx_buffer_push(uint8_t data);
static __inline uint8_t SerialP__rx_buffer_top(void );
static __inline uint8_t SerialP__rx_buffer_pop(void );
static __inline uint16_t SerialP__rx_current_crc(void );

static void SerialP__rx_state_machine(bool isDelimeter, uint8_t data);
static void SerialP__MaybeScheduleTx(void );




static __inline void SerialP__txInit(void );
#line 205
static __inline void SerialP__rxInit(void );








static __inline void SerialP__ackInit(void );



static inline error_t SerialP__Init__init(void );
#line 232
static __inline bool SerialP__ack_queue_is_full(void );









static __inline bool SerialP__ack_queue_is_empty(void );





static __inline void SerialP__ack_queue_push(uint8_t token);









static __inline uint8_t SerialP__ack_queue_top(void );









static inline uint8_t SerialP__ack_queue_pop(void );
#line 295
static __inline void SerialP__rx_buffer_push(uint8_t data);



static __inline uint8_t SerialP__rx_buffer_top(void );



static __inline uint8_t SerialP__rx_buffer_pop(void );





static __inline uint16_t SerialP__rx_current_crc(void );










static inline void SerialP__startDoneTask__runTask(void );





static inline void SerialP__stopDoneTask__runTask(void );



static inline void SerialP__SerialFlush__flushDone(void );




static inline void SerialP__defaultSerialFlushTask__runTask(void );


static inline void SerialP__SerialFlush__default__flush(void );



static inline error_t SerialP__SplitControl__start(void );




static void SerialP__testOff(void );
#line 384
static inline void SerialP__SerialFrameComm__delimiterReceived(void );


static inline void SerialP__SerialFrameComm__dataReceived(uint8_t data);



static inline bool SerialP__valid_rx_proto(uint8_t proto);










static void SerialP__rx_state_machine(bool isDelimeter, uint8_t data);
#line 502
static void SerialP__MaybeScheduleTx(void );










static inline error_t SerialP__SendBytePacket__completeSend(void );








static inline error_t SerialP__SendBytePacket__startSend(uint8_t b);
#line 539
static inline void SerialP__RunTx__runTask(void );
#line 642
static inline void SerialP__SerialFrameComm__putDone(void );
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__postTask(void );
# 89 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Send.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__sendDone(
# 40 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x407d0840, 
# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__postTask(void );
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__receive(
# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x407d0200, 
# 60 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 31 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__upperLength(
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x407cf398, 
# 31 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x407cf398);
# 23 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__dataLinkLength(
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x407cf398, 
# 23 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t upperLen);
# 60 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SendBytePacket.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__completeSend(void );
#line 51
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__startSend(uint8_t first_byte);
# 147 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4369 {
#line 147
  SerialDispatcherP__0__signalSendDone = 4U
};
#line 147
typedef int /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_sillytask_signalSendDone[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone];
#line 264
enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4370 {
#line 264
  SerialDispatcherP__0__receiveTask = 5U
};
#line 264
typedef int /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_sillytask_receiveTask[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask];
#line 55
#line 51
typedef enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4371 {
  SerialDispatcherP__0__SEND_STATE_IDLE = 0, 
  SerialDispatcherP__0__SEND_STATE_BEGIN = 1, 
  SerialDispatcherP__0__SEND_STATE_DATA = 2
} /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__send_state_t;

enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4372 {
  SerialDispatcherP__0__RECV_STATE_IDLE = 0, 
  SerialDispatcherP__0__RECV_STATE_BEGIN = 1, 
  SerialDispatcherP__0__RECV_STATE_DATA = 2
};






#line 63
typedef struct /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4373 {
  uint8_t which : 1;
  uint8_t bufZeroLocked : 1;
  uint8_t bufOneLocked : 1;
  uint8_t state : 2;
} /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recv_state_t;



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recv_state_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState = { 0, 0, 0, /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_IDLE };
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType = TOS_SERIAL_UNKNOWN_ID;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex = 0;


message_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[2];
message_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messagePtrs[2] = { &/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[0], &/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[1] };




uint8_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer = (uint8_t * )&/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[0];

uint8_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer = (void *)0;
/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__send_state_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendLen = 0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex = 0;
error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError = SUCCESS;
bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendCancelled = FALSE;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendId = 0;


uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending = FALSE;
uart_id_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskType = 0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskWhich;
message_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskBuf = (void *)0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskSize = 0;

static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(uint8_t id, message_t *msg, uint8_t len);
#line 147
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask(void );
#line 167
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte(void );
#line 183
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error_t error);




static inline bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__isCurrentBufferLocked(void );



static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__lockCurrentBuffer(void );








static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(uint8_t which);








static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBufferSwap(void );




static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket(void );
#line 233
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(uint8_t b);
#line 264
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask(void );
#line 285
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(error_t result);
#line 347
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(uart_id_t id);


static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(uart_id_t id, message_t *msg, 
uint8_t upperLen);


static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(uart_id_t id, message_t *msg, 
uint8_t dataLinkLen);




static inline message_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(uart_id_t idxxx, message_t *msg, 
void *payload, 
uint8_t len);


static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(uart_id_t idxxx, message_t *msg, error_t error);
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/UartStream.nc"
static error_t HdlcTranslateC__UartStream__send(
#line 44
uint8_t * buf, 



uint16_t len);
# 83 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialFrameComm.nc"
static void HdlcTranslateC__SerialFrameComm__dataReceived(uint8_t data);





static void HdlcTranslateC__SerialFrameComm__putDone(void );
#line 74
static void HdlcTranslateC__SerialFrameComm__delimiterReceived(void );
# 47 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/HdlcTranslateC.nc"
#line 44
typedef struct HdlcTranslateC____nesc_unnamed4374 {
  uint8_t sendEscape : 1;
  uint8_t receiveEscape : 1;
} HdlcTranslateC__HdlcState;


HdlcTranslateC__HdlcState HdlcTranslateC__state = { 0, 0 };
uint8_t HdlcTranslateC__txTemp;
uint8_t HdlcTranslateC__m_data;


static inline void HdlcTranslateC__SerialFrameComm__resetReceive(void );





static inline void HdlcTranslateC__UartStream__receivedByte(uint8_t data);
#line 86
static error_t HdlcTranslateC__SerialFrameComm__putDelimiter(void );





static error_t HdlcTranslateC__SerialFrameComm__putData(uint8_t data);
#line 104
static void HdlcTranslateC__UartStream__sendDone(uint8_t *buf, uint16_t len, 
error_t error);










static inline void HdlcTranslateC__UartStream__receiveDone(uint8_t *buf, uint16_t len, error_t error);
# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartConfigure.nc"
static msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__getConfig(
# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408746c0);
# 97 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__resetUsart(bool reset);
#line 179
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableIntr(void );


static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__enableIntr(void );
#line 224
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__tx(uint8_t data);
#line 128
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableUart(void );
#line 174
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__setModeUart(msp430_uart_union_config_t *config);
# 79 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/UartStream.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receivedByte(
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40877770, 
# 79 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/UartStream.nc"
uint8_t byte);
#line 99
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receiveDone(
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40877770, 
# 95 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__sendDone(
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40877770, 
# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__release(
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40875bb0);
# 87 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__immediateRequest(
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40875bb0);
# 118 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__isOwner(
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40875bb0);
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__granted(
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408785a8);
#line 59
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len;
#line 59
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_len;
uint8_t * /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf;
#line 60
uint8_t * /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf;
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_pos;
#line 61
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_pos;
uint8_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_byte_time;
uint8_t /*Msp430Uart1P.UartP*/Msp430UartP__0__current_owner;

static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__immediateRequest(uint8_t id);
#line 77
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__release(uint8_t id);







static void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__configure(uint8_t id);






static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(uint8_t id);








static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__granted(uint8_t id);
#line 134
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__rxDone(uint8_t id, uint8_t data);
#line 147
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__send(uint8_t id, uint8_t *buf, uint16_t len);
#line 162
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__txDone(uint8_t id);
#line 208
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__overflow(void );

static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__isOwner(uint8_t id);

static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__immediateRequest(uint8_t id);
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__release(uint8_t id);
static inline msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(uint8_t id);



static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__default__granted(uint8_t id);

static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__sendDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error);
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(uint8_t id, uint8_t byte);
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error);
# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart1P__UCLK__selectIOFunc(void );
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void HplMsp430Usart1P__Interrupts__rxDone(uint8_t data);
#line 49
static void HplMsp430Usart1P__Interrupts__txDone(void );
# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart1P__URXD__selectIOFunc(void );
#line 78
static void HplMsp430Usart1P__URXD__selectModuleFunc(void );






static void HplMsp430Usart1P__UTXD__selectIOFunc(void );
#line 78
static void HplMsp430Usart1P__UTXD__selectModuleFunc(void );






static void HplMsp430Usart1P__SOMI__selectIOFunc(void );
#line 85
static void HplMsp430Usart1P__SIMO__selectIOFunc(void );
# 87 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static volatile uint8_t HplMsp430Usart1P__IE2 __asm ("0x0001");
static volatile uint8_t HplMsp430Usart1P__ME2 __asm ("0x0005");
static volatile uint8_t HplMsp430Usart1P__IFG2 __asm ("0x0003");
static volatile uint8_t HplMsp430Usart1P__U1TCTL __asm ("0x0079");
static volatile uint8_t HplMsp430Usart1P__U1RCTL __asm ("0x007A");
static volatile uint8_t HplMsp430Usart1P__U1TXBUF __asm ("0x007F");



void sig_UART1RX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(6)))  ;




void sig_UART1TX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(4)))  ;



static inline error_t HplMsp430Usart1P__AsyncStdControl__start(void );



static inline error_t HplMsp430Usart1P__AsyncStdControl__stop(void );
#line 140
static inline void HplMsp430Usart1P__Usart__setUbr(uint16_t control);










static inline void HplMsp430Usart1P__Usart__setUmctl(uint8_t control);







static inline void HplMsp430Usart1P__Usart__resetUsart(bool reset);
#line 203
static inline void HplMsp430Usart1P__Usart__enableUart(void );







static void HplMsp430Usart1P__Usart__disableUart(void );








static inline void HplMsp430Usart1P__Usart__enableUartTx(void );




static inline void HplMsp430Usart1P__Usart__disableUartTx(void );





static inline void HplMsp430Usart1P__Usart__enableUartRx(void );




static inline void HplMsp430Usart1P__Usart__disableUartRx(void );
#line 251
static void HplMsp430Usart1P__Usart__disableSpi(void );
#line 283
static inline void HplMsp430Usart1P__configUart(msp430_uart_union_config_t *config);









static inline void HplMsp430Usart1P__Usart__setModeUart(msp430_uart_union_config_t *config);
#line 347
static inline void HplMsp430Usart1P__Usart__clrIntr(void );
#line 359
static inline void HplMsp430Usart1P__Usart__disableIntr(void );
#line 377
static inline void HplMsp430Usart1P__Usart__enableIntr(void );






static inline void HplMsp430Usart1P__Usart__tx(uint8_t data);
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void );
#line 48
static inline uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void );
#line 48
static inline uint8_t /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void );
#line 54
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void );
#line 54
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void );
#line 54
static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc(void );
#line 54
static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc(void );
#line 48
static inline uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc(void );
#line 45
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set(void );
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void );
#line 45
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void );
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void );



static inline void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );






static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 45
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
#line 52
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__makeOutput(void );
#line 52
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__makeOutput(void );
#line 52
static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__makeOutput(void );
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void );
static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void );
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void );
# 38 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );




static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );









static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 35 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
static void LedsP__Led0__makeOutput(void );
#line 29
static void LedsP__Led0__set(void );
static void LedsP__Led0__clr(void );




static void LedsP__Led1__makeOutput(void );
#line 29
static void LedsP__Led1__set(void );





static void LedsP__Led2__makeOutput(void );
#line 29
static void LedsP__Led2__set(void );
static void LedsP__Led2__clr(void );
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void );
#line 63
static inline void LedsP__Leds__led0On(void );
#line 93
static inline void LedsP__Leds__led2On(void );




static inline void LedsP__Leds__led2Off(void );
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void );
#line 34
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void );




static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__clr(void );
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr(void );




static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void );
#line 34
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void );
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );





static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void );
#line 34
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void );




static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr(void );
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void );




static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
# 80 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(void );
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(
# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40a89010, 
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(
# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40a89010);









static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );




static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);









static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id);
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data);
# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0____nesc_unnamed4375 {
#line 39
  FcfsResourceQueueC__0__NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[1U];
uint8_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
uint8_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

static inline error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void );




static inline bool /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );







static inline resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40aac948);
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(
# 60 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40aaacf8);
# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(
# 60 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40aaacf8);
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceQueue.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void );
#line 60
static resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void );
# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void );
#line 81
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void );
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40a99e98);
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void );
# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4376 {
#line 75
  ArbiterP__0__grantedTask = 6U
};
#line 75
typedef int /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_sillytask_grantedTask[/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask];
#line 67
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4377 {
#line 67
  ArbiterP__0__RES_CONTROLLED, ArbiterP__0__RES_GRANTING, ArbiterP__0__RES_IMM_GRANTING, ArbiterP__0__RES_BUSY
};
#line 68
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4378 {
#line 68
  ArbiterP__0__default_owner_id = 1U
};
#line 69
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4379 {
#line 69
  ArbiterP__0__NO_RES = 0xFF
};
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
#line 90
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id);
#line 108
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id);
#line 130
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
#line 150
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );
#line 163
static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );










static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id);
#line 187
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
#line 199
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id);



static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id);









static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id);

static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id);
# 52 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/power/PowerDownCleanup.nc"
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__cleanup(void );
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__release(void );
# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AsyncStdControl.nc"
static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__start(void );









static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__stop(void );
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested(void );




static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted(void );




static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup(void );
# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t TelosSerialP__Resource__release(void );
#line 87
static error_t TelosSerialP__Resource__immediateRequest(void );
# 8 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosa/TelosSerialP.nc"
msp430_uart_union_config_t TelosSerialP__msp430_uart_telos_config = { { .ubr = UBR_1MHZ_115200, .umctl = UMCTL_1MHZ_115200, .ssel = 0x02, .pena = 0, .pev = 0, .spb = 0, .clen = 1, .listen = 0, .mm = 0, .ckpl = 0, .urxse = 0, .urxeie = 1, .urxwie = 0, .utxe = 1, .urxe = 1 } };

static inline error_t TelosSerialP__StdControl__start(void );


static inline error_t TelosSerialP__StdControl__stop(void );



static inline void TelosSerialP__Resource__granted(void );

static inline msp430_uart_union_config_t *TelosSerialP__Msp430UartConfigure__getConfig(void );
# 40 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__offset(void );


static inline uint8_t SerialPacketInfoActiveMessageP__Info__dataLinkLength(message_t *msg, uint8_t upperLen);


static inline uint8_t SerialPacketInfoActiveMessageP__Info__upperLength(message_t *msg, uint8_t dataLinkLen);
# 99 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AMSend.nc"
static void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Send.nc"
static error_t /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(
#line 56
message_t * msg, 







uint8_t len);
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AMPacket.nc"
static void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 151
static void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/AMQueueEntryP.nc"
static error_t /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len);









static inline void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(message_t *m, error_t err);
# 69 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AMSend.nc"
static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(
# 40 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/AMQueueImplP.nc"
am_id_t arg_0x40b1e908, 
# 69 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 89 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Send.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(
# 38 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/AMQueueImplP.nc"
uint8_t arg_0x40afbe70, 
# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Packet.nc"
static uint8_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(
#line 63
message_t * msg);
#line 83
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(
#line 79
message_t * msg, 



uint8_t len);
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask(void );
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AMPacket.nc"
static am_addr_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(
#line 63
message_t * amsg);
#line 136
static am_id_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(
#line 132
message_t * amsg);
# 118 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/AMQueueImplP.nc"
enum /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4380 {
#line 118
  AMQueueImplP__0__CancelTask = 7U
};
#line 118
typedef int /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_CancelTask[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask];
#line 161
enum /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4381 {
#line 161
  AMQueueImplP__0__errorTask = 8U
};
#line 161
typedef int /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_errorTask[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask];
#line 49
#line 47
typedef struct /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4382 {
  message_t * msg;
} /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue_entry_t;

uint8_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue_entry_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[1];
uint8_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[1 / 8 + 1];

static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void );

static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket(void );
#line 82
static inline error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(uint8_t clientId, message_t *msg, 
uint8_t len);
#line 118
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void );
#line 155
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(uint8_t last, message_t * msg, error_t err);





static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void );




static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void );
#line 181
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(am_id_t id, message_t *msg, error_t err);
#line 207
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(uint8_t id, message_t *msg, error_t err);
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/BigQueueC.nc"
/*PrintfC.QueueC*/BigQueueC__0__queue_t /*PrintfC.QueueC*/BigQueueC__0__queue[1000];
uint16_t /*PrintfC.QueueC*/BigQueueC__0__head = 0;
uint16_t /*PrintfC.QueueC*/BigQueueC__0__tail = 0;
uint16_t /*PrintfC.QueueC*/BigQueueC__0__size = 0;

static inline bool /*PrintfC.QueueC*/BigQueueC__0__Queue__empty(void );



static inline uint16_t /*PrintfC.QueueC*/BigQueueC__0__Queue__size(void );



static inline uint16_t /*PrintfC.QueueC*/BigQueueC__0__Queue__maxSize(void );



static inline /*PrintfC.QueueC*/BigQueueC__0__queue_t /*PrintfC.QueueC*/BigQueueC__0__Queue__head(void );



static inline void /*PrintfC.QueueC*/BigQueueC__0__printQueue(void );
#line 85
static inline /*PrintfC.QueueC*/BigQueueC__0__queue_t /*PrintfC.QueueC*/BigQueueC__0__Queue__dequeue(void );
#line 97
static inline error_t /*PrintfC.QueueC*/BigQueueC__0__Queue__enqueue(/*PrintfC.QueueC*/BigQueueC__0__queue_t newVal);
# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Boot.nc"
static void PrintfP__Boot__booted(void );
# 83 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/SplitControl.nc"
static error_t PrintfP__SerialControl__start(void );
# 90 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/BigQueue.nc"
static error_t PrintfP__Queue__enqueue(PrintfP__Queue__t newVal);
#line 81
static PrintfP__Queue__t PrintfP__Queue__dequeue(void );
#line 50
static bool PrintfP__Queue__empty(void );







static uint16_t PrintfP__Queue__size(void );
# 69 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AMSend.nc"
static error_t PrintfP__AMSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 115 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Packet.nc"
static 
#line 112
void * 


PrintfP__Packet__getPayload(
#line 110
message_t * msg, 




uint8_t len);
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t PrintfP__retrySend__postTask(void );
# 127 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/printf/PrintfP.nc"
enum PrintfP____nesc_unnamed4383 {
#line 127
  PrintfP__retrySend = 9U
};
#line 127
typedef int PrintfP____nesc_sillytask_retrySend[PrintfP__retrySend];
#line 100
enum PrintfP____nesc_unnamed4384 {
  PrintfP__S_STOPPED, 
  PrintfP__S_STARTED, 
  PrintfP__S_FLUSHING
};

message_t PrintfP__printfMsg;
uint8_t PrintfP__state = PrintfP__S_STOPPED;

static inline void PrintfP__MainBoot__booted(void );



static inline void PrintfP__SerialControl__startDone(error_t error);









static inline void PrintfP__SerialControl__stopDone(error_t error);



static inline void PrintfP__retrySend__runTask(void );




static void PrintfP__sendNext(void );










int printfflush(void )   ;
#line 155
static void PrintfP__AMSend__sendDone(message_t *msg, error_t error);









int putchar(int c) __attribute((noinline))   ;
# 32 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/07-App/appDATA/Device/appDATADevP.nc"
static inline error_t appDATADevP__SoftwareInit__init(void );




static inline void appDATADevP__OpenSendToLower__sendDone(OpenQueueEntry_t *msg, error_t error);
# 20 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/packetfunctions/PacketFunctionsP.nc"
static inline void PacketFunctionsP__PacketFunctions__ip128bToMac64b(
open_addr_t *ip128b, 
open_addr_t *prefix64btoWrite, 
open_addr_t *mac64btoWrite);
#line 36
static void PacketFunctionsP__PacketFunctions__mac64bToIp128b(
open_addr_t *prefix64b, 
open_addr_t *mac64b, 
open_addr_t *ip128bToWrite);
#line 53
static void PacketFunctionsP__PacketFunctions__mac64bToMac16b(open_addr_t *mac64b, open_addr_t *mac16btoWrite);
#line 65
static inline void PacketFunctionsP__PacketFunctions__mac16bToMac64b(open_addr_t *mac16b, open_addr_t *mac64btoWrite);
#line 86
static bool PacketFunctionsP__PacketFunctions__isBroadcastMulticast(open_addr_t *address);
#line 121
static bool PacketFunctionsP__PacketFunctions__sameAddress(open_addr_t *address_1, open_addr_t *address_2);
#line 152
static void PacketFunctionsP__PacketFunctions__readAddress(uint8_t *payload, uint8_t type, open_addr_t *writeToAddress, bool littleEndian);
#line 183
static void PacketFunctionsP__PacketFunctions__writeAddress(OpenQueueEntry_t *msg, open_addr_t *address, bool littleEndian);
#line 217
static void PacketFunctionsP__PacketFunctions__reserveHeaderSize(OpenQueueEntry_t *pkt, uint8_t header_length);








static inline void PacketFunctionsP__PacketFunctions__tossHeader(OpenQueueEntry_t *pkt, uint8_t header_length);









static inline void PacketFunctionsP__PacketFunctions__reserveFooterSize(OpenQueueEntry_t *pkt, uint8_t header_length);







static inline void PacketFunctionsP__PacketFunctions__tossFooter(OpenQueueEntry_t *pkt, uint8_t header_length);
# 11 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/PacketFunctions.nc"
static bool IDManagerP__PacketFunctions__sameAddress(open_addr_t *address_1, open_addr_t *address_2);
#line 7
static void IDManagerP__PacketFunctions__mac64bToMac16b(open_addr_t *mac64b, open_addr_t *mac16btoWrite);
# 25 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/idmanager/IDManagerP.nc"
enum IDManagerP____nesc_unnamed4385 {
#line 25
  IDManagerP__taskPrint = 10U
};
#line 25
typedef int IDManagerP____nesc_sillytask_taskPrint[IDManagerP__taskPrint];
enum IDManagerP____nesc_unnamed4386 {
#line 26
  IDManagerP__taskHandleBridge = 11U
};
#line 26
typedef int IDManagerP____nesc_sillytask_taskHandleBridge[IDManagerP__taskHandleBridge];
enum IDManagerP____nesc_unnamed4387 {
#line 27
  IDManagerP__taskHandleRoot = 12U
};
#line 27
typedef int IDManagerP____nesc_sillytask_taskHandleRoot[IDManagerP__taskHandleRoot];
#line 16
bool IDManagerP__isDAGroot;
bool IDManagerP__isBridge;
open_addr_t IDManagerP__my16bID;
open_addr_t IDManagerP__my64bID;
open_addr_t IDManagerP__myPANID;
open_addr_t IDManagerP__myPrefix;
#line 34
static inline void IDManagerP__taskHandleBridge__runTask(void );
#line 69
static inline void IDManagerP__taskHandleRoot__runTask(void );
#line 99
static inline error_t IDManagerP__SoftwareInit__init(void );
#line 139
static bool IDManagerP__IDManager__getIsDAGroot(void );


static inline error_t IDManagerP__IDManager__setIsDAGroot(bool newRole);




static bool IDManagerP__IDManager__getIsBridge(void );


static inline error_t IDManagerP__IDManager__setIsBridge(bool newRole);




static open_addr_t *IDManagerP__IDManager__getMyID(uint8_t type);
#line 199
static bool IDManagerP__IDManager__isMyAddress(open_addr_t *addr);
#line 233
static inline void IDManagerP__taskPrint__runTask(void );
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Notify.nc"
static error_t LatinResP__UserButton__enable(void );
# 2 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/MAC.nc"
static void LatinResP__MAC__changeNetworkState(uint8_t tmp_networkState);
# 5 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/Malloc.nc"
static error_t LatinResP__Malloc__freePacketBuffer(OpenQueueEntry_t *pkt);
#line 4
static OpenQueueEntry_t *LatinResP__Malloc__getFreePacketBuffer(void );
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t LatinResP__taskBuildAndSendReqMsg__postTask(void );
# 81 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Timer.nc"
static bool LatinResP__timerAdvWait__isRunning(void );
#line 62
static void LatinResP__timerAdvWait__startOneShot(uint32_t dt);
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/IDManager.nc"
static bool LatinResP__IDManager__getIsDAGroot(void );



static open_addr_t *LatinResP__IDManager__getMyID(uint8_t type);
#line 6
static bool LatinResP__IDManager__getIsBridge(void );
# 5 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/OpenSend.nc"
static void LatinResP__OpenSendFromUpper__sendDone(OpenQueueEntry_t *msg, error_t error);
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t LatinResP__taskBuildAndSendAdvMsg__postTask(void );
#line 56
static error_t LatinResP__taskBuildAndSendSeedMsgToChildren__postTask(void );
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/OpenSend.nc"
static error_t LatinResP__OpenSendToLower__send(OpenQueueEntry_t *msg);
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t LatinResP__taskBuildAndSendDataToChildren__postTask(void );
# 8 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/PacketFunctions.nc"
static void LatinResP__PacketFunctions__mac16bToMac64b(open_addr_t *mac16b, open_addr_t *mac64btoWrite);
#line 6
static void LatinResP__PacketFunctions__mac64bToIp128b(open_addr_t *prefix64b, open_addr_t *mac64b, open_addr_t *ip128bToWrite);




static bool LatinResP__PacketFunctions__sameAddress(open_addr_t *address_1, open_addr_t *address_2);
#line 7
static void LatinResP__PacketFunctions__mac64bToMac16b(open_addr_t *mac64b, open_addr_t *mac16btoWrite);
#line 5
static void LatinResP__PacketFunctions__ip128bToMac64b(open_addr_t *ip128b, open_addr_t *prefix64btoWrite, open_addr_t *mac64btoWrite);










static void LatinResP__PacketFunctions__reserveHeaderSize(OpenQueueEntry_t *pkt, uint8_t header_length);
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t LatinResP__taskBuildAndSendScanMsg__postTask(void );
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02b-RES/LatinRes/LatinResP.nc"
enum LatinResP____nesc_unnamed4388 {
#line 43
  LatinResP__taskBuildAndSendScanMsg = 13U
};
#line 43
typedef int LatinResP____nesc_sillytask_taskBuildAndSendScanMsg[LatinResP__taskBuildAndSendScanMsg];
enum LatinResP____nesc_unnamed4389 {
#line 44
  LatinResP__taskBuildAndSendAdvMsg = 14U
};
#line 44
typedef int LatinResP____nesc_sillytask_taskBuildAndSendAdvMsg[LatinResP__taskBuildAndSendAdvMsg];
enum LatinResP____nesc_unnamed4390 {
#line 45
  LatinResP__taskBuildAndSendReqMsg = 15U
};
#line 45
typedef int LatinResP____nesc_sillytask_taskBuildAndSendReqMsg[LatinResP__taskBuildAndSendReqMsg];
enum LatinResP____nesc_unnamed4391 {
#line 46
  LatinResP__taskBuildAndSendSeedMsgToChildren = 16U
};
#line 46
typedef int LatinResP____nesc_sillytask_taskBuildAndSendSeedMsgToChildren[LatinResP__taskBuildAndSendSeedMsgToChildren];
enum LatinResP____nesc_unnamed4392 {
#line 47
  LatinResP__taskBuildAndSendDataToChildren = 17U
};
#line 47
typedef int LatinResP____nesc_sillytask_taskBuildAndSendDataToChildren[LatinResP__taskBuildAndSendDataToChildren];
#line 38
uint8_t LatinResP__state;
neighborList_t LatinResP__neighborList;








static inline void LatinResP__taskBuildAndSendConfMsg(OpenQueueEntry_t *msg);
static void LatinResP__updateNeighborList(OpenQueueEntry_t *msg);
static inline void LatinResP__registerNewNeighbor(OpenQueueEntry_t *msg);


static inline void LatinResP__resetNeighborList(void );

static inline bool LatinResP__isThisRowMatching(open_addr_t *address, uint8_t rowNumber);




static inline void LatinResP__taskBuildAndSendScanMsg__runTask(void );
#line 95
static inline void LatinResP__taskBuildAndSendAdvMsg__runTask(void );
#line 129
static inline void LatinResP__taskBuildAndSendReqMsg__runTask(void );
#line 158
static inline void LatinResP__taskBuildAndSendConfMsg(OpenQueueEntry_t *msg);
#line 186
static inline void LatinResP__taskBuildAndSendSeedMsgToChildren__runTask(void );
#line 222
static inline void LatinResP__taskBuildAndSendDataToChildren__runTask(void );
#line 275
static void LatinResP__updateNeighborList(OpenQueueEntry_t *msg);
#line 316
static inline bool LatinResP__isThisRowMatching(open_addr_t *address, uint8_t rowNumber);
#line 334
static inline void LatinResP__registerNewNeighbor(OpenQueueEntry_t *msg);
#line 409
static inline void LatinResP__resetNeighborList(void );
#line 446
static inline error_t LatinResP__SoftwareInit__init(void );
#line 467
static inline void LatinResP__OpenSendToLower__sendDone(OpenQueueEntry_t *msg, error_t error);
#line 538
static inline void LatinResP__OpenReceiveFromLower__receive(OpenQueueEntry_t *msg);
#line 605
static inline bool LatinResP__LatinMatrix__childCanTransmit(uint8_t minLatinWindow, uint8_t *latinChannelOffset);
#line 618
static inline bool LatinResP__LatinMatrix__siblingCanTransmit(uint8_t minLatinWindow, uint8_t *latinChannelOffset);
#line 636
static inline bool LatinResP__LatinMatrix__parentCanTransmit(uint8_t minLatinWindow, uint8_t *latinChannelOffset);










static inline bool LatinResP__LatinMatrix__canTransmit(uint8_t minLatinWindow, uint8_t *latinChannelOffset);










static inline uint8_t LatinResP__LatinMatrix__getParentSeed(void );





static inline void LatinResP__timerAdvWait__fired(void );






static void LatinResP__UserButton__notify(button_state_t button);
# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void HplMsp430InterruptP__Port14__fired(void );
#line 61
static void HplMsp430InterruptP__Port26__fired(void );
#line 61
static void HplMsp430InterruptP__Port17__fired(void );
#line 61
static void HplMsp430InterruptP__Port21__fired(void );
#line 61
static void HplMsp430InterruptP__Port12__fired(void );
#line 61
static void HplMsp430InterruptP__Port24__fired(void );
#line 61
static void HplMsp430InterruptP__Port15__fired(void );
#line 61
static void HplMsp430InterruptP__Port27__fired(void );
#line 61
static void HplMsp430InterruptP__Port10__fired(void );
#line 61
static void HplMsp430InterruptP__Port22__fired(void );
#line 61
static void HplMsp430InterruptP__Port13__fired(void );
#line 61
static void HplMsp430InterruptP__Port25__fired(void );
#line 61
static void HplMsp430InterruptP__Port16__fired(void );
#line 61
static void HplMsp430InterruptP__Port20__fired(void );
#line 61
static void HplMsp430InterruptP__Port11__fired(void );
#line 61
static void HplMsp430InterruptP__Port23__fired(void );
# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
void sig_PORT1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(8)))  ;
#line 68
static inline void HplMsp430InterruptP__Port11__default__fired(void );
static inline void HplMsp430InterruptP__Port12__default__fired(void );
static inline void HplMsp430InterruptP__Port13__default__fired(void );

static inline void HplMsp430InterruptP__Port15__default__fired(void );
static inline void HplMsp430InterruptP__Port16__default__fired(void );
static inline void HplMsp430InterruptP__Port17__default__fired(void );
static inline void HplMsp430InterruptP__Port10__enable(void );



static inline void HplMsp430InterruptP__Port14__enable(void );



static inline void HplMsp430InterruptP__Port10__disable(void );



static inline void HplMsp430InterruptP__Port14__disable(void );



static inline void HplMsp430InterruptP__Port10__clear(void );
static inline void HplMsp430InterruptP__Port11__clear(void );
static inline void HplMsp430InterruptP__Port12__clear(void );
static inline void HplMsp430InterruptP__Port13__clear(void );
static inline void HplMsp430InterruptP__Port14__clear(void );
static inline void HplMsp430InterruptP__Port15__clear(void );
static inline void HplMsp430InterruptP__Port16__clear(void );
static inline void HplMsp430InterruptP__Port17__clear(void );








static inline void HplMsp430InterruptP__Port10__edge(bool l2h);
#line 131
static inline void HplMsp430InterruptP__Port14__edge(bool l2h);
#line 158
void sig_PORT2_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(2)))  ;
#line 171
static inline void HplMsp430InterruptP__Port20__default__fired(void );
static inline void HplMsp430InterruptP__Port21__default__fired(void );
static inline void HplMsp430InterruptP__Port22__default__fired(void );
static inline void HplMsp430InterruptP__Port23__default__fired(void );
static inline void HplMsp430InterruptP__Port24__default__fired(void );
static inline void HplMsp430InterruptP__Port25__default__fired(void );
static inline void HplMsp430InterruptP__Port26__default__fired(void );








static inline void HplMsp430InterruptP__Port27__enable(void );







static inline void HplMsp430InterruptP__Port27__disable(void );
static inline void HplMsp430InterruptP__Port20__clear(void );
static inline void HplMsp430InterruptP__Port21__clear(void );
static inline void HplMsp430InterruptP__Port22__clear(void );
static inline void HplMsp430InterruptP__Port23__clear(void );
static inline void HplMsp430InterruptP__Port24__clear(void );
static inline void HplMsp430InterruptP__Port25__clear(void );
static inline void HplMsp430InterruptP__Port26__clear(void );
static inline void HplMsp430InterruptP__Port27__clear(void );
#line 253
static inline void HplMsp430InterruptP__Port27__edge(bool l2h);
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplUserButtonC.UserButtonC*/Msp430GpioC__3__HplGeneralIO__makeInput(void );
#line 59
static bool /*HplUserButtonC.UserButtonC*/Msp430GpioC__3__HplGeneralIO__get(void );
# 40 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplUserButtonC.UserButtonC*/Msp430GpioC__3__GeneralIO__get(void );
static inline void /*HplUserButtonC.UserButtonC*/Msp430GpioC__3__GeneralIO__makeInput(void );
# 41 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__HplInterrupt__clear(void );
#line 36
static void /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__HplInterrupt__disable(void );
#line 56
static void /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__HplInterrupt__edge(bool low_to_high);
#line 31
static void /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__HplInterrupt__enable(void );
# 57 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioInterrupt.nc"
static void /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__Interrupt__fired(void );
# 41 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430InterruptC.nc"
static error_t /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__enable(bool rising);








static inline error_t /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void );



static inline error_t /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__Interrupt__enableFallingEdge(void );



static inline error_t /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__Interrupt__disable(void );







static inline void /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__HplInterrupt__fired(void );
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__sendEvent__postTask(void );
# 50 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioInterrupt.nc"
static error_t /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__GpioInterrupt__disable(void );
#line 43
static error_t /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__GpioInterrupt__enableFallingEdge(void );
#line 42
static error_t /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__GpioInterrupt__enableRisingEdge(void );
# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Notify.nc"
static void /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__Notify__notify(/*UserButtonC.SwitchToggleC*/SwitchToggleC__0__Notify__val_t val);
# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
static void /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__GeneralIO__makeInput(void );
#line 32
static bool /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__GeneralIO__get(void );
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosb/SwitchToggleC.nc"
enum /*UserButtonC.SwitchToggleC*/SwitchToggleC__0____nesc_unnamed4393 {
#line 51
  SwitchToggleC__0__sendEvent = 18U
};
#line 51
typedef int /*UserButtonC.SwitchToggleC*/SwitchToggleC__0____nesc_sillytask_sendEvent[/*UserButtonC.SwitchToggleC*/SwitchToggleC__0__sendEvent];
#line 49
bool /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__m_pinHigh;





static inline error_t /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__Notify__enable(void );
#line 71
static inline void /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__GpioInterrupt__fired(void );







static inline void /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__sendEvent__runTask(void );
# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Notify.nc"
static void UserButtonP__Notify__notify(UserButtonP__Notify__val_t val);
#line 59
static error_t UserButtonP__NotifyLower__enable(void );
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosb/UserButtonP.nc"
static inline error_t UserButtonP__Notify__enable(void );







static inline void UserButtonP__NotifyLower__notify(bool val);
# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time);

static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta);
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void );
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void );
# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void );
#line 36
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void );










static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void );
#line 33
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 42 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
#line 54
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );




static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void );






static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void );
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformCounterC.nc"
/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC__0____nesc_unnamed4394 {

  TransformCounterC__0__LOW_SHIFT_RIGHT = 5, 
  TransformCounterC__0__HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT, 
  TransformCounterC__0__NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) + 5, 



  TransformCounterC__0__OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
#line 122
static inline void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void );
#line 92
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt);
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void );
# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void );
# 66 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0____nesc_unnamed4395 {

  TransformAlarmC__0__MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type ) - 1 - 5, 
  TransformAlarmC__0__MAX_DELAY = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type )1 << /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY_LOG2
};

static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );




static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm(void );
#line 136
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type dt);
#line 151
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
#line 166
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void );
# 98 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt);
#line 105
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void );
# 72 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void );
# 63 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_unnamed4396 {
#line 63
  AlarmToTimerC__0__fired = 19U
};
#line 63
typedef int /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_sillytask_fired[/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired];
#line 44
uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt;
bool /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot;

static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot);
#line 60
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );


static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );






static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
#line 82
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);


static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void );
# 125 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void );
#line 118
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x40e3f9f0);
#line 60
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4397 {
#line 60
  VirtualizeTimerC__0__updateFromTimer = 20U
};
#line 60
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer];
#line 42
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4398 {

  VirtualizeTimerC__0__NUM_TIMERS = 1U, 
  VirtualizeTimerC__0__END_OF_LIST = 255
};








#line 48
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4399 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t;

/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS];




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now);
#line 89
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
#line 128
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);
#line 148
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt);









static inline bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(uint8_t num);
#line 193
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num);
# 47 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t LatinMacP__taskReceive__postTask(void );
# 98 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static LatinMacP__SlotAlarm__size_type LatinMacP__SlotAlarm__getNow(void );
#line 92
static void LatinMacP__SlotAlarm__startAt(LatinMacP__SlotAlarm__size_type t0, LatinMacP__SlotAlarm__size_type dt);
#line 105
static LatinMacP__SlotAlarm__size_type LatinMacP__SlotAlarm__getAlarm(void );
# 26 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/RadioControl.nc"
static error_t LatinMacP__RadioControl__receiveNow(bool timeLimitedRx, uint16_t rxWaitTime);








static error_t LatinMacP__RadioControl__rfOff(void );
#line 17
static error_t LatinMacP__RadioControl__prepareReceive(uint8_t frequencyChannel);
#line 7
static error_t LatinMacP__RadioControl__start(void );
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/OpenReceive.nc"
static void LatinMacP__OpenReceiveToUpper__receive(OpenQueueEntry_t *msg);
# 5 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/Malloc.nc"
static error_t LatinMacP__Malloc__freePacketBuffer(OpenQueueEntry_t *pkt);
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t LatinMacP__taskSendDone__postTask(void );
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/IDManager.nc"
static bool LatinMacP__IDManager__getIsDAGroot(void );



static open_addr_t *LatinMacP__IDManager__getMyID(uint8_t type);

static bool LatinMacP__IDManager__isMyAddress(open_addr_t *addr);
# 5 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/OpenSend.nc"
static void LatinMacP__OpenSendFromUpper__sendDone(OpenQueueEntry_t *msg, error_t error);
# 83 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Leds.nc"
static void LatinMacP__Leds__led2Off(void );
#line 45
static void LatinMacP__Leds__led0On(void );
#line 78
static void LatinMacP__Leds__led2On(void );
# 8 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/RadioSend.nc"
static error_t LatinMacP__RadioSend__prepareSend(OpenQueueEntry_t *msg);







static error_t LatinMacP__RadioSend__sendNow(void );
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/OpenQueue.nc"
static OpenQueueEntry_t *LatinMacP__OpenQueue__inQueueBySeed(uint8_t seedNumber);
static OpenQueueEntry_t *LatinMacP__OpenQueue__inQueueToChild(uint8_t seedNumber);
# 13 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/PacketFunctions.nc"
static void LatinMacP__PacketFunctions__readAddress(uint8_t *payload, uint8_t type, open_addr_t *writeToAddress, bool littleEndian);




static void LatinMacP__PacketFunctions__reserveFooterSize(OpenQueueEntry_t *pkt, uint8_t header_length);
static void LatinMacP__PacketFunctions__tossFooter(OpenQueueEntry_t *pkt, uint8_t header_length);
#line 14
static void LatinMacP__PacketFunctions__writeAddress(OpenQueueEntry_t *msg, open_addr_t *address, bool littleEndian);


static void LatinMacP__PacketFunctions__tossHeader(OpenQueueEntry_t *pkt, uint8_t header_length);
#line 10
static bool LatinMacP__PacketFunctions__isBroadcastMulticast(open_addr_t *address);





static void LatinMacP__PacketFunctions__reserveHeaderSize(OpenQueueEntry_t *pkt, uint8_t header_length);
# 98 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static LatinMacP__FastAlarm__size_type LatinMacP__FastAlarm__getNow(void );
#line 92
static void LatinMacP__FastAlarm__startAt(LatinMacP__FastAlarm__size_type t0, LatinMacP__FastAlarm__size_type dt);
#line 55
static void LatinMacP__FastAlarm__start(LatinMacP__FastAlarm__size_type dt);
# 6 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/LatinMatrix.nc"
static uint8_t LatinMacP__LatinMatrix__getParentSeed(void );
#line 2
static bool LatinMacP__LatinMatrix__childCanTransmit(uint8_t minLatinWindow, uint8_t *latinChannelOffset);
static bool LatinMacP__LatinMatrix__siblingCanTransmit(uint8_t minLatinWindow, uint8_t *latinChannelOffset);

static bool LatinMacP__LatinMatrix__canTransmit(uint8_t minLatinWindow, uint8_t *latinChannelOffset);
#line 4
static bool LatinMacP__LatinMatrix__parentCanTransmit(uint8_t minLatinWindow, uint8_t *latinChannelOffset);
# 79 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/LatinMac/LatinMacP.nc"
enum LatinMacP____nesc_unnamed4400 {
#line 79
  LatinMacP__taskReceive = 21U
};
#line 79
typedef int LatinMacP____nesc_sillytask_taskReceive[LatinMacP__taskReceive];

enum LatinMacP____nesc_unnamed4401 {
#line 81
  LatinMacP__taskSendDone = 22U
};
#line 81
typedef int LatinMacP____nesc_sillytask_taskSendDone[LatinMacP__taskSendDone];
#line 41
timervalue_t LatinMacP__fastAlarmStartSlotTimestamp;
timervalue_t LatinMacP__slotAlarmStartSlotTimestamp;
uint8_t LatinMacP__state;
asn_t LatinMacP__asn;
OpenQueueEntry_t *LatinMacP__dataFrameToSend;
OpenQueueEntry_t *LatinMacP__packetACK;
OpenQueueEntry_t *LatinMacP__frameReceived;




bool LatinMacP__isSync;
uint8_t LatinMacP__dsn;
uint8_t LatinMacP__frequencyChannel;
OpenQueueEntry_t *LatinMacP__sendDoneMessage;
error_t LatinMacP__sendDoneError;


uint8_t LatinMacP__init;
uint8_t LatinMacP__numReceiveNothing;
uint8_t LatinMacP__numADV;
uint8_t LatinMacP__numPrepareReceiveFailed;
uint8_t LatinMacP__numSlotsInSynchStatus;


bool LatinMacP__beacon_channel;
bool LatinMacP__change;



uint8_t LatinMacP__slotType;
uint8_t LatinMacP__networkState;
# 1 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/IEEE802154_common.c"
static inline void LatinMacP__prependIEEE802154header(OpenQueueEntry_t *msg, 
uint8_t frameType, 
bool securityEnabled, 
uint8_t sequenceNumber, 
open_addr_t *nextHop);
#line 98
static ieee802154_header_iht LatinMacP__retrieveIEEE802154header(OpenQueueEntry_t *msg);
# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/LatinMac/LatinMacP.nc"
static void LatinMacP__change_state(uint8_t newstate);
static inline void LatinMacP__resynchronize(bool resyncType, open_addr_t *node_id, timervalue_t dataGlobalSlotOffset, int16_t timeCorrection);
static void LatinMacP__endSlot(void );


static void LatinMacP__postTaskSendDone(OpenQueueEntry_t *param_sendDoneMessage, error_t param_sendDoneError);


static inline void LatinMacP__Boot__booted(void );
#line 99
static inline void LatinMacP__RadioControl__startDone(error_t error);


static inline void LatinMacP__RadioControl__stopDone(error_t error);




static error_t LatinMacP__OpenSendFromUpper__send(OpenQueueEntry_t *msg);
#line 124
static inline void LatinMacP__SlotAlarm__fired(void );
#line 449
static void LatinMacP__RadioSend__prepareSendDone(error_t error);
#line 486
static void LatinMacP__RadioControl__prepareReceiveDone(error_t error);
#line 518
static inline void LatinMacP__FastAlarm__fired(void );
#line 589
static void LatinMacP__RadioControl__receivedNothing(void );
#line 627
static inline void LatinMacP__RadioSend__sendNowDone(error_t error);
#line 679
static void LatinMacP__postTaskSendDone(OpenQueueEntry_t *param_sendDoneMessage, 
error_t param_sendDoneError);










static inline void LatinMacP__taskSendDone__runTask(void );
#line 703
static void LatinMacP__endSlot(void );
#line 723
static inline void LatinMacP__RadioReceive__receive(OpenQueueEntry_t *msg);
#line 864
static inline void LatinMacP__taskReceive__runTask(void );










static inline void LatinMacP__resynchronize(bool resyncType, open_addr_t *node_id, 
timervalue_t dataGlobalSlotOffset, int16_t timeCorrection);
#line 947
static inline error_t LatinMacP__SoftwareInit__init(void );










static timervalue_t LatinMacP__GlobalTime__getGlobalSlotOffset(void );








static inline asn_t LatinMacP__GlobalTime__getASN(void );







static void LatinMacP__change_state(uint8_t newstate);
#line 996
static void LatinMacP__MAC__changeNetworkState(uint8_t tmp_networkState);
# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time);

static void /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta);
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void );
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void );
# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void );
static void /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void );
#line 33
static void /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void );
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );










static inline void /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
static /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__size_type /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__get(void );






static bool /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending(void );










static void /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__overflow(void );
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformCounterC.nc"
/*Counter32khz32C.Transform*/TransformCounterC__1__upper_count_type /*Counter32khz32C.Transform*/TransformCounterC__1__m_upper;

enum /*Counter32khz32C.Transform*/TransformCounterC__1____nesc_unnamed4402 {

  TransformCounterC__1__LOW_SHIFT_RIGHT = 0, 
  TransformCounterC__1__HIGH_SHIFT_LEFT = 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__1__from_size_type ) - /*Counter32khz32C.Transform*/TransformCounterC__1__LOW_SHIFT_RIGHT, 
  TransformCounterC__1__NUM_UPPER_BITS = 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type ) - 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__1__from_size_type ) + 0, 



  TransformCounterC__1__OVERFLOW_MASK = /*Counter32khz32C.Transform*/TransformCounterC__1__NUM_UPPER_BITS ? ((/*Counter32khz32C.Transform*/TransformCounterC__1__upper_count_type )2 << (/*Counter32khz32C.Transform*/TransformCounterC__1__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__get(void );
#line 122
static inline void /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void );
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Alarm__fired(void );
#line 92
static void /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__AlarmFrom__startAt(/*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__AlarmFrom__size_type t0, /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__AlarmFrom__size_type dt);
# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
static /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Counter__size_type /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Counter__get(void );
# 66 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
/*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__to_size_type /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__m_t0;
/*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__to_size_type /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__m_dt;

enum /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1____nesc_unnamed4403 {

  TransformAlarmC__1__MAX_DELAY_LOG2 = 8 * sizeof(/*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__from_size_type ) - 1 - 0, 
  TransformAlarmC__1__MAX_DELAY = (/*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__to_size_type )1 << /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__MAX_DELAY_LOG2
};

static inline /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__to_size_type /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Alarm__getNow(void );




static inline /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__to_size_type /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Alarm__getAlarm(void );
#line 96
static void /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__set_alarm(void );
#line 136
static void /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Alarm__startAt(/*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__to_size_type t0, /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__to_size_type dt);
#line 151
static inline void /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__AlarmFrom__fired(void );
#line 166
static inline void /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Counter__overflow(void );
# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEvent(uint16_t time);

static void /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEventFromNow(uint16_t delta);
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__get(void );
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Alarm__fired(void );
# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__enableEvents(void );
static void /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__disableEvents(void );
#line 33
static void /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__clearPendingInterrupt(void );
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired(void );










static inline void /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow(void );
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Alarm__fired(void );
#line 92
static void /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__AlarmFrom__startAt(/*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__AlarmFrom__size_type t0, /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__AlarmFrom__size_type dt);
# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
static /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Counter__size_type /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Counter__get(void );
# 66 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
/*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__to_size_type /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__m_t0;
/*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__to_size_type /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__m_dt;

enum /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2____nesc_unnamed4404 {

  TransformAlarmC__2__MAX_DELAY_LOG2 = 8 * sizeof(/*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__from_size_type ) - 1 - 0, 
  TransformAlarmC__2__MAX_DELAY = (/*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__to_size_type )1 << /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__MAX_DELAY_LOG2
};

static inline /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__to_size_type /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Alarm__getNow(void );
#line 96
static void /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__set_alarm(void );
#line 136
static void /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Alarm__startAt(/*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__to_size_type t0, /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__to_size_type dt);









static inline void /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Alarm__start(/*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__to_size_type dt);




static inline void /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__AlarmFrom__fired(void );
#line 166
static inline void /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Counter__overflow(void );
# 35 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
static void PinDebugP__PinADC2__makeOutput(void );
#line 35
static void PinDebugP__PinADC0__makeOutput(void );
#line 35
static void PinDebugP__PinADC1__makeOutput(void );
# 52 "/home/david/workspace/tsch-project/tos/lib/debug/PinDebugP.nc"
static inline error_t PinDebugP__Reset__init(void );
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PinDebugC.PinADC0*/Msp430GpioC__4__HplGeneralIO__makeOutput(void );
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PinDebugC.PinADC0*/Msp430GpioC__4__GeneralIO__makeOutput(void );
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PinDebugC.PinADC1*/Msp430GpioC__5__HplGeneralIO__makeOutput(void );
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PinDebugC.PinADC1*/Msp430GpioC__5__GeneralIO__makeOutput(void );
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PinDebugC.PinADC2*/Msp430GpioC__6__HplGeneralIO__makeOutput(void );
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PinDebugC.PinADC2*/Msp430GpioC__6__GeneralIO__makeOutput(void );
# 41 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/RandomMlcgC.nc"
uint32_t RandomMlcgC__seed;


static inline error_t RandomMlcgC__Init__init(void );
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Config.nc"
static error_t CC2420DriversP__CC2420Config__startOscillator(void );






static error_t CC2420DriversP__CC2420Config__rfOff(void );
#line 8
static error_t CC2420DriversP__CC2420Config__setChannel(uint8_t channel);
#line 6
static error_t CC2420DriversP__CC2420Config__writeId(void );



static error_t CC2420DriversP__CC2420Config__rxOn(void );
#line 2
static error_t CC2420DriversP__CC2420Config__startVReg(void );










static error_t CC2420DriversP__CC2420Config__stopVReg(void );
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void CC2420DriversP__RxWaitAlarm__start(CC2420DriversP__RxWaitAlarm__size_type dt);






static void CC2420DriversP__RxWaitAlarm__stop(void );
# 12 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/RadioControl.nc"
static void CC2420DriversP__RadioControl__startDone(error_t error);








static void CC2420DriversP__RadioControl__prepareReceiveDone(error_t error);
#line 43
static void CC2420DriversP__RadioControl__stopDone(error_t error);
#line 30
static void CC2420DriversP__RadioControl__receivedNothing(void );
# 3 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/StdAsyncControl.nc"
static error_t CC2420DriversP__TxControl__start(void );
static error_t CC2420DriversP__TxControl__stop(void );
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Transmit.nc"
static error_t CC2420DriversP__CC2420Transmit__sendNow(bool useCca);
#line 2
static error_t CC2420DriversP__CC2420Transmit__loadPacket(OpenQueueEntry_t *msg);
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t CC2420DriversP__taskSendDone__postTask(void );
#line 56
static error_t CC2420DriversP__taskStartOscillatorDone__postTask(void );
#line 56
static error_t CC2420DriversP__taskStopDone__postTask(void );
# 12 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/RadioSend.nc"
static void CC2420DriversP__RadioSend__prepareSendDone(error_t error);







static void CC2420DriversP__RadioSend__sendNowDone(error_t error);
# 3 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/StdAsyncControl.nc"
static error_t CC2420DriversP__RxControl__start(void );
static error_t CC2420DriversP__RxControl__stop(void );
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t CC2420DriversP__taskStartDone__postTask(void );
# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/CC2420DriversP.nc"
enum CC2420DriversP____nesc_unnamed4405 {
#line 46
  CC2420DriversP__taskStartDone = 23U
};
#line 46
typedef int CC2420DriversP____nesc_sillytask_taskStartDone[CC2420DriversP__taskStartDone];
enum CC2420DriversP____nesc_unnamed4406 {
#line 47
  CC2420DriversP__taskStopDone = 24U
};
#line 47
typedef int CC2420DriversP____nesc_sillytask_taskStopDone[CC2420DriversP__taskStopDone];
enum CC2420DriversP____nesc_unnamed4407 {
#line 48
  CC2420DriversP__taskSendDone = 25U
};
#line 48
typedef int CC2420DriversP____nesc_sillytask_taskSendDone[CC2420DriversP__taskSendDone];
enum CC2420DriversP____nesc_unnamed4408 {
#line 49
  CC2420DriversP__taskStartOscillatorDone = 26U
};
#line 49
typedef int CC2420DriversP____nesc_sillytask_taskStartOscillatorDone[CC2420DriversP__taskStartOscillatorDone];
#line 22
enum CC2420DriversP____nesc_unnamed4409 {
  CC2420DriversP__S_STOPPED = 0, 
  CC2420DriversP__S_STARTING_VREG = 1, 
  CC2420DriversP__S_STARTING_OSCILLATOR = 2, 
  CC2420DriversP__S_STARTED = 3, 
  CC2420DriversP__S_LOADING_PACKET = 4, 
  CC2420DriversP__S_SETTING_CHANNEL = 5, 
  CC2420DriversP__S_READY_TX = 6, 
  CC2420DriversP__S_TRANSMITTING = 7, 
  CC2420DriversP__S_READY_RX = 8, 
  CC2420DriversP__S_RECEIVING = 9, 
  CC2420DriversP__S_STOPPING = 10, 
  CC2420DriversP__S_WRITING_ID = 11
};

uint8_t CC2420DriversP__state = CC2420DriversP__S_STOPPED;
bool CC2420DriversP__timeLimitedRx = TRUE;
uint16_t CC2420DriversP__rxWaitTime;
error_t CC2420DriversP__sendErr = SUCCESS;
uint8_t CC2420DriversP__frequencyChannel = 0;
bool CC2420DriversP__syncForSend;







static inline void CC2420DriversP__shutdown(void );



static inline error_t CC2420DriversP__RadioControl__start(void );
#line 72
static inline void CC2420DriversP__CC2420Config__startVRegDone(void );



static inline void CC2420DriversP__CC2420Config__startOscillatorDone(void );


static inline void CC2420DriversP__taskStartOscillatorDone__runTask(void );



static inline void CC2420DriversP__CC2420Config__writeIdDone(void );



static inline void CC2420DriversP__taskStartDone__runTask(void );








static error_t CC2420DriversP__RadioControl__prepareReceive(uint8_t frequencyChannel_param);
#line 115
static error_t CC2420DriversP__RadioControl__receiveNow(bool timeLimitedRx_param, uint16_t rxWaitTime_param);
#line 127
static inline void CC2420DriversP__CC2420sfd__sfd(uint32_t txTimestamp);









static void CC2420DriversP__CC2420ReceivedNothing__receivedNothing(void );





static inline void CC2420DriversP__RxWaitAlarm__fired(void );








static inline error_t CC2420DriversP__RadioSend__prepareSend(OpenQueueEntry_t *msg);
#line 180
static inline void CC2420DriversP__CC2420Transmit__loadPacketDone(error_t error);
#line 198
static inline void CC2420DriversP__CC2420Config__setChannelDone(error_t error);
#line 221
static error_t CC2420DriversP__RadioSend__sendNow(void );




static inline void CC2420DriversP__CC2420Transmit__sendNowDone(error_t err);



static inline void CC2420DriversP__taskSendDone__runTask(void );
#line 269
static inline void CC2420DriversP__shutdown(void );





static inline void CC2420DriversP__taskStopDone__runTask(void );






static error_t CC2420DriversP__RadioControl__rfOff(void );
# 9 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Config.nc"
static void CC2420ControlP__CC2420Config__setChannelDone(error_t error);
#line 5
static void CC2420ControlP__CC2420Config__startOscillatorDone(void );

static void CC2420ControlP__CC2420Config__writeIdDone(void );
#line 3
static void CC2420ControlP__CC2420Config__startVRegDone(void );
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__RXCTRL1__write(uint16_t data);
# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t CC2420ControlP__OscResource__release(void );
#line 87
static error_t CC2420ControlP__OscResource__immediateRequest(void );
#line 78
static error_t CC2420ControlP__OscResource__request(void );
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__MDMCTRL0__write(uint16_t data);
# 35 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
static void CC2420ControlP__RSTN__makeOutput(void );
#line 29
static void CC2420ControlP__RSTN__set(void );
static void CC2420ControlP__RSTN__clr(void );
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__IOCFG0__write(uint16_t data);
# 35 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
static void CC2420ControlP__CSN__makeOutput(void );
#line 29
static void CC2420ControlP__CSN__set(void );
static void CC2420ControlP__CSN__clr(void );




static void CC2420ControlP__VREN__makeOutput(void );
#line 29
static void CC2420ControlP__VREN__set(void );
static void CC2420ControlP__VREN__clr(void );
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SXOSCON__strobe(void );
# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t CC2420ControlP__RxOnResource__release(void );
#line 87
static error_t CC2420ControlP__RxOnResource__immediateRequest(void );
#line 78
static error_t CC2420ControlP__RxOnResource__request(void );
#line 110
static error_t CC2420ControlP__RfOffResource__release(void );
#line 87
static error_t CC2420ControlP__RfOffResource__immediateRequest(void );
#line 78
static error_t CC2420ControlP__RfOffResource__request(void );
#line 110
static error_t CC2420ControlP__SpiResource__release(void );
#line 87
static error_t CC2420ControlP__SpiResource__immediateRequest(void );
#line 78
static error_t CC2420ControlP__SpiResource__request(void );
#line 110
static error_t CC2420ControlP__SyncResource__release(void );
#line 87
static error_t CC2420ControlP__SyncResource__immediateRequest(void );
#line 78
static error_t CC2420ControlP__SyncResource__request(void );
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__IOCFG1__write(uint16_t data);
#line 55
static cc2420_status_t CC2420ControlP__FSCTRL__write(uint16_t data);
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SRXON__strobe(void );
# 63 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420ControlP__IEEEADR__write(uint8_t offset, uint8_t * data, uint8_t length);
# 50 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioInterrupt.nc"
static error_t CC2420ControlP__InterruptCCA__disable(void );
#line 42
static error_t CC2420ControlP__InterruptCCA__enableRisingEdge(void );
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void CC2420ControlP__StartupAlarm__start(CC2420ControlP__StartupAlarm__size_type dt);
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SRFOFF__strobe(void );
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/control/CC2420ControlP.nc"
#line 48
typedef enum CC2420ControlP____nesc_unnamed4410 {
  CC2420ControlP__S_VREG_STOPPED, 
  CC2420ControlP__S_VREG_STARTING, 
  CC2420ControlP__S_VREG_STARTED, 
  CC2420ControlP__S_XOSC_STARTING, 
  CC2420ControlP__S_XOSC_STARTED
} CC2420ControlP__cc2420_control_state_t;

uint8_t CC2420ControlP__m_channel;

bool CC2420ControlP__m_sync_busy = FALSE;
bool CC2420ControlP__m_rxOn_busy = FALSE;
bool CC2420ControlP__m_rfOff_busy = FALSE;

CC2420ControlP__cc2420_control_state_t CC2420ControlP__m_state = CC2420ControlP__S_VREG_STOPPED;



static void CC2420ControlP__startOscillator(void );
static void CC2420ControlP__writeId(void );
static void CC2420ControlP__syncConfig(void );
static void CC2420ControlP__rxOn(void );
static void CC2420ControlP__rfOff(void );






static inline error_t CC2420ControlP__CC2420Config__startVReg(void );










static inline void CC2420ControlP__StartupAlarm__fired(void );










static inline error_t CC2420ControlP__CC2420Config__startOscillator(void );
#line 114
static inline void CC2420ControlP__OscResource__granted(void );


static void CC2420ControlP__startOscillator(void );
#line 153
static inline void CC2420ControlP__InterruptCCA__fired(void );
#line 165
static inline error_t CC2420ControlP__CC2420Config__writeId(void );









static inline void CC2420ControlP__SpiResource__granted(void );


static void CC2420ControlP__writeId(void );
#line 195
static error_t CC2420ControlP__CC2420Config__setChannel(uint8_t channel);
#line 214
static inline void CC2420ControlP__SyncResource__granted(void );


static void CC2420ControlP__syncConfig(void );
#line 235
static inline error_t CC2420ControlP__CC2420Config__rxOn(void );
#line 253
static inline void CC2420ControlP__RxOnResource__granted(void );


static void CC2420ControlP__rxOn(void );







static inline error_t CC2420ControlP__CC2420Config__rfOff(void );
#line 283
static inline void CC2420ControlP__RfOffResource__granted(void );


static void CC2420ControlP__rfOff(void );
#line 309
static inline error_t CC2420ControlP__CC2420Config__stopVReg(void );










static inline error_t CC2420ControlP__Init__init(void );
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__7__HplGeneralIO__makeInput(void );
# 41 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CCAM*/Msp430GpioC__7__GeneralIO__makeInput(void );
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__HplGeneralIO__makeOutput(void );
#line 34
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__HplGeneralIO__set(void );




static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__HplGeneralIO__clr(void );
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__GeneralIO__set(void );
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__GeneralIO__clr(void );




static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__GeneralIO__makeOutput(void );
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__9__HplGeneralIO__get(void );
# 40 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__9__GeneralIO__get(void );
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__10__HplGeneralIO__get(void );
# 40 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__10__GeneralIO__get(void );
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__11__HplGeneralIO__makeOutput(void );
#line 34
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__11__HplGeneralIO__set(void );




static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__11__HplGeneralIO__clr(void );
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__11__GeneralIO__set(void );
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__11__GeneralIO__clr(void );




static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__11__GeneralIO__makeOutput(void );
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__12__HplGeneralIO__makeInput(void );
#line 59
static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__12__HplGeneralIO__get(void );
# 40 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__12__GeneralIO__get(void );
static inline void /*HplCC2420PinsC.SFDM*/Msp430GpioC__12__GeneralIO__makeInput(void );
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__13__HplGeneralIO__makeOutput(void );
#line 34
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__13__HplGeneralIO__set(void );




static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__13__HplGeneralIO__clr(void );
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__13__GeneralIO__set(void );
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__13__GeneralIO__clr(void );




static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__13__GeneralIO__makeOutput(void );
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/SpiPacket.nc"
static error_t CC2420SpiP__SpiPacket__send(
#line 48
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
# 91 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420SpiP__Fifo__writeDone(
# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104a950, 
# 91 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420SpiP__Fifo__readDone(
# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104a950, 
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
# 24 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420SpiP__ChipSpiResource__releasing(void );
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/SpiByte.nc"
static uint8_t CC2420SpiP__SpiByte__write(uint8_t tx);
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/State.nc"
static void CC2420SpiP__WorkingState__toIdle(void );




static bool CC2420SpiP__WorkingState__isIdle(void );
#line 45
static error_t CC2420SpiP__WorkingState__requestState(uint8_t reqState);
# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__SpiResource__release(void );
#line 87
static error_t CC2420SpiP__SpiResource__immediateRequest(void );
#line 78
static error_t CC2420SpiP__SpiResource__request(void );
#line 118
static bool CC2420SpiP__SpiResource__isOwner(void );
#line 92
static void CC2420SpiP__Resource__granted(
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104def0);
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t CC2420SpiP__grant__postTask(void );
# 88 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
enum CC2420SpiP____nesc_unnamed4411 {
#line 88
  CC2420SpiP__grant = 27U
};
#line 88
typedef int CC2420SpiP____nesc_sillytask_grant[CC2420SpiP__grant];
#line 63
enum CC2420SpiP____nesc_unnamed4412 {
  CC2420SpiP__RESOURCE_COUNT = 8U, 
  CC2420SpiP__NO_HOLDER = 0xFF
};


enum CC2420SpiP____nesc_unnamed4413 {
  CC2420SpiP__S_IDLE, 
  CC2420SpiP__S_BUSY
};


uint16_t CC2420SpiP__m_addr;


uint8_t CC2420SpiP__m_requests = 0;


uint8_t CC2420SpiP__m_holder = CC2420SpiP__NO_HOLDER;


bool CC2420SpiP__release;


static inline error_t CC2420SpiP__attemptRelease(void );
#line 107
static error_t CC2420SpiP__Resource__request(uint8_t id);
#line 126
static error_t CC2420SpiP__Resource__immediateRequest(uint8_t id);
#line 149
static error_t CC2420SpiP__Resource__release(uint8_t id);
#line 178
static inline uint8_t CC2420SpiP__Resource__isOwner(uint8_t id);





static inline void CC2420SpiP__SpiResource__granted(void );




static inline cc2420_status_t CC2420SpiP__Fifo__beginRead(uint8_t addr, uint8_t *data, 
uint8_t len);
#line 209
static inline error_t CC2420SpiP__Fifo__continueRead(uint8_t addr, uint8_t *data, 
uint8_t len);



static inline cc2420_status_t CC2420SpiP__Fifo__write(uint8_t addr, uint8_t *data, 
uint8_t len);
#line 260
static inline cc2420_status_t CC2420SpiP__Ram__write(uint16_t addr, uint8_t offset, 
uint8_t *data, 
uint8_t len);
#line 305
static cc2420_status_t CC2420SpiP__Reg__write(uint8_t addr, uint16_t data);
#line 318
static cc2420_status_t CC2420SpiP__Strobe__strobe(uint8_t addr);










static void CC2420SpiP__SpiPacket__sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error);








static inline error_t CC2420SpiP__attemptRelease(void );
#line 358
static inline void CC2420SpiP__grant__runTask(void );








static inline void CC2420SpiP__Resource__default__granted(uint8_t id);


static inline void CC2420SpiP__Fifo__default__readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error);


static inline void CC2420SpiP__Fifo__default__writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error);
# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/StateImplP.nc"
uint8_t StateImplP__state[1U];

enum StateImplP____nesc_unnamed4414 {
  StateImplP__S_IDLE = 0
};


static inline error_t StateImplP__Init__init(void );
#line 96
static error_t StateImplP__State__requestState(uint8_t id, uint8_t reqState);
#line 118
static inline void StateImplP__State__toIdle(uint8_t id);







static inline bool StateImplP__State__isIdle(uint8_t id);






static inline bool StateImplP__State__isState(uint8_t id, uint8_t myState);
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/SpiPacket.nc"
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiPacket__sendDone(
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x410c30c0, 
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Msp430SpiConfigure__getConfig(
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x410c1418);
# 197 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__clrRxIntr(void );
#line 97
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__resetUsart(bool reset);
#line 224
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__tx(uint8_t data);
#line 168
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 231
static uint8_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__rx(void );
#line 192
static bool /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__isRxIntrPending(void );
#line 158
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__disableSpi(void );
# 38 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannel.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel1__setupTransfer(dma_transfer_mode_t transfer_mode, 
dma_trigger_t trigger, 
dma_level_t level, 
void *src_addr, 
void *dst_addr, 
uint16_t size, 
dma_byte_t src_byte, 
dma_byte_t dst_byte, 
dma_incr_t src_incr, 
dma_incr_t dst_incr);
#line 73
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel1__startTransfer(void );
# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__release(
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x410c29a0);
# 87 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__immediateRequest(
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x410c29a0);
# 78 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__request(
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x410c29a0);
# 118 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static bool /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__isOwner(
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x410c29a0);
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__granted(
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x410c6b00);
# 38 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannel.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel2__setupTransfer(dma_transfer_mode_t transfer_mode, 
dma_trigger_t trigger, 
dma_level_t level, 
void *src_addr, 
void *dst_addr, 
uint16_t size, 
dma_byte_t src_byte, 
dma_byte_t dst_byte, 
dma_incr_t src_incr, 
dma_incr_t dst_incr);
#line 73
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel2__startTransfer(void );
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone_task__postTask(void );
# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
enum /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0____nesc_unnamed4415 {
#line 74
  Msp430SpiDmaP__0__signalDone_task = 28U
};
#line 74
typedef int /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0____nesc_sillytask_signalDone_task[/*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone_task];
#line 67
uint8_t */*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_tx_buf;
uint8_t */*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_rx_buf;
uint16_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_len;
uint8_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_client;
uint8_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_dump;

static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone(error_t error);


static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__immediateRequest(uint8_t id);



static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__request(uint8_t id);



static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__release(uint8_t id);



static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__ResourceConfigure__configure(uint8_t id);



static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__ResourceConfigure__unconfigure(uint8_t id);





static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__granted(uint8_t id);



static inline uint8_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__isOwner(uint8_t id);



static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__isOwner(uint8_t id);
static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__request(uint8_t id);
static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__immediateRequest(uint8_t id);
static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__release(uint8_t id);
static inline msp430_spi_union_config_t */*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Msp430SpiConfigure__default__getConfig(uint8_t id);



static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__default__granted(uint8_t id);

static uint8_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiByte__write(uint8_t tx);








static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiPacket__send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len);
#line 180
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone_task__runTask(void );



static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel1__transferDone(error_t error);



static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel2__transferDone(error_t error);

static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone(error_t error);



static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartInterrupts__txDone(void );
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartInterrupts__rxDone(uint8_t data);

static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error);
# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__UCLK__selectIOFunc(void );
#line 78
static void HplMsp430Usart0P__UCLK__selectModuleFunc(void );
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void HplMsp430Usart0P__Interrupts__rxDone(uint8_t data);
#line 49
static void HplMsp430Usart0P__Interrupts__txDone(void );
# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__URXD__selectIOFunc(void );
#line 85
static void HplMsp430Usart0P__UTXD__selectIOFunc(void );
# 7 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430I2C.nc"
static void HplMsp430Usart0P__HplI2C__clearModeI2C(void );
#line 6
static bool HplMsp430Usart0P__HplI2C__isI2C(void );
# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__SOMI__selectIOFunc(void );
#line 78
static void HplMsp430Usart0P__SOMI__selectModuleFunc(void );
# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void HplMsp430Usart0P__I2CInterrupts__fired(void );
# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__SIMO__selectIOFunc(void );
#line 78
static void HplMsp430Usart0P__SIMO__selectModuleFunc(void );
# 89 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static volatile uint8_t HplMsp430Usart0P__IE1 __asm ("0x0000");
static volatile uint8_t HplMsp430Usart0P__ME1 __asm ("0x0004");
static volatile uint8_t HplMsp430Usart0P__IFG1 __asm ("0x0002");
static volatile uint8_t HplMsp430Usart0P__U0TCTL __asm ("0x0071");

static volatile uint8_t HplMsp430Usart0P__U0TXBUF __asm ("0x0077");

void sig_UART0RX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(18)))  ;




void sig_UART0TX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(16)))  ;
#line 132
static inline void HplMsp430Usart0P__Usart__setUbr(uint16_t control);










static inline void HplMsp430Usart0P__Usart__setUmctl(uint8_t control);







static inline void HplMsp430Usart0P__Usart__resetUsart(bool reset);
#line 207
static inline void HplMsp430Usart0P__Usart__disableUart(void );
#line 238
static inline void HplMsp430Usart0P__Usart__enableSpi(void );








static void HplMsp430Usart0P__Usart__disableSpi(void );








static inline void HplMsp430Usart0P__configSpi(msp430_spi_union_config_t *config);








static void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 330
static inline bool HplMsp430Usart0P__Usart__isRxIntrPending(void );










static inline void HplMsp430Usart0P__Usart__clrRxIntr(void );



static inline void HplMsp430Usart0P__Usart__clrIntr(void );
#line 357
static inline void HplMsp430Usart0P__Usart__disableIntr(void );
#line 382
static inline void HplMsp430Usart0P__Usart__tx(uint8_t data);



static inline uint8_t HplMsp430Usart0P__Usart__rx(void );
# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaInterrupt.nc"
static void HplMsp430DmaP__Interrupt__fired(void );
# 72 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaP.nc"
void sig_DACDMA_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0)))  ;
# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
static void /*HplMsp430DmaC.Dma0*/HplMsp430DmaXP__0__DMA__transferDone(error_t success);
# 82 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaXP.nc"
static inline void /*HplMsp430DmaC.Dma0*/HplMsp430DmaXP__0__Interrupt__fired(void );
# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
static void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__transferDone(error_t success);
# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaXP.nc"
static volatile uint16_t /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMACTL0 __asm ("0x0122");






static inline void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__Interrupt__fired(void );








static inline error_t /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setTrigger(dma_trigger_t trigger);
#line 195
static inline void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__enableDMA(void );
#line 235
static inline void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setState(dma_channel_state_t s, 
dma_channel_trigger_t t, 
void *src, void *dest, 
uint16_t size);




static inline void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setStateRaw(uint16_t s, uint16_t t, 
void *src, void *dest, 
uint16_t size);
# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
static void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__transferDone(error_t success);
# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaXP.nc"
static volatile uint16_t /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMACTL0 __asm ("0x0122");






static inline void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__Interrupt__fired(void );








static inline error_t /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setTrigger(dma_trigger_t trigger);
#line 195
static inline void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__enableDMA(void );
#line 235
static inline void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setState(dma_channel_state_t s, 
dma_channel_trigger_t t, 
void *src, void *dest, 
uint16_t size);




static inline void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setStateRaw(uint16_t s, uint16_t t, 
void *src, void *dest, 
uint16_t size);
# 96 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannel.nc"
static void /*Msp430DmaC.Channel0P*/Msp430DmaChannelP__0__Channel__transferDone(error_t success);
# 143 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannelP.nc"
static inline void /*Msp430DmaC.Channel0P*/Msp430DmaChannelP__0__HplChannel__transferDone(error_t error);



static inline void /*Msp430DmaC.Channel0P*/Msp430DmaChannelP__0__Channel__default__transferDone(error_t error);
# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
static void /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__HplChannel__enableDMA(void );
#line 64
static void /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__HplChannel__setState(dma_channel_state_t s, dma_channel_trigger_t t, void *src, void *dest, uint16_t size);
# 96 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannel.nc"
static void /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__Channel__transferDone(error_t success);
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannelP.nc"
dma_channel_state_t /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState;
dma_channel_trigger_t /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelTrigger;







static inline error_t /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__Channel__setupTransfer(dma_transfer_mode_t transfer_mode, 
dma_trigger_t trigger, 
dma_level_t level, 
void *src_addr, 
void *dst_addr, 
uint16_t size, 
dma_byte_t src_byte, 
dma_byte_t dst_byte, 
dma_incr_t src_incr, 
dma_incr_t dst_incr);
#line 112
static inline error_t /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__Channel__startTransfer(void );
#line 143
static inline void /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__HplChannel__transferDone(error_t error);
# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
static void /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__HplChannel__enableDMA(void );
#line 64
static void /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__HplChannel__setState(dma_channel_state_t s, dma_channel_trigger_t t, void *src, void *dest, uint16_t size);
# 96 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannel.nc"
static void /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__Channel__transferDone(error_t success);
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannelP.nc"
dma_channel_state_t /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState;
dma_channel_trigger_t /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelTrigger;







static inline error_t /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__Channel__setupTransfer(dma_transfer_mode_t transfer_mode, 
dma_trigger_t trigger, 
dma_level_t level, 
void *src_addr, 
void *dst_addr, 
uint16_t size, 
dma_byte_t src_byte, 
dma_byte_t dst_byte, 
dma_incr_t src_incr, 
dma_incr_t dst_incr);
#line 112
static inline error_t /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__Channel__startTransfer(void );
#line 143
static inline void /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__HplChannel__transferDone(error_t error);
# 96 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaControlP.nc"
static inline void Msp430DmaControlP__HplChannel0__transferDone(error_t error);
static inline void Msp430DmaControlP__HplChannel1__transferDone(error_t error);
static inline void Msp430DmaControlP__HplChannel2__transferDone(error_t error);
# 80 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__userId(void );
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__rxDone(
# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40a89010, 
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__txDone(
# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40a89010);
# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__I2CInterrupts__fired(
# 40 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40a88010);








static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__txDone(void );




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__rxDone(uint8_t data);




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawI2CInterrupts__fired(void );




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__txDone(uint8_t id);
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__rxDone(uint8_t id, uint8_t data);
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__I2CInterrupts__default__fired(uint8_t id);
# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1____nesc_unnamed4416 {
#line 39
  FcfsResourceQueueC__1__NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[1U];
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void );




static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void );



static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void );
#line 72
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(resource_client_id_t id);
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__requested(
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40aac948);
# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__immediateRequested(
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40aac948);
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(
# 60 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40aaacf8);
# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(
# 60 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40aaacf8);
# 69 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__enqueue(resource_client_id_t id);
#line 43
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__isEmpty(void );
#line 60
static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__dequeue(void );
# 73 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested(void );
#line 46
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted(void );
#line 81
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__immediateRequested(void );
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__granted(
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
uint8_t arg_0x40a99e98);
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask(void );
# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4417 {
#line 75
  ArbiterP__1__grantedTask = 29U
};
#line 75
typedef int /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_sillytask_grantedTask[/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask];
#line 67
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4418 {
#line 67
  ArbiterP__1__RES_CONTROLLED, ArbiterP__1__RES_GRANTING, ArbiterP__1__RES_IMM_GRANTING, ArbiterP__1__RES_BUSY
};
#line 68
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4419 {
#line 68
  ArbiterP__1__default_owner_id = 1U
};
#line 69
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4420 {
#line 69
  ArbiterP__1__NO_RES = 0xFF
};
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED;
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__default_owner_id;
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId;



static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__request(uint8_t id);
#line 90
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(uint8_t id);
#line 108
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(uint8_t id);
#line 130
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void );
#line 150
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse(void );
#line 163
static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId(void );










static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(uint8_t id);
#line 187
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask(void );
#line 199
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__granted(void );

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__requested(void );


static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__immediateRequested(void );


static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(uint8_t id);
# 97 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430I2C0P__HplUsart__resetUsart(bool reset);
# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static volatile uint8_t HplMsp430I2C0P__U0CTL __asm ("0x0070");





static inline bool HplMsp430I2C0P__HplI2C__isI2C(void );



static inline void HplMsp430I2C0P__HplI2C__clearModeI2C(void );
# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEvent(uint16_t time);

static void /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEventFromNow(uint16_t delta);
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__get(void );
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__fired(void );
# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__enableEvents(void );
static void /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__disableEvents(void );
#line 33
static void /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__clearPendingInterrupt(void );
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__fired(void );










static inline void /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__overflow(void );
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Alarm__fired(void );
#line 92
static void /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__AlarmFrom__startAt(/*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__AlarmFrom__size_type t0, /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__AlarmFrom__size_type dt);
# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
static /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Counter__size_type /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Counter__get(void );
# 66 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
/*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__to_size_type /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__m_t0;
/*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__to_size_type /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__m_dt;

enum /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3____nesc_unnamed4421 {

  TransformAlarmC__3__MAX_DELAY_LOG2 = 8 * sizeof(/*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__from_size_type ) - 1 - 0, 
  TransformAlarmC__3__MAX_DELAY = (/*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__to_size_type )1 << /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__MAX_DELAY_LOG2
};

static inline /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__to_size_type /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Alarm__getNow(void );
#line 96
static void /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__set_alarm(void );
#line 136
static inline void /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Alarm__startAt(/*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__to_size_type t0, /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__to_size_type dt);









static inline void /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Alarm__start(/*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__to_size_type dt);




static inline void /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__AlarmFrom__fired(void );
#line 166
static inline void /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Counter__overflow(void );
# 32 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
static bool CC2420ReceiveP__FIFO__get(void );
#line 32
static bool CC2420ReceiveP__FIFOP__get(void );
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/Malloc.nc"
static OpenQueueEntry_t *CC2420ReceiveP__Malloc__getFreePacketBuffer(void );
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static error_t CC2420ReceiveP__taskReceiveDone__postTask(void );
# 7 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/RadioReceive.nc"
static void CC2420ReceiveP__RadioReceive__receive(OpenQueueEntry_t *msg);
# 29 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
static void CC2420ReceiveP__CSN__set(void );
static void CC2420ReceiveP__CSN__clr(void );
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420ReceivedNothing.nc"
static void CC2420ReceiveP__CC2420ReceivedNothing__receivedNothing(void );
# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t CC2420ReceiveP__SpiResource__release(void );
#line 87
static error_t CC2420ReceiveP__SpiResource__immediateRequest(void );
#line 78
static error_t CC2420ReceiveP__SpiResource__request(void );
#line 118
static bool CC2420ReceiveP__SpiResource__isOwner(void );
# 62 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Fifo.nc"
static error_t CC2420ReceiveP__RXFIFO__continueRead(uint8_t * data, uint8_t length);
#line 51
static cc2420_status_t CC2420ReceiveP__RXFIFO__beginRead(uint8_t * data, uint8_t length);
# 50 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioInterrupt.nc"
static error_t CC2420ReceiveP__InterruptFIFOP__disable(void );
#line 43
static error_t CC2420ReceiveP__InterruptFIFOP__enableFallingEdge(void );
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ReceiveP__SFLUSHRX__strobe(void );
# 57 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/receive/CC2420ReceiveP.nc"
enum CC2420ReceiveP____nesc_unnamed4422 {
#line 57
  CC2420ReceiveP__taskReceiveDone = 30U
};
#line 57
typedef int CC2420ReceiveP____nesc_sillytask_taskReceiveDone[CC2420ReceiveP__taskReceiveDone];
#line 38
#line 33
typedef enum CC2420ReceiveP____nesc_unnamed4423 {
  CC2420ReceiveP__S_STOPPED, 
  CC2420ReceiveP__S_STARTED, 
  CC2420ReceiveP__S_RX_LENGTH, 
  CC2420ReceiveP__S_RX_PAYLOAD
} CC2420ReceiveP__cc2420_receive_state_t;

enum CC2420ReceiveP____nesc_unnamed4424 {
  CC2420ReceiveP__RXFIFO_SIZE = 128, 
  CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE = 8, 
  CC2420ReceiveP__SACK_HEADER_LENGTH = 7
};

uint32_t CC2420ReceiveP__timestamp_queue[CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE];
uint8_t CC2420ReceiveP__timestamp_head;
uint8_t CC2420ReceiveP__timestamp_size;
uint8_t CC2420ReceiveP__missed_packets;
bool CC2420ReceiveP__receivingPacket;
uint8_t CC2420ReceiveP__free_bytes_left_in_fifo;
OpenQueueEntry_t *CC2420ReceiveP__receptionBuffer;
CC2420ReceiveP__cc2420_receive_state_t CC2420ReceiveP__state;





static void CC2420ReceiveP__reset(void );
static void CC2420ReceiveP__beginReceive(void );
static void CC2420ReceiveP__receiveLengthByte(void );
static void CC2420ReceiveP__waitForNextPacket(void );



static inline void CC2420ReceiveP__CC2420sfd__sfd(uint32_t time);
#line 79
static inline void CC2420ReceiveP__InterruptFIFOP__fired(void );







static void CC2420ReceiveP__beginReceive(void );
#line 99
static inline void CC2420ReceiveP__SpiResource__granted(void );



static void CC2420ReceiveP__receiveLengthByte(void );




static inline void CC2420ReceiveP__RXFIFO__readDone(uint8_t *rx_buf, uint8_t rx_len, error_t error);
#line 186
static inline void CC2420ReceiveP__taskReceiveDone__runTask(void );
#line 213
static void CC2420ReceiveP__waitForNextPacket(void );
#line 246
static inline error_t CC2420ReceiveP__Init__init(void );
#line 260
static error_t CC2420ReceiveP__StdAsyncControl__start(void );
#line 274
static void CC2420ReceiveP__reset(void );
#line 292
static inline error_t CC2420ReceiveP__StdAsyncControl__stop(void );









static inline void CC2420ReceiveP__RXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error);
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioCapture.nc"
static error_t CC2420TransmitP__CaptureSFD__captureFallingEdge(void );
#line 55
static void CC2420TransmitP__CaptureSFD__disable(void );
#line 42
static error_t CC2420TransmitP__CaptureSFD__captureRisingEdge(void );
# 3 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Transmit.nc"
static void CC2420TransmitP__CC2420Transmit__loadPacketDone(error_t error);

static void CC2420TransmitP__CC2420Transmit__sendNowDone(error_t error);
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420TransmitP__TXCTRL__write(uint16_t data);
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__SFLUSHTX__strobe(void );
# 35 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP__CSN__makeOutput(void );
#line 29
static void CC2420TransmitP__CSN__set(void );
static void CC2420TransmitP__CSN__clr(void );
# 8 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420sfd.nc"
static void CC2420TransmitP__CC2420sfd__sfd(uint32_t time);
# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
static error_t CC2420TransmitP__SpiResource__release(void );
#line 87
static error_t CC2420TransmitP__SpiResource__immediateRequest(void );
#line 78
static error_t CC2420TransmitP__SpiResource__request(void );
# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP__CCA__makeInput(void );
# 2 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/GlobalTime.nc"
static timervalue_t CC2420TransmitP__GlobalTime__getGlobalSlotOffset(void );
# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP__SFD__makeInput(void );
#line 32
static bool CC2420TransmitP__SFD__get(void );
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void CC2420TransmitP__AlarmWatchdog__start(CC2420TransmitP__AlarmWatchdog__size_type dt);






static void CC2420TransmitP__AlarmWatchdog__stop(void );
# 82 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Fifo.nc"
static cc2420_status_t CC2420TransmitP__TXFIFO__write(uint8_t * data, uint8_t length);
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__STXON__strobe(void );
# 47 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/transmit/CC2420TransmitP.nc"
#line 40
typedef enum CC2420TransmitP____nesc_unnamed4425 {
  CC2420TransmitP__S_STOPPED = 0, 
  CC2420TransmitP__S_READY = 1, 
  CC2420TransmitP__S_LOAD = 2, 
  CC2420TransmitP__S_LOADED = 3, 
  CC2420TransmitP__S_SFD = 4, 
  CC2420TransmitP__S_EFD = 5
} CC2420TransmitP__cc2420_transmit_state_t;

enum CC2420TransmitP____nesc_unnamed4426 {

  CC2420TransmitP__CC2420_ABORT_PERIOD_SFD = 32, 

  CC2420TransmitP__CC2420_ABORT_PERIOD_LOAD = 320
};

OpenQueueEntry_t *CC2420TransmitP__m_msg;
bool CC2420TransmitP__m_cca;
uint8_t CC2420TransmitP__m_tx_power;
CC2420TransmitP__cc2420_transmit_state_t CC2420TransmitP__state = CC2420TransmitP__S_STOPPED;
bool CC2420TransmitP__m_receiving = FALSE;
uint16_t CC2420TransmitP__m_prev_time;
bool CC2420TransmitP__sfdHigh;



static void CC2420TransmitP__loadTXFIFO(void );
static void CC2420TransmitP__send(void );
static inline void CC2420TransmitP__signalDone(error_t err);
static void CC2420TransmitP__reset(void );



static inline error_t CC2420TransmitP__CC2420Transmit__loadPacket(OpenQueueEntry_t *p_msg);
#line 94
static inline void CC2420TransmitP__SpiResource__granted(void );
#line 112
static void CC2420TransmitP__loadTXFIFO(void );
#line 129
static inline void CC2420TransmitP__AlarmWatchdog__fired(void );
#line 147
static inline void CC2420TransmitP__TXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error);
#line 164
static inline error_t CC2420TransmitP__CC2420Transmit__sendNow(bool useCca);
#line 183
static void CC2420TransmitP__send(void );
#line 207
static inline void CC2420TransmitP__CaptureSFD__captured(uint16_t time);
#line 295
static inline void CC2420TransmitP__signalDone(error_t err);







static void CC2420TransmitP__reset(void );







static inline error_t CC2420TransmitP__Init__init(void );






static error_t CC2420TransmitP__StdAsyncControl__start(void );










static inline error_t CC2420TransmitP__StdAsyncControl__stop(void );










static inline void CC2420TransmitP__ChipSpiResource__releasing(void );


static inline void CC2420TransmitP__TXFIFO__readDone(uint8_t *tx_buf, uint8_t tx_len, error_t error);
# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Compare__setEvent(uint16_t time);

static void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Compare__setEventFromNow(uint16_t delta);
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Timer__get(void );
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__fired(void );
# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430TimerControl__enableEvents(void );
static void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430TimerControl__disableEvents(void );
#line 33
static void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430TimerControl__clearPendingInterrupt(void );
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__stop(void );




static inline void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Compare__fired(void );










static inline void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Timer__overflow(void );
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__fired(void );
#line 92
static void /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__AlarmFrom__startAt(/*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__AlarmFrom__size_type t0, /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__AlarmFrom__size_type dt);
#line 62
static void /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__AlarmFrom__stop(void );
# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
static /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Counter__size_type /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Counter__get(void );
# 66 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
/*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__to_size_type /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__m_t0;
/*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__to_size_type /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__m_dt;

enum /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4____nesc_unnamed4427 {

  TransformAlarmC__4__MAX_DELAY_LOG2 = 8 * sizeof(/*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__from_size_type ) - 1 - 0, 
  TransformAlarmC__4__MAX_DELAY = (/*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__to_size_type )1 << /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__MAX_DELAY_LOG2
};

static inline /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__to_size_type /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__getNow(void );
#line 91
static inline void /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__stop(void );




static void /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__set_alarm(void );
#line 136
static void /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__startAt(/*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__to_size_type t0, /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__to_size_type dt);









static inline void /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__start(/*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__to_size_type dt);




static inline void /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__AlarmFrom__fired(void );
#line 166
static inline void /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Counter__overflow(void );
# 57 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow(void );
# 50 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioCapture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captured(uint16_t time);
# 44 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(uint8_t cm);

static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__enableEvents(void );
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents(void );
#line 33
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectIOFunc(void );
#line 78
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectModuleFunc(void );
# 38 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/GpioCaptureC.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(uint8_t mode);
#line 50
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge(void );



static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge(void );



static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable(void );






static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time);
# 41 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__HplInterrupt__clear(void );
#line 36
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__HplInterrupt__disable(void );
#line 56
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__HplInterrupt__edge(bool low_to_high);
#line 31
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__HplInterrupt__enable(void );
# 57 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioInterrupt.nc"
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__Interrupt__fired(void );
# 41 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__enable(bool rising);








static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__Interrupt__enableRisingEdge(void );







static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__Interrupt__disable(void );







static inline void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__HplInterrupt__fired(void );
# 41 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__HplInterrupt__clear(void );
#line 36
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__HplInterrupt__disable(void );
#line 56
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__HplInterrupt__edge(bool low_to_high);
#line 31
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__HplInterrupt__enable(void );
# 57 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioInterrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__Interrupt__fired(void );
# 41 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__enable(bool rising);
#line 54
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__Interrupt__enableFallingEdge(void );



static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__Interrupt__disable(void );







static inline void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__HplInterrupt__fired(void );
# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430Compare__setEvent(uint16_t time);

static void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430Compare__setEventFromNow(uint16_t delta);
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430Timer__get(void );
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Alarm__fired(void );
# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430TimerControl__enableEvents(void );
static void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430TimerControl__disableEvents(void );
#line 33
static void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430TimerControl__clearPendingInterrupt(void );
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Alarm__stop(void );




static inline void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430Compare__fired(void );










static inline void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430Timer__overflow(void );
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
static void /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__fired(void );
#line 92
static void /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__AlarmFrom__startAt(/*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__AlarmFrom__size_type t0, /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__AlarmFrom__size_type dt);
#line 62
static void /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__AlarmFrom__stop(void );
# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
static /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Counter__size_type /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Counter__get(void );
# 66 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
/*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__to_size_type /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__m_t0;
/*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__to_size_type /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__m_dt;

enum /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5____nesc_unnamed4428 {

  TransformAlarmC__5__MAX_DELAY_LOG2 = 8 * sizeof(/*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__from_size_type ) - 1 - 0, 
  TransformAlarmC__5__MAX_DELAY = (/*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__to_size_type )1 << /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__MAX_DELAY_LOG2
};

static inline /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__to_size_type /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__getNow(void );
#line 91
static inline void /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__stop(void );




static void /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__set_alarm(void );
#line 136
static inline void /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__startAt(/*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__to_size_type t0, /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__to_size_type dt);









static inline void /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__start(/*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__to_size_type dt);




static inline void /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__AlarmFrom__fired(void );
#line 166
static inline void /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Counter__overflow(void );
# 21 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/openqueue/OpenQueueP.nc"
enum OpenQueueP____nesc_unnamed4429 {
#line 21
  OpenQueueP__taskDebugPrint = 31U
};
#line 21
typedef int OpenQueueP____nesc_sillytask_taskDebugPrint[OpenQueueP__taskDebugPrint];
#line 17
OpenQueueEntry_t OpenQueueP__queue[QUEUELENGTH];




static void OpenQueueP__reset_entry(uint8_t i);



static inline void OpenQueueP__taskDebugPrint__runTask(void );
#line 38
static void OpenQueueP__reset_entry(uint8_t i);
#line 59
static inline error_t OpenQueueP__SoftwareInit__init(void );








static OpenQueueEntry_t *OpenQueueP__Malloc__getFreePacketBuffer(void );









static error_t OpenQueueP__Malloc__freePacketBuffer(OpenQueueEntry_t *pkt);
#line 99
static OpenQueueEntry_t *OpenQueueP__OpenQueue__inQueueBySeed(uint8_t seedNumber);








static inline OpenQueueEntry_t *OpenQueueP__OpenQueue__inQueueToChild(uint8_t seedNumber);
# 212 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_enable_interrupt(void )
{
   __asm volatile ("eint");}

# 185 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void )
{
}

# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void ){
#line 37
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow();
#line 37
}
#line 37
# 126 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow();
}





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n)
{
}

# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(uint8_t arg_0x4062c4f0){
#line 28
  switch (arg_0x4062c4f0) {
#line 28
    case 0:
#line 28
      /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired();
#line 28
      break;
#line 28
    case 1:
#line 28
      /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired();
#line 28
      break;
#line 28
    case 2:
#line 28
      /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired();
#line 28
      break;
#line 28
    case 5:
#line 28
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired();
#line 28
      break;
#line 28
    default:
#line 28
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(arg_0x4062c4f0);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 115 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(0);
}

# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA0__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired();
#line 28
}
#line 28
# 47 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4430 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(* (volatile uint16_t * )354U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void )
{
  return * (volatile uint16_t * )370U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void )
{
}

# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired();
#line 34
}
#line 34
# 47 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4431 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(* (volatile uint16_t * )356U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void )
{
  return * (volatile uint16_t * )372U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void )
{
}

# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired();
#line 34
}
#line 34
# 47 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2____nesc_unnamed4432 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(* (volatile uint16_t * )358U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void )
{
  return * (volatile uint16_t * )374U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void )
{
}

# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired();
#line 34
}
#line 34
# 120 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )302U;

#line 123
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(n >> 1);
}

# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA1__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired();
#line 28
}
#line 28
# 115 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(0);
}

# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB0__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired();
#line 28
}
#line 28
# 185 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void )
{
}

# 208 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__overflow(void )
#line 208
{
}

# 47 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void )
{
}

# 166 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void )
{
}

# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
inline static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void ){
#line 71
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow();
#line 71
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow();
#line 71
}
#line 71
# 122 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformCounterC.nc"
static inline void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper++;
    if ((/*CounterMilli32C.Transform*/TransformCounterC__0__m_upper & /*CounterMilli32C.Transform*/TransformCounterC__0__OVERFLOW_MASK) == 0) {
      /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow();
      }
  }
}

# 166 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static inline void /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Counter__overflow(void )
{
}

#line 166
static inline void /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Counter__overflow(void )
{
}

#line 166
static inline void /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Counter__overflow(void )
{
}

#line 166
static inline void /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Counter__overflow(void )
{
}

#line 166
static inline void /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Counter__overflow(void )
{
}

# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
inline static void /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__overflow(void ){
#line 71
  /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Counter__overflow();
#line 71
  /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Counter__overflow();
#line 71
  /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Counter__overflow();
#line 71
  /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Counter__overflow();
#line 71
  /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Counter__overflow();
#line 71
}
#line 71
# 122 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformCounterC.nc"
static inline void /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*Counter32khz32C.Transform*/TransformCounterC__1__m_upper++;
    if ((/*Counter32khz32C.Transform*/TransformCounterC__1__m_upper & /*Counter32khz32C.Transform*/TransformCounterC__1__OVERFLOW_MASK) == 0) {
      /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__overflow();
      }
  }
}

# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void ){
#line 71
  /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__overflow();
#line 71
  /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow();
#line 71
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__overflow();
#line 71
}
#line 71
# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void )
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow();
}

# 103 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430Timer__overflow(void )
{
}

#line 103
static inline void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Timer__overflow(void )
{
}

#line 103
static inline void /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__overflow(void )
{
}

#line 103
static inline void /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow(void )
{
}

#line 103
static inline void /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void )
{
}

#line 103
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void )
{
}

# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void ){
#line 37
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow();
#line 37
  /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow();
#line 37
  /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow();
#line 37
  /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__overflow();
#line 37
  /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Timer__overflow();
#line 37
  /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430Timer__overflow();
#line 37
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow();
#line 37
}
#line 37
# 126 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow();
}

# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 70 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void )
{
#line 71
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask();
}

# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired();
#line 67
}
#line 67
# 151 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt == 0) 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired();
      }
    else 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm();
      }
  }
}

# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void ){
#line 67
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired();
#line 67
}
#line 67
# 124 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void )
{
  * (volatile uint16_t * )386U &= ~0x0010;
}

# 47 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents();
#line 47
}
#line 47
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired();
}

# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void ){
#line 34
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired();
#line 34
}
#line 34
# 139 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void )
{
  return * (volatile uint16_t * )402U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4433 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(* (volatile uint16_t * )386U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired();
    }
}

# 86 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP__isWaiting(uint8_t id)
{
  return SchedulerBasicP__m_next[id] != SchedulerBasicP__NO_TASK || SchedulerBasicP__m_tail == id;
}

static inline bool SchedulerBasicP__pushTask(uint8_t id)
{
  if (!SchedulerBasicP__isWaiting(id)) 
    {
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_head = id;
          SchedulerBasicP__m_tail = id;
        }
      else 
        {
          SchedulerBasicP__m_next[SchedulerBasicP__m_tail] = id;
          SchedulerBasicP__m_tail = id;
        }
      return TRUE;
    }
  else 
    {
      return FALSE;
    }
}

# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 38 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get();
}

# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
inline static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void ){
#line 53
  unsigned int __nesc_result;
#line 53

#line 53
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 70 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void )
{
  return * (volatile uint16_t * )384U & 1U;
}

# 35 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
inline static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void ){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending();
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending();
}

# 60 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
inline static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 119 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void )
{
  * (volatile uint16_t * )386U |= 0x0010;
}

# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents();
#line 46
}
#line 46
# 84 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )386U &= ~0x0001;
}

# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )402U = x;
}

# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(time);
#line 30
}
#line 30
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 154 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )402U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get() + x;
}

# 32 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 70 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 86
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 88
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt();
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents();
  }
}

# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 181 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void )
{
}

# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired();
#line 34
}
#line 34
# 139 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void )
{
  return * (volatile uint16_t * )404U;
}

# 50 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge(void )
#line 50
{
  return /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(MSP430TIMER_CM_RISING);
}

# 42 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioCapture.nc"
inline static error_t CC2420TransmitP__CaptureSFD__captureRisingEdge(void ){
#line 42
  unsigned char __nesc_result;
#line 42

#line 42
  __nesc_result = /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge();
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw(void )
#line 48
{
#line 48
  return * (volatile uint8_t * )28U & (0x01 << 1);
}

#line 49
static inline bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get(void )
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw() != 0;
}

# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__12__HplGeneralIO__get(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 40 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__12__GeneralIO__get(void )
#line 40
{
#line 40
  return /*HplCC2420PinsC.SFDM*/Msp430GpioC__12__HplGeneralIO__get();
}

# 32 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static bool CC2420TransmitP__SFD__get(void ){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = /*HplCC2420PinsC.SFDM*/Msp430GpioC__12__GeneralIO__get();
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 124 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__disableEvents(void )
{
  * (volatile uint16_t * )398U &= ~0x0010;
}

# 47 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__disableEvents();
#line 47
}
#line 47
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Alarm__stop(void )
{
  /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430TimerControl__disableEvents();
}

# 62 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__AlarmFrom__stop(void ){
#line 62
  /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Alarm__stop();
#line 62
}
#line 62
# 91 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static inline void /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__stop(void )
{
  /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__AlarmFrom__stop();
}

# 62 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void CC2420DriversP__RxWaitAlarm__stop(void ){
#line 62
  /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__stop();
#line 62
}
#line 62
# 127 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/CC2420DriversP.nc"
static inline void CC2420DriversP__CC2420sfd__sfd(uint32_t txTimestamp)
#line 127
{
  CC2420DriversP__RxWaitAlarm__stop();
}

# 66 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__CC2420sfd__sfd(uint32_t time)
#line 66
{
  if (CC2420ReceiveP__timestamp_size < CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE) {
      uint8_t tail = (CC2420ReceiveP__timestamp_head + CC2420ReceiveP__timestamp_size) % CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE;

#line 69
      CC2420ReceiveP__timestamp_queue[tail] = time;
      CC2420ReceiveP__timestamp_size++;
    }
}

# 8 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420sfd.nc"
inline static void CC2420TransmitP__CC2420sfd__sfd(uint32_t time){
#line 8
  CC2420ReceiveP__CC2420sfd__sfd(time);
#line 8
  CC2420DriversP__CC2420sfd__sfd(time);
#line 8
}
#line 8
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge(void )
#line 54
{
  return /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(MSP430TIMER_CM_FALLING);
}

# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioCapture.nc"
inline static error_t CC2420TransmitP__CaptureSFD__captureFallingEdge(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420DriversP__taskSendDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420DriversP__taskSendDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 226 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/CC2420DriversP.nc"
static inline void CC2420DriversP__CC2420Transmit__sendNowDone(error_t err)
#line 226
{
  /* atomic removed: atomic calls only */
#line 227
  CC2420DriversP__sendErr = err;
  CC2420DriversP__taskSendDone__postTask();
}

# 5 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Transmit.nc"
inline static void CC2420TransmitP__CC2420Transmit__sendNowDone(error_t error){
#line 5
  CC2420DriversP__CC2420Transmit__sendNowDone(error);
#line 5
}
#line 5
# 295 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__signalDone(error_t err)
#line 295
{
  CC2420TransmitP__CaptureSFD__captureRisingEdge();
  /* atomic removed: atomic calls only */
#line 297
  CC2420TransmitP__state = CC2420TransmitP__S_READY;
  CC2420TransmitP__CC2420Transmit__sendNowDone(err);
}

# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP__SpiResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420TransmitC.CC2420SpiC*/CC2420SpiC__7__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 124 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__disableEvents(void )
{
  * (volatile uint16_t * )396U &= ~0x0010;
}

# 47 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__disableEvents();
#line 47
}
#line 47
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__stop(void )
{
  /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430TimerControl__disableEvents();
}

# 62 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__AlarmFrom__stop(void ){
#line 62
  /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__stop();
#line 62
}
#line 62
# 91 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static inline void /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__stop(void )
{
  /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__AlarmFrom__stop();
}

# 62 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void CC2420TransmitP__AlarmWatchdog__stop(void ){
#line 62
  /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__stop();
#line 62
}
#line 62
# 2 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/GlobalTime.nc"
inline static timervalue_t CC2420TransmitP__GlobalTime__getGlobalSlotOffset(void ){
#line 2
  unsigned long __nesc_result;
#line 2

#line 2
  __nesc_result = LatinMacP__GlobalTime__getGlobalSlotOffset();
#line 2

#line 2
  return __nesc_result;
#line 2
}
#line 2
# 207 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__CaptureSFD__captured(uint16_t time)
#line 207
{
  uint32_t globalSlotOffset;
  uint8_t sfd_state = 0;
  bool falling_through = FALSE;

  /* atomic removed: atomic calls only */
#line 211
  {
    globalSlotOffset = (unsigned long )CC2420TransmitP__GlobalTime__getGlobalSlotOffset();
    switch (CC2420TransmitP__state) {
        case CC2420TransmitP__S_SFD: 
          CC2420TransmitP__AlarmWatchdog__stop();
        CC2420TransmitP__SpiResource__release();
        CC2420TransmitP__state = CC2420TransmitP__S_EFD;
        CC2420TransmitP__sfdHigh = TRUE;


        CC2420TransmitP__m_receiving = FALSE;
        CC2420TransmitP__CaptureSFD__captureFallingEdge();

        if (CC2420TransmitP__SFD__get()) {


            break;
          }



        case CC2420TransmitP__S_EFD: 
          CC2420TransmitP__sfdHigh = FALSE;
        CC2420TransmitP__signalDone(SUCCESS);
        if (!CC2420TransmitP__SFD__get()) {

            break;
          }


        default: 
          if (CC2420TransmitP__m_receiving == FALSE && CC2420TransmitP__sfdHigh == FALSE) {
              CC2420TransmitP__sfdHigh = TRUE;
              CC2420TransmitP__CaptureSFD__captureFallingEdge();
              sfd_state = CC2420TransmitP__SFD__get();
              falling_through = TRUE;
              CC2420TransmitP__m_receiving = TRUE;
              CC2420TransmitP__m_prev_time = time;
              CC2420TransmitP__CC2420sfd__sfd(globalSlotOffset);
              if (CC2420TransmitP__SFD__get()) {


                  return;
                }
            }


        if (CC2420TransmitP__sfdHigh == TRUE) {
            CC2420TransmitP__sfdHigh = FALSE;
            CC2420TransmitP__CaptureSFD__captureRisingEdge();
            CC2420TransmitP__m_receiving = FALSE;
#line 284
            break;
          }
      }
  }
}

# 50 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioCapture.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captured(uint16_t time){
#line 50
  CC2420TransmitP__CaptureSFD__captured(time);
#line 50
}
#line 50
# 164 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void )
{
  * (volatile uint16_t * )388U &= ~0x0002;
}

# 57 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow();
#line 57
}
#line 57
# 84 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )388U &= ~0x0001;
}

# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 65 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time)
#line 65
{
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt();
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow();
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captured(time);
}

# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time){
#line 75
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(time);
#line 75
}
#line 75
# 47 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4434 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(* (volatile uint16_t * )388U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired();
    }
}

# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
inline static /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__size_type /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__get(void ){
#line 53
  unsigned int __nesc_result;
#line 53

#line 53
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53







inline static bool /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 118 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/StateImplP.nc"
static inline void StateImplP__State__toIdle(uint8_t id)
#line 118
{
  /* atomic removed: atomic calls only */
#line 119
  StateImplP__state[id] = StateImplP__S_IDLE;
}

# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/State.nc"
inline static void CC2420SpiP__WorkingState__toIdle(void ){
#line 56
  StateImplP__State__toIdle(0U);
#line 56
}
#line 56
# 205 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__granted(void )
#line 205
{
}

# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted(void ){
#line 46
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__granted();
#line 46
}
#line 46
# 151 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__resetUsart(bool reset)
#line 151
{
  if (reset) {
      U0CTL = 0x01;
    }
  else {
      U0CTL &= ~0x01;
    }
}

# 97 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__resetUsart(bool reset){
#line 97
  HplMsp430Usart0P__Usart__resetUsart(reset);
#line 97
}
#line 97
#line 158
inline static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__disableSpi(void ){
#line 158
  HplMsp430Usart0P__Usart__disableSpi();
#line 158
}
#line 158
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__ResourceConfigure__unconfigure(uint8_t id)
#line 92
{
  /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__resetUsart(TRUE);
  /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__disableSpi();
  /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__resetUsart(FALSE);
}

# 215 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(uint8_t id)
#line 215
{
}

# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(uint8_t arg_0x40aaacf8){
#line 55
  switch (arg_0x40aaacf8) {
#line 55
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 55
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__ResourceConfigure__unconfigure(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 55
      break;
#line 55
    default:
#line 55
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(arg_0x40aaacf8);
#line 55
      break;
#line 55
    }
#line 55
}
#line 55
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 58 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead != /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
        uint8_t id = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead;

#line 62
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead];
        if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
          }
#line 65
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[id] = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
        {
          unsigned char __nesc_temp = 
#line 66
          id;

#line 66
          return __nesc_temp;
        }
      }
#line 68
    {
      unsigned char __nesc_temp = 
#line 68
      /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

#line 68
      return __nesc_temp;
    }
  }
}

# 60 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__dequeue(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 50 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 51
  {
    unsigned char __nesc_temp = 
#line 51
    /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

#line 51
    return __nesc_temp;
  }
}

# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceQueue.nc"
inline static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__isEmpty(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 108 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(uint8_t id)
#line 108
{
  /* atomic removed: atomic calls only */
#line 109
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY && /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId == id) {
        if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__isEmpty() == FALSE) {
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__dequeue();
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__NO_RES;
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_GRANTING;
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask();
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(id);
          }
        else {
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__default_owner_id;
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED;
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(id);
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted();
          }
        {
          unsigned char __nesc_temp = 
#line 124
          SUCCESS;

#line 124
          return __nesc_temp;
        }
      }
  }
#line 127
  return FAIL;
}

# 109 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__release(uint8_t id)
#line 109
{
#line 109
  return FAIL;
}

# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__release(uint8_t arg_0x410c29a0){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  switch (arg_0x410c29a0) {
#line 110
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 110
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 110
      break;
#line 110
    default:
#line 110
      __nesc_result = /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__release(arg_0x410c29a0);
#line 110
      break;
#line 110
    }
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 84 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__release(uint8_t id)
#line 84
{
  return /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__release(id);
}

# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__release(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 340 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__ChipSpiResource__releasing(void )
#line 340
{
}

# 24 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/ChipSpiResource.nc"
inline static void CC2420SpiP__ChipSpiResource__releasing(void ){
#line 24
  CC2420TransmitP__ChipSpiResource__releasing();
#line 24
}
#line 24
# 133 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/StateImplP.nc"
static inline bool StateImplP__State__isState(uint8_t id, uint8_t myState)
#line 133
{
  bool isState;

  /* atomic removed: atomic calls only */
#line 135
  isState = StateImplP__state[id] == myState;
  return isState;
}

#line 126
static inline bool StateImplP__State__isIdle(uint8_t id)
#line 126
{
  return StateImplP__State__isState(id, StateImplP__S_IDLE);
}

# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/State.nc"
inline static bool CC2420SpiP__WorkingState__isIdle(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = StateImplP__State__isIdle(0U);
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 339 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
static inline error_t CC2420SpiP__attemptRelease(void )
#line 339
{


  if ((
#line 340
  CC2420SpiP__m_requests > 0
   || CC2420SpiP__m_holder != CC2420SpiP__NO_HOLDER)
   || !CC2420SpiP__WorkingState__isIdle()) {
      return FAIL;
    }
  /* atomic removed: atomic calls only */
  CC2420SpiP__release = TRUE;
  CC2420SpiP__ChipSpiResource__releasing();
  /* atomic removed: atomic calls only */
#line 348
  {
    if (CC2420SpiP__release) {
        CC2420SpiP__SpiResource__release();
        {
          unsigned char __nesc_temp = 
#line 351
          SUCCESS;

#line 351
          return __nesc_temp;
        }
      }
  }
  return EBUSY;
}

# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 1);
}

# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SIMO__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 2);
}

# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SOMI__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 3);
}

# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UCLK__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc();
#line 85
}
#line 85
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )31U |= 0x01 << 1;
}

# 78 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc();
#line 78
}
#line 78
# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4435 {
#line 46
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

#line 61
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__captureControl(uint8_t l_cm)
{
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x = { 
  .cm = l_cm & 0x03, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 1, 
  .scs = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(x);
}

#line 99
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(uint8_t cm)
{
  * (volatile uint16_t * )388U = /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__captureControl(cm);
}

# 44 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(uint8_t cm){
#line 44
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(cm);
#line 44
}
#line 44
# 119 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void )
{
  * (volatile uint16_t * )388U |= 0x0010;
}

# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents();
#line 46
}
#line 46
# 281 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_hton_leuint16(void * target, uint16_t value)
#line 281
{
  uint8_t *base = target;

#line 283
  base[0] = value;
  base[1] = value >> 8;
  return value;
}

#line 246
static __inline  uint8_t __nesc_ntoh_leuint8(const void * source)
#line 246
{
  const uint8_t *base = source;

#line 248
  return base[0];
}

# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void LatinMacP__FastAlarm__startAt(LatinMacP__FastAlarm__size_type t0, LatinMacP__FastAlarm__size_type dt){
#line 92
  /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 647 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02b-RES/LatinRes/LatinResP.nc"
static inline bool LatinResP__LatinMatrix__canTransmit(uint8_t minLatinWindow, uint8_t *latinChannelOffset)
#line 647
{
  uint8_t i;

#line 649
  for (i = 0; i < NUMCHANNELS; i = i + 1) {
      if ((minLatinWindow + i) % MAXNUMNEIGHBORS == TOS_NODE_ID) {
          *latinChannelOffset = i;
          return TRUE;
        }
    }
  return FALSE;
}

# 5 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/LatinMatrix.nc"
inline static bool LatinMacP__LatinMatrix__canTransmit(uint8_t minLatinWindow, uint8_t *latinChannelOffset){
#line 5
  unsigned char __nesc_result;
#line 5

#line 5
  __nesc_result = LatinResP__LatinMatrix__canTransmit(minLatinWindow, latinChannelOffset);
#line 5

#line 5
  return __nesc_result;
#line 5
}
#line 5
# 658 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02b-RES/LatinRes/LatinResP.nc"
static inline uint8_t LatinResP__LatinMatrix__getParentSeed(void )
#line 658
{
  return LatinResP__neighborList.neighbors[LatinResP__neighborList.parent].seedNumber;
}

# 6 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/LatinMatrix.nc"
inline static uint8_t LatinMacP__LatinMatrix__getParentSeed(void ){
#line 6
  unsigned char __nesc_result;
#line 6

#line 6
  __nesc_result = LatinResP__LatinMatrix__getParentSeed();
#line 6

#line 6
  return __nesc_result;
#line 6
}
#line 6
# 108 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/openqueue/OpenQueueP.nc"
static inline OpenQueueEntry_t *OpenQueueP__OpenQueue__inQueueToChild(uint8_t seedNumber)
#line 108
{
  uint8_t i;

#line 110
  for (i = 0; i < QUEUELENGTH; i++) {
      if (OpenQueueP__queue[i].l2_seedNumber != 255 && OpenQueueP__queue[i].l2_seedNumber != seedNumber) {
        return &OpenQueueP__queue[i];
        }
    }
#line 114
  return (void *)0;
}

# 5 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/OpenQueue.nc"
inline static OpenQueueEntry_t *LatinMacP__OpenQueue__inQueueToChild(uint8_t seedNumber){
#line 5
  struct OpenQueueEntry_t *__nesc_result;
#line 5

#line 5
  __nesc_result = OpenQueueP__OpenQueue__inQueueToChild(seedNumber);
#line 5

#line 5
  return __nesc_result;
#line 5
}
#line 5
# 17 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/RadioControl.nc"
inline static error_t LatinMacP__RadioControl__prepareReceive(uint8_t frequencyChannel){
#line 17
  unsigned char __nesc_result;
#line 17

#line 17
  __nesc_result = CC2420DriversP__RadioControl__prepareReceive(frequencyChannel);
#line 17

#line 17
  return __nesc_result;
#line 17
}
#line 17
# 636 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02b-RES/LatinRes/LatinResP.nc"
static inline bool LatinResP__LatinMatrix__parentCanTransmit(uint8_t minLatinWindow, uint8_t *latinChannelOffset)
#line 636
{
  uint8_t i;

#line 638
  for (i = 0; i < NUMCHANNELS; i = i + 1) {
      if ((minLatinWindow + i) % MAXNUMNEIGHBORS == LatinResP__neighborList.neighbors[LatinResP__neighborList.parent].seedNumber) {
          *latinChannelOffset = i;
          return TRUE;
        }
    }
  return FALSE;
}

# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/LatinMatrix.nc"
inline static bool LatinMacP__LatinMatrix__parentCanTransmit(uint8_t minLatinWindow, uint8_t *latinChannelOffset){
#line 4
  unsigned char __nesc_result;
#line 4

#line 4
  __nesc_result = LatinResP__LatinMatrix__parentCanTransmit(minLatinWindow, latinChannelOffset);
#line 4

#line 4
  return __nesc_result;
#line 4
}
#line 4
# 618 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02b-RES/LatinRes/LatinResP.nc"
static inline bool LatinResP__LatinMatrix__siblingCanTransmit(uint8_t minLatinWindow, uint8_t *latinChannelOffset)
#line 618
{
  uint8_t i;
#line 619
  uint8_t j;

#line 620
  for (i = 0; i < NUMCHANNELS; i = i + 1) {
      for (j = 0; j < LatinResP__neighborList.numSibling; j = j + 1) {
          if ((minLatinWindow + i) % MAXNUMNEIGHBORS == LatinResP__neighborList.siblingList[j]) {
              if (LatinResP__neighborList.siblingList[j] == TOS_NODE_ID) {
                  *latinChannelOffset = i;
                  return FALSE;
                }
              else 
#line 626
                {
                  *latinChannelOffset = i;
                  return TRUE;
                }
            }
        }
    }
  return TRUE;
}

# 3 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/LatinMatrix.nc"
inline static bool LatinMacP__LatinMatrix__siblingCanTransmit(uint8_t minLatinWindow, uint8_t *latinChannelOffset){
#line 3
  unsigned char __nesc_result;
#line 3

#line 3
  __nesc_result = LatinResP__LatinMatrix__siblingCanTransmit(minLatinWindow, latinChannelOffset);
#line 3

#line 3
  return __nesc_result;
#line 3
}
#line 3
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/OpenQueue.nc"
inline static OpenQueueEntry_t *LatinMacP__OpenQueue__inQueueBySeed(uint8_t seedNumber){
#line 4
  struct OpenQueueEntry_t *__nesc_result;
#line 4

#line 4
  __nesc_result = OpenQueueP__OpenQueue__inQueueBySeed(seedNumber);
#line 4

#line 4
  return __nesc_result;
#line 4
}
#line 4
# 605 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02b-RES/LatinRes/LatinResP.nc"
static inline bool LatinResP__LatinMatrix__childCanTransmit(uint8_t minLatinWindow, uint8_t *latinChannelOffset)
#line 605
{
  uint8_t i;
#line 606
  uint8_t j;

#line 607
  for (i = 0; i < NUMCHANNELS; i = i + 1) {
      for (j = 0; j < LatinResP__neighborList.numChildren; j = j + 1) {
          if ((minLatinWindow + i) % MAXNUMNEIGHBORS == LatinResP__neighborList.seedList[j]) {
              *latinChannelOffset = i;
              return TRUE;
            }
        }
    }
  return FALSE;
}

# 2 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/LatinMatrix.nc"
inline static bool LatinMacP__LatinMatrix__childCanTransmit(uint8_t minLatinWindow, uint8_t *latinChannelOffset){
#line 2
  unsigned char __nesc_result;
#line 2

#line 2
  __nesc_result = LatinResP__LatinMatrix__childCanTransmit(minLatinWindow, latinChannelOffset);
#line 2

#line 2
  return __nesc_result;
#line 2
}
#line 2
# 78 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP__SpiResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420TransmitC.CC2420SpiC*/CC2420SpiC__7__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78









inline static error_t CC2420TransmitP__SpiResource__immediateRequest(void ){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = CC2420SpiP__Resource__immediateRequest(/*CC2420TransmitC.CC2420SpiC*/CC2420SpiC__7__CLIENT_ID);
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 73 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__CC2420Transmit__loadPacket(OpenQueueEntry_t *p_msg)
#line 73
{
  /* atomic removed: atomic calls only */
  {
    if (CC2420TransmitP__state != CC2420TransmitP__S_READY) {
        CC2420TransmitP__reset();



        {
          unsigned char __nesc_temp = 
#line 81
          FAIL;

#line 81
          return __nesc_temp;
        }
      }
#line 83
    CC2420TransmitP__state = CC2420TransmitP__S_LOAD;
    CC2420TransmitP__m_msg = p_msg;
  }
  if (CC2420TransmitP__SpiResource__immediateRequest() == SUCCESS) {
      CC2420TransmitP__loadTXFIFO();
    }
  else 
#line 88
    {
      CC2420TransmitP__SpiResource__request();
    }
  return SUCCESS;
}

# 2 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Transmit.nc"
inline static error_t CC2420DriversP__CC2420Transmit__loadPacket(OpenQueueEntry_t *msg){
#line 2
  unsigned char __nesc_result;
#line 2

#line 2
  __nesc_result = CC2420TransmitP__CC2420Transmit__loadPacket(msg);
#line 2

#line 2
  return __nesc_result;
#line 2
}
#line 2
# 152 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/CC2420DriversP.nc"
static inline error_t CC2420DriversP__RadioSend__prepareSend(OpenQueueEntry_t *msg)
#line 152
{
  uint8_t temp_state;
  bool temp_timeLimitedRx;
  uint16_t temp_rxWaitTime;

  /* atomic removed: atomic calls only */
#line 156
  {
    temp_state = CC2420DriversP__state;
    temp_timeLimitedRx = CC2420DriversP__timeLimitedRx;
    temp_rxWaitTime = CC2420DriversP__rxWaitTime;
  }

  if (temp_state != CC2420DriversP__S_STARTED && temp_state != CC2420DriversP__S_RECEIVING) {



      if (temp_timeLimitedRx == FALSE) {
          CC2420DriversP__RadioControl__receiveNow(temp_timeLimitedRx, temp_rxWaitTime);
        }
      else 
#line 168
        {
          CC2420DriversP__RadioControl__rfOff();
        }
      return FAIL;
    }
  /* atomic removed: atomic calls only */
#line 173
  {
    CC2420DriversP__state = CC2420DriversP__S_LOADING_PACKET;
    CC2420DriversP__frequencyChannel = msg->l1_channel;
  }
  CC2420DriversP__CC2420Transmit__loadPacket(msg);
  return SUCCESS;
}

# 8 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/RadioSend.nc"
inline static error_t LatinMacP__RadioSend__prepareSend(OpenQueueEntry_t *msg){
#line 8
  unsigned char __nesc_result;
#line 8

#line 8
  __nesc_result = CC2420DriversP__RadioSend__prepareSend(msg);
#line 8

#line 8
  return __nesc_result;
#line 8
}
#line 8
# 967 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/LatinMac/LatinMacP.nc"
static inline asn_t LatinMacP__GlobalTime__getASN(void )
#line 967
{
  /* atomic removed: atomic calls only */
#line 968
  {
    unsigned int __nesc_temp = 
#line 968
    LatinMacP__asn;

#line 968
    return __nesc_temp;
  }
}

# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr();
#line 39
}
#line 39
# 38 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void )
#line 38
{
#line 38
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr();
}

# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__clr(void ){
#line 30
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr();
#line 30
}
#line 30
# 93 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/LedsP.nc"
static inline void LedsP__Leds__led2On(void )
#line 93
{
  LedsP__Led2__clr();
  ;
#line 95
  ;
}

# 78 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Leds.nc"
inline static void LatinMacP__Leds__led2On(void ){
#line 78
  LedsP__Leds__led2On();
#line 78
}
#line 78
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/IDManager.nc"
inline static bool LatinMacP__IDManager__getIsDAGroot(void ){
#line 4
  unsigned char __nesc_result;
#line 4

#line 4
  __nesc_result = IDManagerP__IDManager__getIsDAGroot();
#line 4

#line 4
  return __nesc_result;
#line 4
}
#line 4
# 80 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static inline /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__to_size_type /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Alarm__getAlarm(void )
{
  /* atomic removed: atomic calls only */
#line 82
  {
    /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__to_size_type __nesc_temp = 
#line 82
    /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__m_t0 + /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__m_dt;

#line 82
    return __nesc_temp;
  }
}

# 105 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static LatinMacP__SlotAlarm__size_type LatinMacP__SlotAlarm__getAlarm(void ){
#line 105
  unsigned long __nesc_result;
#line 105

#line 105
  __nesc_result = /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Alarm__getAlarm();
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
#line 92
inline static void LatinMacP__SlotAlarm__startAt(LatinMacP__SlotAlarm__size_type t0, LatinMacP__SlotAlarm__size_type dt){
#line 92
  /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
inline static /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Counter__size_type /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Counter__get(void ){
#line 53
  unsigned long __nesc_result;
#line 53

#line 53
  __nesc_result = /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static inline /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__to_size_type /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Alarm__getNow(void )
{
  return /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Counter__get();
}

# 98 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static LatinMacP__SlotAlarm__size_type LatinMacP__SlotAlarm__getNow(void ){
#line 98
  unsigned long __nesc_result;
#line 98

#line 98
  __nesc_result = /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Alarm__getNow();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
inline static /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Counter__size_type /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Counter__get(void ){
#line 53
  unsigned long __nesc_result;
#line 53

#line 53
  __nesc_result = /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static inline /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__to_size_type /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Alarm__getNow(void )
{
  return /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Counter__get();
}

# 98 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static LatinMacP__FastAlarm__size_type LatinMacP__FastAlarm__getNow(void ){
#line 98
  unsigned long __nesc_result;
#line 98

#line 98
  __nesc_result = /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Alarm__getNow();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 124 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/LatinMac/LatinMacP.nc"
static inline void LatinMacP__SlotAlarm__fired(void )
#line 124
{
  asn_t temp_asn;
  uint8_t temp_state;
  bool temp_isSync;
  OpenQueueEntry_t *temp_dataFrameToSend;
  error_t temp_error;
  ieee802154_header_iht transmitted_ieee154_header;
  uint8_t *latinChannelOffset;




  LatinMacP__fastAlarmStartSlotTimestamp = LatinMacP__FastAlarm__getNow();
  LatinMacP__slotAlarmStartSlotTimestamp = LatinMacP__SlotAlarm__getNow();
  /* atomic removed: atomic calls only */
#line 138
  LatinMacP__asn++;
  if (LatinMacP__slotType == UPLINK) {
      LatinMacP__slotType = DOWNLINK;
    }
  else 
#line 141
    {
      LatinMacP__slotType = UPLINK;
    }

  LatinMacP__SlotAlarm__startAt(LatinMacP__SlotAlarm__getAlarm(), SLOT_TIME);
  /* atomic removed: atomic calls only */

  WDTCTL = 0x5A00 + 0x0008 + 0x0004;
  /* atomic removed: atomic calls only */
  LatinMacP__dataFrameToSend = (void *)0;
  /* atomic removed: atomic calls only */
  {
    temp_asn = LatinMacP__asn;
    temp_state = LatinMacP__state;
    temp_isSync = LatinMacP__isSync;
    temp_dataFrameToSend = LatinMacP__dataFrameToSend;
  }

  if (LatinMacP__IDManager__getIsDAGroot() == TRUE) {
      /* atomic removed: atomic calls only */
      LatinMacP__isSync = TRUE;
      LatinMacP__Leds__led2On();
      /* atomic removed: atomic calls only */
#line 163
      {
        if (LatinMacP__state == S_SYNCHRONIZING) {
            LatinMacP__endSlot();
            return;
          }
      }
    }
  else {
#line 169
    if (temp_isSync == FALSE) {

        if (temp_state != S_SYNCHRONIZING) {
            LatinMacP__change_state(S_SYNCHRONIZING);
            if (LatinMacP__RadioControl__prepareReceive(11) != SUCCESS) {
                LatinMacP__endSlot();
              }
          }
        return;
      }
    }
  if (temp_state != S_SLEEP) {
      LatinMacP__endSlot();
      return;
    }

  switch (LatinMacP__networkState) {
      case BUILDING: 
        LatinMacP__frequencyChannel = 11;

      if ((TOS_NODE_ID == LatinMacP__asn / 2 % MAXNUMNEIGHBORS && LatinMacP__slotType == UPLINK) || (TOS_NODE_ID != LatinMacP__asn / 2 % MAXNUMNEIGHBORS && LatinMacP__slotType == DOWNLINK)) {
          if (LatinMacP__slotType == UPLINK) {

              LatinMacP__dataFrameToSend = LatinMacP__OpenQueue__inQueueBySeed(DEFAULT_SEED);
            }
          else {
              LatinMacP__dataFrameToSend = LatinMacP__OpenQueue__inQueueBySeed(LatinMacP__asn / 2 % MAXNUMNEIGHBORS);
            }
          if (LatinMacP__dataFrameToSend != (void *)0) {
              /* atomic removed: atomic calls only */


              LatinMacP__dataFrameToSend->l1_channel = LatinMacP__frequencyChannel;

              transmitted_ieee154_header = LatinMacP__retrieveIEEE802154header(LatinMacP__dataFrameToSend);

              if (LatinMacP__dataFrameToSend->l2_frameType == IEEE154_TYPE_CMD) {
                  if (__nesc_ntoh_leuint8(((CMD_MSG_t *)(LatinMacP__dataFrameToSend->payload + transmitted_ieee154_header.headerLength))->commandFrameId.data) == SCAN_MSG) {


                      __nesc_hton_leuint16(((CMD_MSG_t *)(LatinMacP__dataFrameToSend->payload + transmitted_ieee154_header.headerLength))->timingInformation.data, LatinMacP__GlobalTime__getASN());
                    }
                }

              LatinMacP__change_state(S_TX_TXDATAPREPARE);
              /* atomic removed: atomic calls only */
#line 214
              temp_error = LatinMacP__RadioSend__prepareSend(LatinMacP__dataFrameToSend);
              if (temp_error != SUCCESS) {

                  temp_dataFrameToSend->l2_retriesLeft--;
                  if (temp_dataFrameToSend->l2_retriesLeft == 0) {
                      LatinMacP__postTaskSendDone(temp_dataFrameToSend, FAIL);
                    }
                  LatinMacP__endSlot();
                }
#line 222
              ;
              /* atomic removed: atomic calls only */
#line 223
              LatinMacP__FastAlarm__startAt(LatinMacP__fastAlarmStartSlotTimestamp, TsTxOffset);
            }
          else {
              LatinMacP__endSlot();
            }
        }
      else {
#line 228
        if ((TOS_NODE_ID == LatinMacP__asn / 2 % MAXNUMNEIGHBORS && LatinMacP__slotType == DOWNLINK) || (TOS_NODE_ID != LatinMacP__asn / 2 % MAXNUMNEIGHBORS && LatinMacP__slotType == UPLINK)) {

            LatinMacP__change_state(S_RX_RXDATAPREPARE);
            /* atomic removed: atomic calls only */
#line 231
            temp_error = LatinMacP__RadioControl__prepareReceive(LatinMacP__frequencyChannel);
            if (temp_error != SUCCESS) {

                LatinMacP__endSlot();
              }
#line 235
            ;
            /* atomic removed: atomic calls only */
#line 236
            LatinMacP__FastAlarm__startAt(LatinMacP__fastAlarmStartSlotTimestamp, TsRxOffset);
          }
        }
#line 238
      break;

      case SCHEDULING: 
        if (LatinMacP__slotType == UPLINK) {

            if (LatinMacP__LatinMatrix__childCanTransmit(LatinMacP__asn / 2 % MAXNUMNEIGHBORS, latinChannelOffset)) {

                printf("asn: %u\n", LatinMacP__asn);
                printf("UPLINK\n");
                printf("C\n");
                printfflush();

                LatinMacP__frequencyChannel = *latinChannelOffset + 12;

                LatinMacP__change_state(S_RX_RXDATAPREPARE);
                /* atomic removed: atomic calls only */
#line 253
                temp_error = LatinMacP__RadioControl__prepareReceive(LatinMacP__frequencyChannel);
                if (temp_error != SUCCESS) {

                    LatinMacP__endSlot();
                  }
#line 257
                ;
                /* atomic removed: atomic calls only */
#line 258
                LatinMacP__FastAlarm__startAt(LatinMacP__fastAlarmStartSlotTimestamp, TsRxOffset);
              }
            else 
#line 259
              {


                LatinMacP__dataFrameToSend = LatinMacP__OpenQueue__inQueueBySeed(LatinMacP__LatinMatrix__getParentSeed());

                if (LatinMacP__dataFrameToSend != (void *)0) {


                    if (LatinMacP__LatinMatrix__siblingCanTransmit(LatinMacP__asn / 2 % MAXNUMNEIGHBORS, latinChannelOffset)) {
                        printf("asn: %u\n", LatinMacP__asn);
                        printf("UPLINK\n");
                        printf("S\n");
                        printfflush();
                        LatinMacP__endSlot();
                      }
                    else {
                        printf("asn: %u\n", LatinMacP__asn);
                        printf("UPLINK\n");
                        printf("MP\n");
                        printfflush();

                        ;

                        LatinMacP__frequencyChannel = *latinChannelOffset + 12;
                        /* atomic removed: atomic calls only */
#line 283
                        LatinMacP__dataFrameToSend->l1_channel = LatinMacP__frequencyChannel;

                        LatinMacP__change_state(S_TX_TXDATAPREPARE);
                        /* atomic removed: atomic calls only */








                        LatinMacP__FastAlarm__startAt(LatinMacP__fastAlarmStartSlotTimestamp, TsTxOffset);
                      }
                  }
                else 
                  {
                    LatinMacP__endSlot();
                  }
              }
          }
        else 
          {

            if (LatinMacP__LatinMatrix__parentCanTransmit(LatinMacP__asn / 2 % MAXNUMNEIGHBORS, latinChannelOffset)) {






                LatinMacP__frequencyChannel = *latinChannelOffset + 12;

                LatinMacP__change_state(S_RX_RXDATAPREPARE);
                /* atomic removed: atomic calls only */
#line 317
                temp_error = LatinMacP__RadioControl__prepareReceive(LatinMacP__frequencyChannel);
                if (temp_error != SUCCESS) {

                    LatinMacP__endSlot();
                  }
#line 321
                ;
                /* atomic removed: atomic calls only */
#line 322
                LatinMacP__FastAlarm__startAt(LatinMacP__fastAlarmStartSlotTimestamp, TsRxOffset);
              }
            else 
#line 323
              {
                LatinMacP__dataFrameToSend = LatinMacP__OpenQueue__inQueueToChild(LatinMacP__LatinMatrix__getParentSeed());
                if (LatinMacP__dataFrameToSend != (void *)0) {

                    if (LatinMacP__LatinMatrix__canTransmit(LatinMacP__asn / 2 % MAXNUMNEIGHBORS, latinChannelOffset)) {







                        LatinMacP__frequencyChannel = *latinChannelOffset + 12;
                        /* atomic removed: atomic calls only */
#line 336
                        LatinMacP__dataFrameToSend->l1_channel = LatinMacP__frequencyChannel;

                        LatinMacP__change_state(S_TX_TXDATAPREPARE);
                        /* atomic removed: atomic calls only */








                        LatinMacP__FastAlarm__startAt(LatinMacP__fastAlarmStartSlotTimestamp, TsTxOffset);
                      }
                    else {




                        LatinMacP__endSlot();
                      }
                  }
                else 
#line 357
                  {
                    LatinMacP__endSlot();
                  }
              }
          }


      break;
      default: ;
    }
}

# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Alarm__fired(void ){
#line 67
  LatinMacP__SlotAlarm__fired();
#line 67
}
#line 67
# 151 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static inline void /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__m_dt == 0) 
      {
        /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Alarm__fired();
      }
    else 
      {
        /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__set_alarm();
      }
  }
}

# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void ){
#line 67
  /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__AlarmFrom__fired();
#line 67
}
#line 67
# 124 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void )
{
  * (volatile uint16_t * )390U &= ~0x0010;
}

# 47 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents();
#line 47
}
#line 47
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void )
{
  /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
  /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired();
}

# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void ){
#line 34
  /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired();
#line 34
}
#line 34
# 139 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void )
{
  return * (volatile uint16_t * )406U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4436 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(* (volatile uint16_t * )390U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired();
    }
}

#line 119
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void )
{
  * (volatile uint16_t * )390U |= 0x0010;
}

# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents();
#line 46
}
#line 46
# 84 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )390U &= ~0x0001;
}

# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )406U = x;
}

# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(time);
#line 30
}
#line 30
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 154 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )406U = /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get() + x;
}

# 32 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 70 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 86
          /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 88
    /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt();
    /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents();
  }
}

# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__AlarmFrom__startAt(/*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__AlarmFrom__size_type t0, /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__AlarmFrom__size_type dt){
#line 92
  /*LatinMacC.SlotAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 35 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/RadioControl.nc"
inline static error_t LatinMacP__RadioControl__rfOff(void ){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = CC2420DriversP__RadioControl__rfOff();
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 78 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__RfOffResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ControlC.RfOffSpiC*/CC2420SpiC__5__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78









inline static error_t CC2420ControlP__RfOffResource__immediateRequest(void ){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = CC2420SpiP__Resource__immediateRequest(/*CC2420ControlC.RfOffSpiC*/CC2420SpiC__5__CLIENT_ID);
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 264 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Config__rfOff(void )
#line 264
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 265
    {
      if (CC2420ControlP__m_rfOff_busy) {
          {
            unsigned char __nesc_temp = 
#line 267
            FAIL;

            {
#line 267
              __nesc_atomic_end(__nesc_atomic); 
#line 267
              return __nesc_temp;
            }
          }
        }
#line 269
      CC2420ControlP__m_rfOff_busy = TRUE;
      if (CC2420ControlP__m_state == CC2420ControlP__S_XOSC_STARTED) {
          if (CC2420ControlP__RfOffResource__immediateRequest() != SUCCESS) {
              CC2420ControlP__RfOffResource__request();
            }
          else 
#line 273
            {
              CC2420ControlP__rfOff();
            }
        }
      else 
#line 276
        {
          CC2420ControlP__m_rfOff_busy = FALSE;
          {
            unsigned char __nesc_temp = 
#line 278
            FAIL;

            {
#line 278
              __nesc_atomic_end(__nesc_atomic); 
#line 278
              return __nesc_temp;
            }
          }
        }
    }
#line 282
    __nesc_atomic_end(__nesc_atomic); }
#line 281
  return SUCCESS;
}

# 11 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Config.nc"
inline static error_t CC2420DriversP__CC2420Config__rfOff(void ){
#line 11
  unsigned char __nesc_result;
#line 11

#line 11
  __nesc_result = CC2420ControlP__CC2420Config__rfOff();
#line 11

#line 11
  return __nesc_result;
#line 11
}
#line 11
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/State.nc"
inline static error_t CC2420SpiP__WorkingState__requestState(uint8_t reqState){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = StateImplP__State__requestState(0U, reqState);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 106 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__isOwner(uint8_t id)
#line 106
{
#line 106
  return FAIL;
}

# 118 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static bool /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__isOwner(uint8_t arg_0x410c29a0){
#line 118
  unsigned char __nesc_result;
#line 118

#line 118
  switch (arg_0x410c29a0) {
#line 118
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 118
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 118
      break;
#line 118
    default:
#line 118
      __nesc_result = /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__isOwner(arg_0x410c29a0);
#line 118
      break;
#line 118
    }
#line 118

#line 118
  return __nesc_result;
#line 118
}
#line 118
# 102 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline uint8_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__isOwner(uint8_t id)
#line 102
{
  return /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__isOwner(id);
}

# 118 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static bool CC2420SpiP__SpiResource__isOwner(void ){
#line 118
  unsigned char __nesc_result;
#line 118

#line 118
  __nesc_result = /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__isOwner(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 118

#line 118
  return __nesc_result;
#line 118
}
#line 118
# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline msp430_spi_union_config_t */*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Msp430SpiConfigure__default__getConfig(uint8_t id)
#line 110
{
  return &msp430_spi_default_config;
}

# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
inline static msp430_spi_union_config_t */*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Msp430SpiConfigure__getConfig(uint8_t arg_0x410c1418){
#line 39
  union __nesc_unnamed4277 *__nesc_result;
#line 39

#line 39
    __nesc_result = /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Msp430SpiConfigure__default__getConfig(arg_0x410c1418);
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 168 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__setModeSpi(msp430_spi_union_config_t *config){
#line 168
  HplMsp430Usart0P__Usart__setModeSpi(config);
#line 168
}
#line 168
# 88 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__ResourceConfigure__configure(uint8_t id)
#line 88
{
  /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__setModeSpi(/*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Msp430SpiConfigure__getConfig(id));
}

# 213 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(uint8_t id)
#line 213
{
}

# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(uint8_t arg_0x40aaacf8){
#line 49
  switch (arg_0x40aaacf8) {
#line 49
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 49
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__ResourceConfigure__configure(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(arg_0x40aaacf8);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 210 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__immediateRequested(void )
#line 210
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release();
}

# 81 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__immediateRequested(void ){
#line 81
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__immediateRequested();
#line 81
}
#line 81
# 203 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(uint8_t id)
#line 203
{
}

# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__immediateRequested(uint8_t arg_0x40aac948){
#line 51
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(arg_0x40aac948);
#line 51
}
#line 51
# 90 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(uint8_t id)
#line 90
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__immediateRequested(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId);
  /* atomic removed: atomic calls only */
#line 92
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED) {
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_IMM_GRANTING;
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId = id;
      }
    else {
        unsigned char __nesc_temp = 
#line 97
        FAIL;

#line 97
        return __nesc_temp;
      }
  }
#line 99
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__immediateRequested();
  if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId == id) {
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId);
      return SUCCESS;
    }
  /* atomic removed: atomic calls only */
#line 104
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED;
  return FAIL;
}

# 108 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__immediateRequest(uint8_t id)
#line 108
{
#line 108
  return FAIL;
}

# 87 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__immediateRequest(uint8_t arg_0x410c29a0){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  switch (arg_0x410c29a0) {
#line 87
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 87
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 87
      break;
#line 87
    default:
#line 87
      __nesc_result = /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__immediateRequest(arg_0x410c29a0);
#line 87
      break;
#line 87
    }
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 76 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__immediateRequest(uint8_t id)
#line 76
{
  return /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__immediateRequest(id);
}

# 87 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__immediateRequest(void ){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__immediateRequest(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 97 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void HplMsp430I2C0P__HplUsart__resetUsart(bool reset){
#line 97
  HplMsp430Usart0P__Usart__resetUsart(reset);
#line 97
}
#line 97
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static inline void HplMsp430I2C0P__HplI2C__clearModeI2C(void )
#line 59
{
  /* atomic removed: atomic calls only */
#line 60
  {
    HplMsp430I2C0P__U0CTL &= ~((0x20 | 0x04) | 0x01);
    HplMsp430I2C0P__HplUsart__resetUsart(TRUE);
  }
}

# 7 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static void HplMsp430Usart0P__HplI2C__clearModeI2C(void ){
#line 7
  HplMsp430I2C0P__HplI2C__clearModeI2C();
#line 7
}
#line 7
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 5);
}

# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__URXD__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 4);
}

# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UTXD__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc();
#line 85
}
#line 85
# 207 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__disableUart(void )
#line 207
{
  /* atomic removed: atomic calls only */
#line 208
  {
    HplMsp430Usart0P__ME1 &= ~((1 << 7) | (1 << 6));
    HplMsp430Usart0P__UTXD__selectIOFunc();
    HplMsp430Usart0P__URXD__selectIOFunc();
  }
}

#line 143
static inline void HplMsp430Usart0P__Usart__setUmctl(uint8_t control)
#line 143
{
  U0MCTL = control;
}

#line 132
static inline void HplMsp430Usart0P__Usart__setUbr(uint16_t control)
#line 132
{
  /* atomic removed: atomic calls only */
#line 133
  {
    U0BR0 = control & 0x00FF;
    U0BR1 = (control >> 8) & 0x00FF;
  }
}

#line 256
static inline void HplMsp430Usart0P__configSpi(msp430_spi_union_config_t *config)
#line 256
{

  U0CTL = (config->spiRegisters.uctl | 0x04) | 0x01;
  HplMsp430Usart0P__U0TCTL = config->spiRegisters.utctl;

  HplMsp430Usart0P__Usart__setUbr(config->spiRegisters.ubr);
  HplMsp430Usart0P__Usart__setUmctl(0x00);
}

# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 3;
}

# 78 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UCLK__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc();
#line 78
}
#line 78
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 2;
}

# 78 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SOMI__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc();
#line 78
}
#line 78
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 1;
}

# 78 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SIMO__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc();
#line 78
}
#line 78
# 238 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__enableSpi(void )
#line 238
{
  /* atomic removed: atomic calls only */
#line 239
  {
    HplMsp430Usart0P__SIMO__selectModuleFunc();
    HplMsp430Usart0P__SOMI__selectModuleFunc();
    HplMsp430Usart0P__UCLK__selectModuleFunc();
  }
  HplMsp430Usart0P__ME1 |= 1 << 6;
}

#line 345
static inline void HplMsp430Usart0P__Usart__clrIntr(void )
#line 345
{
  HplMsp430Usart0P__IFG1 &= ~((1 << 7) | (1 << 6));
}









static inline void HplMsp430Usart0P__Usart__disableIntr(void )
#line 357
{
  HplMsp430Usart0P__IE1 &= ~((1 << 7) | (1 << 6));
}

# 207 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__requested(void )
#line 207
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release();
}

# 73 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested(void ){
#line 73
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__requested();
#line 73
}
#line 73
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 54
{
  /* atomic removed: atomic calls only */
#line 55
  {
    unsigned char __nesc_temp = 
#line 55
    /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[id] != /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY || /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail == id;

#line 55
    return __nesc_temp;
  }
}

#line 72
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(resource_client_id_t id)
#line 72
{
  /* atomic removed: atomic calls only */
#line 73
  {
    if (!/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(id)) {
        if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = id;
          }
        else {
#line 78
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail] = id;
          }
#line 79
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = id;
        {
          unsigned char __nesc_temp = 
#line 80
          SUCCESS;

#line 80
          return __nesc_temp;
        }
      }
#line 82
    {
      unsigned char __nesc_temp = 
#line 82
      EBUSY;

#line 82
      return __nesc_temp;
    }
  }
}

# 69 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceQueue.nc"
inline static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__enqueue(resource_client_id_t id){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(id);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 201 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(uint8_t id)
#line 201
{
}

# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__requested(uint8_t arg_0x40aac948){
#line 43
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(arg_0x40aac948);
#line 43
}
#line 43
# 77 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__request(uint8_t id)
#line 77
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__requested(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId);
  /* atomic removed: atomic calls only */
#line 79
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED) {
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_GRANTING;
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId = id;
      }
    else {
        unsigned char __nesc_temp = 
#line 84
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__enqueue(id);

#line 84
        return __nesc_temp;
      }
  }
#line 86
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested();
  return SUCCESS;
}

# 107 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__request(uint8_t id)
#line 107
{
#line 107
  return FAIL;
}

# 78 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__request(uint8_t arg_0x410c29a0){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  switch (arg_0x410c29a0) {
#line 78
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 78
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__request(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 78
      break;
#line 78
    default:
#line 78
      __nesc_result = /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__request(arg_0x410c29a0);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 80 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__request(uint8_t id)
#line 80
{
  return /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__request(id);
}

# 78 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__request(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ControlP__SRFOFF__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SRFOFF);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 382 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__tx(uint8_t data)
#line 382
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 383
    HplMsp430Usart0P__U0TXBUF = data;
#line 383
    __nesc_atomic_end(__nesc_atomic); }
}

# 224 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__tx(uint8_t data){
#line 224
  HplMsp430Usart0P__Usart__tx(data);
#line 224
}
#line 224
# 330 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline bool HplMsp430Usart0P__Usart__isRxIntrPending(void )
#line 330
{
  if (HplMsp430Usart0P__IFG1 & (1 << 6)) {
      return TRUE;
    }
  return FALSE;
}

# 192 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static bool /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__isRxIntrPending(void ){
#line 192
  unsigned char __nesc_result;
#line 192

#line 192
  __nesc_result = HplMsp430Usart0P__Usart__isRxIntrPending();
#line 192

#line 192
  return __nesc_result;
#line 192
}
#line 192
# 341 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__clrRxIntr(void )
#line 341
{
  HplMsp430Usart0P__IFG1 &= ~(1 << 6);
}

# 197 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__clrRxIntr(void ){
#line 197
  HplMsp430Usart0P__Usart__clrRxIntr();
#line 197
}
#line 197
# 386 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline uint8_t HplMsp430Usart0P__Usart__rx(void )
#line 386
{
  uint8_t value;

#line 388
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 388
    value = U0RXBUF;
#line 388
    __nesc_atomic_end(__nesc_atomic); }
  return value;
}

# 231 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static uint8_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__rx(void ){
#line 231
  unsigned char __nesc_result;
#line 231

#line 231
  __nesc_result = HplMsp430Usart0P__Usart__rx();
#line 231

#line 231
  return __nesc_result;
#line 231
}
#line 231
# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__RfOffResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ControlC.RfOffSpiC*/CC2420SpiC__5__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr();
#line 39
}
#line 39
# 38 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__GeneralIO__clr(void )
#line 38
{
#line 38
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__HplGeneralIO__clr();
}

# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CSN__clr(void ){
#line 30
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__GeneralIO__clr();
#line 30
}
#line 30
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420TransmitP__SFLUSHTX__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SFLUSHTX);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void CC2420ReceiveP__CSN__clr(void ){
#line 30
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__GeneralIO__clr();
#line 30
}
#line 30
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ReceiveP__SFLUSHRX__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SFLUSHRX);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 178 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
static inline uint8_t CC2420SpiP__Resource__isOwner(uint8_t id)
#line 178
{
  /* atomic removed: atomic calls only */
#line 179
  {
    unsigned char __nesc_temp = 
#line 179
    CC2420SpiP__m_holder == id;

#line 179
    return __nesc_temp;
  }
}

# 118 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static bool CC2420ReceiveP__SpiResource__isOwner(void ){
#line 118
  unsigned char __nesc_result;
#line 118

#line 118
  __nesc_result = CC2420SpiP__Resource__isOwner(/*CC2420ReceiveC.CC2420SpiC*/CC2420SpiC__6__CLIENT_ID);
#line 118

#line 118
  return __nesc_result;
#line 118
}
#line 118
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/SpiPacket.nc"
inline static error_t CC2420SpiP__SpiPacket__send(uint8_t * txBuf, uint8_t * rxBuf, uint16_t len){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiPacket__send(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID, txBuf, rxBuf, len);
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 209 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
static inline error_t CC2420SpiP__Fifo__continueRead(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 210
{
  return CC2420SpiP__SpiPacket__send((void *)0, data, len);
}

# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/SpiByte.nc"
inline static uint8_t CC2420SpiP__SpiByte__write(uint8_t tx){
#line 34
  unsigned char __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiByte__write(tx);
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 189 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
static inline cc2420_status_t CC2420SpiP__Fifo__beginRead(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 190
{

  cc2420_status_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 194
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 196
            status;

            {
#line 196
              __nesc_atomic_end(__nesc_atomic); 
#line 196
              return __nesc_temp;
            }
          }
        }
    }
#line 200
    __nesc_atomic_end(__nesc_atomic); }
#line 200
  CC2420SpiP__m_addr = addr | 0x40;

  status = CC2420SpiP__SpiByte__write(CC2420SpiP__m_addr);
  CC2420SpiP__Fifo__continueRead(addr, data, len);

  return status;
}

# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Fifo.nc"
inline static cc2420_status_t CC2420ReceiveP__RXFIFO__beginRead(uint8_t * data, uint8_t length){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = CC2420SpiP__Fifo__beginRead(CC2420_RXFIFO, data, length);
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 91 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaXP.nc"
static inline error_t /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setTrigger(dma_trigger_t trigger)
#line 91
{

  if (* (volatile uint16_t *)488U & 0x0010) {
    return FAIL;
    }
  /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMACTL0 = (/*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMACTL0 & ~240) | ((
  trigger << 4) & 240);

  return SUCCESS;
}

#line 243
static inline void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setStateRaw(uint16_t s, uint16_t t, 
void *src, void *dest, 
uint16_t size)
#line 245
{
  * (volatile uint16_t *)490U = (uint16_t )src;
  * (volatile uint16_t *)492U = (uint16_t )dest;
  * (volatile uint16_t *)494U = size;
  /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setTrigger((dma_trigger_t )t);
  * (volatile uint16_t *)488U = s;
}

#line 235
static inline void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setState(dma_channel_state_t s, 
dma_channel_trigger_t t, 
void *src, void *dest, 
uint16_t size)
#line 238
{
  /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setStateRaw(* (uint16_t *)&s, * (uint16_t *)&t, 
  src, dest, size);
}

# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
inline static void /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__HplChannel__setState(dma_channel_state_t s, dma_channel_trigger_t t, void *src, void *dest, uint16_t size){
#line 64
  /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setState(s, t, src, dest, size);
#line 64
}
#line 64
# 80 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannelP.nc"
static inline error_t /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__Channel__setupTransfer(dma_transfer_mode_t transfer_mode, 
dma_trigger_t trigger, 
dma_level_t level, 
void *src_addr, 
void *dst_addr, 
uint16_t size, 
dma_byte_t src_byte, 
dma_byte_t dst_byte, 
dma_incr_t src_incr, 
dma_incr_t dst_incr)
#line 89
{

  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState.request = 0;
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState.abort = 0;
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState.interruptEnable = 1;
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState.interruptFlag = 0;
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState.enable = 0;
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState.level = level;
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState.srcByte = src_byte;
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState.dstByte = dst_byte;
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState.srcIncrement = src_incr;
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState.dstIncrement = dst_incr;
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState.transferMode = transfer_mode;

  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelTrigger.trigger = trigger;

  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__HplChannel__setState(/*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState, /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelTrigger, 
  src_addr, dst_addr, size);

  return SUCCESS;
}

# 38 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannel.nc"
inline static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel1__setupTransfer(dma_transfer_mode_t transfer_mode, dma_trigger_t trigger, dma_level_t level, void *src_addr, void *dst_addr, uint16_t size, dma_byte_t src_byte, dma_byte_t dst_byte, dma_incr_t src_incr, dma_incr_t dst_incr){
#line 38
  unsigned char __nesc_result;
#line 38

#line 38
  __nesc_result = /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__Channel__setupTransfer(transfer_mode, trigger, level, src_addr, dst_addr, size, src_byte, dst_byte, src_incr, dst_incr);
#line 38

#line 38
  return __nesc_result;
#line 38
}
#line 38
# 195 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaXP.nc"
static inline void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__enableDMA(void )
#line 195
{
  * (volatile uint16_t *)488U |= 0x0010;
}

# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
inline static void /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__HplChannel__enableDMA(void ){
#line 49
  /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__enableDMA();
#line 49
}
#line 49
# 112 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannelP.nc"
static inline error_t /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__Channel__startTransfer(void )
#line 112
{
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__HplChannel__enableDMA();
  return SUCCESS;
}

# 73 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannel.nc"
inline static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel1__startTransfer(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__Channel__startTransfer();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 91 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaXP.nc"
static inline error_t /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setTrigger(dma_trigger_t trigger)
#line 91
{

  if (* (volatile uint16_t *)496U & 0x0010) {
    return FAIL;
    }
  /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMACTL0 = (/*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMACTL0 & ~3840) | ((
  trigger << 8) & 3840);

  return SUCCESS;
}

#line 243
static inline void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setStateRaw(uint16_t s, uint16_t t, 
void *src, void *dest, 
uint16_t size)
#line 245
{
  * (volatile uint16_t *)498U = (uint16_t )src;
  * (volatile uint16_t *)500U = (uint16_t )dest;
  * (volatile uint16_t *)502U = size;
  /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setTrigger((dma_trigger_t )t);
  * (volatile uint16_t *)496U = s;
}

#line 235
static inline void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setState(dma_channel_state_t s, 
dma_channel_trigger_t t, 
void *src, void *dest, 
uint16_t size)
#line 238
{
  /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setStateRaw(* (uint16_t *)&s, * (uint16_t *)&t, 
  src, dest, size);
}

# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
inline static void /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__HplChannel__setState(dma_channel_state_t s, dma_channel_trigger_t t, void *src, void *dest, uint16_t size){
#line 64
  /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setState(s, t, src, dest, size);
#line 64
}
#line 64
# 80 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannelP.nc"
static inline error_t /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__Channel__setupTransfer(dma_transfer_mode_t transfer_mode, 
dma_trigger_t trigger, 
dma_level_t level, 
void *src_addr, 
void *dst_addr, 
uint16_t size, 
dma_byte_t src_byte, 
dma_byte_t dst_byte, 
dma_incr_t src_incr, 
dma_incr_t dst_incr)
#line 89
{

  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState.request = 0;
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState.abort = 0;
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState.interruptEnable = 1;
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState.interruptFlag = 0;
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState.enable = 0;
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState.level = level;
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState.srcByte = src_byte;
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState.dstByte = dst_byte;
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState.srcIncrement = src_incr;
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState.dstIncrement = dst_incr;
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState.transferMode = transfer_mode;

  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelTrigger.trigger = trigger;

  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__HplChannel__setState(/*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState, /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelTrigger, 
  src_addr, dst_addr, size);

  return SUCCESS;
}

# 38 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannel.nc"
inline static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel2__setupTransfer(dma_transfer_mode_t transfer_mode, dma_trigger_t trigger, dma_level_t level, void *src_addr, void *dst_addr, uint16_t size, dma_byte_t src_byte, dma_byte_t dst_byte, dma_incr_t src_incr, dma_incr_t dst_incr){
#line 38
  unsigned char __nesc_result;
#line 38

#line 38
  __nesc_result = /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__Channel__setupTransfer(transfer_mode, trigger, level, src_addr, dst_addr, size, src_byte, dst_byte, src_incr, dst_incr);
#line 38

#line 38
  return __nesc_result;
#line 38
}
#line 38
# 195 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaXP.nc"
static inline void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__enableDMA(void )
#line 195
{
  * (volatile uint16_t *)496U |= 0x0010;
}

# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
inline static void /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__HplChannel__enableDMA(void ){
#line 49
  /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__enableDMA();
#line 49
}
#line 49
# 112 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannelP.nc"
static inline error_t /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__Channel__startTransfer(void )
#line 112
{
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__HplChannel__enableDMA();
  return SUCCESS;
}

# 73 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannel.nc"
inline static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel2__startTransfer(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__Channel__startTransfer();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone_task__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone_task);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 87 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t CC2420ReceiveP__SpiResource__immediateRequest(void ){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = CC2420SpiP__Resource__immediateRequest(/*CC2420ReceiveC.CC2420SpiC*/CC2420SpiC__6__CLIENT_ID);
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
#line 78
inline static error_t CC2420ReceiveP__SpiResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ReceiveC.CC2420SpiC*/CC2420SpiC__6__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__enable(void )
#line 75
{
#line 75
  P1IE |= 1 << 0;
}

# 31 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__HplInterrupt__enable(void ){
#line 31
  HplMsp430InterruptP__Port10__enable();
#line 31
}
#line 31
# 107 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__edge(bool l2h)
#line 107
{
  /* atomic removed: atomic calls only */
#line 108
  {
    if (l2h) {
#line 109
      P1IES &= ~(1 << 0);
      }
    else {
#line 110
      P1IES |= 1 << 0;
      }
  }
}

# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__HplInterrupt__edge(bool low_to_high){
#line 56
  HplMsp430InterruptP__Port10__edge(low_to_high);
#line 56
}
#line 56
# 91 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__clear(void )
#line 91
{
#line 91
  P1IFG &= ~(1 << 0);
}

# 41 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port10__clear();
#line 41
}
#line 41
# 83 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__disable(void )
#line 83
{
#line 83
  P1IE &= ~(1 << 0);
}

# 36 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__HplInterrupt__disable(void ){
#line 36
  HplMsp430InterruptP__Port10__disable();
#line 36
}
#line 36
# 58 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__Interrupt__disable(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__HplInterrupt__disable();
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__HplInterrupt__clear();
  }
  return SUCCESS;
}

#line 41
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__enable(bool rising)
#line 41
{
  /* atomic removed: atomic calls only */
#line 42
  {
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__Interrupt__disable();
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__HplInterrupt__edge(rising);
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__HplInterrupt__enable();
  }
  return SUCCESS;
}





static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__Interrupt__enableFallingEdge(void )
#line 54
{
  return /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__enable(FALSE);
}

# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ReceiveP__InterruptFIFOP__enableFallingEdge(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__Interrupt__enableFallingEdge();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 87 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SyncResource__immediateRequest(void ){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = CC2420SpiP__Resource__immediateRequest(/*CC2420ControlC.SyncSpiC*/CC2420SpiC__3__CLIENT_ID);
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
#line 78
inline static error_t CC2420ControlP__SyncResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ControlC.SyncSpiC*/CC2420SpiC__3__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__FSCTRL__write(uint16_t data){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420SpiP__Reg__write(CC2420_FSCTRL, data);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SyncResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ControlC.SyncSpiC*/CC2420SpiC__3__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 21 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/RadioControl.nc"
inline static void CC2420DriversP__RadioControl__prepareReceiveDone(error_t error){
#line 21
  LatinMacP__RadioControl__prepareReceiveDone(error);
#line 21
}
#line 21
# 12 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/RadioSend.nc"
inline static void CC2420DriversP__RadioSend__prepareSendDone(error_t error){
#line 12
  LatinMacP__RadioSend__prepareSendDone(error);
#line 12
}
#line 12
# 198 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/CC2420DriversP.nc"
static inline void CC2420DriversP__CC2420Config__setChannelDone(error_t error)
#line 198
{
  bool temp_syncForSend;

#line 200
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 200
    {
      temp_syncForSend = CC2420DriversP__syncForSend;
    }
#line 202
    __nesc_atomic_end(__nesc_atomic); }
  if (temp_syncForSend) {
      if (error == SUCCESS) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 205
            CC2420DriversP__state = CC2420DriversP__S_READY_TX;
#line 205
            __nesc_atomic_end(__nesc_atomic); }
          CC2420DriversP__RadioSend__prepareSendDone(SUCCESS);
        }
      else 
#line 207
        {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 208
            CC2420DriversP__state = CC2420DriversP__S_STARTED;
#line 208
            __nesc_atomic_end(__nesc_atomic); }
          CC2420DriversP__RadioSend__prepareSendDone(FAIL);
        }
    }
  else 
#line 211
    {
      if (error == SUCCESS) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 213
            CC2420DriversP__state = CC2420DriversP__S_READY_RX;
#line 213
            __nesc_atomic_end(__nesc_atomic); }
          CC2420DriversP__RadioControl__prepareReceiveDone(SUCCESS);
        }
      else 
#line 215
        {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 216
            CC2420DriversP__state = CC2420DriversP__S_STARTED;
#line 216
            __nesc_atomic_end(__nesc_atomic); }
          CC2420DriversP__RadioControl__prepareReceiveDone(FAIL);
        }
    }
}

# 9 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Config.nc"
inline static void CC2420ControlP__CC2420Config__setChannelDone(error_t error){
#line 9
  CC2420DriversP__CC2420Config__setChannelDone(error);
#line 9
}
#line 9
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t LatinMacP__taskSendDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(LatinMacP__taskSendDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
inline static /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Counter__size_type /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Counter__get(void ){
#line 53
  unsigned long __nesc_result;
#line 53

#line 53
  __nesc_result = /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static inline /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__to_size_type /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__getNow(void )
{
  return /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Counter__get();
}

#line 136
static inline void /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__startAt(/*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__to_size_type t0, /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__m_t0 = t0;
      /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__m_dt = dt;
      /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__set_alarm();
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

static inline void /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__start(/*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__to_size_type dt)
{
  /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__startAt(/*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__getNow(), dt);
}

# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void CC2420DriversP__RxWaitAlarm__start(CC2420DriversP__RxWaitAlarm__size_type dt){
#line 55
  /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__start(dt);
#line 55
}
#line 55
# 119 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__enableEvents(void )
{
  * (volatile uint16_t * )398U |= 0x0010;
}

# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__enableEvents();
#line 46
}
#line 46
# 84 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )398U &= ~0x0001;
}

# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )414U = x;
}

# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430Compare__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__setEvent(time);
#line 30
}
#line 30
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 154 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )414U = /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__get() + x;
}

# 32 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430Compare__setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 70 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 86
          /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 88
    /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430TimerControl__clearPendingInterrupt();
    /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430TimerControl__enableEvents();
  }
}

# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__AlarmFrom__startAt(/*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__AlarmFrom__size_type t0, /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__AlarmFrom__size_type dt){
#line 92
  /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 78 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__RxOnResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ControlC.RxOnSpiC*/CC2420SpiC__4__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78









inline static error_t CC2420ControlP__RxOnResource__immediateRequest(void ){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = CC2420SpiP__Resource__immediateRequest(/*CC2420ControlC.RxOnSpiC*/CC2420SpiC__4__CLIENT_ID);
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 235 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Config__rxOn(void )
#line 235
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 236
    {
      if (CC2420ControlP__m_rxOn_busy) {
          {
            unsigned char __nesc_temp = 
#line 238
            FAIL;

            {
#line 238
              __nesc_atomic_end(__nesc_atomic); 
#line 238
              return __nesc_temp;
            }
          }
        }
#line 240
      CC2420ControlP__m_rxOn_busy = TRUE;
      if (CC2420ControlP__m_state == CC2420ControlP__S_XOSC_STARTED) {
          if (CC2420ControlP__RxOnResource__immediateRequest() != SUCCESS) {
              CC2420ControlP__RxOnResource__request();
            }
          else 
#line 244
            {
              CC2420ControlP__rxOn();
            }
        }
      else 
#line 247
        {
          {
            unsigned char __nesc_temp = 
#line 248
            FAIL;

            {
#line 248
              __nesc_atomic_end(__nesc_atomic); 
#line 248
              return __nesc_temp;
            }
          }
        }
    }
#line 252
    __nesc_atomic_end(__nesc_atomic); }
#line 251
  return SUCCESS;
}

# 10 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Config.nc"
inline static error_t CC2420DriversP__CC2420Config__rxOn(void ){
#line 10
  unsigned char __nesc_result;
#line 10

#line 10
  __nesc_result = CC2420ControlP__CC2420Config__rxOn();
#line 10

#line 10
  return __nesc_result;
#line 10
}
#line 10
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ControlP__SRXON__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SRXON);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__RxOnResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ControlC.RxOnSpiC*/CC2420SpiC__4__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 13 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/PacketFunctions.nc"
inline static void LatinMacP__PacketFunctions__readAddress(uint8_t *payload, uint8_t type, open_addr_t *writeToAddress, bool littleEndian){
#line 13
  PacketFunctionsP__PacketFunctions__readAddress(payload, type, writeToAddress, littleEndian);
#line 13
}
#line 13
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420TransmitP__TXCTRL__write(uint16_t data){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420SpiP__Reg__write(CC2420_TXCTRL, data);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
inline static /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Counter__size_type /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Counter__get(void ){
#line 53
  unsigned long __nesc_result;
#line 53

#line 53
  __nesc_result = /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static inline /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__to_size_type /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__getNow(void )
{
  return /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Counter__get();
}

#line 146
static inline void /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__start(/*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__to_size_type dt)
{
  /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__startAt(/*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__getNow(), dt);
}

# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void CC2420TransmitP__AlarmWatchdog__start(CC2420TransmitP__AlarmWatchdog__size_type dt){
#line 55
  /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__start(dt);
#line 55
}
#line 55
# 119 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__enableEvents(void )
{
  * (volatile uint16_t * )396U |= 0x0010;
}

# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__enableEvents();
#line 46
}
#line 46
# 84 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )396U &= ~0x0001;
}

# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )412U = x;
}

# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Compare__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__setEvent(time);
#line 30
}
#line 30
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 154 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )412U = /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__get() + x;
}

# 32 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Compare__setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 70 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 86
          /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 88
    /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430TimerControl__clearPendingInterrupt();
    /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430TimerControl__enableEvents();
  }
}

# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__AlarmFrom__startAt(/*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__AlarmFrom__size_type t0, /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__AlarmFrom__size_type dt){
#line 92
  /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 214 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
static inline cc2420_status_t CC2420SpiP__Fifo__write(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 215
{

  uint8_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 219
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 221
            status;

            {
#line 221
              __nesc_atomic_end(__nesc_atomic); 
#line 221
              return __nesc_temp;
            }
          }
        }
    }
#line 225
    __nesc_atomic_end(__nesc_atomic); }
#line 225
  CC2420SpiP__m_addr = addr;

  status = CC2420SpiP__SpiByte__write(CC2420SpiP__m_addr);
  CC2420SpiP__SpiPacket__send(data, (void *)0, len);

  return status;
}

# 82 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Fifo.nc"
inline static cc2420_status_t CC2420TransmitP__TXFIFO__write(uint8_t * data, uint8_t length){
#line 82
  unsigned char __nesc_result;
#line 82

#line 82
  __nesc_result = CC2420SpiP__Fifo__write(CC2420_TXFIFO, data, length);
#line 82

#line 82
  return __nesc_result;
#line 82
}
#line 82
# 119 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__enableEvents(void )
{
  * (volatile uint16_t * )392U |= 0x0010;
}

# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__enableEvents();
#line 46
}
#line 46
# 84 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )392U &= ~0x0001;
}

# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )408U = x;
}

# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEvent(time);
#line 30
}
#line 30
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 154 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )408U = /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__get() + x;
}

# 32 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 70 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 86
          /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 88
    /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__clearPendingInterrupt();
    /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__enableEvents();
  }
}

# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__AlarmFrom__startAt(/*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__AlarmFrom__size_type t0, /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__AlarmFrom__size_type dt){
#line 92
  /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/BigQueueC.nc"
static inline bool /*PrintfC.QueueC*/BigQueueC__0__Queue__empty(void )
#line 53
{
  return /*PrintfC.QueueC*/BigQueueC__0__size == 0;
}

# 50 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/BigQueue.nc"
inline static bool PrintfP__Queue__empty(void ){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = /*PrintfC.QueueC*/BigQueueC__0__Queue__empty();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 115 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Packet.nc"
inline static void * PrintfP__Packet__getPayload(message_t * msg, uint8_t len){
#line 115
  void *__nesc_result;
#line 115

#line 115
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__getPayload(msg, len);
#line 115

#line 115
  return __nesc_result;
#line 115
}
#line 115
# 120 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialActiveMessageP.nc"
static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength(void )
#line 120
{
  return 28;
}

# 57 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/BigQueueC.nc"
static inline uint16_t /*PrintfC.QueueC*/BigQueueC__0__Queue__size(void )
#line 57
{
  return /*PrintfC.QueueC*/BigQueueC__0__size;
}

# 58 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/BigQueue.nc"
inline static uint16_t PrintfP__Queue__size(void ){
#line 58
  unsigned int __nesc_result;
#line 58

#line 58
  __nesc_result = /*PrintfC.QueueC*/BigQueueC__0__Queue__size();
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 69 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/BigQueueC.nc"
static inline void /*PrintfC.QueueC*/BigQueueC__0__printQueue(void )
#line 69
{
}

#line 65
static inline /*PrintfC.QueueC*/BigQueueC__0__queue_t /*PrintfC.QueueC*/BigQueueC__0__Queue__head(void )
#line 65
{
  return /*PrintfC.QueueC*/BigQueueC__0__queue[/*PrintfC.QueueC*/BigQueueC__0__head];
}

#line 85
static inline /*PrintfC.QueueC*/BigQueueC__0__queue_t /*PrintfC.QueueC*/BigQueueC__0__Queue__dequeue(void )
#line 85
{
  /*PrintfC.QueueC*/BigQueueC__0__queue_t t = /*PrintfC.QueueC*/BigQueueC__0__Queue__head();

#line 87
  ;
  if (!/*PrintfC.QueueC*/BigQueueC__0__Queue__empty()) {
      /*PrintfC.QueueC*/BigQueueC__0__head++;
      /*PrintfC.QueueC*/BigQueueC__0__head %= 1000;
      /*PrintfC.QueueC*/BigQueueC__0__size--;
      /*PrintfC.QueueC*/BigQueueC__0__printQueue();
    }
  return t;
}

# 81 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/BigQueue.nc"
inline static PrintfP__Queue__t PrintfP__Queue__dequeue(void ){
#line 81
  unsigned char __nesc_result;
#line 81

#line 81
  __nesc_result = /*PrintfC.QueueC*/BigQueueC__0__Queue__dequeue();
#line 81

#line 81
  return __nesc_result;
#line 81
}
#line 81
# 269 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_hton_uint16(void * target, uint16_t value)
#line 269
{
  uint8_t *base = target;

#line 271
  base[1] = value;
  base[0] = value >> 8;
  return value;
}

# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialActiveMessageP.nc"
static inline serial_header_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(message_t * msg)
#line 49
{
  return (serial_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(serial_header_t ));
}

#line 147
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setDestination(message_t *amsg, am_addr_t addr)
#line 147
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(amsg);

#line 149
  __nesc_hton_uint16(header->dest.data, addr);
}

# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AMPacket.nc"
inline static void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(message_t * amsg, am_addr_t addr){
#line 92
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setDestination(amsg, addr);
#line 92
}
#line 92
# 240 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_hton_uint8(void * target, uint8_t value)
#line 240
{
  uint8_t *base = target;

#line 242
  base[0] = value;
  return value;
}

# 166 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setType(message_t *amsg, am_id_t type)
#line 166
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(amsg);

#line 168
  __nesc_hton_uint8(header->type.data, type);
}

# 151 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AMPacket.nc"
inline static void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(message_t * amsg, am_id_t t){
#line 151
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setType(amsg, t);
#line 151
}
#line 151
# 69 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AMSend.nc"
inline static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(am_id_t arg_0x40b1e908, am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(arg_0x40b1e908, addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AMPacket.nc"
inline static am_addr_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(message_t * amsg){
#line 67
  unsigned int __nesc_result;
#line 67

#line 67
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__destination(amsg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
#line 136
inline static am_id_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(message_t * amsg){
#line 136
  unsigned char __nesc_result;
#line 136

#line 136
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(amsg);
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 116 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__setPayloadLength(message_t *msg, uint8_t len)
#line 116
{
  __nesc_hton_uint8(/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(msg)->length.data, len);
}

# 83 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Packet.nc"
inline static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(message_t * msg, uint8_t len){
#line 83
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__setPayloadLength(msg, len);
#line 83
}
#line 83
# 82 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/AMQueueImplP.nc"
static inline error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(uint8_t clientId, message_t *msg, 
uint8_t len)
#line 83
{
  if (clientId >= 1) {
      return FAIL;
    }
  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg != (void *)0) {
      return EBUSY;
    }
  ;

  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg = msg;
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(msg, len);

  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current >= 1) {
      error_t err;
      am_id_t amId = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(msg);
      am_addr_t dest = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(msg);

      ;
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = clientId;

      err = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(amId, dest, msg, len);
      if (err != SUCCESS) {
          ;
          /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
          /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg = (void *)0;
        }

      return err;
    }
  else {
      ;
    }
  return SUCCESS;
}

# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Send.nc"
inline static error_t /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(0U, msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 264 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_ntoh_uint16(const void * source)
#line 264
{
  const uint8_t *base = source;

#line 266
  return ((uint16_t )base[0] << 8) | base[1];
}

# 522 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialP.nc"
static inline error_t SerialP__SendBytePacket__startSend(uint8_t b)
#line 522
{
  bool not_busy = FALSE;

#line 524
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 524
    {
      if (SerialP__txBuf[SerialP__TX_DATA_INDEX].state == SerialP__BUFFER_AVAILABLE) {
          SerialP__txBuf[SerialP__TX_DATA_INDEX].state = SerialP__BUFFER_FILLING;
          SerialP__txBuf[SerialP__TX_DATA_INDEX].buf = b;
          not_busy = TRUE;
        }
    }
#line 530
    __nesc_atomic_end(__nesc_atomic); }
  if (not_busy) {
      SerialP__MaybeScheduleTx();
      return SUCCESS;
    }
  return EBUSY;
}

# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SendBytePacket.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__startSend(uint8_t first_byte){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = SerialP__SendBytePacket__startSend(first_byte);
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__dataLinkLength(message_t *msg, uint8_t upperLen)
#line 43
{
  return upperLen + sizeof(serial_header_t );
}

# 350 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(uart_id_t id, message_t *msg, 
uint8_t upperLen)
#line 351
{
  return 0;
}

# 23 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__dataLinkLength(uart_id_t arg_0x407cf398, message_t *msg, uint8_t upperLen){
#line 23
  unsigned char __nesc_result;
#line 23

#line 23
  switch (arg_0x407cf398) {
#line 23
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 23
      __nesc_result = SerialPacketInfoActiveMessageP__Info__dataLinkLength(msg, upperLen);
#line 23
      break;
#line 23
    default:
#line 23
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(arg_0x407cf398, msg, upperLen);
#line 23
      break;
#line 23
    }
#line 23

#line 23
  return __nesc_result;
#line 23
}
#line 23
# 40 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__offset(void )
#line 40
{
  return (uint8_t )(sizeof(message_header_t ) - sizeof(serial_header_t ));
}

# 347 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(uart_id_t id)
#line 347
{
  return 0;
}

# 15 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(uart_id_t arg_0x407cf398){
#line 15
  unsigned char __nesc_result;
#line 15

#line 15
  switch (arg_0x407cf398) {
#line 15
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 15
      __nesc_result = SerialPacketInfoActiveMessageP__Info__offset();
#line 15
      break;
#line 15
    default:
#line 15
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(arg_0x407cf398);
#line 15
      break;
#line 15
    }
#line 15

#line 15
  return __nesc_result;
#line 15
}
#line 15
# 100 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(uint8_t id, message_t *msg, uint8_t len)
#line 100
{
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState != /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE) {
      return EBUSY;
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 105
    {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(id);
      if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex > sizeof(message_header_t )) {
          {
            unsigned char __nesc_temp = 
#line 108
            ESIZE;

            {
#line 108
              __nesc_atomic_end(__nesc_atomic); 
#line 108
              return __nesc_temp;
            }
          }
        }
#line 111
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError = SUCCESS;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer = (uint8_t *)msg;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_DATA;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendId = id;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendCancelled = FALSE;






      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendLen = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__dataLinkLength(id, msg, len) + /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex;
    }
#line 123
    __nesc_atomic_end(__nesc_atomic); }
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__startSend(id) == SUCCESS) {
      return SUCCESS;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE;
      return FAIL;
    }
}

# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Send.nc"
inline static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(TOS_SERIAL_ACTIVE_MESSAGE_ID, msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP__RunTx__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SerialP__RunTx);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 16 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/RadioSend.nc"
inline static error_t LatinMacP__RadioSend__sendNow(void ){
#line 16
  unsigned char __nesc_result;
#line 16

#line 16
  __nesc_result = CC2420DriversP__RadioSend__sendNow();
#line 16

#line 16
  return __nesc_result;
#line 16
}
#line 16
# 26 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/RadioControl.nc"
inline static error_t LatinMacP__RadioControl__receiveNow(bool timeLimitedRx, uint16_t rxWaitTime){
#line 26
  unsigned char __nesc_result;
#line 26

#line 26
  __nesc_result = CC2420DriversP__RadioControl__receiveNow(timeLimitedRx, rxWaitTime);
#line 26

#line 26
  return __nesc_result;
#line 26
}
#line 26
# 518 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/LatinMac/LatinMacP.nc"
static inline void LatinMacP__FastAlarm__fired(void )
#line 518
{
  asn_t temp_asn;
  uint8_t temp_state;
  OpenQueueEntry_t *temp_dataFrameToSend;

  /* atomic removed: atomic calls only */
#line 522
  {
    temp_asn = LatinMacP__asn;
    temp_state = LatinMacP__state;
    temp_dataFrameToSend = LatinMacP__dataFrameToSend;
  }
  switch (temp_state) {

      case S_TX_TXDATAPREPARE: 

        LatinMacP__postTaskSendDone(temp_dataFrameToSend, FAIL);
      LatinMacP__endSlot();
      break;
      case S_TX_TXDATAREADY: 

        LatinMacP__change_state(S_TX_TXDATA);
      if (LatinMacP__RadioSend__sendNow() != SUCCESS) {

          temp_dataFrameToSend->l2_retriesLeft--;
          if (temp_dataFrameToSend->l2_retriesLeft == 0) {
              LatinMacP__postTaskSendDone(temp_dataFrameToSend, FAIL);
            }
          LatinMacP__endSlot();
        }
      break;
      case S_TX_RXACKREADY: 



        LatinMacP__change_state(S_TX_RXACK);
      if (LatinMacP__RadioControl__receiveNow(TIME_LIMITED_RX, TsRxWaitTime) != SUCCESS) {

          LatinMacP__endSlot();
        }
#line 554
      ;
      break;


      case S_RX_RXDATAPREPARE: 


        LatinMacP__endSlot();
      break;
      case S_RX_RXDATAREADY: 

        LatinMacP__change_state(S_RX_RXDATA);
      if (LatinMacP__RadioControl__receiveNow(TIME_LIMITED_RX, TsRxWaitTime) != SUCCESS) {

          LatinMacP__endSlot();
        }
#line 569
      ;
      break;
      case S_RX_TXACKPREPARE: 

        LatinMacP__endSlot();
      break;
      case S_RX_TXACKREADY: 

        LatinMacP__change_state(S_RX_TXACK);
      if (LatinMacP__RadioSend__sendNow() != SUCCESS) {

          LatinMacP__endSlot();
        }
      break;
      default: 
        LatinMacP__endSlot();
      break;
    }
}

# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Alarm__fired(void ){
#line 67
  LatinMacP__FastAlarm__fired();
#line 67
}
#line 67
# 151 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static inline void /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__m_dt == 0) 
      {
        /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Alarm__fired();
      }
    else 
      {
        /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__set_alarm();
      }
  }
}

# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Alarm__fired(void ){
#line 67
  /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__AlarmFrom__fired();
#line 67
}
#line 67
# 124 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__disableEvents(void )
{
  * (volatile uint16_t * )392U &= ~0x0010;
}

# 47 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__disableEvents();
#line 47
}
#line 47
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired(void )
{
  /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__disableEvents();
  /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Alarm__fired();
}

# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void ){
#line 34
  /*LatinMacC.FastAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired();
#line 34
}
#line 34
# 139 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void )
{
  return * (volatile uint16_t * )408U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6____nesc_unnamed4437 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(* (volatile uint16_t * )392U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired();
    }
}

# 164 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__CC2420Transmit__sendNow(bool useCca)
#line 164
{
  CC2420TransmitP__m_cca = useCca;
  /* atomic removed: atomic calls only */
#line 166
  {
    if (CC2420TransmitP__state != CC2420TransmitP__S_LOADED) {
        CC2420TransmitP__reset();



        {
          unsigned char __nesc_temp = 
#line 172
          FAIL;

#line 172
          return __nesc_temp;
        }
      }
  }
#line 175
  if (CC2420TransmitP__SpiResource__immediateRequest() == SUCCESS) {
      CC2420TransmitP__send();
    }
  else 
#line 177
    {
      CC2420TransmitP__SpiResource__request();
    }
  return SUCCESS;
}

# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Transmit.nc"
inline static error_t CC2420DriversP__CC2420Transmit__sendNow(bool useCca){
#line 4
  unsigned char __nesc_result;
#line 4

#line 4
  __nesc_result = CC2420TransmitP__CC2420Transmit__sendNow(useCca);
#line 4

#line 4
  return __nesc_result;
#line 4
}
#line 4
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420TransmitP__STXON__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_STXON);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 78 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__OscResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ControlC.OscSpiC*/CC2420SpiC__1__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78









inline static error_t CC2420ControlP__OscResource__immediateRequest(void ){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = CC2420SpiP__Resource__immediateRequest(/*CC2420ControlC.OscSpiC*/CC2420SpiC__1__CLIENT_ID);
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 99 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Config__startOscillator(void )
#line 99
{
  /* atomic removed: atomic calls only */
#line 100
  {
    if (CC2420ControlP__m_state == CC2420ControlP__S_VREG_STARTED) {
        CC2420ControlP__m_state = CC2420ControlP__S_XOSC_STARTING;
        if (CC2420ControlP__OscResource__immediateRequest() != SUCCESS) {
            CC2420ControlP__OscResource__request();
          }
        else 
#line 105
          {
            CC2420ControlP__startOscillator();
          }
      }
    else 
#line 108
      {
        {
          unsigned char __nesc_temp = 
#line 109
          FAIL;

#line 109
          return __nesc_temp;
        }
      }
  }
#line 112
  return SUCCESS;
}

# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Config.nc"
inline static error_t CC2420DriversP__CC2420Config__startOscillator(void ){
#line 4
  unsigned char __nesc_result;
#line 4

#line 4
  __nesc_result = CC2420ControlP__CC2420Config__startOscillator();
#line 4

#line 4
  return __nesc_result;
#line 4
}
#line 4
# 72 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/CC2420DriversP.nc"
static inline void CC2420DriversP__CC2420Config__startVRegDone(void )
#line 72
{
  /* atomic removed: atomic calls only */
#line 73
  CC2420DriversP__state = CC2420DriversP__S_STARTING_OSCILLATOR;
  CC2420DriversP__CC2420Config__startOscillator();
}

# 3 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Config.nc"
inline static void CC2420ControlP__CC2420Config__startVRegDone(void ){
#line 3
  CC2420DriversP__CC2420Config__startVRegDone();
#line 3
}
#line 3
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__11__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set();
#line 34
}
#line 34
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__11__GeneralIO__set(void )
#line 37
{
#line 37
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__11__HplGeneralIO__set();
}

# 29 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__set(void ){
#line 29
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__11__GeneralIO__set();
#line 29
}
#line 29
# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__11__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr();
#line 39
}
#line 39
# 38 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__11__GeneralIO__clr(void )
#line 38
{
#line 38
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__11__HplGeneralIO__clr();
}

# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__clr(void ){
#line 30
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__11__GeneralIO__clr();
#line 30
}
#line 30
# 88 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__StartupAlarm__fired(void )
#line 88
{
  if (CC2420ControlP__m_state == CC2420ControlP__S_VREG_STARTING) {
      CC2420ControlP__m_state = CC2420ControlP__S_VREG_STARTED;
      CC2420ControlP__RSTN__clr();
      CC2420ControlP__RSTN__set();
      CC2420ControlP__CC2420Config__startVRegDone();
    }
}

# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Alarm__fired(void ){
#line 67
  CC2420ControlP__StartupAlarm__fired();
#line 67
}
#line 67
# 151 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static inline void /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__m_dt == 0) 
      {
        /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Alarm__fired();
      }
    else 
      {
        /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__set_alarm();
      }
  }
}

# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__fired(void ){
#line 67
  /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__AlarmFrom__fired();
#line 67
}
#line 67
# 124 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__disableEvents(void )
{
  * (volatile uint16_t * )394U &= ~0x0010;
}

# 47 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__disableEvents();
#line 47
}
#line 47
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__fired(void )
{
  /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__disableEvents();
  /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__fired();
}

# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void ){
#line 34
  /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__fired();
#line 34
}
#line 34
# 139 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void )
{
  return * (volatile uint16_t * )410U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7____nesc_unnamed4438 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(* (volatile uint16_t * )394U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired();
    }
}

# 79 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__enable(void )
#line 79
{
#line 79
  P1IE |= 1 << 4;
}

# 31 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__HplInterrupt__enable(void ){
#line 31
  HplMsp430InterruptP__Port14__enable();
#line 31
}
#line 31
# 131 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__edge(bool l2h)
#line 131
{
  /* atomic removed: atomic calls only */
#line 132
  {
    if (l2h) {
#line 133
      P1IES &= ~(1 << 4);
      }
    else {
#line 134
      P1IES |= 1 << 4;
      }
  }
}

# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__HplInterrupt__edge(bool low_to_high){
#line 56
  HplMsp430InterruptP__Port14__edge(low_to_high);
#line 56
}
#line 56
# 95 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__clear(void )
#line 95
{
#line 95
  P1IFG &= ~(1 << 4);
}

# 41 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port14__clear();
#line 41
}
#line 41
# 87 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__disable(void )
#line 87
{
#line 87
  P1IE &= ~(1 << 4);
}

# 36 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__HplInterrupt__disable(void ){
#line 36
  HplMsp430InterruptP__Port14__disable();
#line 36
}
#line 36
# 58 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__Interrupt__disable(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__HplInterrupt__disable();
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__HplInterrupt__clear();
  }
  return SUCCESS;
}

#line 41
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__enable(bool rising)
#line 41
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 42
    {
      /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__Interrupt__disable();
      /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__HplInterrupt__edge(rising);
      /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__HplInterrupt__enable();
    }
#line 46
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__Interrupt__enableRisingEdge(void )
#line 50
{
  return /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__enable(TRUE);
}

# 42 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ControlP__InterruptCCA__enableRisingEdge(void ){
#line 42
  unsigned char __nesc_result;
#line 42

#line 42
  __nesc_result = /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__Interrupt__enableRisingEdge();
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ControlP__SXOSCON__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SXOSCON);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__IOCFG0__write(uint16_t data){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420SpiP__Reg__write(CC2420_IOCFG0, data);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
inline static cc2420_status_t CC2420ControlP__MDMCTRL0__write(uint16_t data){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420SpiP__Reg__write(CC2420_MDMCTRL0, data);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
inline static cc2420_status_t CC2420ControlP__RXCTRL1__write(uint16_t data){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420SpiP__Reg__write(CC2420_RXCTRL1, data);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 119 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__enableEvents(void )
{
  * (volatile uint16_t * )394U |= 0x0010;
}

# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__enableEvents();
#line 46
}
#line 46
# 84 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )394U &= ~0x0001;
}

# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )410U = x;
}

# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__setEvent(time);
#line 30
}
#line 30
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 154 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )410U = /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__get() + x;
}

# 32 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 70 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 86
          /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 88
    /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__clearPendingInterrupt();
    /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__enableEvents();
  }
}

# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__AlarmFrom__startAt(/*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__AlarmFrom__size_type t0, /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__AlarmFrom__size_type dt){
#line 92
  /*CC2420ControlC.StartupAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__3__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 129 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__AlarmWatchdog__fired(void )
#line 129
{
  CC2420TransmitP__AlarmWatchdog__stop();
  /* atomic removed: atomic calls only */
#line 131
  {
    switch (CC2420TransmitP__state) {
        case CC2420TransmitP__S_LOAD: 


          case CC2420TransmitP__S_SFD: 

            CC2420TransmitP__reset();
        CC2420TransmitP__signalDone(FAIL);
        break;
        default: 
          break;
      }
  }
}

# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__fired(void ){
#line 67
  CC2420TransmitP__AlarmWatchdog__fired();
#line 67
}
#line 67
# 151 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static inline void /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__m_dt == 0) 
      {
        /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__fired();
      }
    else 
      {
        /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__set_alarm();
      }
  }
}

# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__fired(void ){
#line 67
  /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__AlarmFrom__fired();
#line 67
}
#line 67
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Compare__fired(void )
{
  /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430TimerControl__disableEvents();
  /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Alarm__fired();
}

# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void ){
#line 34
  /*CC2420TransmitC.AlarmWatchdogC.AlarmC.Msp430Alarm*/Msp430AlarmC__4__Msp430Compare__fired();
#line 34
}
#line 34
# 139 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void )
{
  return * (volatile uint16_t * )412U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8____nesc_unnamed4439 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(* (volatile uint16_t * )396U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired();
    }
}

# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/RadioControl.nc"
inline static void CC2420DriversP__RadioControl__receivedNothing(void ){
#line 30
  LatinMacP__RadioControl__receivedNothing();
#line 30
}
#line 30
# 3 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/StdAsyncControl.nc"
inline static error_t CC2420DriversP__RxControl__start(void ){
#line 3
  unsigned char __nesc_result;
#line 3

#line 3
  __nesc_result = CC2420ReceiveP__StdAsyncControl__start();
#line 3

#line 3
  return __nesc_result;
#line 3
}
#line 3
inline static error_t CC2420DriversP__TxControl__start(void ){
#line 3
  unsigned char __nesc_result;
#line 3

#line 3
  __nesc_result = CC2420TransmitP__StdAsyncControl__start();
#line 3

#line 3
  return __nesc_result;
#line 3
}
#line 3
# 143 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/CC2420DriversP.nc"
static inline void CC2420DriversP__RxWaitAlarm__fired(void )
#line 143
{
  CC2420DriversP__RxWaitAlarm__stop();
  CC2420DriversP__TxControl__start();
  CC2420DriversP__RxControl__start();
  CC2420DriversP__RadioControl__receivedNothing();
}

# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__fired(void ){
#line 67
  CC2420DriversP__RxWaitAlarm__fired();
#line 67
}
#line 67
# 151 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static inline void /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__m_dt == 0) 
      {
        /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Alarm__fired();
      }
    else 
      {
        /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__set_alarm();
      }
  }
}

# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Alarm__fired(void ){
#line 67
  /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__AlarmFrom__fired();
#line 67
}
#line 67
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430Compare__fired(void )
{
  /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430TimerControl__disableEvents();
  /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Alarm__fired();
}

# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void ){
#line 34
  /*CC2420DriversC.RxWaitAlarmC.AlarmC.Msp430Alarm*/Msp430AlarmC__5__Msp430Compare__fired();
#line 34
}
#line 34
# 139 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void )
{
  return * (volatile uint16_t * )414U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9____nesc_unnamed4440 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(* (volatile uint16_t * )398U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired();
    }
}

# 120 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )286U;

#line 123
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(n >> 1);
}

# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB1__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired();
#line 28
}
#line 28
# 113 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__Scheduler__init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP__m_next, SchedulerBasicP__NO_TASK, sizeof SchedulerBasicP__m_next);
    SchedulerBasicP__m_head = SchedulerBasicP__NO_TASK;
    SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
  }
}

# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__init(void ){
#line 46
  SchedulerBasicP__Scheduler__init();
#line 46
}
#line 46
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set();
#line 34
}
#line 34
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set();
}

# 29 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__set(void ){
#line 29
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set();
#line 29
}
#line 29
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t * )49U |= 0x01 << 5;
}

# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set();
#line 34
}
#line 34
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set();
}

# 29 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__set(void ){
#line 29
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set();
#line 29
}
#line 29
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t * )49U |= 0x01 << 4;
}

# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set();
#line 34
}
#line 34
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set();
}

# 29 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__set(void ){
#line 29
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set();
#line 29
}
#line 29
# 52 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 6;
}

# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput();
}

# 35 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__makeOutput(void ){
#line 35
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 5;
}

# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput();
}

# 35 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__makeOutput(void ){
#line 35
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 4;
}

# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput();
}

# 35 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__makeOutput(void ){
#line 35
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput();
#line 35
}
#line 35
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 46
  {
    ;
    LedsP__Led0__makeOutput();
    LedsP__Led1__makeOutput();
    LedsP__Led2__makeOutput();
    LedsP__Led0__set();
    LedsP__Led1__set();
    LedsP__Led2__set();
  }
  return SUCCESS;
}

# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
inline static error_t PlatformP__LedsInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = LedsP__Init__init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 36 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_SIMO0_PIN()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x0019");

#line 36
  r |= 1 << 1;
}

#line 37
static inline  void TOSH_SET_UCLK0_PIN()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x0019");

#line 37
  r |= 1 << 3;
}

#line 88
static inline  void TOSH_SET_FLASH_CS_PIN()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001D");

#line 88
  r |= 1 << 4;
}

#line 37
static inline  void TOSH_CLR_UCLK0_PIN()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x0019");

#line 37
  r &= ~(1 << 3);
}

#line 88
static inline  void TOSH_CLR_FLASH_CS_PIN()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001D");

#line 88
  r &= ~(1 << 4);
}

# 11 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__TOSH_wait(void )
#line 11
{
   __asm volatile ("nop"); __asm volatile ("nop");}

# 89 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_FLASH_HOLD_PIN()
#line 89
{
#line 89
  static volatile uint8_t r __asm ("0x001D");

#line 89
  r |= 1 << 7;
}

#line 88
static inline  void TOSH_MAKE_FLASH_CS_OUTPUT()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001E");

#line 88
  r |= 1 << 4;
}

#line 89
static inline  void TOSH_MAKE_FLASH_HOLD_OUTPUT()
#line 89
{
#line 89
  static volatile uint8_t r __asm ("0x001E");

#line 89
  r |= 1 << 7;
}

#line 37
static inline  void TOSH_MAKE_UCLK0_OUTPUT()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x001A");

#line 37
  r |= 1 << 3;
}

#line 36
static inline  void TOSH_MAKE_SIMO0_OUTPUT()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x001A");

#line 36
  r |= 1 << 1;
}

# 27 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosb/MotePlatformC.nc"
static inline void MotePlatformC__TOSH_FLASH_M25P_DP(void )
#line 27
{

  TOSH_MAKE_SIMO0_OUTPUT();
  TOSH_MAKE_UCLK0_OUTPUT();
  TOSH_MAKE_FLASH_HOLD_OUTPUT();
  TOSH_MAKE_FLASH_CS_OUTPUT();
  TOSH_SET_FLASH_HOLD_PIN();
  TOSH_SET_FLASH_CS_PIN();

  MotePlatformC__TOSH_wait();


  TOSH_CLR_FLASH_CS_PIN();
  TOSH_CLR_UCLK0_PIN();

  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);

  TOSH_SET_FLASH_CS_PIN();
  TOSH_SET_UCLK0_PIN();
  TOSH_SET_SIMO0_PIN();
}

#line 6
static __inline void MotePlatformC__uwait(uint16_t u)
#line 6
{
  uint16_t t0 = TA0R;

#line 8
  while (TA0R - t0 <= u) ;
}

#line 56
static inline error_t MotePlatformC__Init__init(void )
#line 56
{
  /* atomic removed: atomic calls only */

  {
    P1SEL = 0;
    P2SEL = 0;
    P3SEL = 0;
    P4SEL = 0;
    P5SEL = 0;
    P6SEL = 0;

    P1OUT = 0x00;
    P1DIR = 0xe0;

    P2OUT = 0x30;
    P2DIR = 0x7b;

    P3OUT = 0x00;
    P3DIR = 0xf1;

    P4OUT = 0xdd;
    P4DIR = 0xfd;

    P5OUT = 0xff;
    P5DIR = 0xff;

    P6OUT = 0x00;
    P6DIR = 0xff;

    P1IE = 0;
    P2IE = 0;






    MotePlatformC__uwait(1024 * 10);

    MotePlatformC__TOSH_FLASH_M25P_DP();
  }

  return SUCCESS;
}

# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
inline static error_t PlatformP__MoteInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = MotePlatformC__Init__init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 152 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__startTimerB(void )
{

  Msp430ClockP__TBCTL = 0x0020 | (Msp430ClockP__TBCTL & ~(0x0020 | 0x0010));
}

#line 140
static inline void Msp430ClockP__startTimerA(void )
{

  Msp430ClockP__TA0CTL = 0x0020 | (Msp430ClockP__TA0CTL & ~(0x0020 | 0x0010));
}

#line 104
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void )
{
  TBR = 0;









  Msp430ClockP__TBCTL = 0x0100 | 0x0002;
}

#line 134
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerB();
}

# 32 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerB(void ){
#line 32
  Msp430ClockP__Msp430ClockInit__default__initTimerB();
#line 32
}
#line 32
# 89 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void )
{
  TA0R = 0;









  Msp430ClockP__TA0CTL = 0x0200 | 0x0002;
}

#line 129
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerA();
}

# 31 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerA(void ){
#line 31
  Msp430ClockP__Msp430ClockInit__default__initTimerA();
#line 31
}
#line 31
# 68 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void )
{





  BCSCTL1 = 0x80 | (BCSCTL1 & ((0x04 | 0x02) | 0x01));







  BCSCTL2 = 0x04;


  Msp430ClockP__IE1 &= ~(1 << 1);
}

#line 124
static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitClocks();
}

# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initClocks(void ){
#line 30
  Msp430ClockP__Msp430ClockInit__default__initClocks();
#line 30
}
#line 30
# 170 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline uint16_t Msp430ClockP__test_calib_busywait_delta(int calib)
{
  int8_t aclk_count = 2;
  uint16_t dco_prev = 0;
  uint16_t dco_curr = 0;

  Msp430ClockP__set_dco_calib(calib);

  while (aclk_count-- > 0) 
    {
      TBCCR0 = TBR + Msp430ClockP__ACLK_CALIB_PERIOD;
      TBCCTL0 &= ~0x0001;
      while ((TBCCTL0 & 0x0001) == 0) ;
      dco_prev = dco_curr;
      dco_curr = TA0R;
    }

  return dco_curr - dco_prev;
}




static inline void Msp430ClockP__busyCalibrateDco(void )
{

  int calib;
  int step;






  for (calib = 0, step = 0x800; step != 0; step >>= 1) 
    {

      if (Msp430ClockP__test_calib_busywait_delta(calib | step) <= Msp430ClockP__TARGET_DCO_DELTA) {
        calib |= step;
        }
    }

  if ((calib & 0x0e0) == 0x0e0) {
    calib &= ~0x01f;
    }
  Msp430ClockP__set_dco_calib(calib);
}

#line 56
static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void )
{



  Msp430ClockP__TA0CTL = 0x0200 | 0x0020;
  Msp430ClockP__TBCTL = 0x0100 | 0x0020;
  BCSCTL1 = 0x80 | 0x04;
  BCSCTL2 = 0;
  TBCCTL0 = 0x4000;
}

#line 119
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void )
{
  Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate();
}

# 29 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void ){
#line 29
  Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate();
#line 29
}
#line 29
# 218 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline error_t Msp430ClockP__Init__init(void )
{

  Msp430ClockP__TA0CTL = 0x0004;
  Msp430ClockP__TA0IV = 0;
  Msp430ClockP__TBCTL = 0x0004;
  Msp430ClockP__TBIV = 0;
  /* atomic removed: atomic calls only */

  {
    Msp430ClockP__Msp430ClockInit__setupDcoCalibrate();
    Msp430ClockP__busyCalibrateDco();
    Msp430ClockP__Msp430ClockInit__initClocks();
    Msp430ClockP__Msp430ClockInit__initTimerA();
    Msp430ClockP__Msp430ClockInit__initTimerB();
    Msp430ClockP__startTimerA();
    Msp430ClockP__startTimerB();
  }

  return SUCCESS;
}

# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
inline static error_t PlatformP__MoteClockInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = Msp430ClockP__Init__init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 10 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void )
#line 10
{
  PlatformP__MoteClockInit__init();
  PlatformP__MoteInit__init();
  PlatformP__LedsInit__init();
  return SUCCESS;
}

# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
inline static error_t RealMainP__PlatformInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = PlatformP__Init__init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 36 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosb/hardware.h"
static inline  void TOSH_CLR_SIMO0_PIN()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x0019");

#line 36
  r &= ~(1 << 1);
}

# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Scheduler.nc"
inline static bool RealMainP__Scheduler__runNextTask(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = SchedulerBasicP__Scheduler__runNextTask();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 26 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/openqueue/OpenQueueP.nc"
static inline void OpenQueueP__taskDebugPrint__runTask(void )
#line 26
{
  debugOpenQueueEntry_t output[QUEUELENGTH];
  uint8_t i;

#line 29
  for (i = 0; i < QUEUELENGTH; i++) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 30
        {
          output[i].creator = OpenQueueP__queue[i].creator;
          output[i].owner = OpenQueueP__queue[i].owner;
        }
#line 33
        __nesc_atomic_end(__nesc_atomic); }
    }
}

# 276 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_ntoh_leuint16(const void * source)
#line 276
{
  const uint8_t *base = source;

#line 278
  return ((uint16_t )base[1] << 8) | base[0];
}

# 5 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/Malloc.nc"
inline static error_t LatinMacP__Malloc__freePacketBuffer(OpenQueueEntry_t *pkt){
#line 5
  unsigned char __nesc_result;
#line 5

#line 5
  __nesc_result = OpenQueueP__Malloc__freePacketBuffer(pkt);
#line 5

#line 5
  return __nesc_result;
#line 5
}
#line 5
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t LatinMacP__taskReceive__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(LatinMacP__taskReceive);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 98 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/LedsP.nc"
static inline void LedsP__Leds__led2Off(void )
#line 98
{
  LedsP__Led2__set();
  ;
#line 100
  ;
}

# 83 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Leds.nc"
inline static void LatinMacP__Leds__led2Off(void ){
#line 83
  LedsP__Leds__led2Off();
#line 83
}
#line 83
# 875 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/LatinMac/LatinMacP.nc"
static inline void LatinMacP__resynchronize(bool resyncType, open_addr_t *node_id, 
timervalue_t dataGlobalSlotOffset, int16_t timeCorrection)
#line 876
{

  bool temp_isSync;

  bool iShouldSynchronize;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 882
    {
      temp_isSync = LatinMacP__isSync;
    }
#line 884
    __nesc_atomic_end(__nesc_atomic); }

  if (LatinMacP__IDManager__getIsDAGroot() == FALSE) {


      iShouldSynchronize = FALSE;
      if (temp_isSync == FALSE) {

          if (resyncType == FRAME_BASED_RESYNC) {
              iShouldSynchronize = TRUE;
              { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 894
                LatinMacP__isSync = TRUE;
#line 894
                __nesc_atomic_end(__nesc_atomic); }

              LatinMacP__Leds__led2On();
            }
#line 911
          if (iShouldSynchronize == TRUE) {
              { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 912
                {
                  if (resyncType == FRAME_BASED_RESYNC) {
                      if (dataGlobalSlotOffset != INVALID_TIMESTAMP) {
                          if (LatinMacP__slotAlarmStartSlotTimestamp + dataGlobalSlotOffset < LatinMacP__SlotAlarm__getNow()) {
                              LatinMacP__SlotAlarm__startAt((uint32_t )((int32_t )LatinMacP__slotAlarmStartSlotTimestamp + (int32_t )timeCorrection), SLOT_TIME);
                            }
                          else 
#line 917
                            {
                              LatinMacP__isSync = FALSE;





                              LatinMacP__Leds__led2Off();
                              LatinMacP__SlotAlarm__startAt(LatinMacP__SlotAlarm__getNow() - SLOT_TIME / 2, SLOT_TIME);
                              LatinMacP__endSlot();
                            }
                        }
                    }
                  else {
                      LatinMacP__SlotAlarm__startAt((uint32_t )((int32_t )LatinMacP__slotAlarmStartSlotTimestamp + (int32_t )timeCorrection), SLOT_TIME);
                    }
                }
#line 933
                __nesc_atomic_end(__nesc_atomic); }
            }
        }
    }
}

# 10 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/PacketFunctions.nc"
inline static bool LatinMacP__PacketFunctions__isBroadcastMulticast(open_addr_t *address){
#line 10
  unsigned char __nesc_result;
#line 10

#line 10
  __nesc_result = PacketFunctionsP__PacketFunctions__isBroadcastMulticast(address);
#line 10

#line 10
  return __nesc_result;
#line 10
}
#line 10
# 10 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/IDManager.nc"
inline static bool LatinMacP__IDManager__isMyAddress(open_addr_t *addr){
#line 10
  unsigned char __nesc_result;
#line 10

#line 10
  __nesc_result = IDManagerP__IDManager__isMyAddress(addr);
#line 10

#line 10
  return __nesc_result;
#line 10
}
#line 10
# 244 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/packetfunctions/PacketFunctionsP.nc"
static inline void PacketFunctionsP__PacketFunctions__tossFooter(OpenQueueEntry_t *pkt, uint8_t header_length)
#line 244
{
  pkt->length -= header_length;
  if (pkt->length > 128) {
    }
}

# 19 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/PacketFunctions.nc"
inline static void LatinMacP__PacketFunctions__tossFooter(OpenQueueEntry_t *pkt, uint8_t header_length){
#line 19
  PacketFunctionsP__PacketFunctions__tossFooter(pkt, header_length);
#line 19
}
#line 19
# 226 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/packetfunctions/PacketFunctionsP.nc"
static inline void PacketFunctionsP__PacketFunctions__tossHeader(OpenQueueEntry_t *pkt, uint8_t header_length)
#line 226
{
  pkt->payload += header_length;
  pkt->length -= header_length;
  if ((uint8_t *)pkt->payload > (uint8_t *)(pkt->packet + 126)) {
    }
}

# 17 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/PacketFunctions.nc"
inline static void LatinMacP__PacketFunctions__tossHeader(OpenQueueEntry_t *pkt, uint8_t header_length){
#line 17
  PacketFunctionsP__PacketFunctions__tossHeader(pkt, header_length);
#line 17
}
#line 17
# 723 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/LatinMac/LatinMacP.nc"
static inline void LatinMacP__RadioReceive__receive(OpenQueueEntry_t *msg)
#line 723
{
  asn_t temp_asn;
  uint8_t temp_state;
  bool temp_isSync;
  OpenQueueEntry_t *temp_dataFrameToSend;
  ieee802154_header_iht received_ieee154_header;
  ieee802154_header_iht transmitted_ieee154_header;

  uint8_t temp_init;
  uint8_t temp_numADV;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 734
    {
      temp_asn = LatinMacP__asn;
      temp_isSync = LatinMacP__isSync;
      temp_state = LatinMacP__state;
      temp_dataFrameToSend = LatinMacP__dataFrameToSend;
      temp_init = LatinMacP__init;
      temp_numADV = LatinMacP__numADV;
    }
#line 741
    __nesc_atomic_end(__nesc_atomic); }
  msg->owner = COMPONENT_MAC;

  received_ieee154_header = LatinMacP__retrieveIEEE802154header(msg);
  LatinMacP__PacketFunctions__tossHeader(msg, received_ieee154_header.headerLength);
  LatinMacP__PacketFunctions__tossFooter(msg, 2);

  msg->l2_frameType = received_ieee154_header.frameType;
  memcpy(& msg->l2_nextORpreviousHop, & received_ieee154_header.src, sizeof(open_addr_t ));

  if (received_ieee154_header.frameType == IEEE154_TYPE_DATA && 
  !LatinMacP__IDManager__isMyAddress(& received_ieee154_header.panid)) {
      LatinMacP__Malloc__freePacketBuffer(msg);
      return;
    }

  switch (temp_state) {


      case S_TX_RXACK: 

        transmitted_ieee154_header = LatinMacP__retrieveIEEE802154header(temp_dataFrameToSend);
      if (received_ieee154_header.dsn == transmitted_ieee154_header.dsn) {








          LatinMacP__postTaskSendDone(temp_dataFrameToSend, SUCCESS);
        }
      else 
#line 773
        {


          temp_dataFrameToSend->l2_retriesLeft--;
          if (temp_dataFrameToSend->l2_retriesLeft == 0) {
              LatinMacP__postTaskSendDone(temp_dataFrameToSend, FAIL);
            }
        }
      LatinMacP__Malloc__freePacketBuffer(msg);
      LatinMacP__endSlot();
      break;


      case S_SYNCHRONIZING: 
        case S_RX_RXDATA: 

          if (LatinMacP__IDManager__isMyAddress(& received_ieee154_header.dest) && received_ieee154_header.ackRequested) {
#line 831
              { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 831
                LatinMacP__frameReceived = msg;
#line 831
                __nesc_atomic_end(__nesc_atomic); }
              LatinMacP__taskReceive__postTask();
              LatinMacP__endSlot();
            }
          else {
#line 835
            if (LatinMacP__PacketFunctions__isBroadcastMulticast(& received_ieee154_header.dest)) {
                if (received_ieee154_header.frameType == IEEE154_TYPE_CMD && __nesc_ntoh_leuint8((
                (CMD_MSG_t *)msg->payload)->commandFrameId.data) == SCAN_MSG) {
                    if (temp_isSync == FALSE) {
                        { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 839
                          LatinMacP__asn = __nesc_ntoh_leuint16(((CMD_MSG_t *)msg->payload)->timingInformation.data);
#line 839
                          __nesc_atomic_end(__nesc_atomic); }
                        LatinMacP__slotType = UPLINK;
                      }
                    LatinMacP__resynchronize(FRAME_BASED_RESYNC, 
                    & received_ieee154_header.src, 
                    msg->l1_rxTimestamp, 
                    (int16_t )((int32_t )msg->l1_rxTimestamp - (int32_t )radio_delay) - (int32_t )TsTxOffset);
                  }
                { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 847
                  LatinMacP__frameReceived = msg;
#line 847
                  __nesc_atomic_end(__nesc_atomic); }
                LatinMacP__taskReceive__postTask();
                LatinMacP__endSlot();
              }
            else {

                LatinMacP__Malloc__freePacketBuffer(msg);
                LatinMacP__endSlot();
              }
            }
#line 856
      break;
      default: 
        LatinMacP__Malloc__freePacketBuffer(msg);
      LatinMacP__endSlot();
      break;
    }
}

# 7 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/RadioReceive.nc"
inline static void CC2420ReceiveP__RadioReceive__receive(OpenQueueEntry_t *msg){
#line 7
  LatinMacP__RadioReceive__receive(msg);
#line 7
}
#line 7
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/Malloc.nc"
inline static OpenQueueEntry_t *CC2420ReceiveP__Malloc__getFreePacketBuffer(void ){
#line 4
  struct OpenQueueEntry_t *__nesc_result;
#line 4

#line 4
  __nesc_result = OpenQueueP__Malloc__getFreePacketBuffer();
#line 4

#line 4
  return __nesc_result;
#line 4
}
#line 4
# 186 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__taskReceiveDone__runTask(void )
#line 186
{
  uint8_t *buf = (uint8_t *)CC2420ReceiveP__receptionBuffer->payload;
  uint8_t length = buf[0] + 1;
  OpenQueueEntry_t *new_receptionBuffer;

  CC2420ReceiveP__receptionBuffer->l1_crc = buf[length - 1] >> 7;
  CC2420ReceiveP__receptionBuffer->l1_lqi = buf[length - 1] & 0x7f;
  CC2420ReceiveP__receptionBuffer->l1_rssi = buf[length - 2];

  new_receptionBuffer = CC2420ReceiveP__Malloc__getFreePacketBuffer();
  if (new_receptionBuffer == (void *)0) {


      return;
    }
  new_receptionBuffer->creator = COMPONENT_CC2420RECEIVE;
  new_receptionBuffer->owner = COMPONENT_CC2420RECEIVE;
  new_receptionBuffer->payload = new_receptionBuffer->packet;
  new_receptionBuffer->length = 0;

  CC2420ReceiveP__RadioReceive__receive(CC2420ReceiveP__receptionBuffer);
  CC2420ReceiveP__receptionBuffer = new_receptionBuffer;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 209
    CC2420ReceiveP__receivingPacket = FALSE;
#line 209
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ReceiveP__waitForNextPacket();
}

# 11 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/PacketFunctions.nc"
inline static bool IDManagerP__PacketFunctions__sameAddress(open_addr_t *address_1, open_addr_t *address_2){
#line 11
  unsigned char __nesc_result;
#line 11

#line 11
  __nesc_result = PacketFunctionsP__PacketFunctions__sameAddress(address_1, address_2);
#line 11

#line 11
  return __nesc_result;
#line 11
}
#line 11
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420SpiP__grant__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420SpiP__grant);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 184 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__SpiResource__granted(void )
#line 184
{
  CC2420SpiP__grant__postTask();
}

# 114 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__default__granted(uint8_t id)
#line 114
{
}

# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__granted(uint8_t arg_0x410c6b00){
#line 92
  switch (arg_0x410c6b00) {
#line 92
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 92
      CC2420SpiP__SpiResource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__default__granted(arg_0x410c6b00);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 98 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__granted(uint8_t id)
#line 98
{
  /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__granted(id);
}

# 199 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(uint8_t id)
#line 199
{
}

# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__granted(uint8_t arg_0x40a99e98){
#line 92
  switch (arg_0x40a99e98) {
#line 92
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 92
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__granted(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(arg_0x40a99e98);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 187 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask(void )
#line 187
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 188
    {
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId;
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY;
    }
#line 191
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId);
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__granted(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId);
}

# 197 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error)
#line 197
{
}

# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/SpiPacket.nc"
inline static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiPacket__sendDone(uint8_t arg_0x410c30c0, uint8_t * txBuf, uint8_t * rxBuf, uint16_t len, error_t error){
#line 71
  switch (arg_0x410c30c0) {
#line 71
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 71
      CC2420SpiP__SpiPacket__sendDone(txBuf, rxBuf, len, error);
#line 71
      break;
#line 71
    default:
#line 71
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiPacket__default__sendDone(arg_0x410c30c0, txBuf, rxBuf, len, error);
#line 71
      break;
#line 71
    }
#line 71
}
#line 71
# 190 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone(error_t error)
#line 190
{
  /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiPacket__sendDone(/*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_client, /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_tx_buf, /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_rx_buf, /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_len, error);
}

#line 180
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone_task__runTask(void )
#line 180
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 181
    /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone(SUCCESS);
#line 181
    __nesc_atomic_end(__nesc_atomic); }
}

# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t CC2420ReceiveP__SpiResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ReceiveC.CC2420SpiC*/CC2420SpiC__6__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set();
#line 34
}
#line 34
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__GeneralIO__set(void )
#line 37
{
#line 37
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__HplGeneralIO__set();
}

# 29 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void CC2420ReceiveP__CSN__set(void ){
#line 29
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__GeneralIO__set();
#line 29
}
#line 29
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420ReceivedNothing.nc"
inline static void CC2420ReceiveP__CC2420ReceivedNothing__receivedNothing(void ){
#line 4
  CC2420DriversP__CC2420ReceivedNothing__receivedNothing();
#line 4
}
#line 4
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420ReceiveP__taskReceiveDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420ReceiveP__taskReceiveDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void )
#line 48
{
#line 48
  return * (volatile uint8_t * )32U & (0x01 << 0);
}

#line 49
static inline bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void )
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw() != 0;
}

# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__10__HplGeneralIO__get(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 40 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__10__GeneralIO__get(void )
#line 40
{
#line 40
  return /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__10__HplGeneralIO__get();
}

# 32 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static bool CC2420ReceiveP__FIFOP__get(void ){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__10__GeneralIO__get();
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void )
#line 48
{
#line 48
  return * (volatile uint8_t * )32U & (0x01 << 3);
}

#line 49
static inline bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void )
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw() != 0;
}

# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__9__HplGeneralIO__get(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 40 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__9__GeneralIO__get(void )
#line 40
{
#line 40
  return /*HplCC2420PinsC.FIFOM*/Msp430GpioC__9__HplGeneralIO__get();
}

# 32 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static bool CC2420ReceiveP__FIFO__get(void ){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = /*HplCC2420PinsC.FIFOM*/Msp430GpioC__9__GeneralIO__get();
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 62 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Fifo.nc"
inline static error_t CC2420ReceiveP__RXFIFO__continueRead(uint8_t * data, uint8_t length){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = CC2420SpiP__Fifo__continueRead(CC2420_RXFIFO, data, length);
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 108 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__RXFIFO__readDone(uint8_t *rx_buf, uint8_t rx_len, error_t error)
#line 108
{
  uint8_t *buf = (uint8_t *)CC2420ReceiveP__receptionBuffer->payload;

#line 110
  CC2420ReceiveP__receptionBuffer->length = buf[0] + 1;

  switch (CC2420ReceiveP__state) {

      case CC2420ReceiveP__S_RX_LENGTH: 
        CC2420ReceiveP__state = CC2420ReceiveP__S_RX_PAYLOAD;
      if (CC2420ReceiveP__receptionBuffer->length > 128) {



          CC2420ReceiveP__CC2420ReceivedNothing__receivedNothing();
          CC2420ReceiveP__reset();
        }
      else {
#line 122
        if (CC2420ReceiveP__receptionBuffer->length > CC2420ReceiveP__free_bytes_left_in_fifo) {



            CC2420ReceiveP__CC2420ReceivedNothing__receivedNothing();
            CC2420ReceiveP__reset();
          }
        else {
#line 128
          if (CC2420ReceiveP__receptionBuffer->length == 1) {
              /* atomic removed: atomic calls only */


              CC2420ReceiveP__receivingPacket = FALSE;
              CC2420ReceiveP__CSN__set();
              CC2420ReceiveP__SpiResource__release();
              CC2420ReceiveP__waitForNextPacket();
            }
          else {
#line 136
            if (CC2420ReceiveP__receptionBuffer->length < 6) {



                CC2420ReceiveP__CC2420ReceivedNothing__receivedNothing();
                CC2420ReceiveP__reset();
              }
            else 
#line 142
              {
                if (!CC2420ReceiveP__FIFO__get() && !CC2420ReceiveP__FIFOP__get()) {
                    CC2420ReceiveP__free_bytes_left_in_fifo -= CC2420ReceiveP__receptionBuffer->length;
                  }
                CC2420ReceiveP__state = CC2420ReceiveP__S_RX_PAYLOAD;
                CC2420ReceiveP__RXFIFO__continueRead(buf + 1, CC2420ReceiveP__receptionBuffer->length - 1);
              }
            }
          }
        }
#line 149
      break;

      case CC2420ReceiveP__S_RX_PAYLOAD: 
        CC2420ReceiveP__CSN__set();
      if (!CC2420ReceiveP__missed_packets) {

          CC2420ReceiveP__SpiResource__release();
        }

      if (((CC2420ReceiveP__missed_packets && CC2420ReceiveP__FIFO__get()) || !CC2420ReceiveP__FIFOP__get()) || !CC2420ReceiveP__timestamp_size) {
          CC2420ReceiveP__receptionBuffer->l1_rxTimestamp = INVALID_TIMESTAMP;
        }
      else 
#line 160
        {
          CC2420ReceiveP__receptionBuffer->l1_rxTimestamp = CC2420ReceiveP__timestamp_queue[CC2420ReceiveP__timestamp_head];
          CC2420ReceiveP__timestamp_head = (CC2420ReceiveP__timestamp_head + 1) % CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE;
          CC2420ReceiveP__timestamp_size--;
        }
      if (buf[CC2420ReceiveP__receptionBuffer->length - 1] >> 7 && rx_buf) {
          CC2420ReceiveP__taskReceiveDone__postTask();
          return;
        }
      else 
#line 168
        {



          CC2420ReceiveP__CC2420ReceivedNothing__receivedNothing();
          CC2420ReceiveP__reset();
        }
      CC2420ReceiveP__waitForNextPacket();
      break;

      default: /* atomic removed: atomic calls only */
        CC2420ReceiveP__receivingPacket = FALSE;
      CC2420ReceiveP__CSN__set();
      CC2420ReceiveP__SpiResource__release();
      break;
    }
}

# 343 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__TXFIFO__readDone(uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 343
{
}

# 370 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Fifo__default__readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error)
#line 370
{
}

# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Fifo.nc"
inline static void CC2420SpiP__Fifo__readDone(uint8_t arg_0x4104a950, uint8_t * data, uint8_t length, error_t error){
#line 71
  switch (arg_0x4104a950) {
#line 71
    case CC2420_TXFIFO:
#line 71
      CC2420TransmitP__TXFIFO__readDone(data, length, error);
#line 71
      break;
#line 71
    case CC2420_RXFIFO:
#line 71
      CC2420ReceiveP__RXFIFO__readDone(data, length, error);
#line 71
      break;
#line 71
    default:
#line 71
      CC2420SpiP__Fifo__default__readDone(arg_0x4104a950, data, length, error);
#line 71
      break;
#line 71
    }
#line 71
}
#line 71
# 302 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__RXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 302
{
}

# 8 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Config.nc"
inline static error_t CC2420DriversP__CC2420Config__setChannel(uint8_t channel){
#line 8
  unsigned char __nesc_result;
#line 8

#line 8
  __nesc_result = CC2420ControlP__CC2420Config__setChannel(channel);
#line 8

#line 8
  return __nesc_result;
#line 8
}
#line 8
# 180 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/CC2420DriversP.nc"
static inline void CC2420DriversP__CC2420Transmit__loadPacketDone(error_t error)
#line 180
{
  uint8_t temp_frequencyChannel;

  /* atomic removed: atomic calls only */
#line 182
  {
    temp_frequencyChannel = CC2420DriversP__frequencyChannel;
  }
  if (error == SUCCESS) {
      /* atomic removed: atomic calls only */
#line 186
      CC2420DriversP__syncForSend = TRUE;
      /* atomic removed: atomic calls only */
#line 187
      CC2420DriversP__state = CC2420DriversP__S_SETTING_CHANNEL;
      if (CC2420DriversP__CC2420Config__setChannel(temp_frequencyChannel) != SUCCESS) {
        }
    }
  else 

    {
      /* atomic removed: atomic calls only */
#line 194
      CC2420DriversP__state = CC2420DriversP__S_STARTED;
      CC2420DriversP__RadioSend__prepareSendDone(FAIL);
    }
}

# 3 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Transmit.nc"
inline static void CC2420TransmitP__CC2420Transmit__loadPacketDone(error_t error){
#line 3
  CC2420DriversP__CC2420Transmit__loadPacketDone(error);
#line 3
}
#line 3
# 29 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CSN__set(void ){
#line 29
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__GeneralIO__set();
#line 29
}
#line 29
# 147 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__TXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 147
{
  CC2420TransmitP__AlarmWatchdog__stop();
  if (error == SUCCESS) {
      /* atomic removed: atomic calls only */
#line 150
      CC2420TransmitP__state = CC2420TransmitP__S_LOADED;
    }
  else 
#line 151
    {
      CC2420TransmitP__reset();
    }



  CC2420TransmitP__CSN__set();
  CC2420TransmitP__SpiResource__release();
  CC2420TransmitP__CC2420Transmit__loadPacketDone(error);
}

# 373 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Fifo__default__writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 373
{
}

# 91 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Fifo.nc"
inline static void CC2420SpiP__Fifo__writeDone(uint8_t arg_0x4104a950, uint8_t * data, uint8_t length, error_t error){
#line 91
  switch (arg_0x4104a950) {
#line 91
    case CC2420_TXFIFO:
#line 91
      CC2420TransmitP__TXFIFO__writeDone(data, length, error);
#line 91
      break;
#line 91
    case CC2420_RXFIFO:
#line 91
      CC2420ReceiveP__RXFIFO__writeDone(data, length, error);
#line 91
      break;
#line 91
    default:
#line 91
      CC2420SpiP__Fifo__default__writeDone(arg_0x4104a950, data, length, error);
#line 91
      break;
#line 91
    }
#line 91
}
#line 91
# 114 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__OscResource__granted(void )
#line 114
{
  CC2420ControlP__startOscillator();
}

#line 175
static inline void CC2420ControlP__SpiResource__granted(void )
#line 175
{
  CC2420ControlP__writeId();
}

#line 214
static inline void CC2420ControlP__SyncResource__granted(void )
#line 214
{
  CC2420ControlP__syncConfig();
}

#line 253
static inline void CC2420ControlP__RxOnResource__granted(void )
#line 253
{
  CC2420ControlP__rxOn();
}

#line 283
static inline void CC2420ControlP__RfOffResource__granted(void )
#line 283
{
  CC2420ControlP__rfOff();
}

# 99 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__SpiResource__granted(void )
#line 99
{
  CC2420ReceiveP__receiveLengthByte();
}

# 94 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__SpiResource__granted(void )
#line 94
{
  uint8_t cur_state;

#line 96
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 96
    {
      cur_state = CC2420TransmitP__state;
    }
#line 98
    __nesc_atomic_end(__nesc_atomic); }
  switch (cur_state) {
      case CC2420TransmitP__S_LOAD: 
        CC2420TransmitP__loadTXFIFO();
      break;
      case CC2420TransmitP__S_LOADED: 
        CC2420TransmitP__send();
      break;
      default: 
        CC2420TransmitP__SpiResource__release();
      break;
    }
}

# 367 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Resource__default__granted(uint8_t id)
#line 367
{
}

# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static void CC2420SpiP__Resource__granted(uint8_t arg_0x4104def0){
#line 92
  switch (arg_0x4104def0) {
#line 92
    case /*CC2420ControlC.OscSpiC*/CC2420SpiC__1__CLIENT_ID:
#line 92
      CC2420ControlP__OscResource__granted();
#line 92
      break;
#line 92
    case /*CC2420ControlC.Spi*/CC2420SpiC__2__CLIENT_ID:
#line 92
      CC2420ControlP__SpiResource__granted();
#line 92
      break;
#line 92
    case /*CC2420ControlC.SyncSpiC*/CC2420SpiC__3__CLIENT_ID:
#line 92
      CC2420ControlP__SyncResource__granted();
#line 92
      break;
#line 92
    case /*CC2420ControlC.RxOnSpiC*/CC2420SpiC__4__CLIENT_ID:
#line 92
      CC2420ControlP__RxOnResource__granted();
#line 92
      break;
#line 92
    case /*CC2420ControlC.RfOffSpiC*/CC2420SpiC__5__CLIENT_ID:
#line 92
      CC2420ControlP__RfOffResource__granted();
#line 92
      break;
#line 92
    case /*CC2420ReceiveC.CC2420SpiC*/CC2420SpiC__6__CLIENT_ID:
#line 92
      CC2420ReceiveP__SpiResource__granted();
#line 92
      break;
#line 92
    case /*CC2420TransmitC.CC2420SpiC*/CC2420SpiC__7__CLIENT_ID:
#line 92
      CC2420TransmitP__SpiResource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      CC2420SpiP__Resource__default__granted(arg_0x4104def0);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 358 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__grant__runTask(void )
#line 358
{
  uint8_t holder;

#line 360
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 360
    {
      holder = CC2420SpiP__m_holder;
    }
#line 362
    __nesc_atomic_end(__nesc_atomic); }
  CC2420SpiP__Resource__granted(holder);
}

#line 260
static inline cc2420_status_t CC2420SpiP__Ram__write(uint16_t addr, uint8_t offset, 
uint8_t *data, 
uint8_t len)
#line 262
{

  cc2420_status_t status = 0;
  uint8_t tmpLen = len;
  uint8_t * tmpData = (uint8_t * )data;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 268
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 270
            status;

            {
#line 270
              __nesc_atomic_end(__nesc_atomic); 
#line 270
              return __nesc_temp;
            }
          }
        }
    }
#line 274
    __nesc_atomic_end(__nesc_atomic); }
#line 274
  addr += offset;

  status = CC2420SpiP__SpiByte__write(addr | 0x80);
  CC2420SpiP__SpiByte__write((addr >> 1) & 0xc0);
  for (; len; len--) {
      CC2420SpiP__SpiByte__write(tmpData[tmpLen - len]);
    }

  return status;
}

# 63 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Ram.nc"
inline static cc2420_status_t CC2420ControlP__IEEEADR__write(uint8_t offset, uint8_t * data, uint8_t length){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Ram__write(CC2420_RAM_IEEEADR, offset, data, length);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SpiResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ControlC.Spi*/CC2420SpiC__2__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420DriversP__taskStartDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420DriversP__taskStartDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 83 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/CC2420DriversP.nc"
static inline void CC2420DriversP__CC2420Config__writeIdDone(void )
#line 83
{
  CC2420DriversP__taskStartDone__postTask();
}

# 7 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Config.nc"
inline static void CC2420ControlP__CC2420Config__writeIdDone(void ){
#line 7
  CC2420DriversP__CC2420Config__writeIdDone();
#line 7
}
#line 7
# 78 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SpiResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ControlC.Spi*/CC2420SpiC__2__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78









inline static error_t CC2420ControlP__SpiResource__immediateRequest(void ){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = CC2420SpiP__Resource__immediateRequest(/*CC2420ControlC.Spi*/CC2420SpiC__2__CLIENT_ID);
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 165 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Config__writeId(void )
#line 165
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 166
    {
      if (CC2420ControlP__SpiResource__immediateRequest() != SUCCESS) {
          CC2420ControlP__SpiResource__request();
        }
      else 
#line 169
        {
          CC2420ControlP__writeId();
        }
    }
#line 172
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 6 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Config.nc"
inline static error_t CC2420DriversP__CC2420Config__writeId(void ){
#line 6
  unsigned char __nesc_result;
#line 6

#line 6
  __nesc_result = CC2420ControlP__CC2420Config__writeId();
#line 6

#line 6
  return __nesc_result;
#line 6
}
#line 6
# 79 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/CC2420DriversP.nc"
static inline void CC2420DriversP__taskStartOscillatorDone__runTask(void )
#line 79
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 80
    CC2420DriversP__state = CC2420DriversP__S_WRITING_ID;
#line 80
    __nesc_atomic_end(__nesc_atomic); }
  CC2420DriversP__CC2420Config__writeId();
}

# 146 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static inline void /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Alarm__start(/*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__to_size_type dt)
{
  /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Alarm__startAt(/*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Alarm__getNow(), dt);
}

# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void LatinMacP__FastAlarm__start(LatinMacP__FastAlarm__size_type dt){
#line 55
  /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Alarm__start(dt);
#line 55
}
#line 55
# 627 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/LatinMac/LatinMacP.nc"
static inline void LatinMacP__RadioSend__sendNowDone(error_t error)
#line 627
{
  asn_t temp_asn;
  uint8_t temp_state;
  OpenQueueEntry_t *temp_dataFrameToSend;

#line 631
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 631
    {
      temp_asn = LatinMacP__asn;
      temp_state = LatinMacP__state;
      temp_dataFrameToSend = LatinMacP__dataFrameToSend;
    }
#line 635
    __nesc_atomic_end(__nesc_atomic); }
  switch (temp_state) {
      case S_TX_TXDATA: 

        if (error != SUCCESS) {

            temp_dataFrameToSend->l2_retriesLeft--;
            if (temp_dataFrameToSend->l2_retriesLeft == 0) {
                LatinMacP__postTaskSendDone(temp_dataFrameToSend, FAIL);
              }
            LatinMacP__endSlot();
            return;
          }

      if (LatinMacP__PacketFunctions__isBroadcastMulticast(& temp_dataFrameToSend->l2_nextORpreviousHop) == TRUE) {

          LatinMacP__postTaskSendDone(temp_dataFrameToSend, SUCCESS);
          LatinMacP__endSlot();
        }
      else 
#line 653
        {
          LatinMacP__FastAlarm__start(TsRxAckDelay);
          LatinMacP__change_state(S_TX_RXACKREADY);
        }
      break;
      case S_RX_TXACK: 

        if (error != SUCCESS) {
          }








      LatinMacP__taskReceive__postTask();
      LatinMacP__endSlot();
      break;
      default: 
        LatinMacP__endSlot();
      break;
    }
}

# 20 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/RadioSend.nc"
inline static void CC2420DriversP__RadioSend__sendNowDone(error_t error){
#line 20
  LatinMacP__RadioSend__sendNowDone(error);
#line 20
}
#line 20
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420DriversP__taskStopDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420DriversP__taskStopDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )29U &= ~(0x01 << 5);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__13__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr();
#line 39
}
#line 39
# 38 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__13__GeneralIO__clr(void )
#line 38
{
#line 38
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__13__HplGeneralIO__clr();
}

# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__VREN__clr(void ){
#line 30
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__13__GeneralIO__clr();
#line 30
}
#line 30
# 309 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Config__stopVReg(void )
#line 309
{
  CC2420ControlP__m_state = CC2420ControlP__S_VREG_STOPPED;
  CC2420ControlP__RSTN__clr();
  CC2420ControlP__VREN__clr();
  CC2420ControlP__RSTN__set();
  return SUCCESS;
}

# 13 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Config.nc"
inline static error_t CC2420DriversP__CC2420Config__stopVReg(void ){
#line 13
  unsigned char __nesc_result;
#line 13

#line 13
  __nesc_result = CC2420ControlP__CC2420Config__stopVReg();
#line 13

#line 13
  return __nesc_result;
#line 13
}
#line 13
# 50 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ReceiveP__InterruptFIFOP__disable(void ){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__Interrupt__disable();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 292 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/receive/CC2420ReceiveP.nc"
static inline error_t CC2420ReceiveP__StdAsyncControl__stop(void )
#line 292
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 293
    {
      CC2420ReceiveP__state = CC2420ReceiveP__S_STOPPED;
      CC2420ReceiveP__reset();
      CC2420ReceiveP__CSN__set();
      CC2420ReceiveP__InterruptFIFOP__disable();
    }
#line 298
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/StdAsyncControl.nc"
inline static error_t CC2420DriversP__RxControl__stop(void ){
#line 4
  unsigned char __nesc_result;
#line 4

#line 4
  __nesc_result = CC2420ReceiveP__StdAsyncControl__stop();
#line 4

#line 4
  return __nesc_result;
#line 4
}
#line 4
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )31U &= ~(0x01 << 1);
}

# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc();
#line 85
}
#line 85
# 124 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void )
{
  * (volatile uint16_t * )388U &= ~0x0010;
}

# 47 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents();
#line 47
}
#line 47
# 58 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectIOFunc();
  }
}

# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioCapture.nc"
inline static void CC2420TransmitP__CaptureSFD__disable(void ){
#line 55
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable();
#line 55
}
#line 55
# 329 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__StdAsyncControl__stop(void )
#line 329
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 330
    {
      CC2420TransmitP__state = CC2420TransmitP__S_STOPPED;
      CC2420TransmitP__AlarmWatchdog__stop();
      CC2420TransmitP__CaptureSFD__disable();
      CC2420TransmitP__SpiResource__release();
      CC2420TransmitP__CSN__set();
    }
#line 336
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/StdAsyncControl.nc"
inline static error_t CC2420DriversP__TxControl__stop(void ){
#line 4
  unsigned char __nesc_result;
#line 4

#line 4
  __nesc_result = CC2420TransmitP__StdAsyncControl__stop();
#line 4

#line 4
  return __nesc_result;
#line 4
}
#line 4
# 269 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/CC2420DriversP.nc"
static inline void CC2420DriversP__shutdown(void )
#line 269
{
  CC2420DriversP__TxControl__stop();
  CC2420DriversP__RxControl__stop();
  CC2420DriversP__CC2420Config__stopVReg();
  CC2420DriversP__taskStopDone__postTask();
}

#line 230
static inline void CC2420DriversP__taskSendDone__runTask(void )
#line 230
{
  uint8_t temp_state;
  error_t temp_sendErr;

#line 233
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 233
    {
      temp_state = CC2420DriversP__state;
      temp_sendErr = CC2420DriversP__sendErr;
    }
#line 236
    __nesc_atomic_end(__nesc_atomic); }
  if (temp_state == CC2420DriversP__S_STOPPING) {
      CC2420DriversP__shutdown();
    }
  else 
#line 239
    {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 240
        CC2420DriversP__state = CC2420DriversP__S_STARTED;
#line 240
        __nesc_atomic_end(__nesc_atomic); }
    }
  CC2420DriversP__RadioSend__sendNowDone(temp_sendErr);
}

# 102 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/LatinMac/LatinMacP.nc"
static inline void LatinMacP__RadioControl__stopDone(error_t error)
#line 102
{
  return;
}

# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/RadioControl.nc"
inline static void CC2420DriversP__RadioControl__stopDone(error_t error){
#line 43
  LatinMacP__RadioControl__stopDone(error);
#line 43
}
#line 43
# 275 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/CC2420DriversP.nc"
static inline void CC2420DriversP__taskStopDone__runTask(void )
#line 275
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 276
    CC2420DriversP__state = CC2420DriversP__S_STOPPED;
#line 276
    __nesc_atomic_end(__nesc_atomic); }
  CC2420DriversP__RadioControl__stopDone(SUCCESS);
}

# 99 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/LatinMac/LatinMacP.nc"
static inline void LatinMacP__RadioControl__startDone(error_t error)
#line 99
{
  LatinMacP__SlotAlarm__startAt(LatinMacP__SlotAlarm__getNow(), SLOT_TIME);
}

# 12 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/RadioControl.nc"
inline static void CC2420DriversP__RadioControl__startDone(error_t error){
#line 12
  LatinMacP__RadioControl__startDone(error);
#line 12
}
#line 12
# 87 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/CC2420DriversP.nc"
static inline void CC2420DriversP__taskStartDone__runTask(void )
#line 87
{
  CC2420DriversP__TxControl__start();
  CC2420DriversP__RxControl__start();
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 90
    CC2420DriversP__state = CC2420DriversP__S_STARTED;
#line 90
    __nesc_atomic_end(__nesc_atomic); }
  CC2420DriversP__RadioControl__startDone(SUCCESS);
}

# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/07-App/appDATA/Device/appDATADevP.nc"
static inline void appDATADevP__OpenSendToLower__sendDone(OpenQueueEntry_t *msg, error_t error)
#line 37
{
}

# 5 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/OpenSend.nc"
inline static void LatinResP__OpenSendFromUpper__sendDone(OpenQueueEntry_t *msg, error_t error){
#line 5
  appDATADevP__OpenSendToLower__sendDone(msg, error);
#line 5
}
#line 5
# 6 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/IDManager.nc"
inline static bool LatinResP__IDManager__getIsBridge(void ){
#line 6
  unsigned char __nesc_result;
#line 6

#line 6
  __nesc_result = IDManagerP__IDManager__getIsBridge();
#line 6

#line 6
  return __nesc_result;
#line 6
}
#line 6
# 5 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/Malloc.nc"
inline static error_t LatinResP__Malloc__freePacketBuffer(OpenQueueEntry_t *pkt){
#line 5
  unsigned char __nesc_result;
#line 5

#line 5
  __nesc_result = OpenQueueP__Malloc__freePacketBuffer(pkt);
#line 5

#line 5
  return __nesc_result;
#line 5
}
#line 5
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t LatinResP__taskBuildAndSendDataToChildren__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(LatinResP__taskBuildAndSendDataToChildren);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 2 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/MAC.nc"
inline static void LatinResP__MAC__changeNetworkState(uint8_t tmp_networkState){
#line 2
  LatinMacP__MAC__changeNetworkState(tmp_networkState);
#line 2
}
#line 2
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t LatinResP__taskBuildAndSendAdvMsg__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(LatinResP__taskBuildAndSendAdvMsg);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
inline static error_t LatinResP__taskBuildAndSendSeedMsgToChildren__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(LatinResP__taskBuildAndSendSeedMsgToChildren);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
inline static error_t LatinResP__taskBuildAndSendScanMsg__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(LatinResP__taskBuildAndSendScanMsg);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/IDManager.nc"
inline static bool LatinResP__IDManager__getIsDAGroot(void ){
#line 4
  unsigned char __nesc_result;
#line 4

#line 4
  __nesc_result = IDManagerP__IDManager__getIsDAGroot();
#line 4

#line 4
  return __nesc_result;
#line 4
}
#line 4
# 467 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02b-RES/LatinRes/LatinResP.nc"
static inline void LatinResP__OpenSendToLower__sendDone(OpenQueueEntry_t *msg, error_t error)
#line 467
{
  msg->owner = COMPONENT_RES;

  switch (msg->creator) {
      case COMPONENT_RES: 

        if (msg->l2_commandFrameId == SCAN_MSG) {

            if (LatinResP__neighborList.numNeighbors == LatinResP__neighborList.old_numNeighbors) {
                { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 476
                  LatinResP__neighborList.counterNoNewNeighbors = LatinResP__neighborList.counterNoNewNeighbors + 1;
#line 476
                  __nesc_atomic_end(__nesc_atomic); }
                if (LatinResP__neighborList.counterNoNewNeighbors >= MAXSCANSAMPLE) {
                    if (LatinResP__IDManager__getIsDAGroot() == TRUE) {
                        LatinResP__state = AdvSend;

                        LatinResP__taskBuildAndSendAdvMsg__postTask();
                      }
                    else 
#line 482
                      {
                        LatinResP__state = AdvWait;
                      }
                  }
                else 
#line 485
                  {
                    LatinResP__taskBuildAndSendScanMsg__postTask();
                  }
              }
            else 
#line 488
              {
                { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 489
                  LatinResP__neighborList.old_numNeighbors = LatinResP__neighborList.numNeighbors;
#line 489
                  __nesc_atomic_end(__nesc_atomic); }
                LatinResP__taskBuildAndSendScanMsg__postTask();
              }
          }
        else {
#line 492
          if (msg->l2_commandFrameId == ADV_MSG) {
              if (LatinResP__neighborList.numChildren == LatinResP__neighborList.old_numChildren) {
                  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 494
                    LatinResP__neighborList.counterNoNewChildren = LatinResP__neighborList.counterNoNewChildren + 1;
#line 494
                    __nesc_atomic_end(__nesc_atomic); }
                  if (LatinResP__neighborList.counterNoNewChildren >= MAXADVSAMPLE) {
                      if (LatinResP__neighborList.numChildren > 0) {
                          LatinResP__state = Build;
                          LatinResP__taskBuildAndSendSeedMsgToChildren__postTask();
                        }
                      else 
#line 499
                        {
                          LatinResP__state = BuildDone;
                          LatinResP__MAC__changeNetworkState(SCHEDULING);
                          LatinResP__taskBuildAndSendDataToChildren__postTask();
                        }
                    }
                  else {
                      LatinResP__taskBuildAndSendAdvMsg__postTask();
                    }
                }
              else 
#line 508
                {
                  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 509
                    LatinResP__neighborList.old_numChildren = LatinResP__neighborList.numChildren;
#line 509
                    __nesc_atomic_end(__nesc_atomic); }
                  LatinResP__taskBuildAndSendAdvMsg__postTask();
                }
            }
          else {
#line 512
            if (msg->l2_commandFrameId == SEED_MSG) {
                LatinResP__neighborList.counterSeedMsgSent = LatinResP__neighborList.counterSeedMsgSent + 1;
                if (LatinResP__neighborList.counterSeedMsgSent >= LatinResP__neighborList.numChildren) {
                    LatinResP__state = BuildDone;
                    LatinResP__MAC__changeNetworkState(SCHEDULING);
                    LatinResP__taskBuildAndSendDataToChildren__postTask();
                  }
              }
            else {
#line 519
              if (msg->l2_frameType == IEEE154_TYPE_DATA) {
                }
              }
            }
          }
#line 522
      LatinResP__Malloc__freePacketBuffer(msg);
      break;

      default: 

        if (LatinResP__IDManager__getIsBridge() == TRUE) {
          }
        else {
            LatinResP__OpenSendFromUpper__sendDone(msg, error);
          }
      break;
    }
}

# 5 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/OpenSend.nc"
inline static void LatinMacP__OpenSendFromUpper__sendDone(OpenQueueEntry_t *msg, error_t error){
#line 5
  LatinResP__OpenSendToLower__sendDone(msg, error);
#line 5
}
#line 5
# 691 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/LatinMac/LatinMacP.nc"
static inline void LatinMacP__taskSendDone__runTask(void )
#line 691
{
  OpenQueueEntry_t *temp_sendDoneMessage;
  error_t temp_sendDoneError;

#line 694
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 694
    {
      temp_sendDoneMessage = LatinMacP__sendDoneMessage;
      temp_sendDoneError = LatinMacP__sendDoneError;
    }
#line 697
    __nesc_atomic_end(__nesc_atomic); }

  LatinMacP__OpenSendFromUpper__sendDone(temp_sendDoneMessage, temp_sendDoneError);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 700
    LatinMacP__sendDoneMessage = (void *)0;
#line 700
    __nesc_atomic_end(__nesc_atomic); }
}

# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )49U &= ~(0x01 << 4);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__clr();
#line 39
}
#line 39
# 38 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr(void )
#line 38
{
#line 38
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__clr();
}

# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__clr(void ){
#line 30
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr();
#line 30
}
#line 30
# 63 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/LedsP.nc"
static inline void LedsP__Leds__led0On(void )
#line 63
{
  LedsP__Led0__clr();
  ;
#line 65
  ;
}

# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Leds.nc"
inline static void LatinMacP__Leds__led0On(void ){
#line 45
  LedsP__Leds__led0On();
#line 45
}
#line 45
# 251 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_hton_leuint8(void * target, uint8_t value)
#line 251
{
  uint8_t *base = target;

#line 253
  base[0] = value;
  return value;
}

# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/OpenSend.nc"
inline static error_t LatinResP__OpenSendToLower__send(OpenQueueEntry_t *msg){
#line 4
  unsigned char __nesc_result;
#line 4

#line 4
  __nesc_result = LatinMacP__OpenSendFromUpper__send(msg);
#line 4

#line 4
  return __nesc_result;
#line 4
}
#line 4
# 16 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/PacketFunctions.nc"
inline static void LatinResP__PacketFunctions__reserveHeaderSize(OpenQueueEntry_t *pkt, uint8_t header_length){
#line 16
  PacketFunctionsP__PacketFunctions__reserveHeaderSize(pkt, header_length);
#line 16
}
#line 16
# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/Malloc.nc"
inline static OpenQueueEntry_t *LatinResP__Malloc__getFreePacketBuffer(void ){
#line 4
  struct OpenQueueEntry_t *__nesc_result;
#line 4

#line 4
  __nesc_result = OpenQueueP__Malloc__getFreePacketBuffer();
#line 4

#line 4
  return __nesc_result;
#line 4
}
#line 4
# 158 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02b-RES/LatinRes/LatinResP.nc"
static inline void LatinResP__taskBuildAndSendConfMsg(OpenQueueEntry_t *msg)
#line 158
{
  OpenQueueEntry_t *pkt;

  pkt = LatinResP__Malloc__getFreePacketBuffer();
  if (pkt == (void *)0) {
      return;
    }

  pkt->creator = COMPONENT_RES;
  pkt->owner = COMPONENT_RES;


  LatinResP__PacketFunctions__reserveHeaderSize(pkt, sizeof(CMD_MSG_t ));
  __nesc_hton_leuint8(((CMD_MSG_t *)pkt->payload)->commandFrameId.data, CONF_MSG);
  __nesc_hton_leuint8(((CMD_MSG_t *)pkt->payload)->seedNumber.data, TOS_NODE_ID);


  memcpy(& pkt->l2_nextORpreviousHop, & msg->l2_nextORpreviousHop, sizeof(open_addr_t ));
  pkt->l2_seedNumber = __nesc_ntoh_leuint8(((CMD_MSG_t *)msg->payload)->seedNumber.data);
  pkt->l2_frameType = IEEE154_TYPE_CMD;
  pkt->l2_commandFrameId = CONF_MSG;


  if (LatinResP__OpenSendToLower__send(pkt) == FAIL) {
      LatinResP__Malloc__freePacketBuffer(pkt);
    }
}

# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
inline static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void ){
#line 53
  unsigned long __nesc_result;
#line 53

#line 53
  __nesc_result = /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void )
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
}

# 98 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void ){
#line 98
  unsigned long __nesc_result;
#line 98

#line 98
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void )
{
#line 86
  return /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow();
}

# 125 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Timer.nc"
inline static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void ){
#line 125
  unsigned long __nesc_result;
#line 125

#line 125
  __nesc_result = /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow();
#line 125

#line 125
  return __nesc_result;
#line 125
}
#line 125
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 133 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

#line 136
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}






static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, TRUE);
}

# 62 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Timer.nc"
inline static void LatinResP__timerAdvWait__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(0U, dt);
#line 62
}
#line 62
# 158 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/VirtualizeTimerC.nc"
static inline bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(uint8_t num)
{
  return /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num].isrunning;
}

# 81 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Timer.nc"
inline static bool LatinResP__timerAdvWait__isRunning(void ){
#line 81
  unsigned char __nesc_result;
#line 81

#line 81
  __nesc_result = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(0U);
#line 81

#line 81
  return __nesc_result;
#line 81
}
#line 81
# 538 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02b-RES/LatinRes/LatinResP.nc"
static inline void LatinResP__OpenReceiveFromLower__receive(OpenQueueEntry_t *msg)
#line 538
{
  msg->owner = COMPONENT_RES;

  switch (msg->l2_frameType) {
      case IEEE154_TYPE_CMD: 
        if (__nesc_ntoh_leuint8(((CMD_MSG_t *)msg->payload)->commandFrameId.data) == SCAN_MSG) {
            { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 544
              {
                if (LatinResP__state == ScanWait || LatinResP__state == ScanSend) {
                    LatinResP__updateNeighborList(msg);
                    if (LatinResP__state == ScanWait) {
                        LatinResP__state = ScanSend;
                        LatinResP__taskBuildAndSendScanMsg__postTask();
                      }
                  }
              }
#line 552
              __nesc_atomic_end(__nesc_atomic); }
          }
        else {
#line 553
          if (__nesc_ntoh_leuint8(((CMD_MSG_t *)msg->payload)->commandFrameId.data) == ADV_MSG) {
              { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 554
                {
                  if (LatinResP__state == AdvWait) {
                      if (LatinResP__timerAdvWait__isRunning() == FALSE && LatinResP__IDManager__getIsDAGroot() == FALSE) {
                          LatinResP__timerAdvWait__startOneShot(5000);
                        }
                      LatinResP__updateNeighborList(msg);
                    }
                }
#line 561
                __nesc_atomic_end(__nesc_atomic); }
            }
          else {
#line 562
            if (__nesc_ntoh_leuint8(((CMD_MSG_t *)msg->payload)->commandFrameId.data) == REQ_MSG) {
                { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 563
                  {
                    if (LatinResP__state == AdvSend) {
                        LatinResP__updateNeighborList(msg);
                        LatinResP__taskBuildAndSendConfMsg(msg);
                      }
                  }
#line 568
                  __nesc_atomic_end(__nesc_atomic); }
              }
            else {
#line 569
              if (__nesc_ntoh_leuint8(((CMD_MSG_t *)msg->payload)->commandFrameId.data) == CONF_MSG) {
                  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 570
                    {
                      if (LatinResP__state == ReqSend) {
                          LatinResP__state = AdvSend;
                          LatinResP__taskBuildAndSendAdvMsg__postTask();
                        }
                    }
#line 575
                    __nesc_atomic_end(__nesc_atomic); }
                }
              else {
#line 576
                if (__nesc_ntoh_leuint8(((CMD_MSG_t *)msg->payload)->commandFrameId.data) == SEED_MSG) {
                    { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 577
                      {
                        if (LatinResP__state == AdvSend || LatinResP__state == Build) {
                            LatinResP__updateNeighborList(msg);
                          }
                      }
#line 581
                      __nesc_atomic_end(__nesc_atomic); }
                  }
                }
              }
            }
          }
#line 583
      LatinResP__Malloc__freePacketBuffer(msg);
      break;

      case IEEE154_TYPE_DATA: 

        if (LatinResP__IDManager__getIsBridge() == TRUE) {
          }
        else {

            LatinResP__Malloc__freePacketBuffer(msg);
          }


      break;

      default: 
        LatinResP__Malloc__freePacketBuffer(msg);

      break;
    }
}

# 4 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/OpenReceive.nc"
inline static void LatinMacP__OpenReceiveToUpper__receive(OpenQueueEntry_t *msg){
#line 4
  LatinResP__OpenReceiveFromLower__receive(msg);
#line 4
}
#line 4
# 864 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/LatinMac/LatinMacP.nc"
static inline void LatinMacP__taskReceive__runTask(void )
#line 864
{
  asn_t temp_asn;
  OpenQueueEntry_t *temp_frameReceived;

#line 867
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 867
    {
      temp_asn = LatinMacP__asn;
      temp_frameReceived = LatinMacP__frameReceived;
    }
#line 870
    __nesc_atomic_end(__nesc_atomic); }
  LatinMacP__OpenReceiveToUpper__receive(temp_frameReceived);
}

# 11 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/PacketFunctions.nc"
inline static bool LatinResP__PacketFunctions__sameAddress(open_addr_t *address_1, open_addr_t *address_2){
#line 11
  unsigned char __nesc_result;
#line 11

#line 11
  __nesc_result = PacketFunctionsP__PacketFunctions__sameAddress(address_1, address_2);
#line 11

#line 11
  return __nesc_result;
#line 11
}
#line 11
# 316 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02b-RES/LatinRes/LatinResP.nc"
static inline bool LatinResP__isThisRowMatching(open_addr_t *address, uint8_t rowNumber)
#line 316
{
  switch (address->type) {
      case ADDR_16B: /* atomic removed: atomic calls only */
        {
          unsigned char __nesc_temp = 
#line 319
          LatinResP__neighborList.neighbors[rowNumber].used && 
          LatinResP__PacketFunctions__sameAddress(address, & LatinResP__neighborList.neighbors[rowNumber].addr_16b);

#line 319
          return __nesc_temp;
        }
      case ADDR_64B: /* atomic removed: atomic calls only */
        {
          unsigned char __nesc_temp = 
#line 322
          LatinResP__neighborList.neighbors[rowNumber].used && 
          LatinResP__PacketFunctions__sameAddress(address, & LatinResP__neighborList.neighbors[rowNumber].addr_64b);

#line 322
          return __nesc_temp;
        }
      case ADDR_128B: /* atomic removed: atomic calls only */
        {
          unsigned char __nesc_temp = 
#line 325
          LatinResP__neighborList.neighbors[rowNumber].used && 
          LatinResP__PacketFunctions__sameAddress(address, & LatinResP__neighborList.neighbors[rowNumber].addr_128b);

#line 325
          return __nesc_temp;
        }
      break;
      default: 
        return FALSE;
      break;
    }
}

# 7 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/PacketFunctions.nc"
inline static void LatinResP__PacketFunctions__mac64bToMac16b(open_addr_t *mac64b, open_addr_t *mac16btoWrite){
#line 7
  PacketFunctionsP__PacketFunctions__mac64bToMac16b(mac64b, mac16btoWrite);
#line 7
}
#line 7
# 20 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/packetfunctions/PacketFunctionsP.nc"
static inline void PacketFunctionsP__PacketFunctions__ip128bToMac64b(
open_addr_t *ip128b, 
open_addr_t *prefix64btoWrite, 
open_addr_t *mac64btoWrite)
#line 23
{
  if (ip128b->type != ADDR_128B) {



      mac64btoWrite->type = ADDR_NONE;
      return;
    }
  prefix64btoWrite->type = ADDR_PREFIX;
  memcpy(prefix64btoWrite->prefix, &ip128b->addr_128b[0], 8);
  mac64btoWrite->type = ADDR_64B;
  memcpy(mac64btoWrite->addr_64b, &ip128b->addr_128b[8], 8);
}

# 5 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/PacketFunctions.nc"
inline static void LatinResP__PacketFunctions__ip128bToMac64b(open_addr_t *ip128b, open_addr_t *prefix64btoWrite, open_addr_t *mac64btoWrite){
#line 5
  PacketFunctionsP__PacketFunctions__ip128bToMac64b(ip128b, prefix64btoWrite, mac64btoWrite);
#line 5
}
#line 5
# 8 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/IDManager.nc"
inline static open_addr_t *LatinResP__IDManager__getMyID(uint8_t type){
#line 8
  struct open_addr_t *__nesc_result;
#line 8

#line 8
  __nesc_result = IDManagerP__IDManager__getMyID(type);
#line 8

#line 8
  return __nesc_result;
#line 8
}
#line 8
# 6 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/PacketFunctions.nc"
inline static void LatinResP__PacketFunctions__mac64bToIp128b(open_addr_t *prefix64b, open_addr_t *mac64b, open_addr_t *ip128bToWrite){
#line 6
  PacketFunctionsP__PacketFunctions__mac64bToIp128b(prefix64b, mac64b, ip128bToWrite);
#line 6
}
#line 6
# 65 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/packetfunctions/PacketFunctionsP.nc"
static inline void PacketFunctionsP__PacketFunctions__mac16bToMac64b(open_addr_t *mac16b, open_addr_t *mac64btoWrite)
#line 65
{
  if (mac16b->type != ADDR_16B) {



      mac64btoWrite->type = ADDR_NONE;
      return;
    }
  mac64btoWrite->type = ADDR_64B;
  mac64btoWrite->addr_64b[0] = 0;
  mac64btoWrite->addr_64b[1] = 0;
  mac64btoWrite->addr_64b[2] = 0;
  mac64btoWrite->addr_64b[3] = 0;
  mac64btoWrite->addr_64b[4] = 0;
  mac64btoWrite->addr_64b[5] = 0;
  mac64btoWrite->addr_64b[6] = mac16b->addr_16b[0];
  mac64btoWrite->addr_64b[7] = mac16b->addr_16b[1];
}

# 8 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/PacketFunctions.nc"
inline static void LatinResP__PacketFunctions__mac16bToMac64b(open_addr_t *mac16b, open_addr_t *mac64btoWrite){
#line 8
  PacketFunctionsP__PacketFunctions__mac16bToMac64b(mac16b, mac64btoWrite);
#line 8
}
#line 8
# 334 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02b-RES/LatinRes/LatinResP.nc"
static inline void LatinResP__registerNewNeighbor(OpenQueueEntry_t *msg)
#line 334
{
  open_addr_t temp_prefix;
  open_addr_t temp_addr16b;
  open_addr_t temp_addr64b;
  open_addr_t temp_addr128b;

  uint8_t i = 0;

#line 341
  while (i < MAXNUMNEIGHBORS) {
      /* atomic removed: atomic calls only */
#line 342
      {
        if (LatinResP__neighborList.neighbors[i].used == FALSE) {
            LatinResP__neighborList.neighbors[i].used = TRUE;
            LatinResP__neighborList.numNeighbors = LatinResP__neighborList.numNeighbors + 1;
            LatinResP__neighborList.neighbors[i].seedNumber = __nesc_ntoh_leuint8(((CMD_MSG_t *)msg->payload)->seedNumber.data);
            LatinResP__neighborList.neighbors[i].addr_16b.type = ADDR_NONE;
            LatinResP__neighborList.neighbors[i].addr_64b.type = ADDR_NONE;
            LatinResP__neighborList.neighbors[i].addr_128b.type = ADDR_NONE;
            switch (msg->l2_nextORpreviousHop.type) {
                case ADDR_16B: 
                  LatinResP__PacketFunctions__mac16bToMac64b(& msg->l2_nextORpreviousHop, &temp_addr64b);
                LatinResP__PacketFunctions__mac64bToIp128b(
                LatinResP__IDManager__getMyID(ADDR_PREFIX), 
                &temp_addr64b, 
                &temp_addr128b);
                memcpy(& LatinResP__neighborList.neighbors[i].addr_16b, & msg->l2_nextORpreviousHop, sizeof(open_addr_t ));
                memcpy(& LatinResP__neighborList.neighbors[i].addr_64b, &temp_addr64b, sizeof(open_addr_t ));
                memcpy(& LatinResP__neighborList.neighbors[i].addr_128b, &temp_addr128b, sizeof(open_addr_t ));
                break;
                case ADDR_64B: 
                  LatinResP__PacketFunctions__mac64bToMac16b(& msg->l2_nextORpreviousHop, &temp_addr16b);
                LatinResP__PacketFunctions__mac64bToIp128b(
                LatinResP__IDManager__getMyID(ADDR_PREFIX), 
                & msg->l2_nextORpreviousHop, 
                &temp_addr128b);
                memcpy(& LatinResP__neighborList.neighbors[i].addr_16b, &temp_addr16b, sizeof(open_addr_t ));
                memcpy(& LatinResP__neighborList.neighbors[i].addr_64b, & msg->l2_nextORpreviousHop, sizeof(open_addr_t ));
                memcpy(& LatinResP__neighborList.neighbors[i].addr_128b, &temp_addr128b, sizeof(open_addr_t ));
                break;
                case ADDR_128B: 
                  LatinResP__PacketFunctions__ip128bToMac64b(
                  & msg->l2_nextORpreviousHop, 
                  &temp_prefix, 
                  &temp_addr64b);
                LatinResP__PacketFunctions__mac64bToMac16b(&temp_addr64b, &temp_addr16b);
                memcpy(& LatinResP__neighborList.neighbors[i].addr_16b, &temp_addr16b, sizeof(open_addr_t ));
                memcpy(& LatinResP__neighborList.neighbors[i].addr_64b, &temp_addr64b, sizeof(open_addr_t ));
                memcpy(& LatinResP__neighborList.neighbors[i].addr_128b, & msg->l2_nextORpreviousHop, sizeof(open_addr_t ));
                break;
              }
            LatinResP__neighborList.neighbors[i].rssi = msg->l1_rssi;

            if (__nesc_ntoh_leuint8(((CMD_MSG_t *)msg->payload)->commandFrameId.data) == ADV_MSG) {
                LatinResP__neighborList.neighbors[i].numHops = __nesc_ntoh_leuint8(((CMD_MSG_t *)msg->payload)->numHops.data);
                if (LatinResP__neighborList.neighbors[i].numHops == LatinResP__neighborList.minHops) {
                    if (LatinResP__neighborList.neighbors[i].rssi > LatinResP__neighborList.maxRssi) {
                        LatinResP__neighborList.maxRssi = LatinResP__neighborList.neighbors[i].rssi;
                        LatinResP__neighborList.parent = i;
                      }
                  }
                else {
#line 391
                  if (LatinResP__neighborList.neighbors[i].numHops < LatinResP__neighborList.minHops) {
                      LatinResP__neighborList.maxRssi = LatinResP__neighborList.neighbors[i].rssi;
                      LatinResP__neighborList.minHops = LatinResP__neighborList.neighbors[i].numHops;
                      LatinResP__neighborList.parent = i;
                    }
                  }
              }
            else {
#line 396
              if (__nesc_ntoh_leuint8(((CMD_MSG_t *)msg->payload)->commandFrameId.data) == REQ_MSG) {
                  LatinResP__neighborList.neighbors[i].isChild = TRUE;
                  LatinResP__neighborList.seedList[LatinResP__neighborList.numChildren] = __nesc_ntoh_leuint8(((CMD_MSG_t *)msg->payload)->seedNumber.data);
                  LatinResP__neighborList.childrenIndex[LatinResP__neighborList.numChildren] = i;
                  LatinResP__neighborList.numChildren = LatinResP__neighborList.numChildren + 1;
                }
              }
#line 402
            return;
          }
      }
      i++;
    }
}

# 16 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/PacketFunctions.nc"
inline static void LatinMacP__PacketFunctions__reserveHeaderSize(OpenQueueEntry_t *pkt, uint8_t header_length){
#line 16
  PacketFunctionsP__PacketFunctions__reserveHeaderSize(pkt, header_length);
#line 16
}
#line 16
# 236 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/packetfunctions/PacketFunctionsP.nc"
static inline void PacketFunctionsP__PacketFunctions__reserveFooterSize(OpenQueueEntry_t *pkt, uint8_t header_length)
#line 236
{
  pkt->length += header_length;
  if (pkt->length > 127) {
    }
}

# 18 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/PacketFunctions.nc"
inline static void LatinMacP__PacketFunctions__reserveFooterSize(OpenQueueEntry_t *pkt, uint8_t header_length){
#line 18
  PacketFunctionsP__PacketFunctions__reserveFooterSize(pkt, header_length);
#line 18
}
#line 18
# 8 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/IDManager.nc"
inline static open_addr_t *LatinMacP__IDManager__getMyID(uint8_t type){
#line 8
  struct open_addr_t *__nesc_result;
#line 8

#line 8
  __nesc_result = IDManagerP__IDManager__getMyID(type);
#line 8

#line 8
  return __nesc_result;
#line 8
}
#line 8
# 14 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/PacketFunctions.nc"
inline static void LatinMacP__PacketFunctions__writeAddress(OpenQueueEntry_t *msg, open_addr_t *address, bool littleEndian){
#line 14
  PacketFunctionsP__PacketFunctions__writeAddress(msg, address, littleEndian);
#line 14
}
#line 14
# 1 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/IEEE802154_common.c"
static inline void LatinMacP__prependIEEE802154header(OpenQueueEntry_t *msg, 
uint8_t frameType, 
bool securityEnabled, 
uint8_t sequenceNumber, 
open_addr_t *nextHop)
#line 5
{
  uint8_t temp_8b;

  if (frameType != IEEE154_TYPE_ACK) {
      switch (nextHop->type) {
          case ADDR_16B: 
            LatinMacP__PacketFunctions__writeAddress(msg, LatinMacP__IDManager__getMyID(ADDR_16B), LITTLE_ENDIAN);
          break;
          case ADDR_64B: 
            LatinMacP__PacketFunctions__writeAddress(msg, LatinMacP__IDManager__getMyID(ADDR_64B), LITTLE_ENDIAN);
          break;
          default: ;
        }
    }




  if (frameType != IEEE154_TYPE_ACK) {
      if (LatinMacP__PacketFunctions__isBroadcastMulticast(nextHop)) {
          LatinMacP__PacketFunctions__reserveHeaderSize(msg, sizeof(uint8_t ));
          * (uint8_t *)msg->payload = 0xFF;
          LatinMacP__PacketFunctions__reserveHeaderSize(msg, sizeof(uint8_t ));
          * (uint8_t *)msg->payload = 0xFF;
        }
      else 
#line 29
        {
          switch (nextHop->type) {
              case ADDR_16B: 
                case ADDR_64B: 
                  LatinMacP__PacketFunctions__writeAddress(msg, nextHop, LITTLE_ENDIAN);
              break;
              default: ;
            }
        }
    }




  if (frameType != IEEE154_TYPE_ACK) {
      LatinMacP__PacketFunctions__writeAddress(msg, LatinMacP__IDManager__getMyID(ADDR_PANID), LITTLE_ENDIAN);
    }

  LatinMacP__PacketFunctions__reserveHeaderSize(msg, sizeof(uint8_t ));
  * (uint8_t *)msg->payload = sequenceNumber;

  temp_8b = 0;
  if (frameType == IEEE154_TYPE_ACK) {
      temp_8b |= IEEE154_ADDR_NONE << IEEE154_FCF_DEST_ADDR_MODE;
    }
  else {
#line 53
    if (LatinMacP__PacketFunctions__isBroadcastMulticast(nextHop)) {
        temp_8b |= IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE;
      }
    else 
#line 55
      {
        switch (nextHop->type) {
            case ADDR_16B: 
              temp_8b |= IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE;
            break;
            case ADDR_64B: 
              temp_8b |= IEEE154_ADDR_EXT << IEEE154_FCF_DEST_ADDR_MODE;
            break;
          }
      }
    }
#line 65
  if (frameType == IEEE154_TYPE_ACK) {
      temp_8b |= IEEE154_ADDR_NONE << IEEE154_FCF_SRC_ADDR_MODE;
    }
  else 
#line 67
    {
      switch (nextHop->type) {
          case ADDR_16B: 
            temp_8b |= IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE;
          break;
          case ADDR_64B: 
            temp_8b |= IEEE154_ADDR_EXT << IEEE154_FCF_SRC_ADDR_MODE;
          break;
        }
    }
  LatinMacP__PacketFunctions__reserveHeaderSize(msg, sizeof(uint8_t ));
  * (uint8_t *)msg->payload = temp_8b;
  temp_8b = 0;
  temp_8b |= frameType << IEEE154_FCF_FRAME_TYPE;
  temp_8b |= securityEnabled << IEEE154_FCF_SECURITY_ENABLED;
  temp_8b |= IEEE154_PENDING_NO_FRAMEPENDING << IEEE154_FCF_FRAME_PENDING;
  if (frameType == IEEE154_TYPE_ACK || LatinMacP__PacketFunctions__isBroadcastMulticast(nextHop)) {
      temp_8b |= IEEE154_ACK_NO_ACK_REQ << IEEE154_FCF_ACK_REQ;
    }
  else 
#line 85
    {
      temp_8b |= IEEE154_ACK_YES_ACK_REQ << IEEE154_FCF_ACK_REQ;
    }
  temp_8b |= IEEE154_PANID_COMPRESSED << IEEE154_FCF_INTRAPAN;
  LatinMacP__PacketFunctions__reserveHeaderSize(msg, sizeof(uint8_t ));
  * (uint8_t *)msg->payload = temp_8b;

  LatinMacP__PacketFunctions__reserveFooterSize(msg, 2);

  LatinMacP__PacketFunctions__reserveHeaderSize(msg, sizeof(uint8_t ));
  * (uint8_t *)msg->payload = msg->length - 1;
}

# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 47 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt = dt;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot = oneshot;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(t0, dt);
}

#line 82
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt)
{
#line 83
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(t0, dt, TRUE);
}

# 118 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt){
#line 118
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(t0, dt);
#line 118
}
#line 118
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
}

# 62 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void ){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop();
#line 62
}
#line 62
# 91 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop();
}

# 62 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void ){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop();
#line 62
}
#line 62
# 60 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void )
{
#line 61
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop();
}

# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop();
#line 67
}
#line 67
# 89 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void )
{




  uint32_t now = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint8_t num;

  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop();

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;
          int32_t remaining = timer->dt - elapsed;

          if (remaining < min_remaining) 
            {
              min_remaining = remaining;
              min_remaining_isset = TRUE;
            }
        }
    }

  if (min_remaining_isset) 
    {
      if (min_remaining <= 0) {
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(now);
        }
      else {
#line 124
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(now, min_remaining);
        }
    }
}

# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t LatinResP__taskBuildAndSendReqMsg__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(LatinResP__taskBuildAndSendReqMsg);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 664 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02b-RES/LatinRes/LatinResP.nc"
static inline void LatinResP__timerAdvWait__fired(void )
#line 664
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 665
    LatinResP__state = ReqSend;
#line 665
    __nesc_atomic_end(__nesc_atomic); }
  LatinResP__taskBuildAndSendReqMsg__postTask();
}

# 193 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num)
{
}

# 72 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(uint8_t arg_0x40e3f9f0){
#line 72
  switch (arg_0x40e3f9f0) {
#line 72
    case 0U:
#line 72
      LatinResP__timerAdvWait__fired();
#line 72
      break;
#line 72
    default:
#line 72
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(arg_0x40e3f9f0);
#line 72
      break;
#line 72
    }
#line 72
}
#line 72
# 128 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void )
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow());
}

# 72 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void ){
#line 72
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired();
#line 72
}
#line 72
# 80 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 82
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type __nesc_temp = 
#line 82
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

      {
#line 82
        __nesc_atomic_end(__nesc_atomic); 
#line 82
        return __nesc_temp;
      }
    }
#line 84
    __nesc_atomic_end(__nesc_atomic); }
}

# 105 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void ){
#line 105
  unsigned long __nesc_result;
#line 105

#line 105
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm();
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 63 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt, FALSE);
    }
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired();
}

# 50 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void )
#line 50
{
  return /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__enable(TRUE);
}

# 42 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioInterrupt.nc"
inline static error_t /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__GpioInterrupt__enableRisingEdge(void ){
#line 42
  unsigned char __nesc_result;
#line 42

#line 42
  __nesc_result = /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__Interrupt__enableRisingEdge();
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__Interrupt__enableFallingEdge(void )
#line 54
{
  return /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__enable(FALSE);
}

# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioInterrupt.nc"
inline static error_t /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__GpioInterrupt__enableFallingEdge(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__Interrupt__enableFallingEdge();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Notify.nc"
inline static void UserButtonP__Notify__notify(UserButtonP__Notify__val_t val){
#line 74
  LatinResP__UserButton__notify(val);
#line 74
}
#line 74
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosb/UserButtonP.nc"
static inline void UserButtonP__NotifyLower__notify(bool val)
#line 67
{

  if (val) {
      UserButtonP__Notify__notify(BUTTON_RELEASED);
    }
  else 
#line 71
    {
      UserButtonP__Notify__notify(BUTTON_PRESSED);
    }
}

# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Notify.nc"
inline static void /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__Notify__notify(/*UserButtonC.SwitchToggleC*/SwitchToggleC__0__Notify__val_t val){
#line 74
  UserButtonP__NotifyLower__notify(val);
#line 74
}
#line 74
# 79 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosb/SwitchToggleC.nc"
static inline void /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__sendEvent__runTask(void )
#line 79
{
  bool pinHigh;

#line 81
  pinHigh = /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__m_pinHigh;

  /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__Notify__notify(pinHigh);

  if (pinHigh) {
      /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__GpioInterrupt__enableFallingEdge();
    }
  else 
#line 87
    {
      /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__GpioInterrupt__enableRisingEdge();
    }
}

# 253 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port27__edge(bool l2h)
#line 253
{
  /* atomic removed: atomic calls only */
#line 254
  {
    if (l2h) {
#line 255
      P2IES &= ~(1 << 7);
      }
    else {
#line 256
      P2IES |= 1 << 7;
      }
  }
}

# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__HplInterrupt__edge(bool low_to_high){
#line 56
  HplMsp430InterruptP__Port27__edge(low_to_high);
#line 56
}
#line 56
# 186 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port27__enable(void )
#line 186
{
#line 186
  P2IE |= 1 << 7;
}

# 31 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__HplInterrupt__enable(void ){
#line 31
  HplMsp430InterruptP__Port27__enable();
#line 31
}
#line 31
# 222 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02b-RES/LatinRes/LatinResP.nc"
static inline void LatinResP__taskBuildAndSendDataToChildren__runTask(void )
#line 222
{
  uint8_t i;
  OpenQueueEntry_t *pkt;

  for (i = 0; i < LatinResP__neighborList.numChildren; i++) {

      pkt = LatinResP__Malloc__getFreePacketBuffer();
      if (pkt == (void *)0) {
          return;
        }

      pkt->creator = COMPONENT_RES;
      pkt->owner = COMPONENT_RES;


      LatinResP__PacketFunctions__reserveHeaderSize(pkt, sizeof(CMD_MSG_t ));


      memcpy(& pkt->l2_nextORpreviousHop, & LatinResP__neighborList.neighbors[LatinResP__neighborList.childrenIndex[i]].addr_16b, sizeof(open_addr_t ));
      pkt->l2_seedNumber = LatinResP__neighborList.neighbors[LatinResP__neighborList.childrenIndex[i]].seedNumber;
      pkt->l2_frameType = IEEE154_TYPE_DATA;


      if (LatinResP__OpenSendToLower__send(pkt) == FAIL) {
          LatinResP__Malloc__freePacketBuffer(pkt);
        }
    }

  if (LatinResP__neighborList.parent != 254) {
      pkt = LatinResP__Malloc__getFreePacketBuffer();
      if (pkt == (void *)0) {
          return;
        }

      pkt->creator = COMPONENT_RES;
      pkt->owner = COMPONENT_RES;


      LatinResP__PacketFunctions__reserveHeaderSize(pkt, sizeof(CMD_MSG_t ));


      memcpy(& pkt->l2_nextORpreviousHop, & LatinResP__neighborList.neighbors[LatinResP__neighborList.parent].addr_16b, sizeof(open_addr_t ));
      pkt->l2_seedNumber = LatinResP__neighborList.neighbors[LatinResP__neighborList.parent].seedNumber;
      pkt->l2_frameType = IEEE154_TYPE_DATA;


      if (LatinResP__OpenSendToLower__send(pkt) == FAIL) {
          LatinResP__Malloc__freePacketBuffer(pkt);
        }
    }
}

#line 186
static inline void LatinResP__taskBuildAndSendSeedMsgToChildren__runTask(void )
#line 186
{
  uint8_t i;
  OpenQueueEntry_t *pkt;

  for (i = 0; i < LatinResP__neighborList.numChildren; i++) {

      pkt = LatinResP__Malloc__getFreePacketBuffer();
      if (pkt == (void *)0) {
          return;
        }

      pkt->creator = COMPONENT_RES;
      pkt->owner = COMPONENT_RES;


      LatinResP__PacketFunctions__reserveHeaderSize(pkt, sizeof(CMD_MSG_t ));
      __nesc_hton_leuint8(((CMD_MSG_t *)pkt->payload)->commandFrameId.data, SEED_MSG);
      __nesc_hton_leuint8(((CMD_MSG_t *)pkt->payload)->seedNumber.data, TOS_NODE_ID);
      __nesc_hton_leuint8(((CMD_MSG_t *)pkt->payload)->numChildren.data, LatinResP__neighborList.numChildren);
      memcpy(((CMD_MSG_t *)pkt->payload)->seedList, LatinResP__neighborList.seedList, sizeof(uint8_t ) * LatinResP__neighborList.numChildren);


      memcpy(& pkt->l2_nextORpreviousHop, & LatinResP__neighborList.neighbors[LatinResP__neighborList.childrenIndex[i]].addr_16b, sizeof(open_addr_t ));
      pkt->l2_seedNumber = LatinResP__neighborList.neighbors[LatinResP__neighborList.childrenIndex[i]].seedNumber;

      pkt->l2_frameType = IEEE154_TYPE_CMD;
      pkt->l2_commandFrameId = SEED_MSG;


      if (LatinResP__OpenSendToLower__send(pkt) == FAIL) {
          LatinResP__Malloc__freePacketBuffer(pkt);
        }
    }
}

#line 129
static inline void LatinResP__taskBuildAndSendReqMsg__runTask(void )
#line 129
{
  OpenQueueEntry_t *pkt;

  pkt = LatinResP__Malloc__getFreePacketBuffer();
  if (pkt == (void *)0) {
      return;
    }

  pkt->creator = COMPONENT_RES;
  pkt->owner = COMPONENT_RES;


  LatinResP__PacketFunctions__reserveHeaderSize(pkt, sizeof(CMD_MSG_t ));
  __nesc_hton_leuint8(((CMD_MSG_t *)pkt->payload)->commandFrameId.data, REQ_MSG);
  __nesc_hton_leuint8(((CMD_MSG_t *)pkt->payload)->seedNumber.data, TOS_NODE_ID);


  memcpy(& pkt->l2_nextORpreviousHop, & LatinResP__neighborList.neighbors[LatinResP__neighborList.parent].addr_16b, sizeof(open_addr_t ));
  pkt->l2_seedNumber = LatinResP__neighborList.neighbors[LatinResP__neighborList.parent].seedNumber;
  pkt->l2_seedNumber = 255;
  pkt->l2_frameType = IEEE154_TYPE_CMD;
  pkt->l2_commandFrameId = REQ_MSG;


  if (LatinResP__OpenSendToLower__send(pkt) == FAIL) {
      LatinResP__Malloc__freePacketBuffer(pkt);
    }
}

#line 95
static inline void LatinResP__taskBuildAndSendAdvMsg__runTask(void )
#line 95
{
  OpenQueueEntry_t *pkt;
  open_addr_t temp_nextHop;


  temp_nextHop.type = ADDR_16B;
  temp_nextHop.addr_16b[0] = 0xFF;
  temp_nextHop.addr_16b[1] = 0xFF;
  pkt = LatinResP__Malloc__getFreePacketBuffer();
  if (pkt == (void *)0) {
      return;
    }

  pkt->creator = COMPONENT_RES;
  pkt->owner = COMPONENT_RES;


  LatinResP__PacketFunctions__reserveHeaderSize(pkt, sizeof(CMD_MSG_t ));
  __nesc_hton_leuint8(((CMD_MSG_t *)pkt->payload)->commandFrameId.data, ADV_MSG);
  __nesc_hton_leuint8(((CMD_MSG_t *)pkt->payload)->seedNumber.data, TOS_NODE_ID);
  __nesc_hton_leuint8(((CMD_MSG_t *)pkt->payload)->numHops.data, LatinResP__neighborList.minHops + 1);


  memcpy(& pkt->l2_nextORpreviousHop, &temp_nextHop, sizeof(open_addr_t ));
  pkt->l2_seedNumber = 255;
  pkt->l2_frameType = IEEE154_TYPE_CMD;
  pkt->l2_commandFrameId = ADV_MSG;


  if (LatinResP__OpenSendToLower__send(pkt) == FAIL) {
      LatinResP__Malloc__freePacketBuffer(pkt);
    }
}

#line 60
static inline void LatinResP__taskBuildAndSendScanMsg__runTask(void )
#line 60
{
  OpenQueueEntry_t *pkt;
  open_addr_t temp_nextHop;


  temp_nextHop.type = ADDR_16B;
  temp_nextHop.addr_16b[0] = 0xFF;
  temp_nextHop.addr_16b[1] = 0xFF;

  pkt = LatinResP__Malloc__getFreePacketBuffer();
  if (pkt == (void *)0) {
      return;
    }

  pkt->creator = COMPONENT_RES;
  pkt->owner = COMPONENT_RES;


  LatinResP__PacketFunctions__reserveHeaderSize(pkt, sizeof(CMD_MSG_t ));
  __nesc_hton_leuint8(((CMD_MSG_t *)pkt->payload)->commandFrameId.data, SCAN_MSG);
  __nesc_hton_leuint8(((CMD_MSG_t *)pkt->payload)->seedNumber.data, TOS_NODE_ID);


  memcpy(& pkt->l2_nextORpreviousHop, &temp_nextHop, sizeof(open_addr_t ));
  pkt->l2_seedNumber = 255;
  pkt->l2_frameType = IEEE154_TYPE_CMD;
  pkt->l2_commandFrameId = SCAN_MSG;


  if (LatinResP__OpenSendToLower__send(pkt) == FAIL) {
      LatinResP__Malloc__freePacketBuffer(pkt);
    }
}

# 142 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/idmanager/IDManagerP.nc"
static inline error_t IDManagerP__IDManager__setIsDAGroot(bool newRole)
#line 142
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 143
    IDManagerP__isDAGroot = newRole;
#line 143
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

#line 69
static inline void IDManagerP__taskHandleRoot__runTask(void )
#line 69
{
  uint8_t number_bytes_from_input_buffer;
  uint8_t input_buffer;


  if (number_bytes_from_input_buffer != sizeof input_buffer) {



      return;
    }
#line 79
  ;

  switch (input_buffer) {
      case 'Y': 
        IDManagerP__IDManager__setIsDAGroot(TRUE);
      break;
      case 'N': 
        IDManagerP__IDManager__setIsDAGroot(FALSE);
      break;
      case 'T': 
        if (IDManagerP__IDManager__getIsDAGroot()) {
            IDManagerP__IDManager__setIsDAGroot(FALSE);
          }
        else 
#line 91
          {
            IDManagerP__IDManager__setIsDAGroot(TRUE);
          }
      break;
    }
  return;
}

#line 150
static inline error_t IDManagerP__IDManager__setIsBridge(bool newRole)
#line 150
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 151
    IDManagerP__isBridge = newRole;
#line 151
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

#line 34
static inline void IDManagerP__taskHandleBridge__runTask(void )
#line 34
{
  uint8_t number_bytes_from_input_buffer;
  uint8_t input_buffer[9];


  if (number_bytes_from_input_buffer != sizeof input_buffer) {



      return;
    }
#line 44
  ;

  switch (input_buffer[0]) {
      case 'Y': 
        IDManagerP__IDManager__setIsBridge(TRUE);
      memcpy(& IDManagerP__myPrefix.prefix, &input_buffer[1], 8);
      break;
      case 'N': 
        IDManagerP__IDManager__setIsBridge(FALSE);
      break;
      case 'T': 
        if (IDManagerP__IDManager__getIsBridge()) {
            IDManagerP__IDManager__setIsBridge(FALSE);
          }
        else 
#line 57
          {
            IDManagerP__IDManager__setIsBridge(TRUE);
            memcpy(& IDManagerP__myPrefix.prefix, &input_buffer[1], 8);
          }
      break;
    }
  return;
}

#line 233
static inline void IDManagerP__taskPrint__runTask(void )
#line 233
{
  debugIDManagerEntry_t output;

#line 235
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 235
    {
      output.isDAGroot = IDManagerP__isDAGroot;
      output.isBridge = IDManagerP__isBridge;
      output.my16bID = IDManagerP__my16bID;
      output.my64bID = IDManagerP__my64bID;
      output.myPANID = IDManagerP__myPANID;
      output.myPrefix = IDManagerP__myPrefix;
    }
#line 242
    __nesc_atomic_end(__nesc_atomic); }
}

# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t PrintfP__retrySend__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(PrintfP__retrySend);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 69 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AMSend.nc"
inline static error_t PrintfP__AMSend__send(am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 127 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/printf/PrintfP.nc"
static inline void PrintfP__retrySend__runTask(void )
#line 127
{
  if (PrintfP__AMSend__send(AM_BROADCAST_ADDR, &PrintfP__printfMsg, sizeof(printf_msg_t )) != SUCCESS) {
    PrintfP__retrySend__postTask();
    }
}

# 99 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AMSend.nc"
inline static void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(message_t * msg, error_t error){
#line 99
  PrintfP__AMSend__sendDone(msg, error);
#line 99
}
#line 99
# 57 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/AMQueueEntryP.nc"
static inline void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(message_t *m, error_t err)
#line 57
{
  /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(m, err);
}

# 207 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/AMQueueImplP.nc"
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(uint8_t id, message_t *msg, error_t err)
#line 207
{
}

# 89 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Send.nc"
inline static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(uint8_t arg_0x40afbe70, message_t * msg, error_t error){
#line 89
  switch (arg_0x40afbe70) {
#line 89
    case 0U:
#line 89
      /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(msg, error);
#line 89
      break;
#line 89
    default:
#line 89
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(arg_0x40afbe70, msg, error);
#line 89
      break;
#line 89
    }
#line 89
}
#line 89
# 118 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/AMQueueImplP.nc"
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void )
#line 118
{
  uint8_t i;
#line 119
  uint8_t j;
#line 119
  uint8_t mask;
#line 119
  uint8_t last;
  message_t *msg;

#line 121
  for (i = 0; i < 1 / 8 + 1; i++) {
      if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i]) {
          for (mask = 1, j = 0; j < 8; j++) {
              if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i] & mask) {
                  last = i * 8 + j;
                  msg = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg;
                  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg = (void *)0;
                  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i] &= ~mask;
                  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(last, msg, ECANCEL);
                }
              mask <<= 1;
            }
        }
    }
}

#line 161
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void )
#line 161
{
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current, /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg, FAIL);
}

# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 235 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_uint8(const void * source)
#line 235
{
  const uint8_t *base = source;

#line 237
  return base[0];
}

# 111 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialActiveMessageP.nc"
static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__payloadLength(message_t *msg)
#line 111
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(msg);

#line 113
  return __nesc_ntoh_uint8(header->length.data);
}

# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Packet.nc"
inline static uint8_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(message_t * msg){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__payloadLength(msg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 57 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/AMQueueImplP.nc"
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket(void )
#line 57
{
  uint8_t i;

#line 59
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current + 1) % 1;
  for (i = 0; i < 1; i++) {
      if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg == (void *)0 || 
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current / 8] & (1 << /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current % 8)) 
        {
          /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current + 1) % 1;
        }
      else {
          break;
        }
    }
  if (i >= 1) {
#line 70
    /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
    }
}

#line 166
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void )
#line 166
{
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket();
  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current < 1) {
      error_t nextErr;
      message_t *nextMsg = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg;
      am_id_t nextId = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(nextMsg);
      am_addr_t nextDest = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(nextMsg);
      uint8_t len = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(nextMsg);

#line 174
      nextErr = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(nextId, nextDest, nextMsg, len);
      if (nextErr != SUCCESS) {
          /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask();
        }
    }
}

# 17 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosa/TelosSerialP.nc"
static inline void TelosSerialP__Resource__granted(void )
#line 17
{
}

# 218 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__default__granted(uint8_t id)
#line 218
{
}

# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__granted(uint8_t arg_0x408785a8){
#line 92
  switch (arg_0x408785a8) {
#line 92
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 92
      TelosSerialP__Resource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__default__granted(arg_0x408785a8);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 101 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__granted(uint8_t id)
#line 101
{
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__granted(id);
}

# 199 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id)
#line 199
{
}

# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(uint8_t arg_0x40a99e98){
#line 92
  switch (arg_0x40a99e98) {
#line 92
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 92
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__granted(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(arg_0x40a99e98);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 213 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id)
#line 213
{
}

# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(uint8_t arg_0x40aaacf8){
#line 49
  switch (arg_0x40aaacf8) {
#line 49
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 49
      /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__configure(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(arg_0x40aaacf8);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 187 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void )
#line 187
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 188
    {
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
    }
#line 191
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId);
}

# 19 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosa/TelosSerialP.nc"
static inline msp430_uart_union_config_t *TelosSerialP__Msp430UartConfigure__getConfig(void )
#line 19
{
  return &TelosSerialP__msp430_uart_telos_config;
}

# 214 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
static inline msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(uint8_t id)
#line 214
{
  return &msp430_uart_default_config;
}

# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartConfigure.nc"
inline static msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__getConfig(uint8_t arg_0x408746c0){
#line 39
  union __nesc_unnamed4281 *__nesc_result;
#line 39

#line 39
  switch (arg_0x408746c0) {
#line 39
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 39
      __nesc_result = TelosSerialP__Msp430UartConfigure__getConfig();
#line 39
      break;
#line 39
    default:
#line 39
      __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(arg_0x408746c0);
#line 39
      break;
#line 39
    }
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 359 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__disableIntr(void )
#line 359
{
  HplMsp430Usart1P__IE2 &= ~((1 << 5) | (1 << 4));
}

#line 347
static inline void HplMsp430Usart1P__Usart__clrIntr(void )
#line 347
{
  HplMsp430Usart1P__IFG2 &= ~((1 << 5) | (1 << 4));
}

#line 159
static inline void HplMsp430Usart1P__Usart__resetUsart(bool reset)
#line 159
{
  if (reset) {
    U1CTL = 0x01;
    }
  else {
#line 163
    U1CTL &= ~0x01;
    }
}

# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 6;
}

# 78 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__UTXD__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectModuleFunc();
#line 78
}
#line 78
# 220 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__enableUartTx(void )
#line 220
{
  HplMsp430Usart1P__UTXD__selectModuleFunc();
  HplMsp430Usart1P__ME2 |= 1 << 5;
}

# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 7);
}

# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__URXD__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc();
#line 85
}
#line 85
# 236 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__disableUartRx(void )
#line 236
{
  HplMsp430Usart1P__ME2 &= ~(1 << 4);
  HplMsp430Usart1P__URXD__selectIOFunc();
}

# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 7;
}

# 78 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__URXD__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectModuleFunc();
#line 78
}
#line 78
# 231 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__enableUartRx(void )
#line 231
{
  HplMsp430Usart1P__URXD__selectModuleFunc();
  HplMsp430Usart1P__ME2 |= 1 << 4;
}

# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 6);
}

# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__UTXD__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc();
#line 85
}
#line 85
# 225 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__disableUartTx(void )
#line 225
{
  HplMsp430Usart1P__ME2 &= ~(1 << 5);
  HplMsp430Usart1P__UTXD__selectIOFunc();
}

#line 203
static inline void HplMsp430Usart1P__Usart__enableUart(void )
#line 203
{
  /* atomic removed: atomic calls only */
#line 204
  {
    HplMsp430Usart1P__UTXD__selectModuleFunc();
    HplMsp430Usart1P__URXD__selectModuleFunc();
  }
  HplMsp430Usart1P__ME2 |= (1 << 5) | (1 << 4);
}

#line 151
static inline void HplMsp430Usart1P__Usart__setUmctl(uint8_t control)
#line 151
{
  U1MCTL = control;
}

#line 140
static inline void HplMsp430Usart1P__Usart__setUbr(uint16_t control)
#line 140
{
  /* atomic removed: atomic calls only */
#line 141
  {
    U1BR0 = control & 0x00FF;
    U1BR1 = (control >> 8) & 0x00FF;
  }
}

#line 283
static inline void HplMsp430Usart1P__configUart(msp430_uart_union_config_t *config)
#line 283
{

  U1CTL = (config->uartRegisters.uctl & ~0x04) | 0x01;
  HplMsp430Usart1P__U1TCTL = config->uartRegisters.utctl;
  HplMsp430Usart1P__U1RCTL = config->uartRegisters.urctl;

  HplMsp430Usart1P__Usart__setUbr(config->uartRegisters.ubr);
  HplMsp430Usart1P__Usart__setUmctl(config->uartRegisters.umctl);
}

static inline void HplMsp430Usart1P__Usart__setModeUart(msp430_uart_union_config_t *config)
#line 293
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 295
    {
      HplMsp430Usart1P__Usart__resetUsart(TRUE);
      HplMsp430Usart1P__Usart__disableSpi();
      HplMsp430Usart1P__configUart(config);
      if (config->uartConfig.utxe == 1 && config->uartConfig.urxe == 1) {
          HplMsp430Usart1P__Usart__enableUart();
        }
      else {
#line 301
        if (config->uartConfig.utxe == 0 && config->uartConfig.urxe == 1) {
            HplMsp430Usart1P__Usart__disableUartTx();
            HplMsp430Usart1P__Usart__enableUartRx();
          }
        else {
#line 304
          if (config->uartConfig.utxe == 1 && config->uartConfig.urxe == 0) {
              HplMsp430Usart1P__Usart__disableUartRx();
              HplMsp430Usart1P__Usart__enableUartTx();
            }
          else 
#line 307
            {
              HplMsp430Usart1P__Usart__disableUart();
            }
          }
        }
#line 310
      HplMsp430Usart1P__Usart__resetUsart(FALSE);
      HplMsp430Usart1P__Usart__clrIntr();
      HplMsp430Usart1P__Usart__disableIntr();
    }
#line 313
    __nesc_atomic_end(__nesc_atomic); }

  return;
}

# 174 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__setModeUart(msp430_uart_union_config_t *config){
#line 174
  HplMsp430Usart1P__Usart__setModeUart(config);
#line 174
}
#line 174
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )51U &= ~(0x01 << 1);
}

# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__SIMO__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )51U &= ~(0x01 << 2);
}

# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__SOMI__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )51U &= ~(0x01 << 3);
}

# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__UCLK__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc();
#line 85
}
#line 85
# 377 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__enableIntr(void )
#line 377
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 378
    {
      HplMsp430Usart1P__IFG2 &= ~((1 << 5) | (1 << 4));
      HplMsp430Usart1P__IE2 |= (1 << 5) | (1 << 4);
    }
#line 381
    __nesc_atomic_end(__nesc_atomic); }
}

# 182 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__enableIntr(void ){
#line 182
  HplMsp430Usart1P__Usart__enableIntr();
#line 182
}
#line 182
# 181 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/AMQueueImplP.nc"
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(am_id_t id, message_t *msg, error_t err)
#line 181
{





  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current >= 1) {
      return;
    }
  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg == msg) {
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current, msg, err);
    }
  else {
      ;
    }
}

# 99 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AMSend.nc"
inline static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__sendDone(am_id_t arg_0x407205f8, message_t * msg, error_t error){
#line 99
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(arg_0x407205f8, msg, error);
#line 99
}
#line 99
# 90 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(message_t *msg, error_t result)
#line 90
{
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__sendDone(/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(msg), msg, result);
}

# 365 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(uart_id_t idxxx, message_t *msg, error_t error)
#line 365
{
  return;
}

# 89 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Send.nc"
inline static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__sendDone(uart_id_t arg_0x407d0840, message_t * msg, error_t error){
#line 89
  switch (arg_0x407d0840) {
#line 89
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 89
      /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(msg, error);
#line 89
      break;
#line 89
    default:
#line 89
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(arg_0x407d0840, msg, error);
#line 89
      break;
#line 89
    }
#line 89
}
#line 89
# 147 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask(void )
#line 147
{
  error_t error;

  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 151
    error = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError;
#line 151
    __nesc_atomic_end(__nesc_atomic); }

  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendCancelled) {
#line 153
    error = ECANCEL;
    }
#line 154
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__sendDone(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendId, (message_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer, error);
}

#line 201
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(uint8_t which)
#line 201
{
  if (which) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufOneLocked = 0;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufZeroLocked = 0;
    }
}

# 98 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialActiveMessageP.nc"
static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(uint8_t id, message_t *msg, void *payload, uint8_t len)
#line 98
{
  return msg;
}

# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Receive.nc"
inline static message_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__receive(am_id_t arg_0x4072f010, message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
    __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(arg_0x4072f010, msg, payload, len);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 102 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialActiveMessageP.nc"
static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 102
{
  return /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__receive(/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(msg), msg, msg->data, len);
}

# 360 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
static inline message_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(uart_id_t idxxx, message_t *msg, 
void *payload, 
uint8_t len)
#line 362
{
  return msg;
}

# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Receive.nc"
inline static message_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__receive(uart_id_t arg_0x407d0200, message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  switch (arg_0x407d0200) {
#line 67
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 67
      __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(msg, payload, len);
#line 67
      break;
#line 67
    default:
#line 67
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(arg_0x407d0200, msg, payload, len);
#line 67
      break;
#line 67
    }
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__upperLength(message_t *msg, uint8_t dataLinkLen)
#line 46
{
  return dataLinkLen - sizeof(serial_header_t );
}

# 354 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(uart_id_t id, message_t *msg, 
uint8_t dataLinkLen)
#line 355
{
  return 0;
}

# 31 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__upperLength(uart_id_t arg_0x407cf398, message_t *msg, uint8_t dataLinkLen){
#line 31
  unsigned char __nesc_result;
#line 31

#line 31
  switch (arg_0x407cf398) {
#line 31
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 31
      __nesc_result = SerialPacketInfoActiveMessageP__Info__upperLength(msg, dataLinkLen);
#line 31
      break;
#line 31
    default:
#line 31
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(arg_0x407cf398, msg, dataLinkLen);
#line 31
      break;
#line 31
    }
#line 31

#line 31
  return __nesc_result;
#line 31
}
#line 31
# 264 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask(void )
#line 264
{
  uart_id_t myType;
  message_t *myBuf;
  uint8_t mySize;
  uint8_t myWhich;

#line 269
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 269
    {
      myType = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskType;
      myBuf = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskBuf;
      mySize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskSize;
      myWhich = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskWhich;
    }
#line 274
    __nesc_atomic_end(__nesc_atomic); }
  mySize -= /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(myType);
  mySize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__upperLength(myType, myBuf, mySize);
  myBuf = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__receive(myType, myBuf, myBuf, mySize);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 278
    {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messagePtrs[myWhich] = myBuf;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(myWhich);
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending = FALSE;
    }
#line 282
    __nesc_atomic_end(__nesc_atomic); }
}

# 123 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/printf/PrintfP.nc"
static inline void PrintfP__SerialControl__stopDone(error_t error)
#line 123
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 124
    PrintfP__state = PrintfP__S_STOPPED;
#line 124
    __nesc_atomic_end(__nesc_atomic); }
}

# 117 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/SplitControl.nc"
inline static void SerialP__SplitControl__stopDone(error_t error){
#line 117
  PrintfP__SerialControl__stopDone(error);
#line 117
}
#line 117
# 109 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline error_t HplMsp430Usart1P__AsyncStdControl__stop(void )
#line 109
{
  HplMsp430Usart1P__Usart__disableSpi();
  HplMsp430Usart1P__Usart__disableUart();
  return SUCCESS;
}

# 84 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AsyncStdControl.nc"
inline static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__stop(void ){
#line 84
  unsigned char __nesc_result;
#line 84

#line 84
  __nesc_result = HplMsp430Usart1P__AsyncStdControl__stop();
#line 84

#line 84
  return __nesc_result;
#line 84
}
#line 84
# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup(void )
#line 74
{
}

# 52 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/power/PowerDownCleanup.nc"
inline static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__cleanup(void ){
#line 52
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup();
#line 52
}
#line 52
# 69 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted(void )
#line 69
{
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__cleanup();
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__stop();
}

# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void ){
#line 46
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted();
#line 46
}
#line 46
# 128 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableUart(void ){
#line 128
  HplMsp430Usart1P__Usart__disableUart();
#line 128
}
#line 128
#line 179
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableIntr(void ){
#line 179
  HplMsp430Usart1P__Usart__disableIntr();
#line 179
}
#line 179
#line 97
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__resetUsart(bool reset){
#line 97
  HplMsp430Usart1P__Usart__resetUsart(reset);
#line 97
}
#line 97
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(uint8_t id)
#line 92
{
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__resetUsart(TRUE);
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableIntr();
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableUart();
}

# 215 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id)
#line 215
{
}

# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(uint8_t arg_0x40aaacf8){
#line 55
  switch (arg_0x40aaacf8) {
#line 55
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 55
      /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 55
      break;
#line 55
    default:
#line 55
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(arg_0x40aaacf8);
#line 55
      break;
#line 55
    }
#line 55
}
#line 55
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 58 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    if (/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead != /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
        uint8_t id = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead;

#line 62
        /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead];
        if (/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead == /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
          /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
          }
#line 65
        /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[id] = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
        {
          unsigned char __nesc_temp = 
#line 66
          id;

#line 66
          return __nesc_temp;
        }
      }
#line 68
    {
      unsigned char __nesc_temp = 
#line 68
      /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

#line 68
      return __nesc_temp;
    }
  }
}

# 60 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 50 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 51
  {
    unsigned char __nesc_temp = 
#line 51
    /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead == /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

#line 51
    return __nesc_temp;
  }
}

# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceQueue.nc"
inline static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 108 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id)
#line 108
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 109
    {
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY && /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId == id) {
          if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty() == FALSE) {
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue();
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING;
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(id);
            }
          else {
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(id);
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted();
            }
          {
            unsigned char __nesc_temp = 
#line 124
            SUCCESS;

            {
#line 124
              __nesc_atomic_end(__nesc_atomic); 
#line 124
              return __nesc_temp;
            }
          }
        }
    }
#line 128
    __nesc_atomic_end(__nesc_atomic); }
#line 127
  return FAIL;
}

# 213 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__release(uint8_t id)
#line 213
{
#line 213
  return FAIL;
}

# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__release(uint8_t arg_0x40875bb0){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  switch (arg_0x40875bb0) {
#line 110
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 110
      __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(/*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID);
#line 110
      break;
#line 110
    default:
#line 110
      __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__release(arg_0x40875bb0);
#line 110
      break;
#line 110
    }
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 210 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__isOwner(uint8_t id)
#line 210
{
#line 210
  return FAIL;
}

# 118 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__isOwner(uint8_t arg_0x40875bb0){
#line 118
  unsigned char __nesc_result;
#line 118

#line 118
  switch (arg_0x40875bb0) {
#line 118
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 118
      __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(/*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID);
#line 118
      break;
#line 118
    default:
#line 118
      __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__isOwner(arg_0x40875bb0);
#line 118
      break;
#line 118
    }
#line 118

#line 118
  return __nesc_result;
#line 118
}
#line 118
# 77 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__release(uint8_t id)
#line 77
{
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__isOwner(id) == FALSE) {
    return FAIL;
    }
#line 80
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf || /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf) {
    return EBUSY;
    }
#line 82
  return /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__release(id);
}

# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t TelosSerialP__Resource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__release(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 13 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosa/TelosSerialP.nc"
static inline error_t TelosSerialP__StdControl__stop(void )
#line 13
{
  TelosSerialP__Resource__release();
  return SUCCESS;
}

# 84 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/StdControl.nc"
inline static error_t SerialP__SerialControl__stop(void ){
#line 84
  unsigned char __nesc_result;
#line 84

#line 84
  __nesc_result = TelosSerialP__StdControl__stop();
#line 84

#line 84
  return __nesc_result;
#line 84
}
#line 84
# 330 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFlush__flushDone(void )
#line 330
{
  SerialP__SerialControl__stop();
  SerialP__SplitControl__stopDone(SUCCESS);
}

static inline void SerialP__defaultSerialFlushTask__runTask(void )
#line 335
{
  SerialP__SerialFlush__flushDone();
}

# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP__defaultSerialFlushTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SerialP__defaultSerialFlushTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 338 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFlush__default__flush(void )
#line 338
{
  SerialP__defaultSerialFlushTask__postTask();
}

# 38 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialFlush.nc"
inline static void SerialP__SerialFlush__flush(void ){
#line 38
  SerialP__SerialFlush__default__flush();
#line 38
}
#line 38
# 326 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialP.nc"
static inline void SerialP__stopDoneTask__runTask(void )
#line 326
{
  SerialP__SerialFlush__flush();
}

# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Counter.nc"
inline static /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Counter__size_type /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Counter__get(void ){
#line 53
  unsigned long __nesc_result;
#line 53

#line 53
  __nesc_result = /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 75 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static inline /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__to_size_type /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Alarm__getNow(void )
{
  return /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Counter__get();
}

#line 136
static inline void /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Alarm__startAt(/*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__to_size_type t0, /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__m_t0 = t0;
      /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__m_dt = dt;
      /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__set_alarm();
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

static inline void /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Alarm__start(/*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__to_size_type dt)
{
  /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Alarm__startAt(/*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Alarm__getNow(), dt);
}

# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/Alarm.nc"
inline static void CC2420ControlP__StartupAlarm__start(CC2420ControlP__StartupAlarm__size_type dt){
#line 55
  /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Alarm__start(dt);
#line 55
}
#line 55
# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )29U |= 0x01 << 5;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 34 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__13__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set();
#line 34
}
#line 34
# 37 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__13__GeneralIO__set(void )
#line 37
{
#line 37
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__13__HplGeneralIO__set();
}

# 29 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__VREN__set(void ){
#line 29
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__13__GeneralIO__set();
#line 29
}
#line 29
# 77 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Config__startVReg(void )
#line 77
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 78
    {
      if (CC2420ControlP__m_state != CC2420ControlP__S_VREG_STOPPED) {
          {
            unsigned char __nesc_temp = 
#line 80
            FAIL;

            {
#line 80
              __nesc_atomic_end(__nesc_atomic); 
#line 80
              return __nesc_temp;
            }
          }
        }
#line 82
      CC2420ControlP__m_state = CC2420ControlP__S_VREG_STARTING;
    }
#line 83
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP__VREN__set();
  CC2420ControlP__StartupAlarm__start(CC2420_TIME_VREN);
  return SUCCESS;
}

# 2 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Config.nc"
inline static error_t CC2420DriversP__CC2420Config__startVReg(void ){
#line 2
  unsigned char __nesc_result;
#line 2

#line 2
  __nesc_result = CC2420ControlP__CC2420Config__startVReg();
#line 2

#line 2
  return __nesc_result;
#line 2
}
#line 2
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/CC2420DriversP.nc"
static inline error_t CC2420DriversP__RadioControl__start(void )
#line 54
{
  uint8_t temp_state;

#line 56
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 56
    {
      temp_state = CC2420DriversP__state;
    }
#line 58
    __nesc_atomic_end(__nesc_atomic); }
  switch (temp_state) {
      case CC2420DriversP__S_STARTED: 
        return EALREADY;
      case CC2420DriversP__S_STARTING_VREG: 
        case CC2420DriversP__S_STARTING_OSCILLATOR: 
          case CC2420DriversP__S_WRITING_ID: 
            return SUCCESS;
      default: 
        { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 67
          CC2420DriversP__state = CC2420DriversP__S_STARTING_VREG;
#line 67
          __nesc_atomic_end(__nesc_atomic); }
      CC2420DriversP__CC2420Config__startVReg();
    }
  return SUCCESS;
}

# 7 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/RadioControl.nc"
inline static error_t LatinMacP__RadioControl__start(void ){
#line 7
  unsigned char __nesc_result;
#line 7

#line 7
  __nesc_result = CC2420DriversP__RadioControl__start();
#line 7

#line 7
  return __nesc_result;
#line 7
}
#line 7
# 83 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/LatinMac/LatinMacP.nc"
static inline void LatinMacP__Boot__booted(void )
#line 83
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 84
    LatinMacP__init = 0;
#line 84
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 85
    LatinMacP__numReceiveNothing = 0;
#line 85
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 86
    LatinMacP__numADV = 0;
#line 86
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 87
    LatinMacP__isSync = FALSE;
#line 87
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 88
    LatinMacP__beacon_channel = TRUE;
#line 88
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 89
    LatinMacP__change = FALSE;
#line 89
    __nesc_atomic_end(__nesc_atomic); }
  LatinMacP__numPrepareReceiveFailed = 0;
  LatinMacP__numSlotsInSynchStatus = 0;
  LatinMacP__Leds__led2Off();
  LatinMacP__RadioControl__start();


  LatinMacP__slotType = UPLINK;
  LatinMacP__networkState = BUILDING;
}

# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Boot.nc"
inline static void PrintfP__Boot__booted(void ){
#line 49
  LatinMacP__Boot__booted();
#line 49
}
#line 49
# 113 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/printf/PrintfP.nc"
static inline void PrintfP__SerialControl__startDone(error_t error)
#line 113
{
  if (PrintfP__state == PrintfP__S_STOPPED) {



      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 118
        PrintfP__state = PrintfP__S_STARTED;
#line 118
        __nesc_atomic_end(__nesc_atomic); }
      PrintfP__Boot__booted();
    }
}

# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/SplitControl.nc"
inline static void SerialP__SplitControl__startDone(error_t error){
#line 92
  PrintfP__SerialControl__startDone(error);
#line 92
}
#line 92
# 130 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void )
#line 130
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 131
    {
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id) {
          if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING) {
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
              {
                unsigned char __nesc_temp = 
#line 135
                SUCCESS;

                {
#line 135
                  __nesc_atomic_end(__nesc_atomic); 
#line 135
                  return __nesc_temp;
                }
              }
            }
          else {
#line 137
            if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING) {
                /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
                /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
                {
                  unsigned char __nesc_temp = 
#line 140
                  SUCCESS;

                  {
#line 140
                    __nesc_atomic_end(__nesc_atomic); 
#line 140
                    return __nesc_temp;
                  }
                }
              }
            }
        }
    }
#line 146
    __nesc_atomic_end(__nesc_atomic); }
#line 144
  return FAIL;
}

# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceDefaultOwner.nc"
inline static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__release(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release();
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 105 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline error_t HplMsp430Usart1P__AsyncStdControl__start(void )
#line 105
{
  return SUCCESS;
}

# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/AsyncStdControl.nc"
inline static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__start(void ){
#line 74
  unsigned char __nesc_result;
#line 74

#line 74
  __nesc_result = HplMsp430Usart1P__AsyncStdControl__start();
#line 74

#line 74
  return __nesc_result;
#line 74
}
#line 74
# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested(void )
#line 64
{
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__start();
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__release();
}

# 81 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void ){
#line 81
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested();
#line 81
}
#line 81
# 203 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id)
#line 203
{
}

# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(uint8_t arg_0x40aac948){
#line 51
    /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(arg_0x40aac948);
#line 51
}
#line 51
# 90 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id)
#line 90
{
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 92
    {
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) {
          /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING;
          /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = id;
        }
      else {
          unsigned char __nesc_temp = 
#line 97
          FAIL;

          {
#line 97
            __nesc_atomic_end(__nesc_atomic); 
#line 97
            return __nesc_temp;
          }
        }
    }
#line 100
    __nesc_atomic_end(__nesc_atomic); }
#line 99
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested();
  if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId == id) {
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId);
      return SUCCESS;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 104
    /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
#line 104
    __nesc_atomic_end(__nesc_atomic); }
  return FAIL;
}

# 212 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__immediateRequest(uint8_t id)
#line 212
{
#line 212
  return FAIL;
}

# 87 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__immediateRequest(uint8_t arg_0x40875bb0){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  switch (arg_0x40875bb0) {
#line 87
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 87
      __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(/*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID);
#line 87
      break;
#line 87
    default:
#line 87
      __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__immediateRequest(arg_0x40875bb0);
#line 87
      break;
#line 87
    }
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 65 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__immediateRequest(uint8_t id)
#line 65
{
  return /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__immediateRequest(id);
}

# 87 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t TelosSerialP__Resource__immediateRequest(void ){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__immediateRequest(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 10 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosa/TelosSerialP.nc"
static inline error_t TelosSerialP__StdControl__start(void )
#line 10
{
  return TelosSerialP__Resource__immediateRequest();
}

# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/StdControl.nc"
inline static error_t SerialP__SerialControl__start(void ){
#line 74
  unsigned char __nesc_result;
#line 74

#line 74
  __nesc_result = TelosSerialP__StdControl__start();
#line 74

#line 74
  return __nesc_result;
#line 74
}
#line 74
# 320 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialP.nc"
static inline void SerialP__startDoneTask__runTask(void )
#line 320
{
  SerialP__SerialControl__start();
  SerialP__SplitControl__startDone(SUCCESS);
}

# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialFrameComm.nc"
inline static error_t SerialP__SerialFrameComm__putDelimiter(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = HdlcTranslateC__SerialFrameComm__putDelimiter();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 183 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error_t error)
#line 183
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 184
    /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError = error;
#line 184
    __nesc_atomic_end(__nesc_atomic); }
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__postTask();
}

# 80 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SendBytePacket.nc"
inline static void SerialP__SendBytePacket__sendCompleted(error_t error){
#line 80
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error);
#line 80
}
#line 80
# 242 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialP.nc"
static __inline bool SerialP__ack_queue_is_empty(void )
#line 242
{
  bool ret;

#line 244
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 244
    ret = SerialP__ackQ.writePtr == SerialP__ackQ.readPtr;
#line 244
    __nesc_atomic_end(__nesc_atomic); }
  return ret;
}











static __inline uint8_t SerialP__ack_queue_top(void )
#line 258
{
  uint8_t tmp = 0;

  /* atomic removed: atomic calls only */
#line 260
  {
    if (!SerialP__ack_queue_is_empty()) {
        tmp = SerialP__ackQ.buf[SerialP__ackQ.readPtr];
      }
  }
  return tmp;
}

static inline uint8_t SerialP__ack_queue_pop(void )
#line 268
{
  uint8_t retval = 0;

#line 270
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 270
    {
      if (SerialP__ackQ.writePtr != SerialP__ackQ.readPtr) {
          retval = SerialP__ackQ.buf[SerialP__ackQ.readPtr];
          if (++ SerialP__ackQ.readPtr > SerialP__ACK_QUEUE_SIZE) {
#line 273
            SerialP__ackQ.readPtr = 0;
            }
        }
    }
#line 276
    __nesc_atomic_end(__nesc_atomic); }
#line 276
  return retval;
}

#line 539
static inline void SerialP__RunTx__runTask(void )
#line 539
{
  uint8_t idle;
  uint8_t done;
  uint8_t fail;









  error_t result = SUCCESS;
  bool send_completed = FALSE;
  bool start_it = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 556
    {
      SerialP__txPending = 0;
      idle = SerialP__txState == SerialP__TXSTATE_IDLE;
      done = SerialP__txState == SerialP__TXSTATE_FINISH;
      fail = SerialP__txState == SerialP__TXSTATE_ERROR;
      if (done || fail) {
          SerialP__txState = SerialP__TXSTATE_IDLE;
          SerialP__txBuf[SerialP__txIndex].state = SerialP__BUFFER_AVAILABLE;
        }
    }
#line 565
    __nesc_atomic_end(__nesc_atomic); }


  if (done || fail) {
      SerialP__txSeqno++;
      if (SerialP__txProto == SERIAL_PROTO_ACK) {
          SerialP__ack_queue_pop();
        }
      else {
          result = done ? SUCCESS : FAIL;
          send_completed = TRUE;
        }
      idle = TRUE;
    }


  if (idle) {
      bool goInactive;

#line 583
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 583
        goInactive = SerialP__offPending;
#line 583
        __nesc_atomic_end(__nesc_atomic); }
      if (goInactive) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 585
            SerialP__txState = SerialP__TXSTATE_INACTIVE;
#line 585
            __nesc_atomic_end(__nesc_atomic); }
        }
      else {

          uint8_t myAckState;
          uint8_t myDataState;

#line 591
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 591
            {
              myAckState = SerialP__txBuf[SerialP__TX_ACK_INDEX].state;
              myDataState = SerialP__txBuf[SerialP__TX_DATA_INDEX].state;
            }
#line 594
            __nesc_atomic_end(__nesc_atomic); }
          if (!SerialP__ack_queue_is_empty() && myAckState == SerialP__BUFFER_AVAILABLE) {
              { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 596
                {
                  SerialP__txBuf[SerialP__TX_ACK_INDEX].state = SerialP__BUFFER_COMPLETE;
                  SerialP__txBuf[SerialP__TX_ACK_INDEX].buf = SerialP__ack_queue_top();
                }
#line 599
                __nesc_atomic_end(__nesc_atomic); }
              SerialP__txProto = SERIAL_PROTO_ACK;
              SerialP__txIndex = SerialP__TX_ACK_INDEX;
              start_it = TRUE;
            }
          else {
#line 604
            if (myDataState == SerialP__BUFFER_FILLING || myDataState == SerialP__BUFFER_COMPLETE) {
                SerialP__txProto = SERIAL_PROTO_PACKET_NOACK;
                SerialP__txIndex = SerialP__TX_DATA_INDEX;
                start_it = TRUE;
              }
            else {
              }
            }
        }
    }
  else {
    }


  if (send_completed) {
      SerialP__SendBytePacket__sendCompleted(result);
    }

  if (SerialP__txState == SerialP__TXSTATE_INACTIVE) {
      SerialP__testOff();
      return;
    }

  if (start_it) {

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 629
        {
          SerialP__txCRC = 0;
          SerialP__txByteCnt = 0;
          SerialP__txState = SerialP__TXSTATE_PROTO;
        }
#line 633
        __nesc_atomic_end(__nesc_atomic); }
      if (SerialP__SerialFrameComm__putDelimiter() != SUCCESS) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 635
            SerialP__txState = SerialP__TXSTATE_ERROR;
#line 635
            __nesc_atomic_end(__nesc_atomic); }
          SerialP__MaybeScheduleTx();
        }
    }
}

# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP__stopDoneTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SerialP__stopDoneTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/UartStream.nc"
inline static error_t HdlcTranslateC__UartStream__send(uint8_t * buf, uint16_t len){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__send(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID, buf, len);
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 58 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 214 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialP.nc"
static __inline void SerialP__ackInit(void )
#line 214
{
  SerialP__ackQ.writePtr = SerialP__ackQ.readPtr = 0;
}

#line 205
static __inline void SerialP__rxInit(void )
#line 205
{
  SerialP__rxBuf.writePtr = SerialP__rxBuf.readPtr = 0;
  SerialP__rxState = SerialP__RXSTATE_NOSYNC;
  SerialP__rxByteCnt = 0;
  SerialP__rxProto = 0;
  SerialP__rxSeqno = 0;
  SerialP__rxCRC = 0;
}

#line 193
static __inline void SerialP__txInit(void )
#line 193
{
  uint8_t i;

  /* atomic removed: atomic calls only */
#line 195
  for (i = 0; i < SerialP__TX_BUFFER_COUNT; i++) SerialP__txBuf[i].state = SerialP__BUFFER_AVAILABLE;
  SerialP__txState = SerialP__TXSTATE_IDLE;
  SerialP__txByteCnt = 0;
  SerialP__txProto = 0;
  SerialP__txSeqno = 0;
  SerialP__txCRC = 0;
  SerialP__txPending = FALSE;
  SerialP__txIndex = 0;
}

#line 218
static inline error_t SerialP__Init__init(void )
#line 218
{

  SerialP__txInit();
  SerialP__rxInit();
  SerialP__ackInit();

  return SUCCESS;
}

# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void )
#line 45
{
  memset(/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ, /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY, sizeof /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ);
  return SUCCESS;
}

# 32 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/07-App/appDATA/Device/appDATADevP.nc"
static inline error_t appDATADevP__SoftwareInit__init(void )
#line 32
{

  return SUCCESS;
}

# 7 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/interfaces/PacketFunctions.nc"
inline static void IDManagerP__PacketFunctions__mac64bToMac16b(open_addr_t *mac64b, open_addr_t *mac16btoWrite){
#line 7
  PacketFunctionsP__PacketFunctions__mac64bToMac16b(mac64b, mac16btoWrite);
#line 7
}
#line 7
# 99 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/idmanager/IDManagerP.nc"
static inline error_t IDManagerP__SoftwareInit__init(void )
#line 99
{
  IDManagerP__isDAGroot = FALSE;
  IDManagerP__isBridge = FALSE;
  IDManagerP__myPANID.type = ADDR_PANID;
  IDManagerP__myPANID.panid[0] = 0xAA;
  IDManagerP__myPANID.panid[1] = 0xAA;
  IDManagerP__myPrefix.type = ADDR_PREFIX;








  IDManagerP__myPrefix.prefix[0] = 0x00;
  IDManagerP__myPrefix.prefix[1] = 0x00;
  IDManagerP__myPrefix.prefix[2] = 0x00;
  IDManagerP__myPrefix.prefix[3] = 0x00;
  IDManagerP__myPrefix.prefix[4] = 0x00;
  IDManagerP__myPrefix.prefix[5] = 0x00;
  IDManagerP__myPrefix.prefix[6] = 0x00;
  IDManagerP__myPrefix.prefix[7] = 0x00;
  IDManagerP__my64bID.type = ADDR_64B;
  IDManagerP__my16bID.addr_64b[0] = 0x00;
  IDManagerP__my64bID.addr_64b[1] = 0x00;
  IDManagerP__my64bID.addr_64b[2] = 0x00;
  IDManagerP__my64bID.addr_64b[3] = 0x00;
  IDManagerP__my64bID.addr_64b[4] = 0x00;
  IDManagerP__my64bID.addr_64b[5] = 0x00;
  IDManagerP__my64bID.addr_64b[6] = 0x00;
  IDManagerP__my64bID.addr_64b[7] = TOS_NODE_ID;




  IDManagerP__PacketFunctions__mac64bToMac16b(&IDManagerP__my64bID, &IDManagerP__my16bID);
  return SUCCESS;
}

# 409 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02b-RES/LatinRes/LatinResP.nc"
static inline void LatinResP__resetNeighborList(void )
#line 409
{
  uint8_t i;

#line 411
  LatinResP__neighborList.numNeighbors = 0;
  LatinResP__neighborList.old_numNeighbors = 0;
  LatinResP__neighborList.counterNoNewNeighbors = 0;
  LatinResP__neighborList.numChildren = 0;
  LatinResP__neighborList.numSibling = 0;
  LatinResP__neighborList.old_numChildren = 0;
  LatinResP__neighborList.counterNoNewChildren = 0;
  LatinResP__neighborList.counterSeedMsgSent = 0;
  LatinResP__neighborList.maxRssi = -64;
  LatinResP__neighborList.minHops = 255;
  LatinResP__neighborList.parent = 254;

  for (i = 0; i < MAXNUMNEIGHBORS; i++) {
      LatinResP__neighborList.neighbors[i].used = FALSE;
      LatinResP__neighborList.neighbors[i].isChild = FALSE;
      LatinResP__neighborList.neighbors[i].seedNumber = 254;
      LatinResP__neighborList.neighbors[i].rssi = -64;
      LatinResP__neighborList.neighbors[i].numHops = 255;
    }
}

# 48 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__getRaw(void )
#line 48
{
#line 48
  return * (volatile uint8_t * )40U & (0x01 << 7);
}

#line 49
static inline bool /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__get(void )
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__getRaw() != 0;
}

# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplUserButtonC.UserButtonC*/Msp430GpioC__3__HplGeneralIO__get(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__get();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 40 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplUserButtonC.UserButtonC*/Msp430GpioC__3__GeneralIO__get(void )
#line 40
{
#line 40
  return /*HplUserButtonC.UserButtonC*/Msp430GpioC__3__HplGeneralIO__get();
}

# 32 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static bool /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__GeneralIO__get(void ){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = /*HplUserButtonC.UserButtonC*/Msp430GpioC__3__GeneralIO__get();
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 50 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )42U &= ~(0x01 << 7);
}

# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplUserButtonC.UserButtonC*/Msp430GpioC__3__HplGeneralIO__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P27*/HplMsp430GeneralIOP__15__IO__makeInput();
#line 64
}
#line 64
# 41 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplUserButtonC.UserButtonC*/Msp430GpioC__3__GeneralIO__makeInput(void )
#line 41
{
#line 41
  /*HplUserButtonC.UserButtonC*/Msp430GpioC__3__HplGeneralIO__makeInput();
}

# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__GeneralIO__makeInput(void ){
#line 33
  /*HplUserButtonC.UserButtonC*/Msp430GpioC__3__GeneralIO__makeInput();
#line 33
}
#line 33
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosb/SwitchToggleC.nc"
static inline error_t /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__Notify__enable(void )
#line 55
{
  /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__GeneralIO__makeInput();

  if (/*UserButtonC.SwitchToggleC*/SwitchToggleC__0__GeneralIO__get()) {
      /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__m_pinHigh = TRUE;
      return /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__GpioInterrupt__enableFallingEdge();
    }
  else 
#line 61
    {
      /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__m_pinHigh = FALSE;
      return /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__GpioInterrupt__enableRisingEdge();
    }
}

# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Notify.nc"
inline static error_t UserButtonP__NotifyLower__enable(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__Notify__enable();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosb/UserButtonP.nc"
static inline error_t UserButtonP__Notify__enable(void )
#line 59
{
  return UserButtonP__NotifyLower__enable();
}

# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Notify.nc"
inline static error_t LatinResP__UserButton__enable(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = UserButtonP__Notify__enable();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 446 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02b-RES/LatinRes/LatinResP.nc"
static inline error_t LatinResP__SoftwareInit__init(void )
#line 446
{
  LatinResP__UserButton__enable();
  LatinResP__resetNeighborList();
  LatinResP__state = ScanWait;
  return SUCCESS;
}

# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4441 {
#line 46
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void )
{
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(x);
}

#line 94
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void )
{
  * (volatile uint16_t * )386U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl();
}

# 36 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void ){
#line 36
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare();
#line 36
}
#line 36
# 42 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 947 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/LatinMac/LatinMacP.nc"
static inline error_t LatinMacP__SoftwareInit__init(void )
#line 947
{
  LatinMacP__change_state(S_SLEEP);
  /* atomic removed: atomic calls only */
#line 949
  LatinMacP__dataFrameToSend = (void *)0;
  /* atomic removed: atomic calls only */
#line 950
  LatinMacP__asn = 0;

  WDTCTL = 0x5A00 + 0x0080;
  WDTCTL = 0x5A00 + 0x0008 + 0x0004;
  return SUCCESS;
}

# 52 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )54U |= 0x01 << 2;
}

# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PinDebugC.PinADC2*/Msp430GpioC__6__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PinDebugC.PinADC2*/Msp430GpioC__6__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PinDebugC.PinADC2*/Msp430GpioC__6__HplGeneralIO__makeOutput();
}

# 35 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void PinDebugP__PinADC2__makeOutput(void ){
#line 35
  /*PinDebugC.PinADC2*/Msp430GpioC__6__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )54U |= 0x01 << 1;
}

# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PinDebugC.PinADC1*/Msp430GpioC__5__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PinDebugC.PinADC1*/Msp430GpioC__5__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PinDebugC.PinADC1*/Msp430GpioC__5__HplGeneralIO__makeOutput();
}

# 35 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void PinDebugP__PinADC1__makeOutput(void ){
#line 35
  /*PinDebugC.PinADC1*/Msp430GpioC__5__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )54U |= 0x01 << 0;
}

# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PinDebugC.PinADC0*/Msp430GpioC__4__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PinDebugC.PinADC0*/Msp430GpioC__4__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PinDebugC.PinADC0*/Msp430GpioC__4__HplGeneralIO__makeOutput();
}

# 35 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void PinDebugP__PinADC0__makeOutput(void ){
#line 35
  /*PinDebugC.PinADC0*/Msp430GpioC__4__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/home/david/workspace/tsch-project/tos/lib/debug/PinDebugP.nc"
static inline error_t PinDebugP__Reset__init(void )
#line 52
{
  PinDebugP__PinADC0__makeOutput();
  PinDebugP__PinADC1__makeOutput();
  PinDebugP__PinADC2__makeOutput();

  return SUCCESS;
}

# 44 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/RandomMlcgC.nc"
static inline error_t RandomMlcgC__Init__init(void )
#line 44
{
  /* atomic removed: atomic calls only */
#line 45
  RandomMlcgC__seed = (uint32_t )(TOS_NODE_ID + 1);

  return SUCCESS;
}

# 52 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )30U |= 0x01 << 5;
}

# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__13__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__13__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__13__HplGeneralIO__makeOutput();
}

# 35 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__VREN__makeOutput(void ){
#line 35
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__13__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )30U |= 0x01 << 6;
}

# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__11__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__11__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__11__HplGeneralIO__makeOutput();
}

# 35 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__makeOutput(void ){
#line 35
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__11__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )30U |= 0x01 << 2;
}

# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__HplGeneralIO__makeOutput();
}

# 35 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__makeOutput(void ){
#line 35
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__GeneralIO__makeOutput();
#line 35
}
#line 35
# 320 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__Init__init(void )
#line 320
{
  CC2420ControlP__CSN__makeOutput();
  CC2420ControlP__RSTN__makeOutput();
  CC2420ControlP__VREN__makeOutput();
  CC2420ControlP__m_channel = 26;
  return SUCCESS;
}

# 81 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/StateImplP.nc"
static inline error_t StateImplP__Init__init(void )
#line 81
{
  int i;

#line 83
  for (i = 0; i < 1U; i++) {
      StateImplP__state[i] = StateImplP__S_IDLE;
    }
  return SUCCESS;
}

# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void )
#line 45
{
  memset(/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ, /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY, sizeof /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ);
  return SUCCESS;
}

# 246 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/receive/CC2420ReceiveP.nc"
static inline error_t CC2420ReceiveP__Init__init(void )
#line 246
{
  CC2420ReceiveP__receptionBuffer = CC2420ReceiveP__Malloc__getFreePacketBuffer();
  if (CC2420ReceiveP__receptionBuffer == (void *)0) {
    }



  CC2420ReceiveP__receptionBuffer->creator = COMPONENT_CC2420RECEIVE;
  CC2420ReceiveP__receptionBuffer->owner = COMPONENT_CC2420RECEIVE;
  CC2420ReceiveP__receptionBuffer->payload = CC2420ReceiveP__receptionBuffer->packet;
  CC2420ReceiveP__receptionBuffer->length = 0;
  return SUCCESS;
}

# 50 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )30U &= ~(0x01 << 1);
}

# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__12__HplGeneralIO__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput();
#line 64
}
#line 64
# 41 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.SFDM*/Msp430GpioC__12__GeneralIO__makeInput(void )
#line 41
{
#line 41
  /*HplCC2420PinsC.SFDM*/Msp430GpioC__12__HplGeneralIO__makeInput();
}

# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__SFD__makeInput(void ){
#line 33
  /*HplCC2420PinsC.SFDM*/Msp430GpioC__12__GeneralIO__makeInput();
#line 33
}
#line 33


inline static void CC2420TransmitP__CSN__makeOutput(void ){
#line 35
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__GeneralIO__makeOutput();
#line 35
}
#line 35
# 50 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )34U &= ~(0x01 << 4);
}

# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__7__HplGeneralIO__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput();
#line 64
}
#line 64
# 41 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CCAM*/Msp430GpioC__7__GeneralIO__makeInput(void )
#line 41
{
#line 41
  /*HplCC2420PinsC.CCAM*/Msp430GpioC__7__HplGeneralIO__makeInput();
}

# 33 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CCA__makeInput(void ){
#line 33
  /*HplCC2420PinsC.CCAM*/Msp430GpioC__7__GeneralIO__makeInput();
#line 33
}
#line 33
# 311 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__Init__init(void )
#line 311
{
  CC2420TransmitP__CCA__makeInput();
  CC2420TransmitP__CSN__makeOutput();
  CC2420TransmitP__SFD__makeInput();
  return SUCCESS;
}

# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/openqueue/OpenQueueP.nc"
static inline error_t OpenQueueP__SoftwareInit__init(void )
#line 59
{
  uint8_t i;

#line 61
  for (i = 0; i < QUEUELENGTH; i++) {
      OpenQueueP__reset_entry(i);
    }
  return SUCCESS;
}

# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Init.nc"
inline static error_t RealMainP__SoftwareInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = OpenQueueP__SoftwareInit__init();
#line 51
  __nesc_result = ecombine(__nesc_result, CC2420TransmitP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, CC2420ReceiveP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, StateImplP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, CC2420ControlP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, RandomMlcgC__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, PinDebugP__Reset__init());
#line 51
  __nesc_result = ecombine(__nesc_result, LatinMacP__SoftwareInit__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, LatinResP__SoftwareInit__init());
#line 51
  __nesc_result = ecombine(__nesc_result, IDManagerP__SoftwareInit__init());
#line 51
  __nesc_result = ecombine(__nesc_result, appDATADevP__SoftwareInit__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, SerialP__Init__init());
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP__startDoneTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SerialP__startDoneTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 342 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialP.nc"
static inline error_t SerialP__SplitControl__start(void )
#line 342
{
  SerialP__startDoneTask__postTask();
  return SUCCESS;
}

# 83 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/SplitControl.nc"
inline static error_t PrintfP__SerialControl__start(void ){
#line 83
  unsigned char __nesc_result;
#line 83

#line 83
  __nesc_result = SerialP__SplitControl__start();
#line 83

#line 83
  return __nesc_result;
#line 83
}
#line 83
# 109 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/printf/PrintfP.nc"
static inline void PrintfP__MainBoot__booted(void )
#line 109
{
  PrintfP__SerialControl__start();
}

# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Boot.nc"
inline static void RealMainP__Boot__booted(void ){
#line 49
  PrintfP__MainBoot__booted();
#line 49
}
#line 49
# 206 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_disable_interrupt(void )
{
   __asm volatile ("dint");
   __asm volatile ("nop");}

# 52 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void )
#line 52
{
  return MSP430_POWER_LPM3;
}

# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/McuPowerOverride.nc"
inline static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = Msp430ClockP__McuPowerOverride__lowestState();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 66 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/McuSleepC.nc"
static inline mcu_power_t McuSleepC__getPowerState(void )
#line 66
{
  mcu_power_t pState = MSP430_POWER_LPM4;









  if ((((((
#line 69
  TA0CCTL0 & 0x0010 || 
  TA0CCTL1 & 0x0010) || 
  TA0CCTL2 & 0x0010) && (
  TA0CTL & (3 << 8)) == 2 << 8) || (
  ME1 & ((1 << 7) | (1 << 6)) && U0TCTL & 0x20)) || (
  ME2 & ((1 << 5) | (1 << 4)) && U1TCTL & 0x20))


   || (U0CTLnr & 0x01 && I2CTCTLnr & 0x20 && 
  I2CDCTLnr & 0x20 && U0CTLnr & 0x04 && U0CTLnr & 0x20)) {


    pState = MSP430_POWER_LPM1;
    }


  if (ADC12CTL0 & 0x0010) {
      if (ADC12CTL1 & (2 << 3)) {

          if (ADC12CTL1 & (1 << 3)) {
            pState = MSP430_POWER_LPM1;
            }
          else {
#line 91
            pState = MSP430_POWER_ACTIVE;
            }
        }
      else {
#line 92
        if (ADC12CTL1 & 0x0400 && (TA0CTL & (3 << 8)) == 2 << 8) {



            pState = MSP430_POWER_LPM1;
          }
        }
    }

  return pState;
}

# 194 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/msp430hardware.h"
static inline  mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 194
{
  return m1 < m2 ? m1 : m2;
}

# 104 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/McuSleepC.nc"
static inline void McuSleepC__computePowerState(void )
#line 104
{
  McuSleepC__powerState = mcombine(McuSleepC__getPowerState(), 
  McuSleepC__McuPowerOverride__lowestState());
}

static inline void McuSleepC__McuSleep__sleep(void )
#line 109
{
  uint16_t temp;

#line 111
  if (McuSleepC__dirty) {
      McuSleepC__computePowerState();
    }

  temp = McuSleepC__msp430PowerBits[McuSleepC__powerState] | 0x0008;
   __asm volatile ("bis  %0, r2" :  : "m"(temp));

   __asm volatile ("" :  :  : "memory");
  __nesc_disable_interrupt();
}

# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP__McuSleep__sleep(void ){
#line 59
  McuSleepC__McuSleep__sleep();
#line 59
}
#line 59
# 67 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP__popTask(void )
{
  if (SchedulerBasicP__m_head != SchedulerBasicP__NO_TASK) 
    {
      uint8_t id = SchedulerBasicP__m_head;

#line 72
      SchedulerBasicP__m_head = SchedulerBasicP__m_next[SchedulerBasicP__m_head];
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
        }
      SchedulerBasicP__m_next[id] = SchedulerBasicP__NO_TASK;
      return id;
    }
  else 
    {
      return SchedulerBasicP__NO_TASK;
    }
}

#line 138
static inline void SchedulerBasicP__Scheduler__taskLoop(void )
{
  for (; ; ) 
    {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          while ((nextTask = SchedulerBasicP__popTask()) == SchedulerBasicP__NO_TASK) 
            {
              SchedulerBasicP__McuSleep__sleep();
            }
        }
#line 150
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP__TaskBasic__runTask(nextTask);
    }
}

# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__taskLoop(void ){
#line 61
  SchedulerBasicP__Scheduler__taskLoop();
#line 61
}
#line 61
# 88 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId();
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 387 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFrameComm__dataReceived(uint8_t data)
#line 387
{
  SerialP__rx_state_machine(FALSE, data);
}

# 83 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC__SerialFrameComm__dataReceived(uint8_t data){
#line 83
  SerialP__SerialFrameComm__dataReceived(data);
#line 83
}
#line 83
# 384 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFrameComm__delimiterReceived(void )
#line 384
{
  SerialP__rx_state_machine(TRUE, 0);
}

# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC__SerialFrameComm__delimiterReceived(void ){
#line 74
  SerialP__SerialFrameComm__delimiterReceived();
#line 74
}
#line 74
# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC__UartStream__receivedByte(uint8_t data)
#line 61
{






  if (data == HDLC_FLAG_BYTE) {

      HdlcTranslateC__SerialFrameComm__delimiterReceived();
      return;
    }
  else {
#line 73
    if (data == HDLC_CTLESC_BYTE) {

        HdlcTranslateC__state.receiveEscape = 1;
        return;
      }
    else {
#line 78
      if (HdlcTranslateC__state.receiveEscape) {

          HdlcTranslateC__state.receiveEscape = 0;
          data = data ^ 0x20;
        }
      }
    }
#line 83
  HdlcTranslateC__SerialFrameComm__dataReceived(data);
}

# 221 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(uint8_t id, uint8_t byte)
#line 221
{
}

# 79 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receivedByte(uint8_t arg_0x40877770, uint8_t byte){
#line 79
  switch (arg_0x40877770) {
#line 79
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 79
      HdlcTranslateC__UartStream__receivedByte(byte);
#line 79
      break;
#line 79
    default:
#line 79
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(arg_0x40877770, byte);
#line 79
      break;
#line 79
    }
#line 79
}
#line 79
# 116 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC__UartStream__receiveDone(uint8_t *buf, uint16_t len, error_t error)
#line 116
{
}

# 222 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error)
#line 222
{
}

# 99 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receiveDone(uint8_t arg_0x40877770, uint8_t * buf, uint16_t len, error_t error){
#line 99
  switch (arg_0x40877770) {
#line 99
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 99
      HdlcTranslateC__UartStream__receiveDone(buf, len, error);
#line 99
      break;
#line 99
    default:
#line 99
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(arg_0x40877770, buf, len, error);
#line 99
      break;
#line 99
    }
#line 99
}
#line 99
# 134 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__rxDone(uint8_t id, uint8_t data)
#line 134
{
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf) {
      /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf[/*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_pos++] = data;
      if (/*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_pos >= /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_len) {
          uint8_t *buf = /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf;

#line 139
          /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf = (void *)0;
          /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receiveDone(id, buf, /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_len, SUCCESS);
        }
    }
  else 
#line 142
    {
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receivedByte(id, data);
    }
}

# 65 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data)
#line 65
{
}

# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(uint8_t arg_0x40a89010, uint8_t data){
#line 54
  switch (arg_0x40a89010) {
#line 54
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 54
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__rxDone(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID, data);
#line 54
      break;
#line 54
    default:
#line 54
      /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(arg_0x40a89010, data);
#line 54
      break;
#line 54
    }
#line 54
}
#line 54
# 80 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ArbiterInfo.nc"
inline static bool /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse(void ){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse();
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data)
#line 54
{
  if (/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(), data);
    }
}

# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart1P__Interrupts__rxDone(uint8_t data){
#line 54
  /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(data);
#line 54
}
#line 54
# 391 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialP.nc"
static inline bool SerialP__valid_rx_proto(uint8_t proto)
#line 391
{
  switch (proto) {
      case SERIAL_PROTO_PACKET_ACK: 
        return TRUE;
      case SERIAL_PROTO_ACK: 
        case SERIAL_PROTO_PACKET_NOACK: 
          default: 
            return FALSE;
    }
}

# 192 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__lockCurrentBuffer(void )
#line 192
{
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufOneLocked = 1;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufZeroLocked = 1;
    }
}

#line 188
static inline bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__isCurrentBufferLocked(void )
#line 188
{
  return /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which ? /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufOneLocked : /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufZeroLocked;
}

#line 215
static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket(void )
#line 215
{
  error_t result = SUCCESS;

  /* atomic removed: atomic calls only */
#line 217
  {
    if (!/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__isCurrentBufferLocked()) {


        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__lockCurrentBuffer();
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_BEGIN;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex = 0;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType = TOS_SERIAL_UNKNOWN_ID;
      }
    else {
        result = EBUSY;
      }
  }
  return result;
}

# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/ReceiveBytePacket.nc"
inline static error_t SerialP__ReceiveBytePacket__startPacket(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 309 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialP.nc"
static __inline uint16_t SerialP__rx_current_crc(void )
#line 309
{
  uint16_t crc;
  uint8_t tmp = SerialP__rxBuf.writePtr;

#line 312
  tmp = tmp == 0 ? SerialP__RX_DATA_BUFFER_SIZE : tmp - 1;
  crc = SerialP__rxBuf.buf[tmp] & 0x00ff;
  crc = (crc << 8) & 0xFF00;
  tmp = tmp == 0 ? SerialP__RX_DATA_BUFFER_SIZE : tmp - 1;
  crc |= SerialP__rxBuf.buf[tmp] & 0x00FF;
  return crc;
}

# 69 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/ReceiveBytePacket.nc"
inline static void SerialP__ReceiveBytePacket__endPacket(error_t result){
#line 69
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(result);
#line 69
}
#line 69
# 210 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBufferSwap(void )
#line 210
{
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which ? 0 : 1;
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer = (uint8_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messagePtrs[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which];
}

# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 232 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialP.nc"
static __inline bool SerialP__ack_queue_is_full(void )
#line 232
{
  uint8_t tmp;
#line 233
  uint8_t tmp2;

  /* atomic removed: atomic calls only */
#line 234
  {
    tmp = SerialP__ackQ.writePtr;
    tmp2 = SerialP__ackQ.readPtr;
  }
  if (++tmp > SerialP__ACK_QUEUE_SIZE) {
#line 238
    tmp = 0;
    }
#line 239
  return tmp == tmp2;
}







static __inline void SerialP__ack_queue_push(uint8_t token)
#line 248
{
  if (!SerialP__ack_queue_is_full()) {
      /* atomic removed: atomic calls only */
#line 250
      {
        SerialP__ackQ.buf[SerialP__ackQ.writePtr] = token;
        if (++ SerialP__ackQ.writePtr > SerialP__ACK_QUEUE_SIZE) {
#line 252
          SerialP__ackQ.writePtr = 0;
          }
      }
#line 254
      SerialP__MaybeScheduleTx();
    }
}

# 233 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(uint8_t b)
#line 233
{
  /* atomic removed: atomic calls only */
#line 234
  {
    switch (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state) {
        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_BEGIN: 
          /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_DATA;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(b);
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType = b;
        break;

        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_DATA: 
          if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex < sizeof(message_t )) {
              /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex] = b;
              /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex++;
            }
          else {
            }




        break;

        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_IDLE: 
          default: 
#line 255
            ;
      }
  }
}

# 58 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/ReceiveBytePacket.nc"
inline static void SerialP__ReceiveBytePacket__byteReceived(uint8_t data){
#line 58
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(data);
#line 58
}
#line 58
# 299 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialP.nc"
static __inline uint8_t SerialP__rx_buffer_top(void )
#line 299
{
  uint8_t tmp = SerialP__rxBuf.buf[SerialP__rxBuf.readPtr];

#line 301
  return tmp;
}

#line 303
static __inline uint8_t SerialP__rx_buffer_pop(void )
#line 303
{
  uint8_t tmp = SerialP__rxBuf.buf[SerialP__rxBuf.readPtr];

#line 305
  if (++ SerialP__rxBuf.readPtr > SerialP__RX_DATA_BUFFER_SIZE) {
#line 305
    SerialP__rxBuf.readPtr = 0;
    }
#line 306
  return tmp;
}

#line 295
static __inline void SerialP__rx_buffer_push(uint8_t data)
#line 295
{
  SerialP__rxBuf.buf[SerialP__rxBuf.writePtr] = data;
  if (++ SerialP__rxBuf.writePtr > SerialP__RX_DATA_BUFFER_SIZE) {
#line 297
    SerialP__rxBuf.writePtr = 0;
    }
}

# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC__SerialFrameComm__resetReceive(void )
#line 55
{
  HdlcTranslateC__state.receiveEscape = 0;
}

# 68 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialFrameComm.nc"
inline static void SerialP__SerialFrameComm__resetReceive(void ){
#line 68
  HdlcTranslateC__SerialFrameComm__resetReceive();
#line 68
}
#line 68
# 220 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__sendDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error)
#line 220
{
}

# 57 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__sendDone(uint8_t arg_0x40877770, uint8_t * buf, uint16_t len, error_t error){
#line 57
  switch (arg_0x40877770) {
#line 57
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 57
      HdlcTranslateC__UartStream__sendDone(buf, len, error);
#line 57
      break;
#line 57
    default:
#line 57
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__sendDone(arg_0x40877770, buf, len, error);
#line 57
      break;
#line 57
    }
#line 57
}
#line 57
# 384 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__tx(uint8_t data)
#line 384
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 385
    HplMsp430Usart1P__U1TXBUF = data;
#line 385
    __nesc_atomic_end(__nesc_atomic); }
}

# 224 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__tx(uint8_t data){
#line 224
  HplMsp430Usart1P__Usart__tx(data);
#line 224
}
#line 224
# 162 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__txDone(uint8_t id)
#line 162
{
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__current_owner != id) {
      uint8_t *buf = /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf;

#line 165
      /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf = (void *)0;
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__sendDone(id, buf, /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len, FAIL);
    }
  else {
#line 168
    if (/*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_pos < /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len) {
        /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__tx(/*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf[/*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_pos++]);
      }
    else {
        uint8_t *buf = /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf;

#line 173
        /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf = (void *)0;
        /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__sendDone(id, buf, /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len, SUCCESS);
      }
    }
}

# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id)
#line 64
{
}

# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(uint8_t arg_0x40a89010){
#line 49
  switch (arg_0x40a89010) {
#line 49
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 49
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__txDone(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(arg_0x40a89010);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void )
#line 49
{
  if (/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId());
    }
}

# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart1P__Interrupts__txDone(void ){
#line 49
  /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone();
#line 49
}
#line 49
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialFrameComm.nc"
inline static error_t SerialP__SerialFrameComm__putData(uint8_t data){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = HdlcTranslateC__SerialFrameComm__putData(data);
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 513 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialP.nc"
static inline error_t SerialP__SendBytePacket__completeSend(void )
#line 513
{
  bool ret = FAIL;

  /* atomic removed: atomic calls only */
#line 515
  {
    SerialP__txBuf[SerialP__TX_DATA_INDEX].state = SerialP__BUFFER_COMPLETE;
    ret = SUCCESS;
  }
  return ret;
}

# 60 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SendBytePacket.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__completeSend(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = SerialP__SendBytePacket__completeSend();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 167 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte(void )
#line 167
{
  uint8_t b;
  uint8_t indx;

  /* atomic removed: atomic calls only */
#line 170
  {
    b = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex];
    /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex++;
    indx = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex;
  }
  if (indx > /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendLen) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__completeSend();
      return 0;
    }
  else {
      return b;
    }
}

# 70 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SendBytePacket.nc"
inline static uint8_t SerialP__SendBytePacket__nextByte(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 642 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFrameComm__putDone(void )
#line 642
{
  {
    error_t txResult = SUCCESS;

    switch (SerialP__txState) {

        case SerialP__TXSTATE_PROTO: 

          txResult = SerialP__SerialFrameComm__putData(SerialP__txProto);

        SerialP__txState = SerialP__TXSTATE_INFO;



        SerialP__txCRC = crcByte(SerialP__txCRC, SerialP__txProto);
        break;

        case SerialP__TXSTATE_SEQNO: 
          txResult = SerialP__SerialFrameComm__putData(SerialP__txSeqno);
        SerialP__txState = SerialP__TXSTATE_INFO;
        SerialP__txCRC = crcByte(SerialP__txCRC, SerialP__txSeqno);
        break;

        case SerialP__TXSTATE_INFO: /* atomic removed: atomic calls only */
          {
            txResult = SerialP__SerialFrameComm__putData(SerialP__txBuf[SerialP__txIndex].buf);
            SerialP__txCRC = crcByte(SerialP__txCRC, SerialP__txBuf[SerialP__txIndex].buf);
            ++SerialP__txByteCnt;

            if (SerialP__txIndex == SerialP__TX_DATA_INDEX) {
                uint8_t nextByte;

#line 673
                nextByte = SerialP__SendBytePacket__nextByte();
                if (SerialP__txBuf[SerialP__txIndex].state == SerialP__BUFFER_COMPLETE || SerialP__txByteCnt >= SerialP__SERIAL_MTU) {
                    SerialP__txState = SerialP__TXSTATE_FCS1;
                  }
                else {
                    SerialP__txBuf[SerialP__txIndex].buf = nextByte;
                  }
              }
            else {
                SerialP__txState = SerialP__TXSTATE_FCS1;
              }
          }
        break;

        case SerialP__TXSTATE_FCS1: 
          txResult = SerialP__SerialFrameComm__putData(SerialP__txCRC & 0xff);
        SerialP__txState = SerialP__TXSTATE_FCS2;
        break;

        case SerialP__TXSTATE_FCS2: 
          txResult = SerialP__SerialFrameComm__putData((SerialP__txCRC >> 8) & 0xff);
        SerialP__txState = SerialP__TXSTATE_ENDFLAG;
        break;

        case SerialP__TXSTATE_ENDFLAG: 
          txResult = SerialP__SerialFrameComm__putDelimiter();
        SerialP__txState = SerialP__TXSTATE_ENDWAIT;
        break;

        case SerialP__TXSTATE_ENDWAIT: 
          SerialP__txState = SerialP__TXSTATE_FINISH;
        case SerialP__TXSTATE_FINISH: 
          SerialP__MaybeScheduleTx();
        break;
        case SerialP__TXSTATE_ERROR: 
          default: 
            txResult = FAIL;
        break;
      }

    if (txResult != SUCCESS) {
        SerialP__txState = SerialP__TXSTATE_ERROR;
        SerialP__MaybeScheduleTx();
      }
  }
}

# 89 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC__SerialFrameComm__putDone(void ){
#line 89
  SerialP__SerialFrameComm__putDone();
#line 89
}
#line 89
# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/BigQueueC.nc"
static inline uint16_t /*PrintfC.QueueC*/BigQueueC__0__Queue__maxSize(void )
#line 61
{
  return 1000;
}

#line 97
static inline error_t /*PrintfC.QueueC*/BigQueueC__0__Queue__enqueue(/*PrintfC.QueueC*/BigQueueC__0__queue_t newVal)
#line 97
{
  if (/*PrintfC.QueueC*/BigQueueC__0__Queue__size() < /*PrintfC.QueueC*/BigQueueC__0__Queue__maxSize()) {
      ;
      /*PrintfC.QueueC*/BigQueueC__0__queue[/*PrintfC.QueueC*/BigQueueC__0__tail] = newVal;
      /*PrintfC.QueueC*/BigQueueC__0__tail++;
      /*PrintfC.QueueC*/BigQueueC__0__tail %= 1000;
      /*PrintfC.QueueC*/BigQueueC__0__size++;
      /*PrintfC.QueueC*/BigQueueC__0__printQueue();
      return SUCCESS;
    }
  else {
      return FAIL;
    }
}

# 90 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/BigQueue.nc"
inline static error_t PrintfP__Queue__enqueue(PrintfP__Queue__t newVal){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = /*PrintfC.QueueC*/BigQueueC__0__Queue__enqueue(newVal);
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 79 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__InterruptFIFOP__fired(void )
#line 79
{
  if (CC2420ReceiveP__state == CC2420ReceiveP__S_STARTED) {
      CC2420ReceiveP__beginReceive();
    }
  else 
#line 82
    {
      CC2420ReceiveP__missed_packets++;
    }
}

# 57 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioInterrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__Interrupt__fired(void ){
#line 57
  CC2420ReceiveP__InterruptFIFOP__fired();
#line 57
}
#line 57
# 66 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__HplInterrupt__fired(void )
#line 66
{
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__HplInterrupt__clear();
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__Interrupt__fired();
}

# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port10__fired(void ){
#line 61
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__2__HplInterrupt__fired();
#line 61
}
#line 61
# 92 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port11__clear(void )
#line 92
{
#line 92
  P1IFG &= ~(1 << 1);
}

#line 68
static inline void HplMsp430InterruptP__Port11__default__fired(void )
#line 68
{
#line 68
  HplMsp430InterruptP__Port11__clear();
}

# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port11__fired(void ){
#line 61
  HplMsp430InterruptP__Port11__default__fired();
#line 61
}
#line 61
# 93 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port12__clear(void )
#line 93
{
#line 93
  P1IFG &= ~(1 << 2);
}

#line 69
static inline void HplMsp430InterruptP__Port12__default__fired(void )
#line 69
{
#line 69
  HplMsp430InterruptP__Port12__clear();
}

# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port12__fired(void ){
#line 61
  HplMsp430InterruptP__Port12__default__fired();
#line 61
}
#line 61
# 94 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port13__clear(void )
#line 94
{
#line 94
  P1IFG &= ~(1 << 3);
}

#line 70
static inline void HplMsp430InterruptP__Port13__default__fired(void )
#line 70
{
#line 70
  HplMsp430InterruptP__Port13__clear();
}

# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port13__fired(void ){
#line 61
  HplMsp430InterruptP__Port13__default__fired();
#line 61
}
#line 61
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420DriversP__taskStartOscillatorDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420DriversP__taskStartOscillatorDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 76 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/CC2420DriversP.nc"
static inline void CC2420DriversP__CC2420Config__startOscillatorDone(void )
#line 76
{
  CC2420DriversP__taskStartOscillatorDone__postTask();
}

# 5 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Config.nc"
inline static void CC2420ControlP__CC2420Config__startOscillatorDone(void ){
#line 5
  CC2420DriversP__CC2420Config__startOscillatorDone();
#line 5
}
#line 5
# 110 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__OscResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ControlC.OscSpiC*/CC2420SpiC__1__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 29 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__set(void ){
#line 29
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__GeneralIO__set();
#line 29
}
#line 29
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__IOCFG1__write(uint16_t data){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420SpiP__Reg__write(CC2420_IOCFG1, data);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 30 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__clr(void ){
#line 30
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__8__GeneralIO__clr();
#line 30
}
#line 30
# 50 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ControlP__InterruptCCA__disable(void ){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__Interrupt__disable();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 153 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__InterruptCCA__fired(void )
#line 153
{
  CC2420ControlP__m_state = CC2420ControlP__S_XOSC_STARTED;
  CC2420ControlP__InterruptCCA__disable();
  CC2420ControlP__CSN__clr();
  CC2420ControlP__IOCFG1__write(0);
  CC2420ControlP__CSN__set();
  CC2420ControlP__OscResource__release();
  CC2420ControlP__CC2420Config__startOscillatorDone();
}

# 57 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioInterrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__Interrupt__fired(void ){
#line 57
  CC2420ControlP__InterruptCCA__fired();
#line 57
}
#line 57
# 66 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__HplInterrupt__fired(void )
#line 66
{
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__HplInterrupt__clear();
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__Interrupt__fired();
}

# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port14__fired(void ){
#line 61
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__1__HplInterrupt__fired();
#line 61
}
#line 61
# 96 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port15__clear(void )
#line 96
{
#line 96
  P1IFG &= ~(1 << 5);
}

#line 72
static inline void HplMsp430InterruptP__Port15__default__fired(void )
#line 72
{
#line 72
  HplMsp430InterruptP__Port15__clear();
}

# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port15__fired(void ){
#line 61
  HplMsp430InterruptP__Port15__default__fired();
#line 61
}
#line 61
# 97 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port16__clear(void )
#line 97
{
#line 97
  P1IFG &= ~(1 << 6);
}

#line 73
static inline void HplMsp430InterruptP__Port16__default__fired(void )
#line 73
{
#line 73
  HplMsp430InterruptP__Port16__clear();
}

# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port16__fired(void ){
#line 61
  HplMsp430InterruptP__Port16__default__fired();
#line 61
}
#line 61
# 98 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port17__clear(void )
#line 98
{
#line 98
  P1IFG &= ~(1 << 7);
}

#line 74
static inline void HplMsp430InterruptP__Port17__default__fired(void )
#line 74
{
#line 74
  HplMsp430InterruptP__Port17__clear();
}

# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port17__fired(void ){
#line 61
  HplMsp430InterruptP__Port17__default__fired();
#line 61
}
#line 61
# 195 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port20__clear(void )
#line 195
{
#line 195
  P2IFG &= ~(1 << 0);
}

#line 171
static inline void HplMsp430InterruptP__Port20__default__fired(void )
#line 171
{
#line 171
  HplMsp430InterruptP__Port20__clear();
}

# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port20__fired(void ){
#line 61
  HplMsp430InterruptP__Port20__default__fired();
#line 61
}
#line 61
# 196 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port21__clear(void )
#line 196
{
#line 196
  P2IFG &= ~(1 << 1);
}

#line 172
static inline void HplMsp430InterruptP__Port21__default__fired(void )
#line 172
{
#line 172
  HplMsp430InterruptP__Port21__clear();
}

# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port21__fired(void ){
#line 61
  HplMsp430InterruptP__Port21__default__fired();
#line 61
}
#line 61
# 197 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port22__clear(void )
#line 197
{
#line 197
  P2IFG &= ~(1 << 2);
}

#line 173
static inline void HplMsp430InterruptP__Port22__default__fired(void )
#line 173
{
#line 173
  HplMsp430InterruptP__Port22__clear();
}

# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port22__fired(void ){
#line 61
  HplMsp430InterruptP__Port22__default__fired();
#line 61
}
#line 61
# 198 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port23__clear(void )
#line 198
{
#line 198
  P2IFG &= ~(1 << 3);
}

#line 174
static inline void HplMsp430InterruptP__Port23__default__fired(void )
#line 174
{
#line 174
  HplMsp430InterruptP__Port23__clear();
}

# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port23__fired(void ){
#line 61
  HplMsp430InterruptP__Port23__default__fired();
#line 61
}
#line 61
# 199 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port24__clear(void )
#line 199
{
#line 199
  P2IFG &= ~(1 << 4);
}

#line 175
static inline void HplMsp430InterruptP__Port24__default__fired(void )
#line 175
{
#line 175
  HplMsp430InterruptP__Port24__clear();
}

# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port24__fired(void ){
#line 61
  HplMsp430InterruptP__Port24__default__fired();
#line 61
}
#line 61
# 200 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port25__clear(void )
#line 200
{
#line 200
  P2IFG &= ~(1 << 5);
}

#line 176
static inline void HplMsp430InterruptP__Port25__default__fired(void )
#line 176
{
#line 176
  HplMsp430InterruptP__Port25__clear();
}

# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port25__fired(void ){
#line 61
  HplMsp430InterruptP__Port25__default__fired();
#line 61
}
#line 61
# 201 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port26__clear(void )
#line 201
{
#line 201
  P2IFG &= ~(1 << 6);
}

#line 177
static inline void HplMsp430InterruptP__Port26__default__fired(void )
#line 177
{
#line 177
  HplMsp430InterruptP__Port26__clear();
}

# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port26__fired(void ){
#line 61
  HplMsp430InterruptP__Port26__default__fired();
#line 61
}
#line 61
# 56 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
inline static error_t /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__sendEvent__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*UserButtonC.SwitchToggleC*/SwitchToggleC__0__sendEvent);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 202 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port27__clear(void )
#line 202
{
#line 202
  P2IFG &= ~(1 << 7);
}

# 41 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port27__clear();
#line 41
}
#line 41
# 194 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port27__disable(void )
#line 194
{
#line 194
  P2IE &= ~(1 << 7);
}

# 36 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__HplInterrupt__disable(void ){
#line 36
  HplMsp430InterruptP__Port27__disable();
#line 36
}
#line 36
# 58 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__Interrupt__disable(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__HplInterrupt__disable();
    /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__HplInterrupt__clear();
  }
  return SUCCESS;
}

# 50 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioInterrupt.nc"
inline static error_t /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__GpioInterrupt__disable(void ){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__Interrupt__disable();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 71 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosb/SwitchToggleC.nc"
static inline void /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__GpioInterrupt__fired(void )
#line 71
{
  /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__GpioInterrupt__disable();

  /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__m_pinHigh = !/*UserButtonC.SwitchToggleC*/SwitchToggleC__0__m_pinHigh;

  /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__sendEvent__postTask();
}

# 57 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/GpioInterrupt.nc"
inline static void /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__Interrupt__fired(void ){
#line 57
  /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__GpioInterrupt__fired();
#line 57
}
#line 57
# 66 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__HplInterrupt__fired(void )
#line 66
{
  /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__HplInterrupt__clear();
  /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__Interrupt__fired();
}

# 61 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port27__fired(void ){
#line 61
  /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__HplInterrupt__fired();
#line 61
}
#line 61
# 88 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__userId(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId();
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 195 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartInterrupts__rxDone(uint8_t data)
#line 195
{
}

# 65 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__rxDone(uint8_t id, uint8_t data)
#line 65
{
}

# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__rxDone(uint8_t arg_0x40a89010, uint8_t data){
#line 54
  switch (arg_0x40a89010) {
#line 54
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 54
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartInterrupts__rxDone(data);
#line 54
      break;
#line 54
    default:
#line 54
      /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__rxDone(arg_0x40a89010, data);
#line 54
      break;
#line 54
    }
#line 54
}
#line 54
# 80 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/ArbiterInfo.nc"
inline static bool /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__inUse(void ){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse();
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__rxDone(uint8_t data)
#line 54
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__rxDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__userId(), data);
    }
}

# 54 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart0P__Interrupts__rxDone(uint8_t data){
#line 54
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__rxDone(data);
#line 54
}
#line 54
# 55 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static inline bool HplMsp430I2C0P__HplI2C__isI2C(void )
#line 55
{
  /* atomic removed: atomic calls only */
#line 56
  {
    unsigned char __nesc_temp = 
#line 56
    HplMsp430I2C0P__U0CTL & 0x20 && HplMsp430I2C0P__U0CTL & 0x04 && HplMsp430I2C0P__U0CTL & 0x01;

#line 56
    return __nesc_temp;
  }
}

# 6 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static bool HplMsp430Usart0P__HplI2C__isI2C(void ){
#line 6
  unsigned char __nesc_result;
#line 6

#line 6
  __nesc_result = HplMsp430I2C0P__HplI2C__isI2C();
#line 6

#line 6
  return __nesc_result;
#line 6
}
#line 6
# 66 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__I2CInterrupts__default__fired(uint8_t id)
#line 66
{
}

# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__I2CInterrupts__fired(uint8_t arg_0x40a88010){
#line 39
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__I2CInterrupts__default__fired(arg_0x40a88010);
#line 39
}
#line 39
# 59 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawI2CInterrupts__fired(void )
#line 59
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__I2CInterrupts__fired(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__userId());
    }
}

# 39 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
inline static void HplMsp430Usart0P__I2CInterrupts__fired(void ){
#line 39
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawI2CInterrupts__fired();
#line 39
}
#line 39
# 194 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartInterrupts__txDone(void )
#line 194
{
}

# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__txDone(uint8_t id)
#line 64
{
}

# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__txDone(uint8_t arg_0x40a89010){
#line 49
  switch (arg_0x40a89010) {
#line 49
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 49
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartInterrupts__txDone();
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__txDone(arg_0x40a89010);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__txDone(void )
#line 49
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__txDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__userId());
    }
}

# 49 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart0P__Interrupts__txDone(void ){
#line 49
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__txDone();
#line 49
}
#line 49
# 98 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaControlP.nc"
static inline void Msp430DmaControlP__HplChannel2__transferDone(error_t error)
#line 98
{
}

# 188 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel2__transferDone(error_t error)
#line 188
{
}

# 96 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannel.nc"
inline static void /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__Channel__transferDone(error_t success){
#line 96
  /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel2__transferDone(success);
#line 96
}
#line 96
# 143 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannelP.nc"
static inline void /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__HplChannel__transferDone(error_t error)
#line 143
{
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__Channel__transferDone(error);
}

# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
inline static void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__transferDone(error_t success){
#line 74
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__HplChannel__transferDone(success);
#line 74
  Msp430DmaControlP__HplChannel2__transferDone(success);
#line 74
}
#line 74
# 82 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaXP.nc"
static inline void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__Interrupt__fired(void )
#line 82
{
  error_t error = * (volatile uint16_t *)496U & 0x0002 ? FAIL : SUCCESS;

#line 84
  if (* (volatile uint16_t *)496U & 0x0008) {
      * (volatile uint16_t *)496U &= ~0x0008;
      * (volatile uint16_t *)496U &= ~0x0002;
      /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__transferDone(error);
    }
}

# 97 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaControlP.nc"
static inline void Msp430DmaControlP__HplChannel1__transferDone(error_t error)
#line 97
{
}

# 184 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel1__transferDone(error_t error)
#line 184
{
  /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone(error);
}

# 96 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannel.nc"
inline static void /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__Channel__transferDone(error_t success){
#line 96
  /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel1__transferDone(success);
#line 96
}
#line 96
# 143 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannelP.nc"
static inline void /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__HplChannel__transferDone(error_t error)
#line 143
{
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__Channel__transferDone(error);
}

# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
inline static void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__transferDone(error_t success){
#line 74
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__HplChannel__transferDone(success);
#line 74
  Msp430DmaControlP__HplChannel1__transferDone(success);
#line 74
}
#line 74
# 82 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaXP.nc"
static inline void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__Interrupt__fired(void )
#line 82
{
  error_t error = * (volatile uint16_t *)488U & 0x0002 ? FAIL : SUCCESS;

#line 84
  if (* (volatile uint16_t *)488U & 0x0008) {
      * (volatile uint16_t *)488U &= ~0x0008;
      * (volatile uint16_t *)488U &= ~0x0002;
      /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__transferDone(error);
    }
}

# 96 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaControlP.nc"
static inline void Msp430DmaControlP__HplChannel0__transferDone(error_t error)
#line 96
{
}

# 147 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannelP.nc"
static inline void /*Msp430DmaC.Channel0P*/Msp430DmaChannelP__0__Channel__default__transferDone(error_t error)
#line 147
{
}

# 96 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannel.nc"
inline static void /*Msp430DmaC.Channel0P*/Msp430DmaChannelP__0__Channel__transferDone(error_t success){
#line 96
  /*Msp430DmaC.Channel0P*/Msp430DmaChannelP__0__Channel__default__transferDone(success);
#line 96
}
#line 96
# 143 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/Msp430DmaChannelP.nc"
static inline void /*Msp430DmaC.Channel0P*/Msp430DmaChannelP__0__HplChannel__transferDone(error_t error)
#line 143
{
  /*Msp430DmaC.Channel0P*/Msp430DmaChannelP__0__Channel__transferDone(error);
}

# 74 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
inline static void /*HplMsp430DmaC.Dma0*/HplMsp430DmaXP__0__DMA__transferDone(error_t success){
#line 74
  /*Msp430DmaC.Channel0P*/Msp430DmaChannelP__0__HplChannel__transferDone(success);
#line 74
  Msp430DmaControlP__HplChannel0__transferDone(success);
#line 74
}
#line 74
# 82 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaXP.nc"
static inline void /*HplMsp430DmaC.Dma0*/HplMsp430DmaXP__0__Interrupt__fired(void )
#line 82
{
  error_t error = * (volatile uint16_t *)480U & 0x0002 ? FAIL : SUCCESS;

#line 84
  if (* (volatile uint16_t *)480U & 0x0008) {
      * (volatile uint16_t *)480U &= ~0x0008;
      * (volatile uint16_t *)480U &= ~0x0002;
      /*HplMsp430DmaC.Dma0*/HplMsp430DmaXP__0__DMA__transferDone(error);
    }
}

# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaInterrupt.nc"
inline static void HplMsp430DmaP__Interrupt__fired(void ){
#line 28
  /*HplMsp430DmaC.Dma0*/HplMsp430DmaXP__0__Interrupt__fired();
#line 28
  /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__Interrupt__fired();
#line 28
  /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__Interrupt__fired();
#line 28
}
#line 28
# 226 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/msp430hardware.h"
  __nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = (({
#line 228
    uint16_t __x;

#line 228
     __asm volatile ("mov	r2, %0" : "=r"((uint16_t )__x));__x;
  }
  )
#line 228
   & 0x0008) != 0;

#line 229
  __nesc_disable_interrupt();
   __asm volatile ("" :  :  : "memory");
  return result;
}

  void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
   __asm volatile ("" :  :  : "memory");
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

# 11 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(12)))  void sig_TIMERA0_VECTOR(void )
#line 11
{
#line 11
  Msp430TimerCommonP__VectorTimerA0__fired();
}

# 169 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired();
    }
}

#line 169
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired();
    }
}

#line 169
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired();
    }
}

# 12 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(10)))  void sig_TIMERA1_VECTOR(void )
#line 12
{
#line 12
  Msp430TimerCommonP__VectorTimerA1__fired();
}

#line 13
__attribute((wakeup)) __attribute((interrupt(26)))  void sig_TIMERB0_VECTOR(void )
#line 13
{
#line 13
  Msp430TimerCommonP__VectorTimerB0__fired();
}

# 135 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n)
{
}

# 28 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(uint8_t arg_0x4062c4f0){
#line 28
  switch (arg_0x4062c4f0) {
#line 28
    case 0:
#line 28
      /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired();
#line 28
      break;
#line 28
    case 1:
#line 28
      /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired();
#line 28
      break;
#line 28
    case 2:
#line 28
      /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired();
#line 28
      break;
#line 28
    case 3:
#line 28
      /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired();
#line 28
      break;
#line 28
    case 4:
#line 28
      /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired();
#line 28
      break;
#line 28
    case 5:
#line 28
      /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired();
#line 28
      break;
#line 28
    case 6:
#line 28
      /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired();
#line 28
      break;
#line 28
    case 7:
#line 28
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired();
#line 28
      break;
#line 28
    default:
#line 28
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(arg_0x4062c4f0);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 159 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/SchedulerBasicP.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 161
    {
#line 161
      {
        unsigned char __nesc_temp = 
#line 161
        SchedulerBasicP__pushTask(id) ? SUCCESS : EBUSY;

        {
#line 161
          __nesc_atomic_end(__nesc_atomic); 
#line 161
          return __nesc_temp;
        }
      }
    }
#line 164
    __nesc_atomic_end(__nesc_atomic); }
}

# 96 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type now = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
#line 98
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type expires;
#line 98
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type remaining;




  expires = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;


  remaining = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type )(expires - now);


  if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 <= now) 
    {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY) 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 = now + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = remaining - /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      remaining = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
    }
  else 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 += /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = 0;
    }
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt((/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type )now << 5, 
  (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type )remaining << 5);
}

# 69 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformCounterC.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void )
{
  /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type rv = 0;

#line 72
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type high = /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;
      /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type low = /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get();

#line 76
      if (/*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get();
        }
      {
        /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type high_to = high;
        /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type low_to = low >> /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT;

#line 90
        rv = (high_to << /*CounterMilli32C.Transform*/TransformCounterC__0__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 92
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 51 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void )
{




  if (1) {
      /* atomic removed: atomic calls only */
#line 58
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t * )400U;

#line 61
        do {
#line 61
            t0 = t1;
#line 61
            t1 = * (volatile uint16_t * )400U;
          }
        while (
#line 61
        t0 != t1);
        {
          unsigned int __nesc_temp = 
#line 62
          t1;

#line 62
          return __nesc_temp;
        }
      }
    }
  else 
#line 65
    {
      return * (volatile uint16_t * )400U;
    }
}

# 958 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/LatinMac/LatinMacP.nc"
static timervalue_t LatinMacP__GlobalTime__getGlobalSlotOffset(void )
#line 958
{



  return LatinMacP__SlotAlarm__getNow() - (LatinMacP__SlotAlarm__getAlarm() - (uint32_t )SLOT_TIME);
}

# 69 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformCounterC.nc"
static /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__get(void )
{
  /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type rv = 0;

#line 72
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*Counter32khz32C.Transform*/TransformCounterC__1__upper_count_type high = /*Counter32khz32C.Transform*/TransformCounterC__1__m_upper;
      /*Counter32khz32C.Transform*/TransformCounterC__1__from_size_type low = /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__get();

#line 76
      if (/*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__get();
        }
      {
        /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type high_to = high;
        /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type low_to = low >> /*Counter32khz32C.Transform*/TransformCounterC__1__LOW_SHIFT_RIGHT;

#line 90
        rv = (high_to << /*Counter32khz32C.Transform*/TransformCounterC__1__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 92
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 149 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
static error_t CC2420SpiP__Resource__release(uint8_t id)
#line 149
{
  uint8_t i;

#line 151
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 151
    {
      if (CC2420SpiP__m_holder != id) {
          {
            unsigned char __nesc_temp = 
#line 153
            FAIL;

            {
#line 153
              __nesc_atomic_end(__nesc_atomic); 
#line 153
              return __nesc_temp;
            }
          }
        }
#line 156
      CC2420SpiP__m_holder = CC2420SpiP__NO_HOLDER;
      if (!CC2420SpiP__m_requests) {
          CC2420SpiP__WorkingState__toIdle();
          CC2420SpiP__attemptRelease();
        }
      else {
          for (i = CC2420SpiP__m_holder + 1; ; i++) {
              i %= CC2420SpiP__RESOURCE_COUNT;

              if (CC2420SpiP__m_requests & (1 << i)) {
                  CC2420SpiP__m_holder = i;
                  CC2420SpiP__m_requests &= ~(1 << i);
                  CC2420SpiP__grant__postTask();
                  {
                    unsigned char __nesc_temp = 
#line 169
                    SUCCESS;

                    {
#line 169
                      __nesc_atomic_end(__nesc_atomic); 
#line 169
                      return __nesc_temp;
                    }
                  }
                }
            }
        }
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
#line 175
  return SUCCESS;
}

# 247 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static void HplMsp430Usart0P__Usart__disableSpi(void )
#line 247
{
  /* atomic removed: atomic calls only */
#line 248
  {
    HplMsp430Usart0P__ME1 &= ~(1 << 6);
    HplMsp430Usart0P__SIMO__selectIOFunc();
    HplMsp430Usart0P__SOMI__selectIOFunc();
    HplMsp430Usart0P__UCLK__selectIOFunc();
  }
}

# 38 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/GpioCaptureC.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(uint8_t mode)
#line 38
{
  /* atomic removed: atomic calls only */
#line 39
  {
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectModuleFunc();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(mode);
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__enableEvents();
  }
  return SUCCESS;
}

# 136 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static void /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Alarm__startAt(/*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__to_size_type t0, /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__m_t0 = t0;
      /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__m_dt = dt;
      /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__set_alarm();
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

#line 96
static void /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__set_alarm(void )
{
  /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__to_size_type now = /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__Counter__get();
#line 98
  /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__to_size_type expires;
#line 98
  /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__to_size_type remaining;




  expires = /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__m_t0 + /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__m_dt;


  remaining = (/*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__to_size_type )(expires - now);


  if (/*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__m_t0 <= now) 
    {
      if (expires >= /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__MAX_DELAY) 
    {
      /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__m_t0 = now + /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__MAX_DELAY;
      /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__m_dt = remaining - /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__MAX_DELAY;
      remaining = /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__MAX_DELAY;
    }
  else 
    {
      /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__m_t0 += /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__m_dt;
      /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__m_dt = 0;
    }
  /*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__AlarmFrom__startAt((/*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__from_size_type )now << 0, 
  (/*LatinMacC.SlotAlarmC.Transform*/TransformAlarmC__1__from_size_type )remaining << 0);
}

# 139 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/idmanager/IDManagerP.nc"
static bool IDManagerP__IDManager__getIsDAGroot(void )
#line 139
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 140
    {
      unsigned char __nesc_temp = 
#line 140
      IDManagerP__isDAGroot;

      {
#line 140
        __nesc_atomic_end(__nesc_atomic); 
#line 140
        return __nesc_temp;
      }
    }
#line 142
    __nesc_atomic_end(__nesc_atomic); }
}

# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )49U &= ~(0x01 << 6);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 703 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/LatinMac/LatinMacP.nc"
static void LatinMacP__endSlot(void )
#line 703
{
  asn_t temp_asn;
  uint8_t temp_state;

#line 706
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 706
    {
      temp_asn = LatinMacP__asn;
      temp_state = LatinMacP__state;
      if (LatinMacP__packetACK != (void *)0) {
          LatinMacP__Malloc__freePacketBuffer(LatinMacP__packetACK);
          LatinMacP__packetACK = (void *)0;
        }
    }
#line 713
    __nesc_atomic_end(__nesc_atomic); }
  if (LatinMacP__RadioControl__rfOff() != SUCCESS) {
    }

  LatinMacP__change_state(S_SLEEP);
}

# 78 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/openqueue/OpenQueueP.nc"
static error_t OpenQueueP__Malloc__freePacketBuffer(OpenQueueEntry_t *pkt)
#line 78
{
  uint8_t i;

#line 80
  for (i = 0; i < QUEUELENGTH; i++) {
      if (&OpenQueueP__queue[i] == pkt) {
          OpenQueueP__reset_entry(i);
          return SUCCESS;
        }
    }
  return FAIL;
}

#line 38
static void OpenQueueP__reset_entry(uint8_t i)
#line 38
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 39
    {

      OpenQueueP__queue[i].creator = COMPONENT_NULL;
      OpenQueueP__queue[i].owner = COMPONENT_NULL;
      OpenQueueP__queue[i].payload = &OpenQueueP__queue[i].packet[126];
      OpenQueueP__queue[i].length = 0;


      OpenQueueP__queue[i].l3_destinationORsource.type = ADDR_NONE;

      OpenQueueP__queue[i].l2_nextORpreviousHop.type = ADDR_NONE;
      OpenQueueP__queue[i].l2_frameType = IEEE154_TYPE_UNDEFINED;
      OpenQueueP__queue[i].l2_retriesLeft = 0;
      OpenQueueP__queue[i].l2_seedNumber = 254;
    }
#line 53
    __nesc_atomic_end(__nesc_atomic); }
}

# 282 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/CC2420DriversP.nc"
static error_t CC2420DriversP__RadioControl__rfOff(void )
#line 282
{
  error_t temp_return;

#line 284
  temp_return = CC2420DriversP__CC2420Config__rfOff();
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 285
    CC2420DriversP__state = CC2420DriversP__S_STARTED;
#line 285
    __nesc_atomic_end(__nesc_atomic); }
  CC2420DriversP__TxControl__start();
  CC2420DriversP__RxControl__start();
  return temp_return;
}

# 126 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
static error_t CC2420SpiP__Resource__immediateRequest(uint8_t id)
#line 126
{
  error_t error;

  /* atomic removed: atomic calls only */
#line 129
  {
    if (CC2420SpiP__WorkingState__requestState(CC2420SpiP__S_BUSY) != SUCCESS) {
        {
          unsigned char __nesc_temp = 
#line 131
          EBUSY;

#line 131
          return __nesc_temp;
        }
      }

    if (CC2420SpiP__SpiResource__isOwner()) {
        CC2420SpiP__m_holder = id;
        error = SUCCESS;
      }
    else {
#line 139
      if ((error = CC2420SpiP__SpiResource__immediateRequest()) == SUCCESS) {
          CC2420SpiP__m_holder = id;
        }
      else {
          CC2420SpiP__WorkingState__toIdle();
        }
      }
  }
#line 146
  return error;
}

# 96 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/StateImplP.nc"
static error_t StateImplP__State__requestState(uint8_t id, uint8_t reqState)
#line 96
{
  error_t returnVal = FAIL;

  /* atomic removed: atomic calls only */
#line 98
  {
    if (reqState == StateImplP__S_IDLE || StateImplP__state[id] == StateImplP__S_IDLE) {
        StateImplP__state[id] = reqState;
        returnVal = SUCCESS;
      }
  }
  return returnVal;
}

# 174 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(uint8_t id)
#line 174
{
  /* atomic removed: atomic calls only */
#line 175
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId == id && /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY) {
        unsigned char __nesc_temp = 
#line 176
        TRUE;

#line 176
        return __nesc_temp;
      }
    else 
#line 177
      {
        unsigned char __nesc_temp = 
#line 177
        FALSE;

#line 177
        return __nesc_temp;
      }
  }
}

#line 130
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void )
#line 130
{
  /* atomic removed: atomic calls only */
#line 131
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__default_owner_id) {
        if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_GRANTING) {
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask();
            {
              unsigned char __nesc_temp = 
#line 135
              SUCCESS;

#line 135
              return __nesc_temp;
            }
          }
        else {
#line 137
          if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_IMM_GRANTING) {
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId;
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY;
              {
                unsigned char __nesc_temp = 
#line 140
                SUCCESS;

#line 140
                return __nesc_temp;
              }
            }
          }
      }
  }
#line 144
  return FAIL;
}

# 265 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config)
#line 265
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 267
    {
      HplMsp430Usart0P__Usart__resetUsart(TRUE);
      HplMsp430Usart0P__HplI2C__clearModeI2C();
      HplMsp430Usart0P__Usart__disableUart();
      HplMsp430Usart0P__configSpi(config);
      HplMsp430Usart0P__Usart__enableSpi();
      HplMsp430Usart0P__Usart__resetUsart(FALSE);
      HplMsp430Usart0P__Usart__clrIntr();
      HplMsp430Usart0P__Usart__disableIntr();
    }
#line 276
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

# 107 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
static error_t CC2420SpiP__Resource__request(uint8_t id)
#line 107
{
  /* atomic removed: atomic calls only */
  {
    if (CC2420SpiP__WorkingState__requestState(CC2420SpiP__S_BUSY) == SUCCESS) {
        CC2420SpiP__m_holder = id;
        if (CC2420SpiP__SpiResource__isOwner()) {
            CC2420SpiP__grant__postTask();
          }
        else {
            CC2420SpiP__SpiResource__request();
          }
      }
    else {
        CC2420SpiP__m_requests |= 1 << id;
      }
  }
  return SUCCESS;
}

# 286 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/control/CC2420ControlP.nc"
static void CC2420ControlP__rfOff(void )
#line 286
{
  CC2420ControlP__CSN__clr();
  CC2420ControlP__SRFOFF__strobe();
  CC2420ControlP__CSN__set();
  CC2420ControlP__RfOffResource__release();
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 291
    CC2420ControlP__m_rfOff_busy = FALSE;
#line 291
    __nesc_atomic_end(__nesc_atomic); }
}

# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )29U &= ~(0x01 << 2);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 318 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Strobe__strobe(uint8_t addr)
#line 318
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 319
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 321
            0;

            {
#line 321
              __nesc_atomic_end(__nesc_atomic); 
#line 321
              return __nesc_temp;
            }
          }
        }
    }
#line 325
    __nesc_atomic_end(__nesc_atomic); }
#line 325
  return CC2420SpiP__SpiByte__write(addr);
}

# 116 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static uint8_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiByte__write(uint8_t tx)
#line 116
{

  /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__tx(tx);
  while (!/*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__isRxIntrPending()) ;
  /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__clrRxIntr();
  return /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__rx();
}

# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )29U |= 0x01 << 2;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 318 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/transmit/CC2420TransmitP.nc"
static error_t CC2420TransmitP__StdAsyncControl__start(void )
#line 318
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 319
    {
      CC2420TransmitP__reset();
      CC2420TransmitP__CaptureSFD__captureRisingEdge();
      CC2420TransmitP__state = CC2420TransmitP__S_READY;
      CC2420TransmitP__m_receiving = FALSE;
      CC2420TransmitP__m_tx_power = 0;
    }
#line 325
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

#line 303
static void CC2420TransmitP__reset(void )
#line 303
{
  CC2420TransmitP__CSN__clr();
  CC2420TransmitP__SFLUSHTX__strobe();
  CC2420TransmitP__CSN__set();
  CC2420TransmitP__SpiResource__release();
  /* atomic removed: atomic calls only */
#line 308
  CC2420TransmitP__state = CC2420TransmitP__S_READY;
}

# 260 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/receive/CC2420ReceiveP.nc"
static error_t CC2420ReceiveP__StdAsyncControl__start(void )
#line 260
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 261
    {
      CC2420ReceiveP__reset();
      CC2420ReceiveP__state = CC2420ReceiveP__S_STARTED;
      CC2420ReceiveP__receivingPacket = FALSE;




      CC2420ReceiveP__InterruptFIFOP__enableFallingEdge();
    }
#line 270
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

static void CC2420ReceiveP__reset(void )
#line 274
{

  CC2420ReceiveP__free_bytes_left_in_fifo = CC2420ReceiveP__RXFIFO_SIZE;
  /* atomic removed: atomic calls only */
#line 277
  CC2420ReceiveP__receivingPacket = FALSE;
  CC2420ReceiveP__timestamp_head = 0;
  CC2420ReceiveP__timestamp_size = 0;
  CC2420ReceiveP__missed_packets = 0;

  CC2420ReceiveP__CSN__set();
  CC2420ReceiveP__CSN__clr();
  CC2420ReceiveP__SFLUSHRX__strobe();
  CC2420ReceiveP__SFLUSHRX__strobe();
  CC2420ReceiveP__CSN__set();
  CC2420ReceiveP__SpiResource__release();

  CC2420ReceiveP__waitForNextPacket();
}

#line 213
static void CC2420ReceiveP__waitForNextPacket(void )
#line 213
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 214
    {
      if (CC2420ReceiveP__state == CC2420ReceiveP__S_STOPPED) {
          CC2420ReceiveP__SpiResource__release();
          {
#line 217
            __nesc_atomic_end(__nesc_atomic); 
#line 217
            return;
          }
        }
#line 219
      CC2420ReceiveP__receivingPacket = FALSE;









      if ((CC2420ReceiveP__missed_packets && CC2420ReceiveP__FIFO__get()) || !CC2420ReceiveP__FIFOP__get()) {

          if (CC2420ReceiveP__missed_packets) {
              CC2420ReceiveP__missed_packets--;
            }
          CC2420ReceiveP__beginReceive();
        }
      else 
#line 235
        {

          CC2420ReceiveP__state = CC2420ReceiveP__S_STARTED;
          CC2420ReceiveP__missed_packets = 0;
          CC2420ReceiveP__SpiResource__release();
        }
    }
#line 241
    __nesc_atomic_end(__nesc_atomic); }
}

#line 87
static void CC2420ReceiveP__beginReceive(void )
#line 87
{
  CC2420ReceiveP__state = CC2420ReceiveP__S_RX_LENGTH;
  /* atomic removed: atomic calls only */
#line 89
  CC2420ReceiveP__receivingPacket = TRUE;
  if (CC2420ReceiveP__SpiResource__isOwner()) {
      CC2420ReceiveP__receiveLengthByte();
    }
  else {
#line 92
    if (CC2420ReceiveP__SpiResource__immediateRequest() == SUCCESS) {
        CC2420ReceiveP__receiveLengthByte();
      }
    else 
#line 94
      {
        CC2420ReceiveP__SpiResource__request();
      }
    }
}




static void CC2420ReceiveP__receiveLengthByte(void )
#line 103
{
  CC2420ReceiveP__CSN__clr();
  CC2420ReceiveP__RXFIFO__beginRead((uint8_t *)CC2420ReceiveP__receptionBuffer->payload, 1);
}

# 125 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiPacket__send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len)
#line 127
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 129
    {
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_client = id;
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_tx_buf = tx_buf;
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_rx_buf = rx_buf;
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_len = len;
    }
#line 134
    __nesc_atomic_end(__nesc_atomic); }

  if (len) {

      * (volatile uint8_t *)2U &= ~(128 | 64);


      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel1__setupTransfer(DMA_SINGLE_TRANSFER, 
      3U, 
      DMA_EDGE_SENSITIVE, 
      (void *)118U, 
      rx_buf ? rx_buf : &/*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_dump, 
      len, 
      DMA_BYTE, 
      DMA_BYTE, 
      DMA_ADDRESS_UNCHANGED, 
      rx_buf ? 
      DMA_ADDRESS_INCREMENTED : 
      DMA_ADDRESS_UNCHANGED);

      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel1__startTransfer();


      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel2__setupTransfer(DMA_SINGLE_TRANSFER, 
      4U, 
      DMA_EDGE_SENSITIVE, 
      tx_buf, 
      (void *)119U, 
      len, 
      DMA_BYTE, 
      DMA_BYTE, 
      DMA_ADDRESS_INCREMENTED, 
      DMA_ADDRESS_UNCHANGED);

      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel2__startTransfer();


      * (volatile uint8_t *)2U |= 128;
    }
  else 
#line 172
    {
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone_task__postTask();
    }

  return SUCCESS;
}

# 975 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/LatinMac/LatinMacP.nc"
static void LatinMacP__change_state(uint8_t newstate)
#line 975
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 976
    LatinMacP__state = newstate;
#line 976
    __nesc_atomic_end(__nesc_atomic); }
  switch (newstate) {
      case S_SYNCHRONIZING: 
        case S_TX_TXDATA: 
          case S_TX_RXACK: 
            case S_RX_RXDATA: 
              case S_RX_TXACK: 
                break;
      case S_TX_TXDATAPREPARE: 
        case S_TX_TXDATAREADY: 
          case S_TX_RXACKPREPARE: 
            case S_TX_RXACKREADY: 
              case S_RX_RXDATAPREPARE: 
                case S_RX_RXDATAREADY: 
                  case S_RX_TXACKPREPARE: 
                    case S_RX_TXACKREADY: 
                      case S_SLEEP: 
                        break;
    }
}

# 96 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/CC2420DriversP.nc"
static error_t CC2420DriversP__RadioControl__prepareReceive(uint8_t frequencyChannel_param)
#line 96
{
  uint8_t temp_state;

  /* atomic removed: atomic calls only */
#line 98
  {
    temp_state = CC2420DriversP__state;
    CC2420DriversP__frequencyChannel = frequencyChannel_param;
  }
  if (temp_state != CC2420DriversP__S_STARTED) {





      return FAIL;
    }
  /* atomic removed: atomic calls only */
#line 110
  CC2420DriversP__syncForSend = FALSE;
  /* atomic removed: atomic calls only */
#line 111
  CC2420DriversP__state = CC2420DriversP__S_SETTING_CHANNEL;
  CC2420DriversP__CC2420Config__setChannel(frequencyChannel_param);
  return SUCCESS;
}

# 195 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/control/CC2420ControlP.nc"
static error_t CC2420ControlP__CC2420Config__setChannel(uint8_t channel)
#line 195
{
  /* atomic removed: atomic calls only */
#line 196
  {
    CC2420ControlP__m_channel = channel;
    if (CC2420ControlP__m_sync_busy) {
        {
          unsigned char __nesc_temp = 
#line 199
          FAIL;

#line 199
          return __nesc_temp;
        }
      }
#line 201
    CC2420ControlP__m_sync_busy = TRUE;
    if (CC2420ControlP__m_state == CC2420ControlP__S_XOSC_STARTED) {
        if (CC2420ControlP__SyncResource__immediateRequest() != SUCCESS) {
            CC2420ControlP__SyncResource__request();
          }
        else 
#line 205
          {
            CC2420ControlP__syncConfig();
          }
      }
    else 
#line 208
      {
        {
          unsigned char __nesc_temp = 
#line 209
          FAIL;

#line 209
          return __nesc_temp;
        }
      }
  }
#line 212
  return SUCCESS;
}



static void CC2420ControlP__syncConfig(void )
#line 217
{
  uint8_t temp_channel;

#line 219
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 219
    {
      temp_channel = CC2420ControlP__m_channel;
    }
#line 221
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP__CSN__clr();
  CC2420ControlP__SRFOFF__strobe();

  CC2420ControlP__FSCTRL__write((1 << CC2420_FSCTRL_LOCK_THR) | (((
  temp_channel - 11) * 5 + 357) << CC2420_FSCTRL_FREQ));
  CC2420ControlP__CSN__set();
  CC2420ControlP__SyncResource__release();
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 229
    CC2420ControlP__m_sync_busy = FALSE;
#line 229
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP__CC2420Config__setChannelDone(SUCCESS);
}

# 305 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Reg__write(uint8_t addr, uint16_t data)
#line 305
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 306
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 308
            0;

            {
#line 308
              __nesc_atomic_end(__nesc_atomic); 
#line 308
              return __nesc_temp;
            }
          }
        }
    }
#line 312
    __nesc_atomic_end(__nesc_atomic); }
#line 311
  CC2420SpiP__SpiByte__write(addr);
  CC2420SpiP__SpiByte__write(data >> 8);
  return CC2420SpiP__SpiByte__write(data & 0xff);
}

# 449 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/LatinMac/LatinMacP.nc"
static void LatinMacP__RadioSend__prepareSendDone(error_t error)
#line 449
{
  asn_t temp_asn;
  uint8_t temp_state;

#line 452
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 452
    {
      temp_asn = LatinMacP__asn;
      temp_state = LatinMacP__state;
    }
#line 455
    __nesc_atomic_end(__nesc_atomic); }
  switch (temp_state) {
      case S_TX_TXDATAPREPARE: 
        if (error == SUCCESS) {
            LatinMacP__change_state(S_TX_TXDATAREADY);
          }
        else 
#line 460
          {
            { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 461
              {
                LatinMacP__dataFrameToSend->l2_retriesLeft--;
                if (LatinMacP__dataFrameToSend->l2_retriesLeft == 0) {
                    LatinMacP__postTaskSendDone(LatinMacP__dataFrameToSend, FAIL);
                  }
              }
#line 466
              __nesc_atomic_end(__nesc_atomic); }
            LatinMacP__endSlot();
          }
      break;
      case S_RX_TXACKPREPARE: 
        if (error == SUCCESS) {
            LatinMacP__change_state(S_RX_TXACKREADY);
          }
        else 
#line 473
          {

            LatinMacP__endSlot();
          }
      break;
      default: 
        LatinMacP__endSlot();
      return;
      break;
    }
}

#line 679
static void LatinMacP__postTaskSendDone(OpenQueueEntry_t *param_sendDoneMessage, 
error_t param_sendDoneError)
#line 680
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 681
    {

      LatinMacP__sendDoneMessage = param_sendDoneMessage;
      LatinMacP__sendDoneError = param_sendDoneError;
    }
#line 685
    __nesc_atomic_end(__nesc_atomic); }

  LatinMacP__taskSendDone__postTask();
}

#line 486
static void LatinMacP__RadioControl__prepareReceiveDone(error_t error)
#line 486
{
  asn_t temp_asn;
  uint8_t temp_state;

#line 489
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 489
    {
      temp_asn = LatinMacP__asn;
      temp_state = LatinMacP__state;
    }
#line 492
    __nesc_atomic_end(__nesc_atomic); }
  if (error != SUCCESS) {

      LatinMacP__endSlot();
    }
  switch (temp_state) {
      case S_TX_RXACKPREPARE: 
        LatinMacP__change_state(S_TX_RXACKREADY);
      break;
      case S_RX_RXDATAPREPARE: 
        LatinMacP__change_state(S_RX_RXDATAREADY);
      break;
      case S_SYNCHRONIZING: 
        if (LatinMacP__RadioControl__receiveNow(TRUE, 3200) != SUCCESS) {

            LatinMacP__endSlot();
          }
      break;
      default: 
        LatinMacP__endSlot();
      return;
      break;
    }
}

# 115 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/CC2420DriversP.nc"
static error_t CC2420DriversP__RadioControl__receiveNow(bool timeLimitedRx_param, uint16_t rxWaitTime_param)
#line 115
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 116
    {
      CC2420DriversP__state = CC2420DriversP__S_RECEIVING;
      CC2420DriversP__timeLimitedRx = timeLimitedRx_param;
      CC2420DriversP__rxWaitTime = rxWaitTime_param;
    }
#line 120
    __nesc_atomic_end(__nesc_atomic); }
  if (timeLimitedRx_param == TRUE) {
      CC2420DriversP__RxWaitAlarm__start(rxWaitTime_param);
    }

  return CC2420DriversP__CC2420Config__rxOn();
}

# 96 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static void /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__set_alarm(void )
{
  /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__to_size_type now = /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__Counter__get();
#line 98
  /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__to_size_type expires;
#line 98
  /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__to_size_type remaining;




  expires = /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__m_t0 + /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__m_dt;


  remaining = (/*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__to_size_type )(expires - now);


  if (/*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__m_t0 <= now) 
    {
      if (expires >= /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__MAX_DELAY) 
    {
      /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__m_t0 = now + /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__MAX_DELAY;
      /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__m_dt = remaining - /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__MAX_DELAY;
      remaining = /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__MAX_DELAY;
    }
  else 
    {
      /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__m_t0 += /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__m_dt;
      /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__m_dt = 0;
    }
  /*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__AlarmFrom__startAt((/*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__from_size_type )now << 0, 
  (/*CC2420DriversC.RxWaitAlarmC.Transform*/TransformAlarmC__5__from_size_type )remaining << 0);
}

# 256 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/control/CC2420ControlP.nc"
static void CC2420ControlP__rxOn(void )
#line 256
{
  CC2420ControlP__CSN__clr();
  CC2420ControlP__SRXON__strobe();
  CC2420ControlP__CSN__set();
  CC2420ControlP__RxOnResource__release();
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 261
    CC2420ControlP__m_rxOn_busy = FALSE;
#line 261
    __nesc_atomic_end(__nesc_atomic); }
}

# 99 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/openqueue/OpenQueueP.nc"
static OpenQueueEntry_t *OpenQueueP__OpenQueue__inQueueBySeed(uint8_t seedNumber)
#line 99
{
  uint8_t i;

#line 101
  for (i = 0; i < QUEUELENGTH; i++) {
      if (OpenQueueP__queue[i].l2_seedNumber == seedNumber) {
        return &OpenQueueP__queue[i];
        }
    }
#line 105
  return (void *)0;
}

# 98 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/IEEE802154_common.c"
static ieee802154_header_iht LatinMacP__retrieveIEEE802154header(OpenQueueEntry_t *msg)
#line 98
{
  ieee802154_header_iht ieee802514_header;
  uint8_t temp_8b;
  uint8_t offset = 0;

  offset += 1;

  temp_8b = *((uint8_t *)msg->payload + offset);
  ieee802514_header.frameType = (temp_8b >> IEEE154_FCF_FRAME_TYPE) & 0x07;
  ieee802514_header.securityEnabled = (temp_8b >> IEEE154_FCF_SECURITY_ENABLED) & 0x01;
  ieee802514_header.framePending = (temp_8b >> IEEE154_FCF_FRAME_PENDING) & 0x01;
  ieee802514_header.ackRequested = (temp_8b >> IEEE154_FCF_ACK_REQ) & 0x01;
  ieee802514_header.panIDCompression = (temp_8b >> IEEE154_FCF_INTRAPAN) & 0x01;
  offset += 1;
  temp_8b = *((uint8_t *)msg->payload + offset);
  switch ((temp_8b >> IEEE154_FCF_DEST_ADDR_MODE) & 0x03) {
      case IEEE154_ADDR_NONE: 
        ieee802514_header.dest.type = ADDR_NONE;
      break;
      case IEEE154_ADDR_SHORT: 
        ieee802514_header.dest.type = ADDR_16B;
      break;
      case IEEE154_ADDR_EXT: 
        ieee802514_header.dest.type = ADDR_64B;
      break;
      default: 



        break;
    }
  switch ((temp_8b >> IEEE154_FCF_SRC_ADDR_MODE) & 0x03) {
      case IEEE154_ADDR_NONE: 
        ieee802514_header.src.type = ADDR_NONE;
      break;
      case IEEE154_ADDR_SHORT: 
        ieee802514_header.src.type = ADDR_16B;
      break;
      case IEEE154_ADDR_EXT: 
        ieee802514_header.src.type = ADDR_64B;
      break;
      default: 



        break;
    }
  offset += 1;

  ieee802514_header.dsn = *((uint8_t *)msg->payload + offset);
  offset += 1;

  if (ieee802514_header.frameType == IEEE154_TYPE_ACK) {
      ieee802514_header.panid.type = ADDR_NONE;
    }
  else 
#line 152
    {
      LatinMacP__PacketFunctions__readAddress((uint8_t *)msg->payload + offset, ADDR_PANID, & ieee802514_header.panid, LITTLE_ENDIAN);
      offset += 2;
    }

  switch (ieee802514_header.dest.type) {
      case ADDR_NONE: 
        break;
      case ADDR_16B: 
        LatinMacP__PacketFunctions__readAddress((uint8_t *)msg->payload + offset, ADDR_16B, & ieee802514_header.dest, LITTLE_ENDIAN);
      offset += 2;
      break;
      case ADDR_64B: 
        LatinMacP__PacketFunctions__readAddress((uint8_t *)msg->payload + offset, ADDR_64B, & ieee802514_header.dest, LITTLE_ENDIAN);
      offset += 8;
      break;
    }

  switch (ieee802514_header.src.type) {
      case ADDR_NONE: 
        break;
      case ADDR_16B: 
        LatinMacP__PacketFunctions__readAddress((uint8_t *)msg->payload + offset, ADDR_16B, & ieee802514_header.src, LITTLE_ENDIAN);
      offset += 2;
      break;
      case ADDR_64B: 
        LatinMacP__PacketFunctions__readAddress((uint8_t *)msg->payload + offset, ADDR_64B, & ieee802514_header.src, LITTLE_ENDIAN);
      offset += 8;
      break;
    }
  ieee802514_header.headerLength = offset;
  return ieee802514_header;
}

# 152 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/packetfunctions/PacketFunctionsP.nc"
static void PacketFunctionsP__PacketFunctions__readAddress(uint8_t *payload, uint8_t type, open_addr_t *writeToAddress, bool littleEndian)
#line 152
{
  uint8_t i;
  uint8_t address_length;

#line 155
  writeToAddress->type = type;
  switch (type) {
      case ADDR_16B: 
        case ADDR_PANID: 
          address_length = 2;
      break;
      case ADDR_64B: 
        case ADDR_PREFIX: 
          address_length = 8;
      break;
      case ADDR_128B: 
        address_length = 16;
      break;
      default: 



        return;
    }
  for (i = 0; i < address_length; i++) {
      if (littleEndian) {
          writeToAddress->addr_128b[address_length - 1 - i] = *(payload + i);
        }
      else 
#line 177
        {
          writeToAddress->addr_128b[i] = *(payload + i);
        }
    }
}

# 112 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP__loadTXFIFO(void )
#line 112
{
  uint8_t tx_power = CC2420TransmitP__m_msg->l1_txPower;

#line 114
  if (!tx_power) {
      tx_power = 31;
    }
  CC2420TransmitP__CSN__clr();
  if (CC2420TransmitP__m_tx_power != tx_power) {
      CC2420TransmitP__TXCTRL__write((((2 << CC2420_TXCTRL_TXMIXBUF_CUR) | (
      3 << CC2420_TXCTRL_PA_CURRENT)) | (
      1 << CC2420_TXCTRL_RESERVED)) | ((
      tx_power & 0x1F) << CC2420_TXCTRL_PA_LEVEL));
    }
  CC2420TransmitP__m_tx_power = tx_power;
  CC2420TransmitP__AlarmWatchdog__start(CC2420TransmitP__CC2420_ABORT_PERIOD_LOAD);
  CC2420TransmitP__TXFIFO__write((uint8_t *)CC2420TransmitP__m_msg->payload, CC2420TransmitP__m_msg->length);
}

# 136 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static void /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Alarm__startAt(/*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__to_size_type t0, /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__m_t0 = t0;
      /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__m_dt = dt;
      /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__set_alarm();
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

#line 96
static void /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__set_alarm(void )
{
  /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__to_size_type now = /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__Counter__get();
#line 98
  /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__to_size_type expires;
#line 98
  /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__to_size_type remaining;




  expires = /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__m_t0 + /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__m_dt;


  remaining = (/*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__to_size_type )(expires - now);


  if (/*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__m_t0 <= now) 
    {
      if (expires >= /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__MAX_DELAY) 
    {
      /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__m_t0 = now + /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__MAX_DELAY;
      /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__m_dt = remaining - /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__MAX_DELAY;
      remaining = /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__MAX_DELAY;
    }
  else 
    {
      /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__m_t0 += /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__m_dt;
      /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__m_dt = 0;
    }
  /*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__AlarmFrom__startAt((/*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__from_size_type )now << 0, 
  (/*CC2420TransmitC.AlarmWatchdogC.Transform*/TransformAlarmC__4__from_size_type )remaining << 0);
}

static void /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Alarm__startAt(/*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__to_size_type t0, /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__m_t0 = t0;
      /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__m_dt = dt;
      /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__set_alarm();
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

#line 96
static void /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__set_alarm(void )
{
  /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__to_size_type now = /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__Counter__get();
#line 98
  /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__to_size_type expires;
#line 98
  /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__to_size_type remaining;




  expires = /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__m_t0 + /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__m_dt;


  remaining = (/*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__to_size_type )(expires - now);


  if (/*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__m_t0 <= now) 
    {
      if (expires >= /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__MAX_DELAY) 
    {
      /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__m_t0 = now + /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__MAX_DELAY;
      /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__m_dt = remaining - /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__MAX_DELAY;
      remaining = /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__MAX_DELAY;
    }
  else 
    {
      /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__m_t0 += /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__m_dt;
      /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__m_dt = 0;
    }
  /*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__AlarmFrom__startAt((/*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__from_size_type )now << 0, 
  (/*LatinMacC.FastAlarmC.Transform*/TransformAlarmC__2__from_size_type )remaining << 0);
}

# 143 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/printf/PrintfP.nc"
  int printfflush(void )
#line 143
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 144
    {
      if (PrintfP__state == PrintfP__S_FLUSHING) 
        {
          int __nesc_temp = 
#line 146
          SUCCESS;

          {
#line 146
            __nesc_atomic_end(__nesc_atomic); 
#line 146
            return __nesc_temp;
          }
        }
#line 147
      if (PrintfP__Queue__empty()) 
        {
          int __nesc_temp = 
#line 148
          FAIL;

          {
#line 148
            __nesc_atomic_end(__nesc_atomic); 
#line 148
            return __nesc_temp;
          }
        }
#line 149
      PrintfP__state = PrintfP__S_FLUSHING;
    }
#line 150
    __nesc_atomic_end(__nesc_atomic); }
  PrintfP__sendNext();
  return SUCCESS;
}

#line 132
static void PrintfP__sendNext(void )
#line 132
{
  int i;
  printf_msg_t *m = (printf_msg_t *)PrintfP__Packet__getPayload(&PrintfP__printfMsg, sizeof(printf_msg_t ));
  uint16_t length_to_send = PrintfP__Queue__size() < sizeof(printf_msg_t ) ? PrintfP__Queue__size() : sizeof(printf_msg_t );

#line 136
  memset(m->buffer, 0, sizeof(printf_msg_t ));
  for (i = 0; i < length_to_send; i++) 
    __nesc_hton_uint8(m->buffer[i].data, PrintfP__Queue__dequeue());
  if (PrintfP__AMSend__send(AM_BROADCAST_ADDR, &PrintfP__printfMsg, sizeof(printf_msg_t )) != SUCCESS) {
    PrintfP__retrySend__postTask();
    }
}

# 124 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialActiveMessageP.nc"
static void */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__getPayload(message_t *msg, uint8_t len)
#line 124
{
  if (len > /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength()) {
      return (void *)0;
    }
  else {
      return (void * )msg->data;
    }
}

# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/AMQueueEntryP.nc"
static error_t /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 47
{
  /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(msg, dest);
  /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(msg, 100);
  return /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(msg, len);
}

# 161 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialActiveMessageP.nc"
static am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(message_t *amsg)
#line 161
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(amsg);

#line 163
  return __nesc_ntoh_uint8(header->type.data);
}

#line 137
static am_addr_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__destination(message_t *amsg)
#line 137
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(amsg);

#line 139
  return __nesc_ntoh_uint16(header->dest.data);
}

#line 57
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(am_id_t id, am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 59
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(msg);

  if (len > /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength()) {
      return ESIZE;
    }

  __nesc_hton_uint16(header->dest.data, dest);





  __nesc_hton_uint8(header->type.data, id);
  __nesc_hton_uint8(header->length.data, len);

  return /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__send(msg, len);
}

# 502 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialP.nc"
static void SerialP__MaybeScheduleTx(void )
#line 502
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 503
    {
      if (SerialP__txPending == 0) {
          if (SerialP__RunTx__postTask() == SUCCESS) {
              SerialP__txPending = 1;
            }
        }
    }
#line 509
    __nesc_atomic_end(__nesc_atomic); }
}

# 221 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/CC2420DriversP.nc"
static error_t CC2420DriversP__RadioSend__sendNow(void )
#line 221
{
  bool useCca = FALSE;

  /* atomic removed: atomic calls only */
#line 223
  CC2420DriversP__state = CC2420DriversP__S_TRANSMITTING;
  return CC2420DriversP__CC2420Transmit__sendNow(useCca);
}

# 183 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP__send(void )
#line 183
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 184
    {
      CC2420TransmitP__CSN__clr();
      CC2420TransmitP__STXON__strobe();
      CC2420TransmitP__CSN__set();
    }
#line 188
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 189
    CC2420TransmitP__state = CC2420TransmitP__S_SFD;
#line 189
    __nesc_atomic_end(__nesc_atomic); }
  CC2420TransmitP__AlarmWatchdog__start(CC2420TransmitP__CC2420_ABORT_PERIOD_SFD);
}

# 46 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )29U &= ~(0x01 << 6);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

#line 45
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )29U |= 0x01 << 6;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 117 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/control/CC2420ControlP.nc"
static void CC2420ControlP__startOscillator(void )
#line 117
{
  CC2420ControlP__CSN__clr();

  CC2420ControlP__IOCFG1__write(CC2420_SFDMUX_XOSC16M_STABLE << CC2420_IOCFG1_CCAMUX);
  CC2420ControlP__InterruptCCA__enableRisingEdge();
  CC2420ControlP__SXOSCON__strobe();

  CC2420ControlP__IOCFG0__write((1 << CC2420_IOCFG0_FIFOP_POLARITY) | (127 << CC2420_IOCFG0_FIFOP_THR));









  CC2420ControlP__MDMCTRL0__write((((((((0 << CC2420_MDMCTRL0_RESERVED_FRAME_MODE) | (
  0 << CC2420_MDMCTRL0_PAN_COORDINATOR)) | (
  1 << CC2420_MDMCTRL0_ADR_DECODE)) | (
  2 << CC2420_MDMCTRL0_CCA_HYST)) | (
  3 << CC2420_MDMCTRL0_CCA_MOD)) | (
  1 << CC2420_MDMCTRL0_AUTOCRC)) | (
  1 << CC2420_MDMCTRL0_AUTOACK)) | (
  2 << CC2420_MDMCTRL0_PREAMBLE_LENGTH));

  CC2420ControlP__RXCTRL1__write(((((((1 << CC2420_RXCTRL1_RXBPF_LOCUR) | (
  1 << CC2420_RXCTRL1_LOW_LOWGAIN)) | (
  1 << CC2420_RXCTRL1_HIGH_HGM)) | (
  1 << CC2420_RXCTRL1_LNA_CAP_ARRAY)) | (
  1 << CC2420_RXCTRL1_RXMIX_TAIL)) | (
  1 << CC2420_RXCTRL1_RXMIX_VCM)) | (
  2 << CC2420_RXCTRL1_RXMIX_CURRENT));

  CC2420ControlP__CSN__set();
}

# 96 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static void /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__set_alarm(void )
{
  /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__to_size_type now = /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__Counter__get();
#line 98
  /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__to_size_type expires;
#line 98
  /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__to_size_type remaining;




  expires = /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__m_t0 + /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__m_dt;


  remaining = (/*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__to_size_type )(expires - now);


  if (/*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__m_t0 <= now) 
    {
      if (expires >= /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__MAX_DELAY) 
    {
      /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__m_t0 = now + /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__MAX_DELAY;
      /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__m_dt = remaining - /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__MAX_DELAY;
      remaining = /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__MAX_DELAY;
    }
  else 
    {
      /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__m_t0 += /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__m_dt;
      /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__m_dt = 0;
    }
  /*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__AlarmFrom__startAt((/*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__from_size_type )now << 0, 
  (/*CC2420ControlC.StartupAlarmC.Transform*/TransformAlarmC__3__from_size_type )remaining << 0);
}

# 589 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/LatinMac/LatinMacP.nc"
static void LatinMacP__RadioControl__receivedNothing(void )
#line 589
{
  asn_t temp_asn;
  uint8_t temp_state;
  OpenQueueEntry_t *temp_dataFrameToSend;

  uint8_t temp_init;

  /* atomic removed: atomic calls only */
#line 596
  {
    temp_asn = LatinMacP__asn;
    temp_state = LatinMacP__state;
    temp_dataFrameToSend = LatinMacP__dataFrameToSend;
    temp_init = LatinMacP__init;
  }
  switch (temp_state) {
      case S_RX_RXDATA: 

        LatinMacP__endSlot();
      break;
      case S_TX_RXACK: 


        temp_dataFrameToSend->l2_retriesLeft--;
      if (temp_dataFrameToSend->l2_retriesLeft == 0) {
          LatinMacP__postTaskSendDone(temp_dataFrameToSend, FAIL);
        }
      LatinMacP__endSlot();
      break;
      case S_SYNCHRONIZING: 

        LatinMacP__endSlot();
      break;
      default: 
        LatinMacP__endSlot();
      break;
    }
}

# 14 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(24)))  void sig_TIMERB1_VECTOR(void )
#line 14
{
#line 14
  Msp430TimerCommonP__VectorTimerB1__fired();
}

# 52 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/RealMainP.nc"
  int main(void )
#line 52
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      {
      }
#line 60
      ;

      RealMainP__Scheduler__init();





      RealMainP__PlatformInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;





      RealMainP__SoftwareInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;
    }
#line 77
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  RealMainP__Boot__booted();


  RealMainP__Scheduler__taskLoop();




  return -1;
}

# 164 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/timer/Msp430ClockP.nc"
static void Msp430ClockP__set_dco_calib(int calib)
{
  BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  DCOCTL = calib & 0xff;
}

# 16 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/platforms/telosb/MotePlatformC.nc"
static void MotePlatformC__TOSH_FLASH_M25P_DP_bit(bool set)
#line 16
{
  if (set) {
    TOSH_SET_SIMO0_PIN();
    }
  else {
#line 20
    TOSH_CLR_SIMO0_PIN();
    }
#line 21
  TOSH_SET_UCLK0_PIN();
  TOSH_CLR_UCLK0_PIN();
}

# 45 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )49U |= 0x01 << 6;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 123 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/SchedulerBasicP.nc"
static bool SchedulerBasicP__Scheduler__runNextTask(void )
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
#line 127
  {
    nextTask = SchedulerBasicP__popTask();
    if (nextTask == SchedulerBasicP__NO_TASK) 
      {
        {
          unsigned char __nesc_temp = 
#line 131
          FALSE;

#line 131
          return __nesc_temp;
        }
      }
  }
#line 134
  SchedulerBasicP__TaskBasic__runTask(nextTask);
  return TRUE;
}

#line 164
static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id)
{
}

# 64 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0x4059ab40){
#line 64
  switch (arg_0x4059ab40) {
#line 64
    case SerialP__RunTx:
#line 64
      SerialP__RunTx__runTask();
#line 64
      break;
#line 64
    case SerialP__startDoneTask:
#line 64
      SerialP__startDoneTask__runTask();
#line 64
      break;
#line 64
    case SerialP__stopDoneTask:
#line 64
      SerialP__stopDoneTask__runTask();
#line 64
      break;
#line 64
    case SerialP__defaultSerialFlushTask:
#line 64
      SerialP__defaultSerialFlushTask__runTask();
#line 64
      break;
#line 64
    case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone:
#line 64
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask();
#line 64
      break;
#line 64
    case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask:
#line 64
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask();
#line 64
      break;
#line 64
    case /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask:
#line 64
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask();
#line 64
      break;
#line 64
    case /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask:
#line 64
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask();
#line 64
      break;
#line 64
    case /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask:
#line 64
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask();
#line 64
      break;
#line 64
    case PrintfP__retrySend:
#line 64
      PrintfP__retrySend__runTask();
#line 64
      break;
#line 64
    case IDManagerP__taskPrint:
#line 64
      IDManagerP__taskPrint__runTask();
#line 64
      break;
#line 64
    case IDManagerP__taskHandleBridge:
#line 64
      IDManagerP__taskHandleBridge__runTask();
#line 64
      break;
#line 64
    case IDManagerP__taskHandleRoot:
#line 64
      IDManagerP__taskHandleRoot__runTask();
#line 64
      break;
#line 64
    case LatinResP__taskBuildAndSendScanMsg:
#line 64
      LatinResP__taskBuildAndSendScanMsg__runTask();
#line 64
      break;
#line 64
    case LatinResP__taskBuildAndSendAdvMsg:
#line 64
      LatinResP__taskBuildAndSendAdvMsg__runTask();
#line 64
      break;
#line 64
    case LatinResP__taskBuildAndSendReqMsg:
#line 64
      LatinResP__taskBuildAndSendReqMsg__runTask();
#line 64
      break;
#line 64
    case LatinResP__taskBuildAndSendSeedMsgToChildren:
#line 64
      LatinResP__taskBuildAndSendSeedMsgToChildren__runTask();
#line 64
      break;
#line 64
    case LatinResP__taskBuildAndSendDataToChildren:
#line 64
      LatinResP__taskBuildAndSendDataToChildren__runTask();
#line 64
      break;
#line 64
    case /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__sendEvent:
#line 64
      /*UserButtonC.SwitchToggleC*/SwitchToggleC__0__sendEvent__runTask();
#line 64
      break;
#line 64
    case /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired:
#line 64
      /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask();
#line 64
      break;
#line 64
    case /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer:
#line 64
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask();
#line 64
      break;
#line 64
    case LatinMacP__taskReceive:
#line 64
      LatinMacP__taskReceive__runTask();
#line 64
      break;
#line 64
    case LatinMacP__taskSendDone:
#line 64
      LatinMacP__taskSendDone__runTask();
#line 64
      break;
#line 64
    case CC2420DriversP__taskStartDone:
#line 64
      CC2420DriversP__taskStartDone__runTask();
#line 64
      break;
#line 64
    case CC2420DriversP__taskStopDone:
#line 64
      CC2420DriversP__taskStopDone__runTask();
#line 64
      break;
#line 64
    case CC2420DriversP__taskSendDone:
#line 64
      CC2420DriversP__taskSendDone__runTask();
#line 64
      break;
#line 64
    case CC2420DriversP__taskStartOscillatorDone:
#line 64
      CC2420DriversP__taskStartOscillatorDone__runTask();
#line 64
      break;
#line 64
    case CC2420SpiP__grant:
#line 64
      CC2420SpiP__grant__runTask();
#line 64
      break;
#line 64
    case /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone_task:
#line 64
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone_task__runTask();
#line 64
      break;
#line 64
    case /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask:
#line 64
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask();
#line 64
      break;
#line 64
    case CC2420ReceiveP__taskReceiveDone:
#line 64
      CC2420ReceiveP__taskReceiveDone__runTask();
#line 64
      break;
#line 64
    case OpenQueueP__taskDebugPrint:
#line 64
      OpenQueueP__taskDebugPrint__runTask();
#line 64
      break;
#line 64
    default:
#line 64
      SchedulerBasicP__TaskBasic__default__runTask(arg_0x4059ab40);
#line 64
      break;
#line 64
    }
#line 64
}
#line 64
# 68 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/openqueue/OpenQueueP.nc"
static OpenQueueEntry_t *OpenQueueP__Malloc__getFreePacketBuffer(void )
#line 68
{
  uint8_t i;

#line 70
  for (i = 0; i < QUEUELENGTH; i++) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 71
        if (OpenQueueP__queue[i].owner == COMPONENT_NULL) {
            OpenQueueP__queue[i].owner = COMPONENT_OPENQUEUE;
            {
              struct OpenQueueEntry_t *__nesc_temp = 
#line 73
              &OpenQueueP__queue[i];

              {
#line 73
                __nesc_atomic_end(__nesc_atomic); 
#line 73
                return __nesc_temp;
              }
            }
          }
#line 76
        __nesc_atomic_end(__nesc_atomic); }
    }
#line 76
  return (void *)0;
}

# 199 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/idmanager/IDManagerP.nc"
static bool IDManagerP__IDManager__isMyAddress(open_addr_t *addr)
#line 199
{
  open_addr_t temp_my128bID;

#line 201
  switch (addr->type) {
      case ADDR_16B: 
        return IDManagerP__PacketFunctions__sameAddress(addr, &IDManagerP__my16bID);
      case ADDR_64B: 
        return IDManagerP__PacketFunctions__sameAddress(addr, &IDManagerP__my64bID);
      case ADDR_128B: 

        temp_my128bID.type = ADDR_128B;
      memcpy(&temp_my128bID.addr_128b[0], & IDManagerP__myPrefix.prefix, 8);
      memcpy(&temp_my128bID.addr_128b[8], & IDManagerP__my64bID.addr_64b, 8);
      return IDManagerP__PacketFunctions__sameAddress(addr, &temp_my128bID);
      case ADDR_PANID: 
        return IDManagerP__PacketFunctions__sameAddress(addr, &IDManagerP__myPANID);
      case ADDR_PREFIX: 
        return IDManagerP__PacketFunctions__sameAddress(addr, &IDManagerP__myPrefix);
      default: 



        return FALSE;
    }
}

# 121 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/packetfunctions/PacketFunctionsP.nc"
static bool PacketFunctionsP__PacketFunctions__sameAddress(open_addr_t *address_1, open_addr_t *address_2)
#line 121
{
  uint8_t address_length;

#line 123
  if (address_1->type != address_2->type) {
      return FALSE;
    }
  switch (address_1->type) {
      case ADDR_16B: 
        case ADDR_PANID: 
          address_length = 2;
      break;
      case ADDR_64B: 
        case ADDR_PREFIX: 
          address_length = 8;
      break;
      case ADDR_128B: 
        address_length = 16;
      break;
      default: 



        return FALSE;
    }
  if (memcmp((void *)address_1->addr_128b, (void *)address_2->addr_128b, address_length) == 0) {
      return TRUE;
    }
  return FALSE;
}

#line 86
static bool PacketFunctionsP__PacketFunctions__isBroadcastMulticast(open_addr_t *address)
#line 86
{
  uint8_t i;
  uint8_t address_length;

  if (address->type == ADDR_128B) {
      if (address->addr_128b[0] == 0xff) {
          return TRUE;
        }
      else 
#line 93
        {
          return FALSE;
        }
    }

  switch (address->type) {
      case ADDR_16B: 
        address_length = 2;
      break;
      case ADDR_64B: 
        address_length = 8;
      break;
      default: 




        return FALSE;
    }
  for (i = 0; i < address_length; i++) {
      if (address->addr_128b[i] != 0xFF) {

          return FALSE;
        }
    }
  return TRUE;
}

# 329 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/spi/CC2420SpiP.nc"
static void CC2420SpiP__SpiPacket__sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error)
#line 330
{
  if (CC2420SpiP__m_addr & 0x40) {
      CC2420SpiP__Fifo__readDone(CC2420SpiP__m_addr & ~0x40, rx_buf, len, error);
    }
  else 
#line 333
    {
      CC2420SpiP__Fifo__writeDone(CC2420SpiP__m_addr, tx_buf, len, error);
    }
}

# 137 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/CC2420DriversP.nc"
static void CC2420DriversP__CC2420ReceivedNothing__receivedNothing(void )
#line 137
{
  CC2420DriversP__RxWaitAlarm__stop();
  CC2420DriversP__TxControl__start();
  CC2420DriversP__RxControl__start();
  CC2420DriversP__RadioControl__receivedNothing();
}

# 178 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/01-PHY/cc2420/control/CC2420ControlP.nc"
static void CC2420ControlP__writeId(void )
#line 178
{
  nxle_uint16_t id[6];

#line 180
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 180
    {

      __nesc_hton_leuint16(id[4].data, PAN_ID);
      __nesc_hton_leuint16(id[5].data, TOS_NODE_ID);
    }
#line 184
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP__CSN__clr();

  CC2420ControlP__IEEEADR__write(0, (uint8_t *)&id, sizeof id);
  CC2420ControlP__CSN__set();
  CC2420ControlP__SpiResource__release();
  CC2420ControlP__CC2420Config__writeIdDone();
}

# 996 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/LatinMac/LatinMacP.nc"
static void LatinMacP__MAC__changeNetworkState(uint8_t tmp_networkState)
#line 996
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 997
    LatinMacP__networkState = tmp_networkState;
#line 997
    __nesc_atomic_end(__nesc_atomic); }
  LatinMacP__Leds__led0On();
}

# 147 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/idmanager/IDManagerP.nc"
static bool IDManagerP__IDManager__getIsBridge(void )
#line 147
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 148
    {
      unsigned char __nesc_temp = 
#line 148
      IDManagerP__isBridge;

      {
#line 148
        __nesc_atomic_end(__nesc_atomic); 
#line 148
        return __nesc_temp;
      }
    }
#line 150
    __nesc_atomic_end(__nesc_atomic); }
}

# 275 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02b-RES/LatinRes/LatinResP.nc"
static void LatinResP__updateNeighborList(OpenQueueEntry_t *msg)
#line 275
{
  uint8_t i;

  if (__nesc_ntoh_leuint8(((CMD_MSG_t *)msg->payload)->commandFrameId.data) == SEED_MSG) {
      LatinResP__neighborList.numSibling = __nesc_ntoh_leuint8(((CMD_MSG_t *)msg->payload)->numChildren.data);
      memcpy(LatinResP__neighborList.siblingList, ((CMD_MSG_t *)msg->payload)->seedList, sizeof(uint8_t ) * LatinResP__neighborList.numSibling);
    }
  else 
#line 281
    {
      for (i = 0; i < MAXNUMNEIGHBORS; i++) 
        {
          /* atomic removed: atomic calls only */
#line 284
          {
            if (LatinResP__isThisRowMatching(& msg->l2_nextORpreviousHop, i)) {

                LatinResP__neighborList.neighbors[i].rssi = msg->l1_rssi;
                if (__nesc_ntoh_leuint8(((CMD_MSG_t *)msg->payload)->commandFrameId.data) == ADV_MSG) {
                    LatinResP__neighborList.neighbors[i].numHops = __nesc_ntoh_leuint8(((CMD_MSG_t *)msg->payload)->numHops.data);
                    if (LatinResP__neighborList.neighbors[i].numHops == LatinResP__neighborList.minHops) {
                        if (LatinResP__neighborList.neighbors[i].rssi > LatinResP__neighborList.maxRssi) {
                            LatinResP__neighborList.maxRssi = LatinResP__neighborList.neighbors[i].rssi;
                            LatinResP__neighborList.parent = i;
                          }
                      }
                    else {
#line 295
                      if (LatinResP__neighborList.neighbors[i].numHops < LatinResP__neighborList.minHops) {
                          LatinResP__neighborList.maxRssi = LatinResP__neighborList.neighbors[i].rssi;
                          LatinResP__neighborList.minHops = LatinResP__neighborList.neighbors[i].numHops;
                          LatinResP__neighborList.parent = i;
                        }
                      }
                  }
                else {
#line 300
                  if (__nesc_ntoh_leuint8(((CMD_MSG_t *)msg->payload)->commandFrameId.data) == REQ_MSG) {
                      if (LatinResP__neighborList.neighbors[i].isChild == FALSE) {
                          LatinResP__neighborList.neighbors[i].isChild = TRUE;
                          LatinResP__neighborList.seedList[LatinResP__neighborList.numChildren] = __nesc_ntoh_leuint8(((CMD_MSG_t *)msg->payload)->seedNumber.data);
                          LatinResP__neighborList.childrenIndex[LatinResP__neighborList.numChildren] = i;
                          LatinResP__neighborList.numChildren = LatinResP__neighborList.numChildren + 1;
                        }
                    }
                  }
#line 308
                return;
              }
          }
        }
      LatinResP__registerNewNeighbor(msg);
    }
}

# 36 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/packetfunctions/PacketFunctionsP.nc"
static void PacketFunctionsP__PacketFunctions__mac64bToIp128b(
open_addr_t *prefix64b, 
open_addr_t *mac64b, 
open_addr_t *ip128bToWrite)
#line 39
{
  if (prefix64b->type != ADDR_PREFIX || mac64b->type != ADDR_64B) {



      ip128bToWrite->type = ADDR_NONE;
      return;
    }
  ip128bToWrite->type = ADDR_128B;
  memcpy(&ip128bToWrite->addr_128b[0], &prefix64b->prefix[0], 8);
  memcpy(&ip128bToWrite->addr_128b[8], &mac64b->addr_64b[0], 8);
}

# 155 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/idmanager/IDManagerP.nc"
static open_addr_t *IDManagerP__IDManager__getMyID(uint8_t type)
#line 155
{
  switch (type) {
      case ADDR_16B: 
        return &IDManagerP__my16bID;
      case ADDR_64B: 
        return &IDManagerP__my64bID;
      case ADDR_PANID: 
        return &IDManagerP__myPANID;
      case ADDR_PREFIX: 
        return &IDManagerP__myPrefix;
      case ADDR_128B: 

        default: 



          return (void *)0;
    }
}

# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/packetfunctions/PacketFunctionsP.nc"
static void PacketFunctionsP__PacketFunctions__mac64bToMac16b(open_addr_t *mac64b, open_addr_t *mac16btoWrite)
#line 53
{
  if (mac64b->type != ADDR_64B) {



      mac16btoWrite->type = ADDR_NONE;
      return;
    }
  mac16btoWrite->type = ADDR_16B;
  mac16btoWrite->addr_16b[0] = mac64b->addr_64b[6];
  mac16btoWrite->addr_16b[1] = mac64b->addr_64b[7];
}

#line 217
static void PacketFunctionsP__PacketFunctions__reserveHeaderSize(OpenQueueEntry_t *pkt, uint8_t header_length)
#line 217
{
  pkt->payload -= header_length;
  pkt->length += header_length;
  if ((uint8_t *)pkt->payload < (uint8_t *)pkt->packet) {
    }
}

# 107 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02a-MAC/LatinMac/LatinMacP.nc"
static error_t LatinMacP__OpenSendFromUpper__send(OpenQueueEntry_t *msg)
#line 107
{
  msg->owner = COMPONENT_MAC;
  if (LatinMacP__PacketFunctions__isBroadcastMulticast(& msg->l2_nextORpreviousHop) == TRUE) {
      msg->l2_retriesLeft = 1;
    }
  else 
#line 111
    {
      msg->l2_retriesLeft = TXRETRIES;
    }
  msg->l1_txPower = TX_POWER;
  LatinMacP__prependIEEE802154header(msg, 
  msg->l2_frameType, 
  IEEE154_SEC_NO_SECURITY, 
  LatinMacP__dsn++, 
  & msg->l2_nextORpreviousHop);

  return SUCCESS;
}

# 183 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/cross-layers/packetfunctions/PacketFunctionsP.nc"
static void PacketFunctionsP__PacketFunctions__writeAddress(OpenQueueEntry_t *msg, open_addr_t *address, bool littleEndian)
#line 183
{
  uint8_t i;
  uint8_t address_length;

#line 186
  switch (address->type) {
      case ADDR_16B: 
        case ADDR_PANID: 
          address_length = 2;
      break;
      case ADDR_64B: 
        case ADDR_PREFIX: 
          address_length = 8;
      break;
      case ADDR_128B: 
        address_length = 16;
      break;
      default: 



        return;
    }
  for (i = 0; i < address_length; i++) {
      msg->payload -= sizeof(uint8_t );
      msg->length += sizeof(uint8_t );
      if (littleEndian) {
          * (uint8_t *)msg->payload = address->addr_128b[i];
        }
      else 
#line 209
        {
          * (uint8_t *)msg->payload = address->addr_128b[address_length - 1 - i];
        }
    }
}

# 62 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now)
{
  uint8_t num;

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
#line 79
                timer->t0 += timer->dt;
                }
              /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(num);
              break;
            }
        }
    }
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

# 136 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 = t0;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm();
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

# 671 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling//apps/CodeScheduling/02b-RES/LatinRes/LatinResP.nc"
static void LatinResP__UserButton__notify(button_state_t button)
#line 671
{
  if (LatinResP__IDManager__getIsDAGroot() == TRUE && LatinResP__state == ScanWait) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 673
        LatinResP__state = ScanSend;
#line 673
        __nesc_atomic_end(__nesc_atomic); }
      LatinResP__taskBuildAndSendScanMsg__postTask();
    }
}

# 41 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/Msp430InterruptC.nc"
static error_t /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__enable(bool rising)
#line 41
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 42
    {
      /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__Interrupt__disable();
      /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__HplInterrupt__edge(rising);
      /*HplUserButtonC.InterruptUserButtonC*/Msp430InterruptC__0__HplInterrupt__enable();
    }
#line 46
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 155 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/printf/PrintfP.nc"
static void PrintfP__AMSend__sendDone(message_t *msg, error_t error)
#line 155
{
  if (error == SUCCESS) {
      if (PrintfP__Queue__size() > 0) {
        PrintfP__sendNext();
        }
      else {
#line 159
        PrintfP__state = PrintfP__S_STARTED;
        }
    }
  else {
#line 161
    PrintfP__retrySend__postTask();
    }
}

# 155 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/AMQueueImplP.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(uint8_t last, message_t * msg, error_t err)
#line 155
{
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg = (void *)0;
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend();
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(last, msg, err);
}

# 85 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__configure(uint8_t id)
#line 85
{
  msp430_uart_union_config_t *config = /*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__getConfig(id);

#line 87
  /*Msp430Uart1P.UartP*/Msp430UartP__0__m_byte_time = config->uartConfig.ubr / 2;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__setModeUart(config);
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__enableIntr();
}

# 251 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static void HplMsp430Usart1P__Usart__disableSpi(void )
#line 251
{
  /* atomic removed: atomic calls only */
#line 252
  {
    HplMsp430Usart1P__ME2 &= ~(1 << 4);
    HplMsp430Usart1P__SIMO__selectIOFunc();
    HplMsp430Usart1P__SOMI__selectIOFunc();
    HplMsp430Usart1P__UCLK__selectIOFunc();
  }
}

#line 211
static void HplMsp430Usart1P__Usart__disableUart(void )
#line 211
{
  /* atomic removed: atomic calls only */
#line 212
  {
    HplMsp430Usart1P__ME2 &= ~((1 << 5) | (1 << 4));
    HplMsp430Usart1P__UTXD__selectIOFunc();
    HplMsp430Usart1P__URXD__selectIOFunc();
  }
}

# 174 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id)
#line 174
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 175
    {
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId == id && /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) {
          unsigned char __nesc_temp = 
#line 176
          TRUE;

          {
#line 176
            __nesc_atomic_end(__nesc_atomic); 
#line 176
            return __nesc_temp;
          }
        }
      else 
#line 177
        {
          unsigned char __nesc_temp = 
#line 177
          FALSE;

          {
#line 177
            __nesc_atomic_end(__nesc_atomic); 
#line 177
            return __nesc_temp;
          }
        }
    }
#line 180
    __nesc_atomic_end(__nesc_atomic); }
}

# 347 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialP.nc"
static void SerialP__testOff(void )
#line 347
{
  bool turnOff = FALSE;

#line 349
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 349
    {
      if (SerialP__txState == SerialP__TXSTATE_INACTIVE && 
      SerialP__rxState == SerialP__RXSTATE_INACTIVE) {
          turnOff = TRUE;
        }
    }
#line 354
    __nesc_atomic_end(__nesc_atomic); }
  if (turnOff) {
      SerialP__stopDoneTask__postTask();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 357
        SerialP__offPending = FALSE;
#line 357
        __nesc_atomic_end(__nesc_atomic); }
    }
  else {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 360
        SerialP__offPending = TRUE;
#line 360
        __nesc_atomic_end(__nesc_atomic); }
    }
}

# 86 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/HdlcTranslateC.nc"
static error_t HdlcTranslateC__SerialFrameComm__putDelimiter(void )
#line 86
{
  HdlcTranslateC__state.sendEscape = 0;
  HdlcTranslateC__m_data = HDLC_FLAG_BYTE;
  return HdlcTranslateC__UartStream__send(&HdlcTranslateC__m_data, 1);
}

# 147 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/Msp430UartP.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__send(uint8_t id, uint8_t *buf, uint16_t len)
#line 147
{
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__isOwner(id) == FALSE) {
    return FAIL;
    }
#line 150
  if (len == 0) {
    return FAIL;
    }
  else {
#line 152
    if (/*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf) {
      return EBUSY;
      }
    }
#line 154
  /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf = buf;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len = len;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_pos = 0;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__current_owner = id;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__tx(buf[/*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_pos++]);
  return SUCCESS;
}

# 96 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
__attribute((wakeup)) __attribute((interrupt(6)))  void sig_UART1RX_VECTOR(void )
#line 96
{
  uint8_t temp = U1RXBUF;

#line 98
  HplMsp430Usart1P__Interrupts__rxDone(temp);
}

# 150 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void )
#line 150
{
  /* atomic removed: atomic calls only */
#line 151
  {
    if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) 
      {
        unsigned char __nesc_temp = 
#line 153
        FALSE;

#line 153
        return __nesc_temp;
      }
  }
#line 155
  return TRUE;
}

# 402 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialP.nc"
static void SerialP__rx_state_machine(bool isDelimeter, uint8_t data)
#line 402
{

  switch (SerialP__rxState) {

      case SerialP__RXSTATE_NOSYNC: 
        if (isDelimeter) {
            SerialP__rxInit();
            SerialP__rxState = SerialP__RXSTATE_PROTO;
          }
      break;

      case SerialP__RXSTATE_PROTO: 
        if (!isDelimeter) {
            SerialP__rxCRC = crcByte(SerialP__rxCRC, data);
            SerialP__rxState = SerialP__RXSTATE_TOKEN;
            SerialP__rxProto = data;
            if (!SerialP__valid_rx_proto(SerialP__rxProto)) {
              goto nosync;
              }
            if (SerialP__rxProto != SERIAL_PROTO_PACKET_ACK) {
                goto nosync;
              }
            if (SerialP__ReceiveBytePacket__startPacket() != SUCCESS) {
                goto nosync;
              }
          }
      break;

      case SerialP__RXSTATE_TOKEN: 
        if (isDelimeter) {
            goto nosync;
          }
        else {
            SerialP__rxSeqno = data;
            SerialP__rxCRC = crcByte(SerialP__rxCRC, SerialP__rxSeqno);
            SerialP__rxState = SerialP__RXSTATE_INFO;
          }
      break;

      case SerialP__RXSTATE_INFO: 
        if (SerialP__rxByteCnt < SerialP__SERIAL_MTU) {
            if (isDelimeter) {
                if (SerialP__rxByteCnt >= 2) {
                    if (SerialP__rx_current_crc() == SerialP__rxCRC) {
                        SerialP__ReceiveBytePacket__endPacket(SUCCESS);
                        SerialP__ack_queue_push(SerialP__rxSeqno);
                        goto nosync;
                      }
                    else {
                        goto nosync;
                      }
                  }
                else {
                    goto nosync;
                  }
              }
            else {
                if (SerialP__rxByteCnt >= 2) {
                    SerialP__ReceiveBytePacket__byteReceived(SerialP__rx_buffer_top());
                    SerialP__rxCRC = crcByte(SerialP__rxCRC, SerialP__rx_buffer_pop());
                  }
                SerialP__rx_buffer_push(data);
                SerialP__rxByteCnt++;
              }
          }
        else 

          {
            goto nosync;
          }
      break;

      default: 
        goto nosync;
    }
  goto done;

  nosync: 

    SerialP__rxInit();
  SerialP__SerialFrameComm__resetReceive();
  SerialP__ReceiveBytePacket__endPacket(FAIL);
  if (SerialP__offPending) {
      SerialP__rxState = SerialP__RXSTATE_INACTIVE;
      SerialP__testOff();
    }
  else {
    if (isDelimeter) {
        SerialP__rxState = SerialP__RXSTATE_PROTO;
      }
    }
  done: ;
}

# 80 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/crc.h"
static uint16_t crcByte(uint16_t crc, uint8_t b)
#line 80
{
  crc = (uint8_t )(crc >> 8) | (crc << 8);
  crc ^= b;
  crc ^= (uint8_t )(crc & 0xff) >> 4;
  crc ^= crc << 12;
  crc ^= (crc & 0xff) << 5;
  return crc;
}

# 285 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/SerialDispatcherP.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(error_t result)
#line 285
{
  uint8_t postsignalreceive = FALSE;

  /* atomic removed: atomic calls only */
#line 287
  {
    if (!/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending && result == SUCCESS) {
        postsignalreceive = TRUE;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending = TRUE;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskType = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskWhich = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskBuf = (message_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskSize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBufferSwap();
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_IDLE;
      }
    else 
#line 297
      {

        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which);
      }
  }
  if (postsignalreceive) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__postTask();
    }
}

# 163 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void )
#line 163
{
  /* atomic removed: atomic calls only */
#line 164
  {
    if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state != /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 166
        /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;

#line 166
        return __nesc_temp;
      }
#line 167
    {
      unsigned char __nesc_temp = 
#line 167
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId;

#line 167
      return __nesc_temp;
    }
  }
}

# 101 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
__attribute((wakeup)) __attribute((interrupt(4)))  void sig_UART1TX_VECTOR(void )
#line 101
{
  HplMsp430Usart1P__Interrupts__txDone();
}

# 104 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/serial/HdlcTranslateC.nc"
static void HdlcTranslateC__UartStream__sendDone(uint8_t *buf, uint16_t len, 
error_t error)
#line 105
{
  if (HdlcTranslateC__state.sendEscape) {
      HdlcTranslateC__state.sendEscape = 0;
      HdlcTranslateC__m_data = HdlcTranslateC__txTemp;
      HdlcTranslateC__UartStream__send(&HdlcTranslateC__m_data, 1);
    }
  else {
      HdlcTranslateC__SerialFrameComm__putDone();
    }
}

#line 92
static error_t HdlcTranslateC__SerialFrameComm__putData(uint8_t data)
#line 92
{
  if (data == HDLC_CTLESC_BYTE || data == HDLC_FLAG_BYTE) {
      HdlcTranslateC__state.sendEscape = 1;
      HdlcTranslateC__txTemp = data ^ 0x20;
      HdlcTranslateC__m_data = HDLC_CTLESC_BYTE;
    }
  else {
      HdlcTranslateC__m_data = data;
    }
  return HdlcTranslateC__UartStream__send(&HdlcTranslateC__m_data, 1);
}

# 165 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/lib/printf/PrintfP.nc"
__attribute((noinline))   int putchar(int c)
#line 165
{
#line 177
  if (PrintfP__state == PrintfP__S_STARTED && PrintfP__Queue__size() >= 1000 / 2) {
      PrintfP__state = PrintfP__S_FLUSHING;
      PrintfP__sendNext();
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 181
    {
      if (PrintfP__Queue__enqueue(c) == SUCCESS) 
        {
          int __nesc_temp = 
#line 183
          0;

          {
#line 183
            __nesc_atomic_end(__nesc_atomic); 
#line 183
            return __nesc_temp;
          }
        }
      else 
#line 184
        {
          int __nesc_temp = 
#line 184
          -1;

          {
#line 184
            __nesc_atomic_end(__nesc_atomic); 
#line 184
            return __nesc_temp;
          }
        }
    }
#line 187
    __nesc_atomic_end(__nesc_atomic); }
}

# 53 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
__attribute((wakeup)) __attribute((interrupt(8)))  void sig_PORT1_VECTOR(void )
{
  volatile int n = P1IFG & P1IE;

  if (n & (1 << 0)) {
#line 57
      HplMsp430InterruptP__Port10__fired();
#line 57
      return;
    }
#line 58
  if (n & (1 << 1)) {
#line 58
      HplMsp430InterruptP__Port11__fired();
#line 58
      return;
    }
#line 59
  if (n & (1 << 2)) {
#line 59
      HplMsp430InterruptP__Port12__fired();
#line 59
      return;
    }
#line 60
  if (n & (1 << 3)) {
#line 60
      HplMsp430InterruptP__Port13__fired();
#line 60
      return;
    }
#line 61
  if (n & (1 << 4)) {
#line 61
      HplMsp430InterruptP__Port14__fired();
#line 61
      return;
    }
#line 62
  if (n & (1 << 5)) {
#line 62
      HplMsp430InterruptP__Port15__fired();
#line 62
      return;
    }
#line 63
  if (n & (1 << 6)) {
#line 63
      HplMsp430InterruptP__Port16__fired();
#line 63
      return;
    }
#line 64
  if (n & (1 << 7)) {
#line 64
      HplMsp430InterruptP__Port17__fired();
#line 64
      return;
    }
}

#line 158
__attribute((wakeup)) __attribute((interrupt(2)))  void sig_PORT2_VECTOR(void )
{
  volatile int n = P2IFG & P2IE;

  if (n & (1 << 0)) {
#line 162
      HplMsp430InterruptP__Port20__fired();
#line 162
      return;
    }
#line 163
  if (n & (1 << 1)) {
#line 163
      HplMsp430InterruptP__Port21__fired();
#line 163
      return;
    }
#line 164
  if (n & (1 << 2)) {
#line 164
      HplMsp430InterruptP__Port22__fired();
#line 164
      return;
    }
#line 165
  if (n & (1 << 3)) {
#line 165
      HplMsp430InterruptP__Port23__fired();
#line 165
      return;
    }
#line 166
  if (n & (1 << 4)) {
#line 166
      HplMsp430InterruptP__Port24__fired();
#line 166
      return;
    }
#line 167
  if (n & (1 << 5)) {
#line 167
      HplMsp430InterruptP__Port25__fired();
#line 167
      return;
    }
#line 168
  if (n & (1 << 6)) {
#line 168
      HplMsp430InterruptP__Port26__fired();
#line 168
      return;
    }
#line 169
  if (n & (1 << 7)) {
#line 169
      HplMsp430InterruptP__Port27__fired();
#line 169
      return;
    }
}

# 96 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
__attribute((wakeup)) __attribute((interrupt(18)))  void sig_UART0RX_VECTOR(void )
#line 96
{
  uint8_t temp = U0RXBUF;

#line 98
  HplMsp430Usart0P__Interrupts__rxDone(temp);
}

# 150 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/system/ArbiterP.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse(void )
#line 150
{
  /* atomic removed: atomic calls only */
#line 151
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED) 
      {
        unsigned char __nesc_temp = 
#line 153
        FALSE;

#line 153
        return __nesc_temp;
      }
  }
#line 155
  return TRUE;
}






static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId(void )
#line 163
{
  /* atomic removed: atomic calls only */
#line 164
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state != /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 166
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__NO_RES;

#line 166
        return __nesc_temp;
      }
#line 167
    {
      unsigned char __nesc_temp = 
#line 167
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId;

#line 167
      return __nesc_temp;
    }
  }
}

# 101 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
__attribute((wakeup)) __attribute((interrupt(16)))  void sig_UART0TX_VECTOR(void )
#line 101
{
  if (HplMsp430Usart0P__HplI2C__isI2C()) {
    HplMsp430Usart0P__I2CInterrupts__fired();
    }
  else {
#line 105
    HplMsp430Usart0P__Interrupts__txDone();
    }
}

# 72 "/home/david/workspace/tinyos-2.x_tsch_code_scheduling/tos/chips/msp430/dma/HplMsp430DmaP.nc"
__attribute((wakeup)) __attribute((interrupt(0)))  void sig_DACDMA_VECTOR(void )
#line 72
{
  HplMsp430DmaP__Interrupt__fired();
}

