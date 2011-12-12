#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 151 "/opt/msp430-z1/lib/gcc-lib/msp430/3.2.3/include/stddef.h" 3
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
# 38 "/opt/msp430-z1/msp430/include/sys/inttypes.h" 3
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
# 385 "/usr/lib/ncc/nesc_nx.h"
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
# 41 "/opt/msp430-z1/msp430/include/sys/types.h" 3
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
# 40 "/opt/msp430-z1/msp430/include/string.h" 3
extern void *memset(void *arg_0x40291220, int arg_0x40291378, size_t arg_0x40291510);
#line 61
extern void *memset(void *arg_0x4029f118, int arg_0x4029f270, size_t arg_0x4029f408);
# 59 "/opt/msp430-z1/msp430/include/stdlib.h" 3
#line 55
typedef struct __nesc_unnamed4242 {

  int quot;
  int rem;
} div_t;







#line 63
typedef struct __nesc_unnamed4243 {

  long quot;
  long rem;
} ldiv_t;
#line 93
void *malloc(size_t size);
# 122 "/opt/msp430-z1/msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/opt/msp430-z1/msp430/include/sys/_types.h" 3
typedef long _off_t;
typedef long _ssize_t;
# 28 "/opt/msp430-z1/msp430/include/sys/reent.h" 3
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

  void (*__cleanup)(struct _reent *arg_0x402bc820);


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


  void (**_sig_func)(int arg_0x402bfe90);




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/opt/msp430-z1/msp430/include/math.h" 3
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
# 23 "/opt/tinyos-2.1.0/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4247 {
#line 24
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;







struct __nesc_attr_atmostonce {
};
#line 35
struct __nesc_attr_atleastonce {
};
#line 36
struct __nesc_attr_exactlyonce {
};
# 40 "/opt/tinyos-2.1.0/tos/types/TinyError.h"
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
# 39 "/opt/msp430-z1/msp430/include/msp430/iostructures.h" 3
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

  ioregister_t ren;
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
# 128 "/opt/msp430-z1/msp430/include/msp430/gpio.h" 3
volatile unsigned char P1IFG __asm ("0x0023");

volatile unsigned char P1IES __asm ("0x0024");

volatile unsigned char P1IE __asm ("0x0025");
#line 145
volatile unsigned char P2OUT __asm ("0x0029");

volatile unsigned char P2DIR __asm ("0x002A");

volatile unsigned char P2IFG __asm ("0x002B");



volatile unsigned char P2IE __asm ("0x002D");




volatile unsigned char P2REN __asm ("0x002F");









volatile unsigned char P3DIR __asm ("0x001A");
#line 181
volatile unsigned char P4OUT __asm ("0x001D");

volatile unsigned char P4DIR __asm ("0x001E");
# 254 "/opt/msp430-z1/msp430/include/msp430/usci.h" 3
volatile unsigned char UCA0CTL1 __asm ("0x0061");

volatile unsigned char UCA0BR0 __asm ("0x0062");

volatile unsigned char UCA0BR1 __asm ("0x0063");

volatile unsigned char UCA0MCTL __asm ("0x0064");



volatile unsigned char UCA0RXBUF __asm ("0x0066");
#line 277
volatile unsigned char UCB0CTL1 __asm ("0x0069");









volatile unsigned char UCB0RXBUF __asm ("0x006E");
#line 302
volatile unsigned char UCA1CTL1 __asm ("0x00D1");
#line 325
volatile unsigned char UCB1CTL1 __asm ("0x00D9");
# 28 "/opt/msp430-z1/msp430/include/msp430/timera.h" 3
volatile unsigned int TA0CTL __asm ("0x0160");

volatile unsigned int TA0R __asm ("0x0170");


volatile unsigned int TA0CCTL0 __asm ("0x0162");

volatile unsigned int TA0CCTL1 __asm ("0x0164");
#line 71
volatile unsigned int TA0CCTL2 __asm ("0x0166");
#line 132
#line 123
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
#line 148
#line 134
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



  volatile unsigned dummy2[4];
  volatile unsigned tar;
  volatile unsigned taccr0;
  volatile unsigned taccr1;

  volatile unsigned taccr2;
};




struct timera_t;
# 26 "/opt/msp430-z1/msp430/include/msp430/timerb.h" 3
volatile unsigned int TBR __asm ("0x0190");
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
# 20 "/opt/msp430-z1/msp430/include/msp430/basic_clock.h" 3
volatile unsigned char DCOCTL __asm ("0x0056");

volatile unsigned char BCSCTL1 __asm ("0x0057");

volatile unsigned char BCSCTL2 __asm ("0x0058");
# 18 "/opt/msp430-z1/msp430/include/msp430/adc12.h" 3
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
# 93 "/opt/msp430-z1/msp430/include/msp430x261x.h" 3
volatile unsigned char IFG2 __asm ("0x0003");
#line 156
volatile unsigned char CALDCO_8MHZ __asm ("0x10FC");

volatile unsigned char CALBC1_8MHZ __asm ("0x10FD");
# 195 "/opt/tinyos-2.1.0/tos/chips/msp430X/msp430hardware.h"
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
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.h"
enum __nesc_unnamed4258 {
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
typedef struct __nesc_unnamed4259 {

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
typedef struct __nesc_unnamed4260 {

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
typedef struct __nesc_unnamed4261 {

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
# 29 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4262 {
#line 29
  int notUsed;
} 
#line 29
TMilli;
typedef struct __nesc_unnamed4263 {
#line 30
  int notUsed;
} 
#line 30
T32khz;
typedef struct __nesc_unnamed4264 {
#line 31
  int notUsed;
} 
#line 31
TMicro;
# 32 "/opt/tinyos-2.1.0/tos/types/Leds.h"
enum __nesc_unnamed4265 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 88 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/msp430usci.h"
#line 82
typedef enum __nesc_unnamed4266 {

  USCI_NONE = 0, 
  USCI_UART = 1, 
  USCI_SPI = 2, 
  USCI_I2C = 3
} msp430_uscimode_t;









#line 90
typedef struct __nesc_unnamed4267 {
  unsigned int ucsync : 1;
  unsigned int ucmode : 2;
  unsigned int ucspb : 1;
  unsigned int uc7bit : 1;
  unsigned int ucmsb : 1;
  unsigned int ucpar : 1;
  unsigned int ucpen : 1;
} __attribute((packed))  msp430_uctl0_t;









#line 100
typedef struct __nesc_unnamed4268 {
  unsigned int ucswrst : 1;
  unsigned int uctxbrk : 1;
  unsigned int uctxaddr : 1;
  unsigned int ucdorm : 1;
  unsigned int ucbrkie : 1;
  unsigned int ucrxeie : 1;
  unsigned int ucssel : 2;
} __attribute((packed))  msp430_uctl1_t;
#line 136
#line 119
typedef struct __nesc_unnamed4269 {
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





#line 138
typedef struct __nesc_unnamed4270 {
  uint16_t ubr;
  uint8_t uctl0;
  uint8_t uctl1;
} msp430_spi_registers_t;




#line 144
typedef union __nesc_unnamed4271 {
  msp430_spi_config_t spiConfig;
  msp430_spi_registers_t spiRegisters;
} msp430_spi_union_config_t;

msp430_spi_union_config_t msp430_spi_default_config = { 
{ 
#line 161
.ubr = 0x0002, 
.ssel = 0x02, 
.clen = 0, 
.listen = 0, 
.mm = 1, 
.ckph = 1, 
.ckpl = 0, 
.stc = 1 } };
#line 211
#line 182
typedef enum __nesc_unnamed4272 {
  UBR_32KHZ_1200 = 0x001B, UMCTL_32KHZ_1200 = 0x20, 
  UBR_32KHZ_2400 = 0x000D, UMCTL_32KHZ_2400 = 0x60, 
  UBR_32KHZ_4800 = 0x0006, UMCTL_32KHZ_4800 = 0x70, 
  UBR_32KHZ_9600 = 0x0003, UMCTL_32KHZ_9600 = 0x30, 

  UBR_1048MHZ_9600 = 0x006D, UMCTL_1048MHZ_9600 = 0x20, 
  UBR_1048MHZ_19200 = 0x0036, UMCTL_1048MHZ_19200 = 0x50, 
  UBR_1048MHZ_38400 = 0x001B, UMCTL_1048MHZ_38400 = 0x20, 
  UBR_1048MHZ_56000 = 0x0012, UMCTL_1048MHZ_56000 = 0x60, 
  UBR_1048MHZ_115200 = 0x0009, UMCTL_1048MHZ_115200 = 0x10, 
  UBR_1048MHZ_128000 = 0x0008, UMCTL_1048MHZ_128000 = 0x10, 
  UBR_1048MHZ_256000 = 0x0004, UMCTL_1048MHZ_230400 = 0x10, 

  UBR_1MHZ_9600 = 0x0068, UMCTL_1MHZ_9600 = 0x10, 
  UBR_1MHZ_19200 = 0x0034, UMCTL_1MHZ_19200 = 0x00, 
  UBR_1MHZ_38400 = 0x001A, UMCTL_1MHZ_38400 = 0x00, 
  UBR_1MHZ_56000 = 0x0011, UMCTL_1MHZ_56000 = 0x70, 
  UBR_1MHZ_115200 = 0x0008, UMCTL_1MHZ_115200 = 0x60, 
  UBR_1MHZ_128000 = 0x0007, UMCTL_1MHZ_128000 = 0x70, 
  UBR_1MHZ_256000 = 0x0003, UMCTL_1MHZ_230400 = 0x70, 

  UBR_8MHZ_9600 = 0x0341, UMCTL_8MHZ_9600 = 0x20, 
  UBR_8MHZ_19200 = 0x01A0, UMCTL_8MHZ_19200 = 0x60, 
  UBR_8MHZ_38400 = 0x00D0, UMCTL_8MHZ_38400 = 0x30, 
  UBR_8MHZ_56000 = 0x008E, UMCTL_8MHZ_56000 = 0x70, 
  UBR_8MHZ_115200 = 0x0045, UMCTL_8MHZ_115200 = 0x08, 
  UBR_8MHZ_128000 = 0x003E, UMCTL_8MHZ_128000 = 0x40, 
  UBR_8MHZ_256000 = 0x001F, UMCTL_8MHZ_230400 = 0x20
} msp430_uart_rate_t;
#line 234
#line 213
typedef struct __nesc_unnamed4273 {
  unsigned int ubr : 16;

  unsigned int umctl : 8;

  unsigned int  : 1;
  unsigned int ucmode : 2;
  unsigned int ucspb : 1;
  unsigned int uc7bit : 1;
  unsigned int  : 1;
  unsigned int ucpar : 1;
  unsigned int ucpen : 1;

  unsigned int  : 5;
  unsigned int ucrxeie : 1;
  unsigned int ucssel : 2;



  unsigned int utxe : 1;
  unsigned int urxe : 1;
} msp430_uart_config_t;







#line 236
typedef struct __nesc_unnamed4274 {
  uint16_t ubr;
  uint8_t umctl;
  uint8_t uctl0;
  uint8_t uctl1;
  uint8_t ume;
} msp430_uart_registers_t;




#line 244
typedef union __nesc_unnamed4275 {
  msp430_uart_config_t uartConfig;
  msp430_uart_registers_t uartRegisters;
} msp430_uart_union_config_t;
#line 280
#line 272
typedef struct __nesc_unnamed4276 {
  unsigned int i2cstt : 1;
  unsigned int i2cstp : 1;
  unsigned int i2cstb : 1;
  unsigned int i2cctrx : 1;
  unsigned int i2cssel : 2;
  unsigned int i2ccrm : 1;
  unsigned int i2cword : 1;
} __attribute((packed))  msp430_i2ctctl_t;
#line 308
#line 285
typedef struct __nesc_unnamed4277 {
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








#line 310
typedef struct __nesc_unnamed4278 {
  uint8_t uctl;
  uint8_t i2ctctl;
  uint8_t i2cpsc;
  uint8_t i2csclh;
  uint8_t i2cscll;
  uint16_t i2coa;
} msp430_i2c_registers_t;




#line 319
typedef union __nesc_unnamed4279 {
  msp430_i2c_config_t i2cConfig;
  msp430_i2c_registers_t i2cRegisters;
} msp430_i2c_union_config_t;
# 33 "/opt/tinyos-2.1.0/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
typedef TMilli fabMACP__Timer__precision_tag;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC__0____nesc_unnamed4280 {
  Msp430Timer32khzC__0__ALARM_ID = 0U
};
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__frequency_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__frequency_tag /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type;
typedef T32khz /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag;
typedef /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__precision_tag;
typedef uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type;
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
typedef TMicro RF231driverP__radioWait__precision_tag;
typedef uint16_t RF231driverP__radioWait__size_type;
typedef TMicro /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__precision_tag;
typedef uint16_t /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__precision_tag /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__precision_tag;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__size_type;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__precision_tag /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__precision_tag;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__size_type;
typedef TMicro /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__frequency_tag;
typedef /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__frequency_tag /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__precision_tag;
typedef uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__size_type;
enum /*RF231driverC.RF231SpiC*/RF231SpiC__0____nesc_unnamed4281 {
  RF231SpiC__0__CLIENT_ID = 0U
};
typedef TMicro RF231SpiP__spiWait__precision_tag;
typedef uint16_t RF231SpiP__spiWait__size_type;
enum /*RF231SpiWireC.HplRF231SpiC.SpiC*/Msp430SpiA0C__0____nesc_unnamed4282 {
  Msp430SpiA0C__0__CLIENT_ID = 0U
};
enum /*RF231SpiWireC.HplRF231SpiC.SpiC.UsciC*/Msp430UsciA0C__0____nesc_unnamed4283 {
  Msp430UsciA0C__0__CLIENT_ID = 0U
};
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t PlatformP__Init__init(void );
# 35 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 32
static void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );



static void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 31
static void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );





static void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 30
static void Msp430ClockP__Msp430ClockInit__default__initClocks(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t Msp430ClockP__Init__init(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(
# 40 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerP.nc"
uint8_t arg_0x404d0ce8);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(
# 40 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerP.nc"
uint8_t arg_0x404d0ce8);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
static bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
# 33 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 33 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 33 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 33 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );
#line 36
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );
# 30 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t delta);
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 33 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 33 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 33 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 33 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 33 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 33 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(
# 45 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x40441ae8);
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
# 45 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x40441ae8);
# 46 "/opt/tinyos-2.1.0/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP__Scheduler__init(void );
#line 61
static void SchedulerBasicP__Scheduler__taskLoop(void );
#line 54
static bool SchedulerBasicP__Scheduler__runNextTask(void );
# 54 "/opt/tinyos-2.1.0/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC__McuPowerOverride__default__lowestState(void );
# 59 "/opt/tinyos-2.1.0/tos/interfaces/McuSleep.nc"
static void McuSleepC__McuSleep__sleep(void );
# 49 "/opt/tinyos-2.1.0/tos/interfaces/Boot.nc"
static void fabMACP__Boot__booted(void );
# 3 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RadioControl.nc"
static void fabMACP__RadioControl__startDone(error_t error);

static void fabMACP__RadioControl__prepareReceiveDone(error_t error);




static void fabMACP__RadioControl__stopDone(error_t error);
# 6 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RadioReceive.nc"
static void fabMACP__RadioReceive__receive(uint8_t *msg);
# 6 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RadioSend.nc"
static void fabMACP__RadioSend__prepareSendDone(error_t error);

static void fabMACP__RadioSend__sendNowDone(error_t error);
# 72 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static void fabMACP__Timer__fired(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t LedsP__Init__init(void );
# 56 "/opt/tinyos-2.1.0/tos/platforms/gina20/Leds.nc"
static void LedsP__Leds__led0Toggle(void );
#line 72
static void LedsP__Leds__led1Toggle(void );
#line 89
static void LedsP__Leds__led2Toggle(void );
#line 105
static void LedsP__Leds__led3Toggle(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
static void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__toggle(void );
#line 71
static void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__makeOutput(void );
#line 59
static bool /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__getRaw(void );
#line 39
static void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__clr(void );




static void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__toggle(void );
#line 71
static void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__makeOutput(void );
#line 59
static bool /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__getRaw(void );
#line 39
static void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__clr(void );




static void /*HplMsp430GeneralIOC.P22*/HplMsp430GeneralIOP__10__IO__toggle(void );
#line 71
static void /*HplMsp430GeneralIOC.P22*/HplMsp430GeneralIOP__10__IO__makeOutput(void );
#line 59
static bool /*HplMsp430GeneralIOC.P22*/HplMsp430GeneralIOP__10__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P22*/HplMsp430GeneralIOP__10__IO__getRaw(void );
#line 39
static void /*HplMsp430GeneralIOC.P22*/HplMsp430GeneralIOP__10__IO__clr(void );




static void /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__toggle(void );
#line 71
static void /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__makeOutput(void );
#line 59
static bool /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__getRaw(void );
#line 39
static void /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__clr(void );
#line 85
static void /*HplMsp430GeneralIOC.P30*/HplMsp430GeneralIOP__16__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P30*/HplMsp430GeneralIOP__16__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectModuleFunc(void );
# 31 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle(void );
static bool /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__get(void );


static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
#line 30
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr(void );
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle(void );
static bool /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__get(void );


static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
#line 30
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr(void );
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle(void );
static bool /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__get(void );


static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
#line 30
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void );
static void /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__GeneralIO__toggle(void );
static bool /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__GeneralIO__get(void );


static void /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__GeneralIO__makeOutput(void );
#line 30
static void /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__GeneralIO__clr(void );
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 92 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type dt);
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
static /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );






static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
#line 53
static /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
# 98 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type dt);
#line 105
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
# 71 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );
# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
# 125 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
#line 118
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
# 72 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );
#line 72
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(
# 37 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x407969f0);
# 62 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(
# 37 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x407969f0, 
# 62 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
uint32_t dt);
# 71 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 6 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RadioControl.nc"
static error_t RF231driverP__RadioControl__receiveNow(void );
#line 2
static error_t RF231driverP__RadioControl__start(void );






static error_t RF231driverP__RadioControl__stop(void );
# 11 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/ChipSpiResource.nc"
static void RF231driverP__ChipSpiResource__loadPacketDone(error_t error);

static void RF231driverP__ChipSpiResource__doneSettingChannel(void );
#line 2
static void RF231driverP__ChipSpiResource__releasing(void );
# 57 "/opt/tinyos-2.1.0/tos/interfaces/GpioInterrupt.nc"
static void RF231driverP__InterruptIRQRFP__fired(void );
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void RF231driverP__taskSendDone__runTask(void );
# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static void RF231driverP__SpiResource__granted(void );
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void RF231driverP__taskStopDone__runTask(void );
# 5 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RadioSend.nc"
static error_t RF231driverP__RadioSend__prepareSend(uint8_t *msg);

static error_t RF231driverP__RadioSend__sendNow(void );
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void RF231driverP__taskStartDone__runTask(void );
# 55 "/opt/tinyos-2.1.0/tos/lib/timer/BusyWait.nc"
static void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__wait(/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__size_type dt);
# 71 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
static void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow(void );
# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
static /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__size_type /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get(void );
# 71 "/opt/tinyos-2.1.0/tos/interfaces/SpiPacket.nc"
static void RF231SpiP__SpiPacket__sendDone(
#line 64
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 6 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/ChipSpiResource.nc"
static void RF231SpiP__ChipSpiResource__changeState(uint8_t newState);
static void RF231SpiP__ChipSpiResource__setChannel(uint8_t newChannel);

static void RF231SpiP__ChipSpiResource__setRegister(uint8_t regAddr, uint8_t newValue);
static void RF231SpiP__ChipSpiResource__loadPacket(uint8_t *data, uint8_t len);
#line 8
static uint8_t RF231SpiP__ChipSpiResource__readRegister(uint8_t registerAddress);



static void RF231SpiP__ChipSpiResource__readPacket(uint8_t *data);
# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static void RF231SpiP__SpiResource__granted(void );
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void RF231SpiP__grant__runTask(void );
# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t RF231SpiP__Resource__release(
# 5 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/spi/RF231SpiP.nc"
uint8_t arg_0x408569b8);
# 87 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t RF231SpiP__Resource__immediateRequest(
# 5 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/spi/RF231SpiP.nc"
uint8_t arg_0x408569b8);
# 78 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t RF231SpiP__Resource__request(
# 5 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/spi/RF231SpiP.nc"
uint8_t arg_0x408569b8);
# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static void RF231SpiP__Resource__default__granted(
# 5 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/spi/RF231SpiP.nc"
uint8_t arg_0x408569b8);
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t StateImplP__Init__init(void );
# 56 "/opt/tinyos-2.1.0/tos/interfaces/State.nc"
static void StateImplP__State__toIdle(
# 67 "/opt/tinyos-2.1.0/tos/system/StateImplP.nc"
uint8_t arg_0x40888378);
# 66 "/opt/tinyos-2.1.0/tos/interfaces/State.nc"
static bool StateImplP__State__isState(
# 67 "/opt/tinyos-2.1.0/tos/system/StateImplP.nc"
uint8_t arg_0x40888378, 
# 66 "/opt/tinyos-2.1.0/tos/interfaces/State.nc"
uint8_t myState);
#line 61
static bool StateImplP__State__isIdle(
# 67 "/opt/tinyos-2.1.0/tos/system/StateImplP.nc"
uint8_t arg_0x40888378);
# 45 "/opt/tinyos-2.1.0/tos/interfaces/State.nc"
static error_t StateImplP__State__requestState(
# 67 "/opt/tinyos-2.1.0/tos/system/StateImplP.nc"
uint8_t arg_0x40888378, 
# 45 "/opt/tinyos-2.1.0/tos/interfaces/State.nc"
uint8_t reqState);
# 55 "/opt/tinyos-2.1.0/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__ResourceConfigure__unconfigure(
# 4 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
uint8_t arg_0x408d6790);
# 49 "/opt/tinyos-2.1.0/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__ResourceConfigure__configure(
# 4 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
uint8_t arg_0x408d6790);
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__signalDone_task__runTask(void );
# 59 "/opt/tinyos-2.1.0/tos/interfaces/SpiPacket.nc"
static error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__SpiPacket__send(
# 6 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
uint8_t arg_0x408d52b0, 
# 48 "/opt/tinyos-2.1.0/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
#line 71
static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__SpiPacket__default__sendDone(
# 6 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
uint8_t arg_0x408d52b0, 
# 64 "/opt/tinyos-2.1.0/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 71 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Msp430SpiConfigure__default__getConfig(
# 9 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
uint8_t arg_0x408d4500);
# 34 "/opt/tinyos-2.1.0/tos/interfaces/SpiByte.nc"
static uint8_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__SpiByte__write(uint8_t tx);
# 85 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciInterrupts__rxDone(uint8_t data);
#line 80
static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciInterrupts__txDone(void );
# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Resource__release(
# 3 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
uint8_t arg_0x408bbca0);
# 87 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Resource__immediateRequest(
# 3 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
uint8_t arg_0x408bbca0);
# 78 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Resource__request(
# 3 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
uint8_t arg_0x408bbca0);
# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Resource__default__granted(
# 3 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
uint8_t arg_0x408bbca0);
# 118 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Resource__isOwner(
# 3 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
uint8_t arg_0x408bbca0);
# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__default__release(
# 8 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
uint8_t arg_0x408d5aa0);
# 87 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__default__immediateRequest(
# 8 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
uint8_t arg_0x408d5aa0);
# 78 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__default__request(
# 8 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
uint8_t arg_0x408d5aa0);
# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__granted(
# 8 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
uint8_t arg_0x408d5aa0);
# 118 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__default__isOwner(
# 8 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
uint8_t arg_0x408d5aa0);
# 85 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciRawInterrupts.nc"
static void HplMsp430UsciA0P__UsciRawInterrupts__rxDone(uint8_t data);
#line 80
static void HplMsp430UsciA0P__UsciRawInterrupts__txDone(void );
# 131 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA.nc"
static void HplMsp430UsciA0P__Usci__resetUsci(bool reset);
#line 164
static void HplMsp430UsciA0P__Usci__enableRxIntr(void );
#line 181
static void HplMsp430UsciA0P__Usci__clrRxIntr(void );
#line 163
static void HplMsp430UsciA0P__Usci__disableIntr(void );
#line 112
static void HplMsp430UsciA0P__Usci__setUmctl(uint8_t umctl);
#line 161
static void HplMsp430UsciA0P__Usci__disableRxIntr(void );
#line 191
static void HplMsp430UsciA0P__Usci__clrIntr(void );
#line 102
static void HplMsp430UsciA0P__Usci__setUbr(uint16_t ubr);
#line 202
static void HplMsp430UsciA0P__Usci__tx(uint8_t data);
#line 143
static void HplMsp430UsciA0P__Usci__enableSpi(void );
#line 158
static void HplMsp430UsciA0P__Usci__setModeSpi(msp430_spi_union_config_t *config);
#line 209
static uint8_t HplMsp430UsciA0P__Usci__rx(void );
#line 176
static bool HplMsp430UsciA0P__Usci__isRxIntrPending(void );
#line 148
static void HplMsp430UsciA0P__Usci__disableSpi(void );
# 85 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciRawInterrupts.nc"
static void HplMsp430UsciAB0RawInterruptsP__UsciB__default__rxDone(uint8_t data);
#line 80
static void HplMsp430UsciAB0RawInterruptsP__UsciB__default__txDone(void );
# 85 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__rxDone(
# 70 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
uint8_t arg_0x409701b8, 
# 85 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
uint8_t data);
#line 80
static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__txDone(
# 70 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
uint8_t arg_0x409701b8);
# 85 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__rxDone(uint8_t data);
#line 80
static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__txDone(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void );
# 69 "/opt/tinyos-2.1.0/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id);
#line 43
static bool /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );








static bool /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
# 43 "/opt/tinyos-2.1.0/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(
# 55 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
uint8_t arg_0x4098fda8);
# 51 "/opt/tinyos-2.1.0/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(
# 55 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
uint8_t arg_0x4098fda8);
# 55 "/opt/tinyos-2.1.0/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(
# 60 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
uint8_t arg_0x4098d1b8);
# 49 "/opt/tinyos-2.1.0/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(
# 60 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
uint8_t arg_0x4098d1b8);
# 56 "/opt/tinyos-2.1.0/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
#line 73
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested(void );
#line 46
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void );
#line 81
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested(void );
# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(
# 54 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
uint8_t arg_0x4098f380);
# 87 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(
# 54 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
uint8_t arg_0x4098f380);
# 78 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(
# 54 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
uint8_t arg_0x4098f380);
# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(
# 54 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
uint8_t arg_0x4098f380);
# 118 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static bool /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(
# 54 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
uint8_t arg_0x4098f380);
# 80 "/opt/tinyos-2.1.0/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
# 41 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
static void HplMsp430InterruptP__Port14__clear(void );
#line 61
static void HplMsp430InterruptP__Port14__default__fired(void );
#line 41
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
#line 61
static void HplMsp430InterruptP__Port27__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port10__clear(void );
#line 61
static void HplMsp430InterruptP__Port10__default__fired(void );
#line 41
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
#line 36
static void HplMsp430InterruptP__Port16__disable(void );
#line 56
static void HplMsp430InterruptP__Port16__edge(bool low_to_high);
#line 31
static void HplMsp430InterruptP__Port16__enable(void );









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
#line 61
static void /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__HplInterrupt__fired(void );
# 50 "/opt/tinyos-2.1.0/tos/interfaces/GpioInterrupt.nc"
static error_t /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__Interrupt__disable(void );
#line 42
static error_t /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t PlatformP__Msp430ClockInit__init(void );
#line 51
static error_t PlatformP__LedsInit__init(void );
# 9 "/opt/tinyos-2.1.0/tos/platforms/gina20/PlatformP.nc"
static inline error_t PlatformP__Init__init(void );
# 32 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__initTimerB(void );
#line 31
static void Msp430ClockP__Msp430ClockInit__initTimerA(void );
#line 30
static void Msp430ClockP__Msp430ClockInit__initClocks(void );
# 73 "/opt/tinyos-2.1.0/tos/platforms/gina20/Msp430ClockP.nc"
static volatile uint8_t Msp430ClockP__IE1 __asm ("0x0000");
static volatile uint16_t Msp430ClockP__TA0CTL __asm ("0x0160");
static volatile uint16_t Msp430ClockP__TA0IV __asm ("0x012E");
static volatile uint16_t Msp430ClockP__TBCTL __asm ("0x0180");
static volatile uint16_t Msp430ClockP__TBIV __asm ("0x011E");

enum Msp430ClockP____nesc_unnamed4284 {

  Msp430ClockP__ACLK_CALIB_PERIOD = 8, 
  Msp430ClockP__TARGET_DCO_DELTA = 8192 / 128 * Msp430ClockP__ACLK_CALIB_PERIOD
};
#line 99
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 136
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 152
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 173
static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );





static inline void Msp430ClockP__startTimerA(void );
#line 201
static inline void Msp430ClockP__startTimerB(void );
#line 267
static inline error_t Msp430ClockP__Init__init(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(
# 40 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerP.nc"
uint8_t arg_0x404d0ce8);
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void );
# 51 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void );
#line 115
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n);
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(
# 40 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerP.nc"
uint8_t arg_0x404d0ce8);
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void );
# 51 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
#line 70
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
#line 115
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );








static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n);
# 75 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
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
# 75 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
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
# 75 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
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
# 75 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void );
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
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
# 75 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t;


static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 75 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t;


static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 75 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t;


static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 75 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t;


static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 75 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t;


static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 75 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t;


static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void Msp430TimerCommonP__VectorTimerB1__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerA0__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerA1__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerB0__fired(void );
# 11 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCommonP.nc"
void sig_TIMERA0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(50)))  ;









void sig_TIMERA1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(48)))  ;









void sig_TIMERB0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(58)))  ;









void sig_TIMERB1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(56)))  ;
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t RealMainP__SoftwareInit__init(void );
# 49 "/opt/tinyos-2.1.0/tos/interfaces/Boot.nc"
static void RealMainP__Boot__booted(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t RealMainP__PlatformInit__init(void );
# 46 "/opt/tinyos-2.1.0/tos/interfaces/Scheduler.nc"
static void RealMainP__Scheduler__init(void );
#line 61
static void RealMainP__Scheduler__taskLoop(void );
#line 54
static bool RealMainP__Scheduler__runNextTask(void );
# 52 "/opt/tinyos-2.1.0/tos/system/RealMainP.nc"
int main(void )   ;
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(
# 45 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x40441ae8);
# 59 "/opt/tinyos-2.1.0/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP__McuSleep__sleep(void );
# 50 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4285 {

  SchedulerBasicP__NUM_TASKS = 8U, 
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
# 54 "/opt/tinyos-2.1.0/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void );
# 53 "/opt/tinyos-2.1.0/tos/chips/msp430X/McuSleepC.nc"
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
#line 102
static inline void McuSleepC__computePowerState(void );




static inline void McuSleepC__McuSleep__sleep(void );
#line 133
static inline mcu_power_t McuSleepC__McuPowerOverride__default__lowestState(void );
# 6 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RadioControl.nc"
static error_t fabMACP__RadioControl__receiveNow(void );
#line 2
static error_t fabMACP__RadioControl__start(void );






static error_t fabMACP__RadioControl__stop(void );
# 56 "/opt/tinyos-2.1.0/tos/platforms/gina20/Leds.nc"
static void fabMACP__Leds__led0Toggle(void );
#line 72
static void fabMACP__Leds__led1Toggle(void );
#line 89
static void fabMACP__Leds__led2Toggle(void );
#line 105
static void fabMACP__Leds__led3Toggle(void );
# 5 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RadioSend.nc"
static error_t fabMACP__RadioSend__prepareSend(uint8_t *msg);

static error_t fabMACP__RadioSend__sendNow(void );
# 62 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static void fabMACP__Timer__startOneShot(uint32_t dt);
# 12 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/02-MAC/fabMACP.nc"
static inline void fabMACP__Boot__booted(void );








static inline void fabMACP__RadioControl__startDone(error_t error);
#line 38
static inline void fabMACP__RadioSend__prepareSendDone(error_t error);




static inline void fabMACP__RadioSend__sendNowDone(error_t error);



static inline void fabMACP__Timer__fired(void );





static inline void fabMACP__RadioControl__stopDone(error_t error);






static inline void fabMACP__RadioReceive__receive(uint8_t *msg);



static inline void fabMACP__RadioControl__prepareReceiveDone(error_t error);
# 31 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
static void LedsP__Led0__toggle(void );
static bool LedsP__Led0__get(void );


static void LedsP__Led0__makeOutput(void );
#line 30
static void LedsP__Led0__clr(void );
static void LedsP__Led3__toggle(void );
static bool LedsP__Led3__get(void );


static void LedsP__Led3__makeOutput(void );
#line 30
static void LedsP__Led3__clr(void );
static void LedsP__Led1__toggle(void );
static bool LedsP__Led1__get(void );


static void LedsP__Led1__makeOutput(void );
#line 30
static void LedsP__Led1__clr(void );
static void LedsP__Led2__toggle(void );
static bool LedsP__Led2__get(void );


static void LedsP__Led2__makeOutput(void );
#line 30
static void LedsP__Led2__clr(void );
# 50 "/opt/tinyos-2.1.0/tos/platforms/gina20/LedsP.nc"
static inline error_t LedsP__Init__init(void );
#line 75
static inline void LedsP__Leds__led0Toggle(void );
#line 95
static inline void LedsP__Leds__led1Toggle(void );
#line 113
static inline void LedsP__Leds__led2Toggle(void );
#line 131
static inline void LedsP__Leds__led3Toggle(void );
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__clr(void );
static void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__toggle(void );
static inline uint8_t /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__get(void );


static inline void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__makeOutput(void );
#line 47
static inline void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__clr(void );
static void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__toggle(void );
static inline uint8_t /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__get(void );


static inline void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__makeOutput(void );
#line 47
static inline void /*HplMsp430GeneralIOC.P22*/HplMsp430GeneralIOP__10__IO__clr(void );
static void /*HplMsp430GeneralIOC.P22*/HplMsp430GeneralIOP__10__IO__toggle(void );
static inline uint8_t /*HplMsp430GeneralIOC.P22*/HplMsp430GeneralIOP__10__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P22*/HplMsp430GeneralIOP__10__IO__get(void );


static inline void /*HplMsp430GeneralIOC.P22*/HplMsp430GeneralIOP__10__IO__makeOutput(void );
#line 47
static inline void /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__clr(void );
static void /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__toggle(void );
static inline uint8_t /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__get(void );


static inline void /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__makeOutput(void );

static inline void /*HplMsp430GeneralIOC.P30*/HplMsp430GeneralIOP__16__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P30*/HplMsp430GeneralIOP__16__IO__selectIOFunc(void );
#line 55
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void );
#line 55
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__toggle(void );
#line 71
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void );
#line 59
static bool /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__get(void );
#line 39
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__clr(void );
# 38 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr(void );
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle(void );
static inline bool /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__get(void );


static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__toggle(void );
#line 71
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void );
#line 59
static bool /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__get(void );
#line 39
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__clr(void );
# 38 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr(void );
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle(void );
static inline bool /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__get(void );


static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__toggle(void );
#line 71
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void );
#line 59
static bool /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__get(void );
#line 39
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr(void );
# 38 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void );
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle(void );
static inline bool /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__get(void );


static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__HplGeneralIO__toggle(void );
#line 71
static void /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__HplGeneralIO__makeOutput(void );
#line 59
static bool /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__HplGeneralIO__get(void );
#line 39
static void /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__HplGeneralIO__clr(void );
# 38 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__GeneralIO__clr(void );
static inline void /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__GeneralIO__toggle(void );
static inline bool /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__GeneralIO__get(void );


static inline void /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__GeneralIO__makeOutput(void );
# 30 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time);

static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void );
# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void );
# 46 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void );
#line 36
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void );










static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void );
#line 33
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 42 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
#line 54
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );




static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void );
static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void );
# 71 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void );
# 38 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );




static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );









static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void );






static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void );
# 56 "/opt/tinyos-2.1.0/tos/lib/timer/TransformCounterC.nc"
/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC__0____nesc_unnamed4286 {

  TransformCounterC__0__LOW_SHIFT_RIGHT = 5, 
  TransformCounterC__0__HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT, 
  TransformCounterC__0__NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) + 5, 



  TransformCounterC__0__OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
#line 122
static inline void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void );
#line 92
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt);
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void );
# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void );
# 66 "/opt/tinyos-2.1.0/tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0____nesc_unnamed4287 {

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
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void );
# 98 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt);
#line 105
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void );
# 72 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void );
# 63 "/opt/tinyos-2.1.0/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_unnamed4288 {
#line 63
  AlarmToTimerC__0__fired = 0U
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
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void );
# 125 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void );
#line 118
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(
# 37 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x407969f0);
#line 60
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4289 {
#line 60
  VirtualizeTimerC__0__updateFromTimer = 1U
};
#line 60
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer];
#line 42
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4290 {

  VirtualizeTimerC__0__NUM_TIMERS = 1U, 
  VirtualizeTimerC__0__END_OF_LIST = 255
};








#line 48
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4291 {

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
#line 193
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num);
# 47 "/opt/tinyos-2.1.0/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 3 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RadioControl.nc"
static void RF231driverP__RadioControl__startDone(error_t error);

static void RF231driverP__RadioControl__prepareReceiveDone(error_t error);




static void RF231driverP__RadioControl__stopDone(error_t error);
# 6 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/ChipSpiResource.nc"
static void RF231driverP__ChipSpiResource__changeState(uint8_t newState);
static void RF231driverP__ChipSpiResource__setChannel(uint8_t newChannel);

static void RF231driverP__ChipSpiResource__setRegister(uint8_t regAddr, uint8_t newValue);
static void RF231driverP__ChipSpiResource__loadPacket(uint8_t *data, uint8_t len);
#line 8
static uint8_t RF231driverP__ChipSpiResource__readRegister(uint8_t registerAddress);



static void RF231driverP__ChipSpiResource__readPacket(uint8_t *data);
# 50 "/opt/tinyos-2.1.0/tos/interfaces/GpioInterrupt.nc"
static error_t RF231driverP__InterruptIRQRFP__disable(void );
#line 42
static error_t RF231driverP__InterruptIRQRFP__enableRisingEdge(void );
# 6 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RadioReceive.nc"
static void RF231driverP__RadioReceive__receive(uint8_t *msg);
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t RF231driverP__taskSendDone__postTask(void );
# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t RF231driverP__SpiResource__release(void );
#line 87
static error_t RF231driverP__SpiResource__immediateRequest(void );
#line 78
static error_t RF231driverP__SpiResource__request(void );
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t RF231driverP__taskStopDone__postTask(void );
# 55 "/opt/tinyos-2.1.0/tos/lib/timer/BusyWait.nc"
static void RF231driverP__radioWait__wait(RF231driverP__radioWait__size_type dt);
# 72 "/opt/tinyos-2.1.0/tos/platforms/gina20/Leds.nc"
static void RF231driverP__Leds__led1Toggle(void );
# 6 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RadioSend.nc"
static void RF231driverP__RadioSend__prepareSendDone(error_t error);

static void RF231driverP__RadioSend__sendNowDone(error_t error);
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t RF231driverP__taskStartDone__postTask(void );
# 59 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RF231driverP.nc"
enum RF231driverP____nesc_unnamed4292 {
#line 59
  RF231driverP__taskStartDone = 2U
};
#line 59
typedef int RF231driverP____nesc_sillytask_taskStartDone[RF231driverP__taskStartDone];
enum RF231driverP____nesc_unnamed4293 {
#line 60
  RF231driverP__taskStopDone = 3U
};
#line 60
typedef int RF231driverP____nesc_sillytask_taskStopDone[RF231driverP__taskStopDone];
#line 359
enum RF231driverP____nesc_unnamed4294 {
#line 359
  RF231driverP__taskSendDone = 4U
};
#line 359
typedef int RF231driverP____nesc_sillytask_taskSendDone[RF231driverP__taskSendDone];
#line 20
uint8_t RF231driverP__radioState;
uint8_t RF231driverP__driverState;

uint8_t RF231driverP__frequencyChannel;
uint8_t RF231driverP__globalCommand;
uint8_t RF231driverP__globalNewReg;
uint8_t RF231driverP__globalFrequency;
uint8_t RF231driverP__returnedRegValue;
uint8_t RF231driverP__internalSpiState;
uint8_t RF231driverP__statusRg;

bool RF231driverP__statusRead;
bool RF231driverP__preparingPacket;

uint8_t *RF231driverP__globalMsg;
uint8_t *RF231driverP__globalReceivedMsg;
bool RF231driverP__isPacketRead;


enum RF231driverP____nesc_unnamed4295 {
  RF231driverP__CHANGINGREG = 0, 
  RF231driverP__READINGREG = 1, 
  RF231driverP__CHANGINGFREQ = 2, 
  RF231driverP__LOADINGPACKET = 3, 
  RF231driverP__READINGPACKET = 4, 
  RF231driverP__CHANGINGSTATE = 5, 
  RF231driverP__READINGSTATUS = 6, 
  RF231driverP__DEFAULT = 7
};

enum RF231driverP____nesc_unnamed4296 {
  RF231driverP__MISC = 0, 
  RF231driverP__STARTING = 1, 
  RF231driverP__RECEIVING = 2, 
  RF231driverP__TRANSMITTING = 3
};





static void RF231driverP__shutdown(void );
static void RF231driverP__ISRserviced(void );



static inline void RF231driverP__SpiResource__granted(void );
#line 122
static inline void RF231driverP__ChipSpiResource__releasing(void );




static inline error_t RF231driverP__RadioControl__start(void );
#line 252
static inline void RF231driverP__taskStartDone__runTask(void );






static inline error_t RF231driverP__RadioSend__prepareSend(uint8_t *msg);
#line 287
static void RF231driverP__ChipSpiResource__loadPacketDone(error_t error);
#line 332
static inline void RF231driverP__ChipSpiResource__doneSettingChannel(void );






static inline error_t RF231driverP__RadioSend__sendNow(void );
#line 359
static inline void RF231driverP__taskSendDone__runTask(void );
#line 394
static inline error_t RF231driverP__RadioControl__receiveNow(void );
#line 417
static inline void RF231driverP__receivedSomething(void );






static inline void RF231driverP__unloadPacket(void );
#line 448
static inline error_t RF231driverP__RadioControl__stop(void );
#line 482
static void RF231driverP__shutdown(void );







static inline void RF231driverP__taskStopDone__runTask(void );
#line 502
static inline void RF231driverP__InterruptIRQRFP__fired(void );
#line 524
static void RF231driverP__ISRserviced(void );
# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
static /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get(void );
# 47 "/opt/tinyos-2.1.0/tos/lib/timer/BusyWaitCounterC.nc"
enum /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0____nesc_unnamed4297 {

  BusyWaitCounterC__0__HALF_MAX_SIZE_TYPE = (/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type )1 << (8 * sizeof(/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type ) - 1)
};

static void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__wait(/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type dt);
#line 72
static inline void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow(void );
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
static uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__get(void );
# 71 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
static void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__overflow(void );
# 38 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get(void );
#line 53
static inline void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow(void );
# 59 "/opt/tinyos-2.1.0/tos/interfaces/SpiPacket.nc"
static error_t RF231SpiP__SpiPacket__send(
#line 48
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
# 56 "/opt/tinyos-2.1.0/tos/interfaces/State.nc"
static void RF231SpiP__WorkingState__toIdle(void );




static bool RF231SpiP__WorkingState__isIdle(void );
#line 45
static error_t RF231SpiP__WorkingState__requestState(uint8_t reqState);
# 11 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/ChipSpiResource.nc"
static void RF231SpiP__ChipSpiResource__loadPacketDone(error_t error);

static void RF231SpiP__ChipSpiResource__doneSettingChannel(void );
#line 2
static void RF231SpiP__ChipSpiResource__releasing(void );
# 34 "/opt/tinyos-2.1.0/tos/interfaces/SpiByte.nc"
static uint8_t RF231SpiP__SpiByte__write(uint8_t tx);
# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t RF231SpiP__SpiResource__release(void );
#line 87
static error_t RF231SpiP__SpiResource__immediateRequest(void );
#line 78
static error_t RF231SpiP__SpiResource__request(void );
#line 118
static bool RF231SpiP__SpiResource__isOwner(void );
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t RF231SpiP__grant__postTask(void );
# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static void RF231SpiP__Resource__granted(
# 5 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/spi/RF231SpiP.nc"
uint8_t arg_0x408569b8);
#line 54
enum RF231SpiP____nesc_unnamed4298 {
#line 54
  RF231SpiP__grant = 5U
};
#line 54
typedef int RF231SpiP____nesc_sillytask_grant[RF231SpiP__grant];
#line 20
enum RF231SpiP____nesc_unnamed4299 {
  RF231SpiP__RESOURCE_COUNT = 1U, 
  RF231SpiP__NO_HOLDER = 0xFF
};


enum RF231SpiP____nesc_unnamed4300 {
  RF231SpiP__S_IDLE, 
  RF231SpiP__S_BUSY
};


enum RF231SpiP____nesc_unnamed4301 {
  RF231SpiP__WAS_READING, 
  RF231SpiP__WAS_WRITING
};





uint8_t RF231SpiP__m_requests = 0;


uint8_t RF231SpiP__m_holder = RF231SpiP__NO_HOLDER;


bool RF231SpiP__release;


uint8_t RF231SpiP__packetOperation;


static inline error_t RF231SpiP__attemptRelease(void );
#line 73
static error_t RF231SpiP__Resource__request(uint8_t id);
#line 92
static error_t RF231SpiP__Resource__immediateRequest(uint8_t id);
#line 115
static error_t RF231SpiP__Resource__release(uint8_t id);
#line 150
static inline void RF231SpiP__SpiResource__granted(void );
#line 295
static inline void RF231SpiP__SpiPacket__sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error);










static void RF231SpiP__ChipSpiResource__changeState(uint8_t newState);









static uint8_t RF231SpiP__ChipSpiResource__readRegister(uint8_t registerAddress);









static void RF231SpiP__ChipSpiResource__setChannel(uint8_t newChannel);
#line 340
static void RF231SpiP__ChipSpiResource__setRegister(uint8_t regAddr, uint8_t newValue);









static void RF231SpiP__ChipSpiResource__loadPacket(uint8_t *data, uint8_t len);
#line 369
static void RF231SpiP__ChipSpiResource__readPacket(uint8_t *data);
#line 386
static inline error_t RF231SpiP__attemptRelease(void );
#line 405
static inline void RF231SpiP__grant__runTask(void );








static inline void RF231SpiP__Resource__default__granted(uint8_t id);
# 74 "/opt/tinyos-2.1.0/tos/system/StateImplP.nc"
uint8_t StateImplP__state[1U];

enum StateImplP____nesc_unnamed4302 {
  StateImplP__S_IDLE = 0
};


static inline error_t StateImplP__Init__init(void );
#line 96
static error_t StateImplP__State__requestState(uint8_t id, uint8_t reqState);
#line 118
static inline void StateImplP__State__toIdle(uint8_t id);







static inline bool StateImplP__State__isIdle(uint8_t id);






static inline bool StateImplP__State__isState(uint8_t id, uint8_t myState);
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__signalDone_task__postTask(void );
# 71 "/opt/tinyos-2.1.0/tos/interfaces/SpiPacket.nc"
static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__SpiPacket__sendDone(
# 6 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
uint8_t arg_0x408d52b0, 
# 64 "/opt/tinyos-2.1.0/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 71 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Msp430SpiConfigure__getConfig(
# 9 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
uint8_t arg_0x408d4500);
# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Resource__granted(
# 3 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
uint8_t arg_0x408bbca0);
# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__release(
# 8 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
uint8_t arg_0x408d5aa0);
# 87 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__immediateRequest(
# 8 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
uint8_t arg_0x408d5aa0);
# 78 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__request(
# 8 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
uint8_t arg_0x408d5aa0);
# 118 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__isOwner(
# 8 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
uint8_t arg_0x408d5aa0);
# 131 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA.nc"
static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__resetUsci(bool reset);
#line 164
static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__enableRxIntr(void );
#line 181
static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__clrRxIntr(void );
#line 161
static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__disableRxIntr(void );
#line 202
static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__tx(uint8_t data);
#line 158
static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__setModeSpi(msp430_spi_union_config_t *config);
#line 209
static uint8_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__rx(void );
#line 176
static bool /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__isRxIntrPending(void );
#line 148
static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__disableSpi(void );
# 29 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
enum /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0____nesc_unnamed4303 {
#line 29
  Msp430SpiNoDmaAP__0__signalDone_task = 6U
};
#line 29
typedef int /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0____nesc_sillytask_signalDone_task[/*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__signalDone_task];
#line 18
enum /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0____nesc_unnamed4304 {
  Msp430SpiNoDmaAP__0__SPI_ATOMIC_SIZE = 2
};

uint16_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_len;
uint8_t * /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_tx_buf;
uint8_t * /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_rx_buf;
uint16_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_pos;
uint8_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_client;

static inline void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__signalDone(void );


static inline error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Resource__immediateRequest(uint8_t id);



static inline error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Resource__request(uint8_t id);



static inline uint8_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Resource__isOwner(uint8_t id);



static inline error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Resource__release(uint8_t id);



static inline void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__ResourceConfigure__configure(uint8_t id);



static inline void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__ResourceConfigure__unconfigure(uint8_t id);





static inline void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__granted(uint8_t id);



static uint8_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__SpiByte__write(uint8_t tx);
#line 73
static inline error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__default__isOwner(uint8_t id);
static inline error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__default__request(uint8_t id);
static inline error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__default__immediateRequest(uint8_t id);
static inline error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__default__release(uint8_t id);
static inline msp430_spi_union_config_t */*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Msp430SpiConfigure__default__getConfig(uint8_t id);



static inline void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Resource__default__granted(uint8_t id);

static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__continueOp(void );
#line 106
static inline error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__SpiPacket__send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len);
#line 128
static inline void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__signalDone_task__runTask(void );



static inline void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciInterrupts__rxDone(uint8_t data);
#line 145
static inline void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__signalDone(void );




static inline void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciInterrupts__txDone(void );

static inline void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error);
# 85 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
static void HplMsp430UsciA0P__UCLK__selectIOFunc(void );
#line 78
static void HplMsp430UsciA0P__UCLK__selectModuleFunc(void );
# 85 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
static void HplMsp430UsciA0P__Interrupts__rxDone(uint8_t data);
#line 80
static void HplMsp430UsciA0P__Interrupts__txDone(void );
# 85 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
static void HplMsp430UsciA0P__SOMI__selectIOFunc(void );
#line 78
static void HplMsp430UsciA0P__SOMI__selectModuleFunc(void );






static void HplMsp430UsciA0P__SIMO__selectIOFunc(void );
#line 78
static void HplMsp430UsciA0P__SIMO__selectModuleFunc(void );
# 121 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA0P.nc"
static volatile uint8_t HplMsp430UsciA0P__IE2 __asm ("0x0001");
static volatile uint8_t HplMsp430UsciA0P__IFG2 __asm ("0x0003");
static volatile uint8_t HplMsp430UsciA0P__UCA0CTL0 __asm ("0x0060");
static volatile uint8_t HplMsp430UsciA0P__UCA0CTL1 __asm ("0x0061");
static volatile uint8_t HplMsp430UsciA0P__UCA0TXBUF __asm ("0x0067");

static inline void HplMsp430UsciA0P__UsciRawInterrupts__rxDone(uint8_t temp);



static inline void HplMsp430UsciA0P__UsciRawInterrupts__txDone(void );
#line 153
static inline void HplMsp430UsciA0P__Usci__setUbr(uint16_t control);










static inline void HplMsp430UsciA0P__Usci__setUmctl(uint8_t control);
#line 181
static inline void HplMsp430UsciA0P__Usci__resetUsci(bool reset);
#line 203
static inline void HplMsp430UsciA0P__Usci__enableSpi(void );







static inline void HplMsp430UsciA0P__Usci__disableSpi(void );







static inline void HplMsp430UsciA0P__configSpi(msp430_spi_union_config_t *config);






static void HplMsp430UsciA0P__Usci__setModeSpi(msp430_spi_union_config_t *config);
#line 247
static inline bool HplMsp430UsciA0P__Usci__isRxIntrPending(void );
#line 260
static inline void HplMsp430UsciA0P__Usci__clrRxIntr(void );




static inline void HplMsp430UsciA0P__Usci__clrIntr(void );




static inline void HplMsp430UsciA0P__Usci__disableRxIntr(void );









static inline void HplMsp430UsciA0P__Usci__disableIntr(void );




static inline void HplMsp430UsciA0P__Usci__enableRxIntr(void );
#line 313
static inline void HplMsp430UsciA0P__Usci__tx(uint8_t data);




static uint8_t HplMsp430UsciA0P__Usci__rx(void );
# 85 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciRawInterrupts.nc"
static void HplMsp430UsciAB0RawInterruptsP__UsciA__rxDone(uint8_t data);
#line 80
static void HplMsp430UsciAB0RawInterruptsP__UsciA__txDone(void );




static void HplMsp430UsciAB0RawInterruptsP__UsciB__rxDone(uint8_t data);
#line 80
static void HplMsp430UsciAB0RawInterruptsP__UsciB__txDone(void );
# 46 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciAB0RawInterruptsP.nc"
void sig_USCIAB0RX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(46)))  ;
#line 58
void sig_USCIAB0TX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(44)))  ;
#line 77
static inline void HplMsp430UsciAB0RawInterruptsP__UsciB__default__txDone(void );



static inline void HplMsp430UsciAB0RawInterruptsP__UsciB__default__rxDone(uint8_t temp);
# 85 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__rxDone(
# 70 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
uint8_t arg_0x409701b8, 
# 85 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
uint8_t data);
#line 80
static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__txDone(
# 70 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
uint8_t arg_0x409701b8);
# 80 "/opt/tinyos-2.1.0/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__userId(void );
# 78 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__txDone(void );




static inline void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__rxDone(uint8_t data);




static inline void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__txDone(uint8_t id);
static inline void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data);
# 39 "/opt/tinyos-2.1.0/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0____nesc_unnamed4305 {
#line 39
  FcfsResourceQueueC__0__NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[1U];
uint8_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
uint8_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

static inline error_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void );




static inline bool /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );



static inline bool /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
#line 72
static inline error_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id);
# 43 "/opt/tinyos-2.1.0/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(
# 55 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
uint8_t arg_0x4098fda8);
# 51 "/opt/tinyos-2.1.0/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(
# 55 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
uint8_t arg_0x4098fda8);
# 55 "/opt/tinyos-2.1.0/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(
# 60 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
uint8_t arg_0x4098d1b8);
# 49 "/opt/tinyos-2.1.0/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(
# 60 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
uint8_t arg_0x4098d1b8);
# 69 "/opt/tinyos-2.1.0/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(resource_client_id_t id);
#line 43
static bool /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void );
#line 60
static resource_client_id_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void );
# 73 "/opt/tinyos-2.1.0/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested(void );
#line 46
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void );
#line 81
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void );
# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(
# 54 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
uint8_t arg_0x4098f380);
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void );
# 75 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
enum /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4306 {
#line 75
  ArbiterP__0__grantedTask = 7U
};
#line 75
typedef int /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_sillytask_grantedTask[/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask];
#line 67
enum /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4307 {
#line 67
  ArbiterP__0__RES_CONTROLLED, ArbiterP__0__RES_GRANTING, ArbiterP__0__RES_IMM_GRANTING, ArbiterP__0__RES_BUSY
};
#line 68
enum /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4308 {
#line 68
  ArbiterP__0__default_owner_id = 1U
};
#line 69
enum /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4309 {
#line 69
  ArbiterP__0__NO_RES = 0xFF
};
uint8_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
uint8_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
uint8_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;



static inline error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(uint8_t id);
#line 90
static inline error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id);
#line 108
static inline error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id);
#line 129
static error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
#line 149
static bool /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );
#line 162
static uint8_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );










static uint8_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id);
#line 186
static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
#line 198
static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id);

static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(uint8_t id);

static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id);

static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void );

static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested(void );


static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested(void );


static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id);

static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id);
# 61 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
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
# 84 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
void sig_PORT1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(36)))  ;
#line 122
static inline void HplMsp430InterruptP__Port10__default__fired(void );
static inline void HplMsp430InterruptP__Port11__default__fired(void );
static inline void HplMsp430InterruptP__Port12__default__fired(void );
static inline void HplMsp430InterruptP__Port13__default__fired(void );
static inline void HplMsp430InterruptP__Port14__default__fired(void );
static inline void HplMsp430InterruptP__Port15__default__fired(void );

static inline void HplMsp430InterruptP__Port17__default__fired(void );






static inline void HplMsp430InterruptP__Port16__enable(void );







static inline void HplMsp430InterruptP__Port16__disable(void );

static inline void HplMsp430InterruptP__Port10__clear(void );
static inline void HplMsp430InterruptP__Port11__clear(void );
static inline void HplMsp430InterruptP__Port12__clear(void );
static inline void HplMsp430InterruptP__Port13__clear(void );
static inline void HplMsp430InterruptP__Port14__clear(void );
static inline void HplMsp430InterruptP__Port15__clear(void );
static inline void HplMsp430InterruptP__Port16__clear(void );
static inline void HplMsp430InterruptP__Port17__clear(void );
#line 198
static inline void HplMsp430InterruptP__Port16__edge(bool l2h);
#line 213
void sig_PORT2_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(38)))  ;
#line 249
static inline void HplMsp430InterruptP__Port20__default__fired(void );
static inline void HplMsp430InterruptP__Port21__default__fired(void );
static inline void HplMsp430InterruptP__Port22__default__fired(void );
static inline void HplMsp430InterruptP__Port23__default__fired(void );
static inline void HplMsp430InterruptP__Port24__default__fired(void );
static inline void HplMsp430InterruptP__Port25__default__fired(void );
static inline void HplMsp430InterruptP__Port26__default__fired(void );
static inline void HplMsp430InterruptP__Port27__default__fired(void );
#line 273
static inline void HplMsp430InterruptP__Port20__clear(void );
static inline void HplMsp430InterruptP__Port21__clear(void );
static inline void HplMsp430InterruptP__Port22__clear(void );
static inline void HplMsp430InterruptP__Port23__clear(void );
static inline void HplMsp430InterruptP__Port24__clear(void );
static inline void HplMsp430InterruptP__Port25__clear(void );
static inline void HplMsp430InterruptP__Port26__clear(void );
static inline void HplMsp430InterruptP__Port27__clear(void );
# 41 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
static void /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__HplInterrupt__clear(void );
#line 36
static void /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__HplInterrupt__disable(void );
#line 56
static void /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__HplInterrupt__edge(bool low_to_high);
#line 31
static void /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__HplInterrupt__enable(void );
# 57 "/opt/tinyos-2.1.0/tos/interfaces/GpioInterrupt.nc"
static void /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__Interrupt__fired(void );
# 41 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430InterruptC.nc"
static error_t /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__enable(bool rising);








static inline error_t /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void );







static inline error_t /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__Interrupt__disable(void );







static inline void /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__HplInterrupt__fired(void );
# 214 "/opt/tinyos-2.1.0/tos/chips/msp430X/msp430hardware.h"
static inline  void __nesc_enable_interrupt(void )
{
   __asm volatile ("eint");}

# 185 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
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

# 72 "/opt/tinyos-2.1.0/tos/lib/timer/BusyWaitCounterC.nc"
static inline void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow(void )
{
}

# 71 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
inline static void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__overflow(void ){
#line 71
  /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow();
#line 71
}
#line 71
# 53 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430CounterC.nc"
static inline void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow(void )
{
  /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__overflow();
}

# 37 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void ){
#line 37
  /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow();
#line 37
}
#line 37
# 126 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow();
}





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n)
{
}

# 28 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(uint8_t arg_0x404d0ce8){
#line 28
  switch (arg_0x404d0ce8) {
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
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(arg_0x404d0ce8);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 115 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(0);
}

# 28 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA0__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired();
#line 28
}
#line 28
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4310 {
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

# 75 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void )
{
  return * (volatile uint16_t * )370U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired();
#line 34
}
#line 34
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4311 {
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

# 75 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void )
{
  return * (volatile uint16_t * )372U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired();
#line 34
}
#line 34
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2____nesc_unnamed4312 {
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

# 75 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void )
{
  return * (volatile uint16_t * )374U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired();
#line 34
}
#line 34
# 120 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )302U;

#line 123
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(n >> 1);
}

# 28 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA1__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired();
#line 28
}
#line 28
# 115 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(0);
}

# 28 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB0__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired();
#line 28
}
#line 28
# 185 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
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

# 103 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void )
{
}

# 47 "/opt/tinyos-2.1.0/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void )
{
}

# 166 "/opt/tinyos-2.1.0/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void )
{
}

# 71 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
inline static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void ){
#line 71
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow();
#line 71
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow();
#line 71
}
#line 71
# 122 "/opt/tinyos-2.1.0/tos/lib/timer/TransformCounterC.nc"
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

# 71 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void ){
#line 71
  /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow();
#line 71
}
#line 71
# 53 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430CounterC.nc"
static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void )
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow();
}

# 37 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void ){
#line 37
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow();
#line 37
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow();
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
# 126 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow();
}

# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 70 "/opt/tinyos-2.1.0/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void )
{
#line 71
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask();
}

# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired();
#line 67
}
#line 67
# 151 "/opt/tinyos-2.1.0/tos/lib/timer/TransformAlarmC.nc"
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

# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void ){
#line 67
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired();
#line 67
}
#line 67
# 124 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void )
{
  * (volatile uint16_t * )386U &= ~0x0010;
}

# 47 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents();
#line 47
}
#line 47
# 59 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired();
}

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void ){
#line 34
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void )
{
  return * (volatile uint16_t * )402U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4313 {
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

# 86 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
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

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void ){
#line 34
  unsigned int result;
#line 34

#line 34
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 38 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get();
}

# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
inline static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void ){
#line 53
  unsigned int result;
#line 53

#line 53
  result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 70 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerP.nc"
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void )
{
  return * (volatile uint16_t * )384U & 1U;
}

# 35 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
inline static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void ){
#line 35
  unsigned char result;
#line 35

#line 35
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending();
#line 35

#line 35
  return result;
#line 35
}
#line 35
# 43 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430CounterC.nc"
static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending();
}

# 60 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
inline static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void ){
#line 60
  unsigned char result;
#line 60

#line 60
  result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 60

#line 60
  return result;
#line 60
}
#line 60
# 119 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void )
{
  * (volatile uint16_t * )386U |= 0x0010;
}

# 46 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents();
#line 46
}
#line 46
# 84 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )386U &= ~0x0001;
}

# 33 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )402U = x;
}

# 30 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(time);
#line 30
}
#line 30
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void ){
#line 34
  unsigned int result;
#line 34

#line 34
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 154 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )402U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get() + x;
}

# 32 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
inline static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void ){
#line 34
  unsigned int result;
#line 34

#line 34
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 70 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430AlarmC.nc"
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

# 92 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 181 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void )
{
  return * (volatile uint16_t * )404U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4314 {
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




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void )
{
  return * (volatile uint16_t * )406U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4315 {
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




static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void )
{
  return * (volatile uint16_t * )408U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6____nesc_unnamed4316 {
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




static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void )
{
  return * (volatile uint16_t * )410U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7____nesc_unnamed4317 {
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




static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void )
{
  return * (volatile uint16_t * )412U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8____nesc_unnamed4318 {
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




static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void )
{
  return * (volatile uint16_t * )414U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9____nesc_unnamed4319 {
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

# 120 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )286U;

#line 123
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(n >> 1);
}

# 28 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB1__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired();
#line 28
}
#line 28
# 113 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__Scheduler__init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP__m_next, SchedulerBasicP__NO_TASK, sizeof SchedulerBasicP__m_next);
    SchedulerBasicP__m_head = SchedulerBasicP__NO_TASK;
    SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
  }
}

# 46 "/opt/tinyos-2.1.0/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__init(void ){
#line 46
  SchedulerBasicP__Scheduler__init();
#line 46
}
#line 46
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__clr(void )
#line 47
{
  /* atomic removed: atomic calls only */
#line 47
  * (volatile uint8_t * )41U &= ~(0x01 << 3);
}

# 39 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__GeneralIO__clr(void )
#line 38
{
#line 38
  /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__HplGeneralIO__clr();
}

# 30 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led3__clr(void ){
#line 30
  /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__GeneralIO__clr();
#line 30
}
#line 30
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P22*/HplMsp430GeneralIOP__10__IO__clr(void )
#line 47
{
  /* atomic removed: atomic calls only */
#line 47
  * (volatile uint8_t * )41U &= ~(0x01 << 2);
}

# 39 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P22*/HplMsp430GeneralIOP__10__IO__clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void )
#line 38
{
#line 38
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr();
}

# 30 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__clr(void ){
#line 30
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr();
#line 30
}
#line 30
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__clr(void )
#line 47
{
  /* atomic removed: atomic calls only */
#line 47
  * (volatile uint8_t * )41U &= ~(0x01 << 1);
}

# 39 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr(void )
#line 38
{
#line 38
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__clr();
}

# 30 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__clr(void ){
#line 30
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr();
#line 30
}
#line 30
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__clr(void )
#line 47
{
  /* atomic removed: atomic calls only */
#line 47
  * (volatile uint8_t * )41U &= ~(0x01 << 0);
}

# 39 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr(void )
#line 38
{
#line 38
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__clr();
}

# 30 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__clr(void ){
#line 30
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr();
#line 30
}
#line 30
# 53 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__makeOutput(void )
#line 53
{
  /* atomic removed: atomic calls only */
#line 53
  * (volatile uint8_t * )42U |= 0x01 << 3;
}

# 71 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__HplGeneralIO__makeOutput();
}

# 35 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led3__makeOutput(void ){
#line 35
  /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__GeneralIO__makeOutput();
#line 35
}
#line 35
# 53 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P22*/HplMsp430GeneralIOP__10__IO__makeOutput(void )
#line 53
{
  /* atomic removed: atomic calls only */
#line 53
  * (volatile uint8_t * )42U |= 0x01 << 2;
}

# 71 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P22*/HplMsp430GeneralIOP__10__IO__makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput();
}

# 35 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__makeOutput(void ){
#line 35
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput();
#line 35
}
#line 35
# 53 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__makeOutput(void )
#line 53
{
  /* atomic removed: atomic calls only */
#line 53
  * (volatile uint8_t * )42U |= 0x01 << 1;
}

# 71 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput();
}

# 35 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__makeOutput(void ){
#line 35
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput();
#line 35
}
#line 35
# 53 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__makeOutput(void )
#line 53
{
  /* atomic removed: atomic calls only */
#line 53
  * (volatile uint8_t * )42U |= 0x01 << 0;
}

# 71 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput();
}

# 35 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__makeOutput(void ){
#line 35
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput();
#line 35
}
#line 35
# 50 "/opt/tinyos-2.1.0/tos/platforms/gina20/LedsP.nc"
static inline error_t LedsP__Init__init(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 51
  {
    ;
    LedsP__Led0__makeOutput();
    LedsP__Led1__makeOutput();
    LedsP__Led2__makeOutput();
    LedsP__Led3__makeOutput();
    LedsP__Led0__clr();
    LedsP__Led1__clr();
    LedsP__Led2__clr();
    LedsP__Led3__clr();
  }
  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
inline static error_t PlatformP__LedsInit__init(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = LedsP__Init__init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 201 "/opt/tinyos-2.1.0/tos/platforms/gina20/Msp430ClockP.nc"
static inline void Msp430ClockP__startTimerB(void )
{

  Msp430ClockP__TBCTL = 0x0020 | (Msp430ClockP__TBCTL & ~(0x0020 | 0x0010));
}

#line 189
static inline void Msp430ClockP__startTimerA(void )
{

  Msp430ClockP__TA0CTL = 0x0020 | (Msp430ClockP__TA0CTL & ~(0x0020 | 0x0010));
}

#line 152
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void )
{
  TBR = 0;










  Msp430ClockP__TBCTL = 0x0100 | 0x0002;
}

#line 183
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerB();
}

# 32 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerB(void ){
#line 32
  Msp430ClockP__Msp430ClockInit__default__initTimerB();
#line 32
}
#line 32
# 136 "/opt/tinyos-2.1.0/tos/platforms/gina20/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void )
{
  TA0R = 0;










  Msp430ClockP__TA0CTL = 0x0200 | 0x0002;
}

#line 178
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerA();
}

# 31 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerA(void ){
#line 31
  Msp430ClockP__Msp430ClockInit__default__initTimerA();
#line 31
}
#line 31
# 99 "/opt/tinyos-2.1.0/tos/platforms/gina20/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void )
{


  if (CALBC1_8MHZ != 0xFF) {
      DCOCTL = 0x00;
      BCSCTL1 = CALBC1_8MHZ;
      DCOCTL = CALDCO_8MHZ;
    }
  else 
#line 107
    {
      DCOCTL = 0x00;
      BCSCTL1 = 0x8D;
      DCOCTL = 0x88;
    }






  BCSCTL1 = 0x80 | (BCSCTL1 & ((0x04 | 0x02) | 0x01));
#line 132
  BCSCTL2 = 0x04;
  Msp430ClockP__IE1 &= ~(1 << 1);
}

#line 173
static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitClocks();
}

# 30 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initClocks(void ){
#line 30
  Msp430ClockP__Msp430ClockInit__default__initClocks();
#line 30
}
#line 30
# 267 "/opt/tinyos-2.1.0/tos/platforms/gina20/Msp430ClockP.nc"
static inline error_t Msp430ClockP__Init__init(void )
{


  Msp430ClockP__TA0CTL = 0x0004;
  Msp430ClockP__TA0IV = 0;
  Msp430ClockP__TBCTL = 0x0004;
  Msp430ClockP__TBIV = 0;
  /* atomic removed: atomic calls only */

  {



    Msp430ClockP__Msp430ClockInit__initClocks();
    Msp430ClockP__Msp430ClockInit__initTimerA();
    Msp430ClockP__Msp430ClockInit__initTimerB();
    Msp430ClockP__startTimerA();
    Msp430ClockP__startTimerB();
  }

  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
inline static error_t PlatformP__Msp430ClockInit__init(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = Msp430ClockP__Init__init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 9 "/opt/tinyos-2.1.0/tos/platforms/gina20/PlatformP.nc"
static inline error_t PlatformP__Init__init(void )
#line 9
{
  PlatformP__Msp430ClockInit__init();

  P2DIR = P2DIR & 0x7F;
  P2REN = P2REN | 0x80;
  P2OUT = P2OUT | 0x80;


  P3DIR = P3DIR & 
  PlatformP__LedsInit__init();
  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
inline static error_t RealMainP__PlatformInit__init(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = PlatformP__Init__init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 54 "/opt/tinyos-2.1.0/tos/interfaces/Scheduler.nc"
inline static bool RealMainP__Scheduler__runNextTask(void ){
#line 54
  unsigned char result;
#line 54

#line 54
  result = SchedulerBasicP__Scheduler__runNextTask();
#line 54

#line 54
  return result;
#line 54
}
#line 54
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t RF231SpiP__grant__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(RF231SpiP__grant);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 150 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/spi/RF231SpiP.nc"
static inline void RF231SpiP__SpiResource__granted(void )
#line 150
{
  RF231SpiP__grant__postTask();
}

# 81 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
static inline void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Resource__default__granted(uint8_t id)
#line 81
{
}

# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Resource__granted(uint8_t arg_0x408bbca0){
#line 92
  switch (arg_0x408bbca0) {
#line 92
    case /*RF231SpiWireC.HplRF231SpiC.SpiC*/Msp430SpiA0C__0__CLIENT_ID:
#line 92
      RF231SpiP__SpiResource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Resource__default__granted(arg_0x408bbca0);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 57 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
static inline void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__granted(uint8_t id)
#line 57
{
  /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Resource__granted(id);
}

# 198 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id)
#line 198
{
}

# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(uint8_t arg_0x4098f380){
#line 92
  switch (arg_0x4098f380) {
#line 92
    case /*RF231SpiWireC.HplRF231SpiC.SpiC.UsciC*/Msp430UsciA0C__0__CLIENT_ID:
#line 92
      /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__granted(/*RF231SpiWireC.HplRF231SpiC.SpiC*/Msp430SpiA0C__0__CLIENT_ID);
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(arg_0x4098f380);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 77 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
static inline msp430_spi_union_config_t */*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Msp430SpiConfigure__default__getConfig(uint8_t id)
#line 77
{
  return &msp430_spi_default_config;
}

# 71 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiConfigure.nc"
inline static msp430_spi_union_config_t */*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Msp430SpiConfigure__getConfig(uint8_t arg_0x408d4500){
#line 71
  union __nesc_unnamed4271 *result;
#line 71

#line 71
    result = /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Msp430SpiConfigure__default__getConfig(arg_0x408d4500);
#line 71

#line 71
  return result;
#line 71
}
#line 71
# 158 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA.nc"
inline static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__setModeSpi(msp430_spi_union_config_t *config){
#line 158
  HplMsp430UsciA0P__Usci__setModeSpi(config);
#line 158
}
#line 158
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
static inline void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__ResourceConfigure__configure(uint8_t id)
#line 47
{
  /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__setModeSpi(/*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Msp430SpiConfigure__getConfig(id));
}

# 212 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id)
#line 212
{
}

# 49 "/opt/tinyos-2.1.0/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(uint8_t arg_0x4098d1b8){
#line 49
  switch (arg_0x4098d1b8) {
#line 49
    case /*RF231SpiWireC.HplRF231SpiC.SpiC.UsciC*/Msp430UsciA0C__0__CLIENT_ID:
#line 49
      /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__ResourceConfigure__configure(/*RF231SpiWireC.HplRF231SpiC.SpiC*/Msp430SpiA0C__0__CLIENT_ID);
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(arg_0x4098d1b8);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 186 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void )
#line 186
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 187
    {
      /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
      /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
    }
#line 190
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
}

# 164 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA0P.nc"
static inline void HplMsp430UsciA0P__Usci__setUmctl(uint8_t control)
#line 164
{
  UCA0MCTL = control;
}

#line 153
static inline void HplMsp430UsciA0P__Usci__setUbr(uint16_t control)
#line 153
{
  /* atomic removed: atomic calls only */
#line 154
  {
    UCA0BR0 = control & 0x00FF;
    UCA0BR1 = (control >> 8) & 0x00FF;
  }
}

#line 219
static inline void HplMsp430UsciA0P__configSpi(msp430_spi_union_config_t *config)
#line 219
{
  HplMsp430UsciA0P__UCA0CTL0 |= 0x08 + 0x01 + 0x40 + 0x20;
  HplMsp430UsciA0P__UCA0CTL1 |= (2 << 6) + 0x01;
  HplMsp430UsciA0P__Usci__setUbr(config->spiRegisters.ubr);
  HplMsp430UsciA0P__Usci__setUmctl(0x00);
}

# 55 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P30*/HplMsp430GeneralIOP__16__IO__selectModuleFunc(void )
#line 55
{
  /* atomic removed: atomic calls only */
#line 55
  * (volatile uint8_t * )27U |= 0x01 << 0;
}

# 78 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430UsciA0P__UCLK__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P30*/HplMsp430GeneralIOP__16__IO__selectModuleFunc();
#line 78
}
#line 78
# 55 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectModuleFunc(void )
#line 55
{
  /* atomic removed: atomic calls only */
#line 55
  * (volatile uint8_t * )27U |= 0x01 << 5;
}

# 78 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430UsciA0P__SOMI__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectModuleFunc();
#line 78
}
#line 78
# 55 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectModuleFunc(void )
#line 55
{
  /* atomic removed: atomic calls only */
#line 55
  * (volatile uint8_t * )27U |= 0x01 << 4;
}

# 78 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430UsciA0P__SIMO__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectModuleFunc();
#line 78
}
#line 78
# 203 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA0P.nc"
static inline void HplMsp430UsciA0P__Usci__enableSpi(void )
#line 203
{
  /* atomic removed: atomic calls only */
#line 204
  {
    HplMsp430UsciA0P__SIMO__selectModuleFunc();
    HplMsp430UsciA0P__SOMI__selectModuleFunc();
    HplMsp430UsciA0P__UCLK__selectModuleFunc();
  }
}

#line 265
static inline void HplMsp430UsciA0P__Usci__clrIntr(void )
#line 265
{
  HplMsp430UsciA0P__IFG2 &= ~((1 << 1) | (1 << 0));
}

#line 280
static inline void HplMsp430UsciA0P__Usci__disableIntr(void )
#line 280
{
  HplMsp430UsciA0P__IE2 &= ~((1 << 1) | (1 << 0));
}

# 11 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/ChipSpiResource.nc"
inline static void RF231SpiP__ChipSpiResource__loadPacketDone(error_t error){
#line 11
  RF231driverP__ChipSpiResource__loadPacketDone(error);
#line 11
}
#line 11
# 295 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/spi/RF231SpiP.nc"
static inline void RF231SpiP__SpiPacket__sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error)
#line 296
{
  if (RF231SpiP__packetOperation == RF231SpiP__WAS_WRITING) {
      RF231SpiP__ChipSpiResource__loadPacketDone(error);
    }
  else {
#line 299
    if (RF231SpiP__packetOperation == RF231SpiP__WAS_READING) {
      }
    }
}

# 152 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
static inline void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error)
#line 152
{
}

# 71 "/opt/tinyos-2.1.0/tos/interfaces/SpiPacket.nc"
inline static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__SpiPacket__sendDone(uint8_t arg_0x408d52b0, uint8_t * txBuf, uint8_t * rxBuf, uint16_t len, error_t error){
#line 71
  switch (arg_0x408d52b0) {
#line 71
    case /*RF231SpiWireC.HplRF231SpiC.SpiC*/Msp430SpiA0C__0__CLIENT_ID:
#line 71
      RF231SpiP__SpiPacket__sendDone(txBuf, rxBuf, len, error);
#line 71
      break;
#line 71
    default:
#line 71
      /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__SpiPacket__default__sendDone(arg_0x408d52b0, txBuf, rxBuf, len, error);
#line 71
      break;
#line 71
    }
#line 71
}
#line 71
# 145 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
static inline void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__signalDone(void )
#line 145
{
  /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__SpiPacket__sendDone(/*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_client, /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_tx_buf, /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_rx_buf, /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_len, 
  SUCCESS);
}

#line 128
static inline void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__signalDone_task__runTask(void )
#line 128
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 129
    /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__signalDone();
#line 129
    __nesc_atomic_end(__nesc_atomic); }
}

# 45 "/opt/tinyos-2.1.0/tos/interfaces/State.nc"
inline static error_t RF231SpiP__WorkingState__requestState(uint8_t reqState){
#line 45
  unsigned char result;
#line 45

#line 45
  result = StateImplP__State__requestState(0U, reqState);
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 73 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
static inline error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__default__isOwner(uint8_t id)
#line 73
{
#line 73
  return 0x0080;
}

# 118 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static bool /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__isOwner(uint8_t arg_0x408d5aa0){
#line 118
  unsigned char result;
#line 118

#line 118
  switch (arg_0x408d5aa0) {
#line 118
    case /*RF231SpiWireC.HplRF231SpiC.SpiC*/Msp430SpiA0C__0__CLIENT_ID:
#line 118
      result = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(/*RF231SpiWireC.HplRF231SpiC.SpiC.UsciC*/Msp430UsciA0C__0__CLIENT_ID);
#line 118
      break;
#line 118
    default:
#line 118
      result = /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__default__isOwner(arg_0x408d5aa0);
#line 118
      break;
#line 118
    }
#line 118

#line 118
  return result;
#line 118
}
#line 118
# 39 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
static inline uint8_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Resource__isOwner(uint8_t id)
#line 39
{
  return /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__isOwner(id);
}

# 118 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static bool RF231SpiP__SpiResource__isOwner(void ){
#line 118
  unsigned char result;
#line 118

#line 118
  result = /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Resource__isOwner(/*RF231SpiWireC.HplRF231SpiC.SpiC*/Msp430SpiA0C__0__CLIENT_ID);
#line 118

#line 118
  return result;
#line 118
}
#line 118
# 209 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested(void )
#line 209
{
  /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release();
}

# 81 "/opt/tinyos-2.1.0/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void ){
#line 81
  /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested();
#line 81
}
#line 81
# 202 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id)
#line 202
{
}

# 51 "/opt/tinyos-2.1.0/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(uint8_t arg_0x4098fda8){
#line 51
    /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(arg_0x4098fda8);
#line 51
}
#line 51
# 90 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id)
#line 90
{
  /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /* atomic removed: atomic calls only */
#line 92
  {
    if (/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) {
        /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING;
        /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = id;
      }
    else {
        unsigned char __nesc_temp = 
#line 97
        0x0080;

#line 97
        return __nesc_temp;
      }
  }
#line 99
  /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested();
  if (/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId == id) {
      /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
      return SUCCESS;
    }
  /* atomic removed: atomic calls only */
#line 104
  /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
  return 0x0080;
}

# 75 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
static inline error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__default__immediateRequest(uint8_t id)
#line 75
{
#line 75
  return 0x0080;
}

# 87 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__immediateRequest(uint8_t arg_0x408d5aa0){
#line 87
  unsigned char result;
#line 87

#line 87
  switch (arg_0x408d5aa0) {
#line 87
    case /*RF231SpiWireC.HplRF231SpiC.SpiC*/Msp430SpiA0C__0__CLIENT_ID:
#line 87
      result = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(/*RF231SpiWireC.HplRF231SpiC.SpiC.UsciC*/Msp430UsciA0C__0__CLIENT_ID);
#line 87
      break;
#line 87
    default:
#line 87
      result = /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__default__immediateRequest(arg_0x408d5aa0);
#line 87
      break;
#line 87
    }
#line 87

#line 87
  return result;
#line 87
}
#line 87
# 31 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
static inline error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Resource__immediateRequest(uint8_t id)
#line 31
{
  return /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__immediateRequest(id);
}

# 87 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static error_t RF231SpiP__SpiResource__immediateRequest(void ){
#line 87
  unsigned char result;
#line 87

#line 87
  result = /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Resource__immediateRequest(/*RF231SpiWireC.HplRF231SpiC.SpiC*/Msp430SpiA0C__0__CLIENT_ID);
#line 87

#line 87
  return result;
#line 87
}
#line 87
# 118 "/opt/tinyos-2.1.0/tos/system/StateImplP.nc"
static inline void StateImplP__State__toIdle(uint8_t id)
#line 118
{
  /* atomic removed: atomic calls only */
#line 119
  StateImplP__state[id] = StateImplP__S_IDLE;
}

# 56 "/opt/tinyos-2.1.0/tos/interfaces/State.nc"
inline static void RF231SpiP__WorkingState__toIdle(void ){
#line 56
  StateImplP__State__toIdle(0U);
#line 56
}
#line 56
# 34 "/opt/tinyos-2.1.0/tos/interfaces/SpiByte.nc"
inline static uint8_t RF231SpiP__SpiByte__write(uint8_t tx){
#line 34
  unsigned char result;
#line 34

#line 34
  result = /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__SpiByte__write(tx);
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 313 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA0P.nc"
static inline void HplMsp430UsciA0P__Usci__tx(uint8_t data)
#line 313
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 314
    HplMsp430UsciA0P__UCA0TXBUF = data;
#line 314
    __nesc_atomic_end(__nesc_atomic); }
}

# 202 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA.nc"
inline static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__tx(uint8_t data){
#line 202
  HplMsp430UsciA0P__Usci__tx(data);
#line 202
}
#line 202
# 247 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA0P.nc"
static inline bool HplMsp430UsciA0P__Usci__isRxIntrPending(void )
#line 247
{
  if (HplMsp430UsciA0P__IFG2 & (1 << 0)) {

      return TRUE;
    }
  return FALSE;
}

# 176 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA.nc"
inline static bool /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__isRxIntrPending(void ){
#line 176
  unsigned char result;
#line 176

#line 176
  result = HplMsp430UsciA0P__Usci__isRxIntrPending();
#line 176

#line 176
  return result;
#line 176
}
#line 176
# 260 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA0P.nc"
static inline void HplMsp430UsciA0P__Usci__clrRxIntr(void )
#line 260
{
  HplMsp430UsciA0P__IFG2 &= ~(1 << 0);
}

# 181 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA.nc"
inline static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__clrRxIntr(void ){
#line 181
  HplMsp430UsciA0P__Usci__clrRxIntr();
#line 181
}
#line 181
#line 209
inline static uint8_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__rx(void ){
#line 209
  unsigned char result;
#line 209

#line 209
  result = HplMsp430UsciA0P__Usci__rx();
#line 209

#line 209
  return result;
#line 209
}
#line 209
# 204 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void )
#line 204
{
}

# 46 "/opt/tinyos-2.1.0/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void ){
#line 46
  /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted();
#line 46
}
#line 46
# 181 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA0P.nc"
static inline void HplMsp430UsciA0P__Usci__resetUsci(bool reset)
#line 181
{
  if (reset) {
      HplMsp430UsciA0P__UCA0CTL1 |= 0x01;
    }
  else {
      HplMsp430UsciA0P__UCA0CTL1 &= ~0x01;
    }
}

# 131 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA.nc"
inline static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__resetUsci(bool reset){
#line 131
  HplMsp430UsciA0P__Usci__resetUsci(reset);
#line 131
}
#line 131
# 57 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P30*/HplMsp430GeneralIOP__16__IO__selectIOFunc(void )
#line 57
{
  /* atomic removed: atomic calls only */
#line 57
  * (volatile uint8_t * )27U &= ~(0x01 << 0);
}

# 85 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430UsciA0P__UCLK__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P30*/HplMsp430GeneralIOP__16__IO__selectIOFunc();
#line 85
}
#line 85
# 57 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void )
#line 57
{
  /* atomic removed: atomic calls only */
#line 57
  * (volatile uint8_t * )27U &= ~(0x01 << 5);
}

# 85 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430UsciA0P__SOMI__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc();
#line 85
}
#line 85
# 57 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void )
#line 57
{
  /* atomic removed: atomic calls only */
#line 57
  * (volatile uint8_t * )27U &= ~(0x01 << 4);
}

# 85 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430UsciA0P__SIMO__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc();
#line 85
}
#line 85
# 211 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA0P.nc"
static inline void HplMsp430UsciA0P__Usci__disableSpi(void )
#line 211
{
  /* atomic removed: atomic calls only */
#line 212
  {
    HplMsp430UsciA0P__SIMO__selectIOFunc();
    HplMsp430UsciA0P__SOMI__selectIOFunc();
    HplMsp430UsciA0P__UCLK__selectIOFunc();
  }
}

# 148 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA.nc"
inline static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__disableSpi(void ){
#line 148
  HplMsp430UsciA0P__Usci__disableSpi();
#line 148
}
#line 148
# 51 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
static inline void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__ResourceConfigure__unconfigure(uint8_t id)
#line 51
{
  /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__resetUsci(TRUE);
  /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__disableSpi();
  /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__resetUsci(FALSE);
}

# 214 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id)
#line 214
{
}

# 55 "/opt/tinyos-2.1.0/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(uint8_t arg_0x4098d1b8){
#line 55
  switch (arg_0x4098d1b8) {
#line 55
    case /*RF231SpiWireC.HplRF231SpiC.SpiC.UsciC*/Msp430UsciA0C__0__CLIENT_ID:
#line 55
      /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__ResourceConfigure__unconfigure(/*RF231SpiWireC.HplRF231SpiC.SpiC*/Msp430SpiA0C__0__CLIENT_ID);
#line 55
      break;
#line 55
    default:
#line 55
      /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(arg_0x4098d1b8);
#line 55
      break;
#line 55
    }
#line 55
}
#line 55
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 58 "/opt/tinyos-2.1.0/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    if (/*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead != /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
        uint8_t id = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead;

#line 62
        /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[/*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead];
        if (/*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead == /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
          /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
          }
#line 65
        /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[id] = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
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
      /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

#line 68
      return __nesc_temp;
    }
  }
}

# 60 "/opt/tinyos-2.1.0/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void ){
#line 60
  unsigned char result;
#line 60

#line 60
  result = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue();
#line 60

#line 60
  return result;
#line 60
}
#line 60
# 50 "/opt/tinyos-2.1.0/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void )
#line 50
{
  return /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead == /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
}

# 43 "/opt/tinyos-2.1.0/tos/interfaces/ResourceQueue.nc"
inline static bool /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void ){
#line 43
  unsigned char result;
#line 43

#line 43
  result = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty();
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 108 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id)
#line 108
{
  /* atomic removed: atomic calls only */
#line 109
  {
    if (/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY && /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId == id) {
        if (/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty() == FALSE) {
            /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue();
            /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING;
            /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
            /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(id);
          }
        else {
            /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
            /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
            /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(id);
            /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted();
          }
        {
          unsigned char __nesc_temp = 
#line 123
          SUCCESS;

#line 123
          return __nesc_temp;
        }
      }
  }
#line 126
  return 0x0080;
}

# 76 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
static inline error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__default__release(uint8_t id)
#line 76
{
#line 76
  return 0x0080;
}

# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__release(uint8_t arg_0x408d5aa0){
#line 110
  unsigned char result;
#line 110

#line 110
  switch (arg_0x408d5aa0) {
#line 110
    case /*RF231SpiWireC.HplRF231SpiC.SpiC*/Msp430SpiA0C__0__CLIENT_ID:
#line 110
      result = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(/*RF231SpiWireC.HplRF231SpiC.SpiC.UsciC*/Msp430UsciA0C__0__CLIENT_ID);
#line 110
      break;
#line 110
    default:
#line 110
      result = /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__default__release(arg_0x408d5aa0);
#line 110
      break;
#line 110
    }
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 43 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
static inline error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Resource__release(uint8_t id)
#line 43
{
  return /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__release(id);
}

# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static error_t RF231SpiP__SpiResource__release(void ){
#line 110
  unsigned char result;
#line 110

#line 110
  result = /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Resource__release(/*RF231SpiWireC.HplRF231SpiC.SpiC*/Msp430SpiA0C__0__CLIENT_ID);
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 122 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RF231driverP.nc"
static inline void RF231driverP__ChipSpiResource__releasing(void )
#line 122
{
}

# 2 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/ChipSpiResource.nc"
inline static void RF231SpiP__ChipSpiResource__releasing(void ){
#line 2
  RF231driverP__ChipSpiResource__releasing();
#line 2
}
#line 2
# 133 "/opt/tinyos-2.1.0/tos/system/StateImplP.nc"
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

# 61 "/opt/tinyos-2.1.0/tos/interfaces/State.nc"
inline static bool RF231SpiP__WorkingState__isIdle(void ){
#line 61
  unsigned char result;
#line 61

#line 61
  result = StateImplP__State__isIdle(0U);
#line 61

#line 61
  return result;
#line 61
}
#line 61
# 386 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/spi/RF231SpiP.nc"
static inline error_t RF231SpiP__attemptRelease(void )
#line 386
{


  if ((
#line 387
  RF231SpiP__m_requests > 0
   || RF231SpiP__m_holder != RF231SpiP__NO_HOLDER)
   || !RF231SpiP__WorkingState__isIdle()) {
      return 0x0080;
    }
  /* atomic removed: atomic calls only */
  RF231SpiP__release = TRUE;
  RF231SpiP__ChipSpiResource__releasing();
  /* atomic removed: atomic calls only */
#line 395
  {
    if (RF231SpiP__release) {
        RF231SpiP__SpiResource__release();
        {
          unsigned char __nesc_temp = 
#line 398
          SUCCESS;

#line 398
          return __nesc_temp;
        }
      }
  }
  return EBUSY;
}

# 206 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested(void )
#line 206
{
  /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release();
}

# 73 "/opt/tinyos-2.1.0/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested(void ){
#line 73
  /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested();
#line 73
}
#line 73
# 54 "/opt/tinyos-2.1.0/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 54
{
  return /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[id] != /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY || /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail == id;
}

#line 72
static inline error_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id)
#line 72
{
  /* atomic removed: atomic calls only */
#line 73
  {
    if (!/*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(id)) {
        if (/*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead == /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
          /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead = id;
          }
        else {
#line 78
          /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[/*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail] = id;
          }
#line 79
        /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail = id;
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

# 69 "/opt/tinyos-2.1.0/tos/interfaces/ResourceQueue.nc"
inline static error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(resource_client_id_t id){
#line 69
  unsigned char result;
#line 69

#line 69
  result = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(id);
#line 69

#line 69
  return result;
#line 69
}
#line 69
# 200 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(uint8_t id)
#line 200
{
}

# 43 "/opt/tinyos-2.1.0/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(uint8_t arg_0x4098fda8){
#line 43
    /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(arg_0x4098fda8);
#line 43
}
#line 43
# 77 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(uint8_t id)
#line 77
{
  /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /* atomic removed: atomic calls only */
#line 79
  {
    if (/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) {
        /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING;
        /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = id;
      }
    else {
        unsigned char __nesc_temp = 
#line 84
        /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(id);

#line 84
        return __nesc_temp;
      }
  }
#line 86
  /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested();
  return SUCCESS;
}

# 74 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
static inline error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__default__request(uint8_t id)
#line 74
{
#line 74
  return 0x0080;
}

# 78 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__request(uint8_t arg_0x408d5aa0){
#line 78
  unsigned char result;
#line 78

#line 78
  switch (arg_0x408d5aa0) {
#line 78
    case /*RF231SpiWireC.HplRF231SpiC.SpiC*/Msp430SpiA0C__0__CLIENT_ID:
#line 78
      result = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(/*RF231SpiWireC.HplRF231SpiC.SpiC.UsciC*/Msp430UsciA0C__0__CLIENT_ID);
#line 78
      break;
#line 78
    default:
#line 78
      result = /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__default__request(arg_0x408d5aa0);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 35 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
static inline error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Resource__request(uint8_t id)
#line 35
{
  return /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciResource__request(id);
}

# 78 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static error_t RF231SpiP__SpiResource__request(void ){
#line 78
  unsigned char result;
#line 78

#line 78
  result = /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Resource__request(/*RF231SpiWireC.HplRF231SpiC.SpiC*/Msp430SpiA0C__0__CLIENT_ID);
#line 78

#line 78
  return result;
#line 78
}
#line 78
inline static error_t RF231driverP__SpiResource__request(void ){
#line 78
  unsigned char result;
#line 78

#line 78
  result = RF231SpiP__Resource__request(/*RF231driverC.RF231SpiC*/RF231SpiC__0__CLIENT_ID);
#line 78

#line 78
  return result;
#line 78
}
#line 78
#line 110
inline static error_t RF231driverP__SpiResource__release(void ){
#line 110
  unsigned char result;
#line 110

#line 110
  result = RF231SpiP__Resource__release(/*RF231driverC.RF231SpiC*/RF231SpiC__0__CLIENT_ID);
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 6 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/ChipSpiResource.nc"
inline static void RF231driverP__ChipSpiResource__changeState(uint8_t newState){
#line 6
  RF231SpiP__ChipSpiResource__changeState(newState);
#line 6
}
#line 6
# 87 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static error_t RF231driverP__SpiResource__immediateRequest(void ){
#line 87
  unsigned char result;
#line 87

#line 87
  result = RF231SpiP__Resource__immediateRequest(/*RF231driverC.RF231SpiC*/RF231SpiC__0__CLIENT_ID);
#line 87

#line 87
  return result;
#line 87
}
#line 87
# 394 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RF231driverP.nc"
static inline error_t RF231driverP__RadioControl__receiveNow(void )
#line 394
{
  /* atomic removed: atomic calls only */
  {
    if (RF231driverP__radioState != 6) {
        if (RF231driverP__SpiResource__immediateRequest()) {
            P4OUT &= ~0x0001;
            RF231driverP__ChipSpiResource__changeState(0x06);
            RF231driverP__SpiResource__release();
            P4OUT |= 0x0001;
          }
        else {
            RF231driverP__internalSpiState = RF231driverP__CHANGINGSTATE;
            RF231driverP__globalNewReg = 0x06;
            RF231driverP__SpiResource__request();
          }
      }
  }
  /* atomic removed: atomic calls only */
#line 409
  RF231driverP__radioState = 6;

  return SUCCESS;
}

# 6 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RadioControl.nc"
inline static error_t fabMACP__RadioControl__receiveNow(void ){
#line 6
  unsigned char result;
#line 6

#line 6
  result = RF231driverP__RadioControl__receiveNow();
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 64 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/02-MAC/fabMACP.nc"
static inline void fabMACP__RadioControl__prepareReceiveDone(error_t error)
#line 64
{
  fabMACP__RadioControl__receiveNow();
}

# 5 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RadioControl.nc"
inline static void RF231driverP__RadioControl__prepareReceiveDone(error_t error){
#line 5
  fabMACP__RadioControl__prepareReceiveDone(error);
#line 5
}
#line 5
# 339 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RF231driverP.nc"
static inline error_t RF231driverP__RadioSend__sendNow(void )
#line 339
{
  /* atomic removed: atomic calls only */
#line 340
  P4OUT |= 0x0080;
  /* atomic removed: atomic calls only */
  P4OUT &= ~0x0080;
#line 356
  return SUCCESS;
}

# 7 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RadioSend.nc"
inline static error_t fabMACP__RadioSend__sendNow(void ){
#line 7
  unsigned char result;
#line 7

#line 7
  result = RF231driverP__RadioSend__sendNow();
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 49 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P22*/HplMsp430GeneralIOP__10__IO__getRaw(void )
#line 49
{
#line 49
  return * (volatile uint8_t * )40U & (0x01 << 2);
}

#line 50
static inline bool /*HplMsp430GeneralIOC.P22*/HplMsp430GeneralIOP__10__IO__get(void )
#line 50
{
#line 50
  return /*HplMsp430GeneralIOC.P22*/HplMsp430GeneralIOP__10__IO__getRaw() != 0;
}

# 59 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static bool /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__get(void ){
#line 59
  unsigned char result;
#line 59

#line 59
  result = /*HplMsp430GeneralIOC.P22*/HplMsp430GeneralIOP__10__IO__get();
#line 59

#line 59
  return result;
#line 59
}
#line 59
# 40 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline bool /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__get(void )
#line 40
{
#line 40
  return /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__get();
}

# 32 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static bool LedsP__Led2__get(void ){
#line 32
  unsigned char result;
#line 32

#line 32
  result = /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__get();
#line 32

#line 32
  return result;
#line 32
}
#line 32
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__toggle(void ){
#line 44
  /*HplMsp430GeneralIOC.P22*/HplMsp430GeneralIOP__10__IO__toggle();
#line 44
}
#line 44
# 39 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle(void )
#line 39
{
#line 39
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__toggle();
}

# 31 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__toggle(void ){
#line 31
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle();
#line 31
}
#line 31
# 113 "/opt/tinyos-2.1.0/tos/platforms/gina20/LedsP.nc"
static inline void LedsP__Leds__led2Toggle(void )
#line 113
{
  LedsP__Led2__toggle();
  if (LedsP__Led2__get()) {
    ;
    }
  else {
#line 118
    ;
    }
}

# 89 "/opt/tinyos-2.1.0/tos/platforms/gina20/Leds.nc"
inline static void fabMACP__Leds__led2Toggle(void ){
#line 89
  LedsP__Leds__led2Toggle();
#line 89
}
#line 89
# 38 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/02-MAC/fabMACP.nc"
static inline void fabMACP__RadioSend__prepareSendDone(error_t error)
#line 38
{
  fabMACP__Leds__led2Toggle();
  fabMACP__RadioSend__sendNow();
}

# 6 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RadioSend.nc"
inline static void RF231driverP__RadioSend__prepareSendDone(error_t error){
#line 6
  fabMACP__RadioSend__prepareSendDone(error);
#line 6
}
#line 6
# 332 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RF231driverP.nc"
static inline void RF231driverP__ChipSpiResource__doneSettingChannel(void )
#line 332
{
  if (RF231driverP__driverState == RF231driverP__TRANSMITTING) {
#line 333
      RF231driverP__RadioSend__prepareSendDone(SUCCESS);
    }
  else {
#line 334
    if (RF231driverP__driverState == RF231driverP__RECEIVING) {
#line 334
        RF231driverP__RadioControl__prepareReceiveDone(SUCCESS);
      }
    else {
#line 335
      if (RF231driverP__driverState == RF231driverP__STARTING) {
#line 335
        return;
        }
      }
    }
}

# 13 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/ChipSpiResource.nc"
inline static void RF231SpiP__ChipSpiResource__doneSettingChannel(void ){
#line 13
  RF231driverP__ChipSpiResource__doneSettingChannel();
#line 13
}
#line 13
#line 8
inline static uint8_t RF231driverP__ChipSpiResource__readRegister(uint8_t registerAddress){
#line 8
  unsigned char result;
#line 8

#line 8
  result = RF231SpiP__ChipSpiResource__readRegister(registerAddress);
#line 8

#line 8
  return result;
#line 8
}
#line 8
# 49 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__getRaw(void )
#line 49
{
#line 49
  return * (volatile uint8_t * )40U & (0x01 << 1);
}

#line 50
static inline bool /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__get(void )
#line 50
{
#line 50
  return /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__getRaw() != 0;
}

# 59 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static bool /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__get(void ){
#line 59
  unsigned char result;
#line 59

#line 59
  result = /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__get();
#line 59

#line 59
  return result;
#line 59
}
#line 59
# 40 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline bool /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__get(void )
#line 40
{
#line 40
  return /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__get();
}

# 32 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static bool LedsP__Led1__get(void ){
#line 32
  unsigned char result;
#line 32

#line 32
  result = /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__get();
#line 32

#line 32
  return result;
#line 32
}
#line 32
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__toggle(void ){
#line 44
  /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__toggle();
#line 44
}
#line 44
# 39 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle(void )
#line 39
{
#line 39
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__toggle();
}

# 31 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__toggle(void ){
#line 31
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle();
#line 31
}
#line 31
# 95 "/opt/tinyos-2.1.0/tos/platforms/gina20/LedsP.nc"
static inline void LedsP__Leds__led1Toggle(void )
#line 95
{
  LedsP__Led1__toggle();
  if (LedsP__Led1__get()) {
    ;
    }
  else {
#line 100
    ;
    }
}

# 72 "/opt/tinyos-2.1.0/tos/platforms/gina20/Leds.nc"
inline static void RF231driverP__Leds__led1Toggle(void ){
#line 72
  LedsP__Leds__led1Toggle();
#line 72
}
#line 72
# 12 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/ChipSpiResource.nc"
inline static void RF231driverP__ChipSpiResource__readPacket(uint8_t *data){
#line 12
  RF231SpiP__ChipSpiResource__readPacket(data);
#line 12
}
#line 12
#line 10
inline static void RF231driverP__ChipSpiResource__loadPacket(uint8_t *data, uint8_t len){
#line 10
  RF231SpiP__ChipSpiResource__loadPacket(data, len);
#line 10
}
#line 10
#line 7
inline static void RF231driverP__ChipSpiResource__setChannel(uint8_t newChannel){
#line 7
  RF231SpiP__ChipSpiResource__setChannel(newChannel);
#line 7
}
#line 7


inline static void RF231driverP__ChipSpiResource__setRegister(uint8_t regAddr, uint8_t newValue){
#line 9
  RF231SpiP__ChipSpiResource__setRegister(regAddr, newValue);
#line 9
}
#line 9
# 66 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RF231driverP.nc"
static inline void RF231driverP__SpiResource__granted(void )
#line 66
{

  uint8_t tempSpiState;

#line 69
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 69
    tempSpiState = RF231driverP__internalSpiState;
#line 69
    __nesc_atomic_end(__nesc_atomic); }


  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 72
    P4OUT &= ~0x0001;
#line 72
    __nesc_atomic_end(__nesc_atomic); }
  if (tempSpiState == RF231driverP__CHANGINGSTATE) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 74
        RF231driverP__ChipSpiResource__changeState(RF231driverP__globalNewReg);
#line 74
        __nesc_atomic_end(__nesc_atomic); }
      RF231driverP__SpiResource__release();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 76
        RF231driverP__internalSpiState = RF231driverP__DEFAULT;
#line 76
        __nesc_atomic_end(__nesc_atomic); }
    }
  else {
    if (tempSpiState == RF231driverP__CHANGINGREG) {
        { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 80
          RF231driverP__ChipSpiResource__setRegister(RF231driverP__globalCommand, RF231driverP__globalNewReg);
#line 80
          __nesc_atomic_end(__nesc_atomic); }
        RF231driverP__SpiResource__release();
        { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 82
          RF231driverP__internalSpiState = RF231driverP__DEFAULT;
#line 82
          __nesc_atomic_end(__nesc_atomic); }
      }
    else {
      if (tempSpiState == RF231driverP__READINGSTATUS) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 86
            RF231driverP__statusRg = RF231driverP__ChipSpiResource__readRegister(0x0f);
#line 86
            __nesc_atomic_end(__nesc_atomic); }
          RF231driverP__SpiResource__release();
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 88
            RF231driverP__statusRead = TRUE;
#line 88
            __nesc_atomic_end(__nesc_atomic); }
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 89
            RF231driverP__internalSpiState = RF231driverP__DEFAULT;
#line 89
            __nesc_atomic_end(__nesc_atomic); }
          RF231driverP__ISRserviced();
        }
      else {
        if (tempSpiState == RF231driverP__CHANGINGFREQ) {
            { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 94
              RF231driverP__ChipSpiResource__setChannel(RF231driverP__globalFrequency);
#line 94
              __nesc_atomic_end(__nesc_atomic); }
            RF231driverP__SpiResource__release();
            { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 96
              RF231driverP__internalSpiState = RF231driverP__DEFAULT;
#line 96
              __nesc_atomic_end(__nesc_atomic); }
          }
        else {
          if (tempSpiState == RF231driverP__LOADINGPACKET) {
              { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 100
                RF231driverP__ChipSpiResource__loadPacket((uint8_t *)&RF231driverP__globalMsg[2], RF231driverP__globalMsg[0]);
#line 100
                __nesc_atomic_end(__nesc_atomic); }
              RF231driverP__SpiResource__release();
              { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 102
                RF231driverP__internalSpiState = RF231driverP__DEFAULT;
#line 102
                __nesc_atomic_end(__nesc_atomic); }
            }
          else {
            if (tempSpiState == RF231driverP__READINGPACKET) {
                RF231driverP__ChipSpiResource__readPacket(RF231driverP__globalReceivedMsg);
                RF231driverP__SpiResource__release();
                { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 108
                  RF231driverP__internalSpiState = RF231driverP__DEFAULT;
#line 108
                  __nesc_atomic_end(__nesc_atomic); }
              }
            else {
              if (tempSpiState == RF231driverP__READINGREG) {
                  RF231driverP__Leds__led1Toggle();
                  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 113
                    RF231driverP__returnedRegValue = RF231driverP__ChipSpiResource__readRegister(RF231driverP__globalCommand);
#line 113
                    __nesc_atomic_end(__nesc_atomic); }

                  RF231driverP__SpiResource__release();

                  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 117
                    RF231driverP__internalSpiState = RF231driverP__DEFAULT;
#line 117
                    __nesc_atomic_end(__nesc_atomic); }
                }
              }
            }
          }
        }
      }
    }
#line 119
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 119
    P4OUT |= 0x0001;
#line 119
    __nesc_atomic_end(__nesc_atomic); }
}

# 414 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/spi/RF231SpiP.nc"
static inline void RF231SpiP__Resource__default__granted(uint8_t id)
#line 414
{
}

# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static void RF231SpiP__Resource__granted(uint8_t arg_0x408569b8){
#line 92
  switch (arg_0x408569b8) {
#line 92
    case /*RF231driverC.RF231SpiC*/RF231SpiC__0__CLIENT_ID:
#line 92
      RF231driverP__SpiResource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      RF231SpiP__Resource__default__granted(arg_0x408569b8);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 405 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/spi/RF231SpiP.nc"
static inline void RF231SpiP__grant__runTask(void )
#line 405
{
  uint8_t holder;

#line 407
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 407
    {
      holder = RF231SpiP__m_holder;
    }
#line 409
    __nesc_atomic_end(__nesc_atomic); }
  RF231SpiP__Resource__granted(holder);
}

# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t RF231driverP__taskStartDone__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(RF231driverP__taskStartDone);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 50 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430InterruptC.nc"
static inline error_t /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void )
#line 50
{
  return /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__enable(TRUE);
}

# 42 "/opt/tinyos-2.1.0/tos/interfaces/GpioInterrupt.nc"
inline static error_t RF231driverP__InterruptIRQRFP__enableRisingEdge(void ){
#line 42
  unsigned char result;
#line 42

#line 42
  result = /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__Interrupt__enableRisingEdge();
#line 42

#line 42
  return result;
#line 42
}
#line 42
# 417 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RF231driverP.nc"
static inline void RF231driverP__receivedSomething(void )
#line 417
{

  RF231driverP__InterruptIRQRFP__enableRisingEdge();
}

# 198 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port16__edge(bool l2h)
#line 198
{
  /* atomic removed: atomic calls only */
#line 199
  {
    if (l2h) {
#line 200
      P1IES &= ~(1 << 6);
      }
    else {
#line 201
      P1IES |= 1 << 6;
      }
  }
}

# 56 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__HplInterrupt__edge(bool low_to_high){
#line 56
  HplMsp430InterruptP__Port16__edge(low_to_high);
#line 56
}
#line 56
# 136 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port16__enable(void )
#line 136
{
#line 136
  P1IE |= 1 << 6;
}

# 31 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__HplInterrupt__enable(void ){
#line 31
  HplMsp430InterruptP__Port16__enable();
#line 31
}
#line 31
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t RF231driverP__taskSendDone__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(RF231driverP__taskSendDone);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 60 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/02-MAC/fabMACP.nc"
static inline void fabMACP__RadioReceive__receive(uint8_t *msg)
#line 60
{
  if (msg[0] == 6) {
#line 61
    fabMACP__Leds__led2Toggle();
    }
}

# 6 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RadioReceive.nc"
inline static void RF231driverP__RadioReceive__receive(uint8_t *msg){
#line 6
  fabMACP__RadioReceive__receive(msg);
#line 6
}
#line 6
# 424 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RF231driverP.nc"
static inline void RF231driverP__unloadPacket(void )
#line 424
{
  /* atomic removed: atomic calls only */
  {
#line 426
    if (RF231driverP__isPacketRead) {
#line 426
      return;
      }
  }
#line 427
  if (RF231driverP__SpiResource__immediateRequest()) {
      /* atomic removed: atomic calls only */
#line 428
      P4OUT &= ~0x0001;

      RF231driverP__ChipSpiResource__readPacket(RF231driverP__globalReceivedMsg);
      RF231driverP__SpiResource__release();
      /* atomic removed: atomic calls only */
#line 432
      P4OUT |= 0x0001;
    }
  else 

    {
      /* atomic removed: atomic calls only */
#line 437
      RF231driverP__internalSpiState = RF231driverP__READINGPACKET;
      RF231driverP__SpiResource__request();
    }

  RF231driverP__RadioReceive__receive(RF231driverP__globalReceivedMsg);
}

# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__signalDone_task__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(/*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__signalDone_task);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 285 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA0P.nc"
static inline void HplMsp430UsciA0P__Usci__enableRxIntr(void )
#line 285
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 286
    {
      HplMsp430UsciA0P__IFG2 &= ~(1 << 0);
      HplMsp430UsciA0P__IE2 |= 1 << 0;
    }
#line 289
    __nesc_atomic_end(__nesc_atomic); }
}

# 164 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA.nc"
inline static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__enableRxIntr(void ){
#line 164
  HplMsp430UsciA0P__Usci__enableRxIntr();
#line 164
}
#line 164
# 106 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
static inline error_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__SpiPacket__send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len)
#line 108
{

  /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_client = id;
  /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_tx_buf = tx_buf;
  /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_rx_buf = rx_buf;
  /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_len = len;
  /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_pos = 0;

  if (len) {
      /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__enableRxIntr();
      /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__continueOp();
    }
  else {
      /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__signalDone_task__postTask();
    }

  return SUCCESS;
}

# 59 "/opt/tinyos-2.1.0/tos/interfaces/SpiPacket.nc"
inline static error_t RF231SpiP__SpiPacket__send(uint8_t * txBuf, uint8_t * rxBuf, uint16_t len){
#line 59
  unsigned char result;
#line 59

#line 59
  result = /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__SpiPacket__send(/*RF231SpiWireC.HplRF231SpiC.SpiC*/Msp430SpiA0C__0__CLIENT_ID, txBuf, rxBuf, len);
#line 59

#line 59
  return result;
#line 59
}
#line 59
# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
inline static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void ){
#line 53
  unsigned long result;
#line 53

#line 53
  result = /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get();
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 75 "/opt/tinyos-2.1.0/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void )
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
}

# 98 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void ){
#line 98
  unsigned long result;
#line 98

#line 98
  result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow();
#line 98

#line 98
  return result;
#line 98
}
#line 98
# 85 "/opt/tinyos-2.1.0/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void )
{
#line 86
  return /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow();
}

# 125 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void ){
#line 125
  unsigned long result;
#line 125

#line 125
  result = /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow();
#line 125

#line 125
  return result;
#line 125
}
#line 125
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 133 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
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

# 62 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static void fabMACP__Timer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(0U, dt);
#line 62
}
#line 62
# 43 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/02-MAC/fabMACP.nc"
static inline void fabMACP__RadioSend__sendNowDone(error_t error)
#line 43
{
  fabMACP__Timer__startOneShot(200);
}

# 8 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RadioSend.nc"
inline static void RF231driverP__RadioSend__sendNowDone(error_t error){
#line 8
  fabMACP__RadioSend__sendNowDone(error);
#line 8
}
#line 8
# 359 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RF231driverP.nc"
static inline void RF231driverP__taskSendDone__runTask(void )
#line 359
{








  RF231driverP__RadioSend__sendNowDone(SUCCESS);
}

# 53 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/02-MAC/fabMACP.nc"
static inline void fabMACP__RadioControl__stopDone(error_t error)
#line 53
{
  fabMACP__Leds__led1Toggle();
  return;
}

# 10 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RadioControl.nc"
inline static void RF231driverP__RadioControl__stopDone(error_t error){
#line 10
  fabMACP__RadioControl__stopDone(error);
#line 10
}
#line 10
# 490 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RF231driverP.nc"
static inline void RF231driverP__taskStopDone__runTask(void )
#line 490
{
  RF231driverP__RadioControl__stopDone(SUCCESS);
}

#line 259
static inline error_t RF231driverP__RadioSend__prepareSend(uint8_t *msg)
#line 259
{

  uint8_t count;

#line 262
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 262
    RF231driverP__driverState = RF231driverP__TRANSMITTING;
#line 262
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 263
    count = 0;
#line 263
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 264
    RF231driverP__frequencyChannel = msg[1];
#line 264
    __nesc_atomic_end(__nesc_atomic); }





  if (RF231driverP__SpiResource__immediateRequest()) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 271
        P4OUT &= ~0x0001;
#line 271
        __nesc_atomic_end(__nesc_atomic); }

      RF231driverP__ChipSpiResource__loadPacket((uint8_t *)&msg[2], msg[0]);
      RF231driverP__SpiResource__release();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 275
        P4OUT |= 0x0001;
#line 275
        __nesc_atomic_end(__nesc_atomic); }
    }
  else 
    {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 279
        RF231driverP__internalSpiState = RF231driverP__LOADINGPACKET;
#line 279
        __nesc_atomic_end(__nesc_atomic); }
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 280
        RF231driverP__globalMsg = msg;
#line 280
        __nesc_atomic_end(__nesc_atomic); }
      RF231driverP__SpiResource__request();
    }

  return SUCCESS;
}

# 5 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RadioSend.nc"
inline static error_t fabMACP__RadioSend__prepareSend(uint8_t *msg){
#line 5
  unsigned char result;
#line 5

#line 5
  result = RF231driverP__RadioSend__prepareSend(msg);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 49 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__getRaw(void )
#line 49
{
#line 49
  return * (volatile uint8_t * )40U & (0x01 << 3);
}

#line 50
static inline bool /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__get(void )
#line 50
{
#line 50
  return /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__getRaw() != 0;
}

# 59 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static bool /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__HplGeneralIO__get(void ){
#line 59
  unsigned char result;
#line 59

#line 59
  result = /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__get();
#line 59

#line 59
  return result;
#line 59
}
#line 59
# 40 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline bool /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__GeneralIO__get(void )
#line 40
{
#line 40
  return /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__HplGeneralIO__get();
}

# 32 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static bool LedsP__Led3__get(void ){
#line 32
  unsigned char result;
#line 32

#line 32
  result = /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__GeneralIO__get();
#line 32

#line 32
  return result;
#line 32
}
#line 32
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__HplGeneralIO__toggle(void ){
#line 44
  /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__toggle();
#line 44
}
#line 44
# 39 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__GeneralIO__toggle(void )
#line 39
{
#line 39
  /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__HplGeneralIO__toggle();
}

# 31 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led3__toggle(void ){
#line 31
  /*PlatformLedsC.Led3Impl*/Msp430GpioC__3__GeneralIO__toggle();
#line 31
}
#line 31
# 131 "/opt/tinyos-2.1.0/tos/platforms/gina20/LedsP.nc"
static inline void LedsP__Leds__led3Toggle(void )
#line 131
{
  LedsP__Led3__toggle();
  if (LedsP__Led3__get()) {
    ;
    }
  else {
#line 136
    ;
    }
}

# 105 "/opt/tinyos-2.1.0/tos/platforms/gina20/Leds.nc"
inline static void fabMACP__Leds__led3Toggle(void ){
#line 105
  LedsP__Leds__led3Toggle();
#line 105
}
#line 105
# 21 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/02-MAC/fabMACP.nc"
static inline void fabMACP__RadioControl__startDone(error_t error)
#line 21
{
  uint8_t packet[10];

#line 23
  fabMACP__Leds__led3Toggle();
  packet[0] = 0x06;
  packet[1] = 0x0E;
  packet[2] = 0xDE;
  packet[3] = 0xAD;
  packet[4] = 0xBE;
  packet[5] = 0xEF;
  fabMACP__RadioSend__prepareSend(packet);
}

# 3 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RadioControl.nc"
inline static void RF231driverP__RadioControl__startDone(error_t error){
#line 3
  fabMACP__RadioControl__startDone(error);
#line 3
}
#line 3
# 252 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RF231driverP.nc"
static inline void RF231driverP__taskStartDone__runTask(void )
#line 252
{
  RF231driverP__RadioControl__startDone(SUCCESS);
}

# 92 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 47 "/opt/tinyos-2.1.0/tos/lib/timer/AlarmToTimerC.nc"
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

# 118 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt){
#line 118
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(t0, dt);
#line 118
}
#line 118
# 54 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
}

# 62 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void ){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop();
#line 62
}
#line 62
# 91 "/opt/tinyos-2.1.0/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop();
}

# 62 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void ){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop();
#line 62
}
#line 62
# 60 "/opt/tinyos-2.1.0/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void )
{
#line 61
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop();
}

# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop();
#line 67
}
#line 67
# 89 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
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

# 55 "/opt/tinyos-2.1.0/tos/lib/timer/BusyWait.nc"
inline static void RF231driverP__radioWait__wait(RF231driverP__radioWait__size_type dt){
#line 55
  /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__wait(dt);
#line 55
}
#line 55
# 448 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RF231driverP.nc"
static inline error_t RF231driverP__RadioControl__stop(void )
#line 448
{
  uint8_t temp_state;

#line 450
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 450
    temp_state = RF231driverP__radioState;
#line 450
    __nesc_atomic_end(__nesc_atomic); }
  switch (temp_state) {
      case 8: 
        RF231driverP__shutdown();
      return SUCCESS;
      case 15: 
        return SUCCESS;
      default: 
        { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 458
          {
            if (RF231driverP__radioState != 8) {
                if (RF231driverP__SpiResource__immediateRequest()) {
                    P4OUT &= ~0x0001;
                    RF231driverP__ChipSpiResource__changeState(0x08);
                    RF231driverP__SpiResource__release();
                    P4OUT |= 0x0001;
                  }
                else {
                    RF231driverP__internalSpiState = RF231driverP__CHANGINGSTATE;
                    RF231driverP__globalNewReg = 0x08;
                    RF231driverP__SpiResource__request();
                  }
                RF231driverP__radioWait__wait(400);
                RF231driverP__radioState = 8;
              }
          }
#line 474
          __nesc_atomic_end(__nesc_atomic); }
#line 474
      RF231driverP__radioWait__wait(1);
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 475
        RF231driverP__radioState = 8;
#line 475
        __nesc_atomic_end(__nesc_atomic); }
    }
  RF231driverP__shutdown();
  return SUCCESS;
}

# 9 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RadioControl.nc"
inline static error_t fabMACP__RadioControl__stop(void ){
#line 9
  unsigned char result;
#line 9

#line 9
  result = RF231driverP__RadioControl__stop();
#line 9

#line 9
  return result;
#line 9
}
#line 9
# 49 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__getRaw(void )
#line 49
{
#line 49
  return * (volatile uint8_t * )40U & (0x01 << 0);
}

#line 50
static inline bool /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__get(void )
#line 50
{
#line 50
  return /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__getRaw() != 0;
}

# 59 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static bool /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__get(void ){
#line 59
  unsigned char result;
#line 59

#line 59
  result = /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__get();
#line 59

#line 59
  return result;
#line 59
}
#line 59
# 40 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline bool /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__get(void )
#line 40
{
#line 40
  return /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__get();
}

# 32 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static bool LedsP__Led0__get(void ){
#line 32
  unsigned char result;
#line 32

#line 32
  result = /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__get();
#line 32

#line 32
  return result;
#line 32
}
#line 32
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__toggle(void ){
#line 44
  /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__toggle();
#line 44
}
#line 44
# 39 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle(void )
#line 39
{
#line 39
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__toggle();
}

# 31 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__toggle(void ){
#line 31
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle();
#line 31
}
#line 31
# 75 "/opt/tinyos-2.1.0/tos/platforms/gina20/LedsP.nc"
static inline void LedsP__Leds__led0Toggle(void )
#line 75
{
  LedsP__Led0__toggle();


  if (LedsP__Led0__get()) {
    ;
    }
  else {
#line 82
    ;
    }
}

# 56 "/opt/tinyos-2.1.0/tos/platforms/gina20/Leds.nc"
inline static void fabMACP__Leds__led0Toggle(void ){
#line 56
  LedsP__Leds__led0Toggle();
#line 56
}
#line 56
# 47 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/02-MAC/fabMACP.nc"
static inline void fabMACP__Timer__fired(void )
{
  fabMACP__Leds__led0Toggle();
  fabMACP__RadioControl__stop();
}

# 193 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num)
{
}

# 72 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(uint8_t arg_0x407969f0){
#line 72
  switch (arg_0x407969f0) {
#line 72
    case 0U:
#line 72
      fabMACP__Timer__fired();
#line 72
      break;
#line 72
    default:
#line 72
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(arg_0x407969f0);
#line 72
      break;
#line 72
    }
#line 72
}
#line 72
# 51 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void )
{




  if (0) {
      /* atomic removed: atomic calls only */
#line 58
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t * )368U;

#line 61
        do {
#line 61
            t0 = t1;
#line 61
            t1 = * (volatile uint16_t * )368U;
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
      return * (volatile uint16_t * )368U;
    }
}

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__get(void ){
#line 34
  unsigned int result;
#line 34

#line 34
  result = /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get();
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 38 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get(void )
{
  return /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__get();
}

# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
inline static /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get(void ){
#line 53
  unsigned int result;
#line 53

#line 53
  result = /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get();
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t RF231driverP__taskStopDone__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(RF231driverP__taskStopDone);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 128 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void )
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow());
}

# 72 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void ){
#line 72
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired();
#line 72
}
#line 72
# 80 "/opt/tinyos-2.1.0/tos/lib/timer/TransformAlarmC.nc"
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

# 105 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void ){
#line 105
  unsigned long result;
#line 105

#line 105
  result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm();
#line 105

#line 105
  return result;
#line 105
}
#line 105
# 63 "/opt/tinyos-2.1.0/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt, FALSE);
    }
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired();
}

# 58 "/opt/tinyos-2.1.0/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 46 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4320 {
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

# 36 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void ){
#line 36
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare();
#line 36
}
#line 36
# 42 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 81 "/opt/tinyos-2.1.0/tos/system/StateImplP.nc"
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

# 45 "/opt/tinyos-2.1.0/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void )
#line 45
{
  memset(/*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ, /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY, sizeof /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ);
  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
inline static error_t RealMainP__SoftwareInit__init(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init();
#line 51
  result = ecombine(result, StateImplP__Init__init());
#line 51
  result = ecombine(result, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init());
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 127 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RF231driverP.nc"
static inline error_t RF231driverP__RadioControl__start(void )
#line 127
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 128
    RF231driverP__radioState = 0;
#line 128
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 129
    RF231driverP__internalSpiState = RF231driverP__DEFAULT;
#line 129
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 130
    RF231driverP__driverState = RF231driverP__STARTING;
#line 130
    __nesc_atomic_end(__nesc_atomic); }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 132
    P4OUT |= 0x0001;
#line 132
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 133
    P4OUT &= ~0x0080;
#line 133
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 134
    P4DIR |= 0x0001;
#line 134
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 135
    P4DIR |= 0x0080;
#line 135
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 136
    P4OUT &= ~0x0080;
#line 136
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 137
    P4OUT &= ~0x0080;
#line 137
    __nesc_atomic_end(__nesc_atomic); }


  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 140
    {
      if (RF231driverP__SpiResource__immediateRequest()) {
          P4OUT &= ~0x0001;
          RF231driverP__ChipSpiResource__setRegister(0x1C, 0x03);
          RF231driverP__SpiResource__release();
          P4OUT |= 0x0001;
        }
      else {
          RF231driverP__internalSpiState = RF231driverP__CHANGINGREG;
          RF231driverP__globalCommand = 0x1C;
          RF231driverP__globalNewReg = 0x03;
          RF231driverP__SpiResource__request();
        }
    }
#line 153
    __nesc_atomic_end(__nesc_atomic); }


  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 156
    RF231driverP__preparingPacket = FALSE;
#line 156
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 157
    {
      if (RF231driverP__SpiResource__immediateRequest()) {
          P4OUT &= ~0x0001;
          RF231driverP__ChipSpiResource__setChannel(0x0D);
          RF231driverP__SpiResource__release();
          P4OUT |= 0x0001;
        }
      else {
          RF231driverP__internalSpiState = RF231driverP__CHANGINGFREQ;
          RF231driverP__globalFrequency = 0x0D;
          RF231driverP__SpiResource__request();
        }
    }
#line 169
    __nesc_atomic_end(__nesc_atomic); }
#line 189
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 189
    {
      if (RF231driverP__radioState != 8) {
          if (RF231driverP__SpiResource__immediateRequest()) {
              P4OUT &= ~0x0001;
              RF231driverP__ChipSpiResource__changeState(0x08);
              RF231driverP__SpiResource__release();
              P4OUT |= 0x0001;
            }
          else {
              RF231driverP__internalSpiState = RF231driverP__CHANGINGSTATE;
              RF231driverP__globalNewReg = 0x08;
              RF231driverP__SpiResource__request();
            }

          RF231driverP__radioWait__wait(500);
          RF231driverP__radioState = 8;
        }
    }
#line 206
    __nesc_atomic_end(__nesc_atomic); }




  if (RF231driverP__SpiResource__immediateRequest()) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 212
        P4OUT &= ~0x0001;
#line 212
        __nesc_atomic_end(__nesc_atomic); }
      RF231driverP__ChipSpiResource__setRegister(0x0e, 0x0D);
      RF231driverP__SpiResource__release();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 215
        P4OUT |= 0x0001;
#line 215
        __nesc_atomic_end(__nesc_atomic); }
    }
  else 
    {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 219
        RF231driverP__internalSpiState = RF231driverP__CHANGINGREG;
#line 219
        __nesc_atomic_end(__nesc_atomic); }
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 220
        RF231driverP__globalCommand = 0x0e;
#line 220
        __nesc_atomic_end(__nesc_atomic); }
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 221
        RF231driverP__globalNewReg = 0x0D;
#line 221
        __nesc_atomic_end(__nesc_atomic); }
      RF231driverP__SpiResource__request();
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 225
    P4OUT |= 0x0001;
#line 225
    __nesc_atomic_end(__nesc_atomic); }

  RF231driverP__InterruptIRQRFP__enableRisingEdge();

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 229
    {
      if (RF231driverP__radioState != 9) {
          if (RF231driverP__SpiResource__immediateRequest()) {
              P4OUT &= ~0x0001;
              RF231driverP__ChipSpiResource__changeState(0x09);
              RF231driverP__SpiResource__release();
              P4OUT |= 0x0001;
            }
          else {
              RF231driverP__internalSpiState = RF231driverP__CHANGINGSTATE;
              RF231driverP__globalNewReg = 0x09;
              RF231driverP__SpiResource__request();
            }
        }
    }
#line 243
    __nesc_atomic_end(__nesc_atomic); }





  return SUCCESS;
}

# 2 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RadioControl.nc"
inline static error_t fabMACP__RadioControl__start(void ){
#line 2
  unsigned char result;
#line 2

#line 2
  result = RF231driverP__RadioControl__start();
#line 2

#line 2
  return result;
#line 2
}
#line 2
# 12 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/02-MAC/fabMACP.nc"
static inline void fabMACP__Boot__booted(void )
{
  fabMACP__Leds__led0Toggle();
  fabMACP__Leds__led1Toggle();
  fabMACP__Leds__led2Toggle();
  fabMACP__Leds__led3Toggle();
  fabMACP__RadioControl__start();
}

# 49 "/opt/tinyos-2.1.0/tos/interfaces/Boot.nc"
inline static void RealMainP__Boot__booted(void ){
#line 49
  fabMACP__Boot__booted();
#line 49
}
#line 49
# 208 "/opt/tinyos-2.1.0/tos/chips/msp430X/msp430hardware.h"
static inline  void __nesc_disable_interrupt(void )
{
   __asm volatile ("dint");
   __asm volatile ("nop");}

# 133 "/opt/tinyos-2.1.0/tos/chips/msp430X/McuSleepC.nc"
static inline mcu_power_t McuSleepC__McuPowerOverride__default__lowestState(void )
#line 133
{
  return MSP430_POWER_LPM4;
}

# 54 "/opt/tinyos-2.1.0/tos/interfaces/McuPowerOverride.nc"
inline static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void ){
#line 54
  unsigned char result;
#line 54

#line 54
  result = McuSleepC__McuPowerOverride__default__lowestState();
#line 54

#line 54
  return result;
#line 54
}
#line 54
# 68 "/opt/tinyos-2.1.0/tos/chips/msp430X/McuSleepC.nc"
static inline mcu_power_t McuSleepC__getPowerState(void )
#line 68
{
  mcu_power_t pState = MSP430_POWER_LPM3;





  if (((((((
#line 71
  TA0CCTL0 & 0x0010 || TA0CCTL1 & 0x0010) || TA0CCTL2 & 0x0010) && (
  TA0CTL & (3 << 8)) == 2 << 8) || (
  UCA0CTL1 & (3 << 6)) != 0 << 6) || (
  UCA1CTL1 & (3 << 6)) != 0 << 6) || (
  UCB0CTL1 & (3 << 6)) != 0 << 6) || (
  UCB1CTL1 & (3 << 6)) != 0 << 6) {

    pState = MSP430_POWER_LPM1;
    }



  if (ADC12CTL0 & 0x0010) {
      if (ADC12CTL1 & (2 << 3)) {

          if (ADC12CTL1 & (1 << 3)) {
            pState = MSP430_POWER_LPM1;
            }
          else {
#line 89
            pState = MSP430_POWER_ACTIVE;
            }
        }
      else {
#line 90
        if (ADC12CTL1 & 0x0400 && (TA0CTL & (3 << 8)) == 2 << 8) {



            pState = MSP430_POWER_LPM1;
          }
        }
    }

  return pState;
}

# 196 "/opt/tinyos-2.1.0/tos/chips/msp430X/msp430hardware.h"
static inline  mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 196
{
  return m1 < m2 ? m1 : m2;
}

# 102 "/opt/tinyos-2.1.0/tos/chips/msp430X/McuSleepC.nc"
static inline void McuSleepC__computePowerState(void )
#line 102
{
  McuSleepC__powerState = mcombine(McuSleepC__getPowerState(), 
  McuSleepC__McuPowerOverride__lowestState());
}

static inline void McuSleepC__McuSleep__sleep(void )
#line 107
{
  uint16_t temp;

#line 109
  if (McuSleepC__dirty) {
      McuSleepC__computePowerState();
    }










  temp = McuSleepC__msp430PowerBits[McuSleepC__powerState] | 0x0008;
   __asm volatile ("bis  %0, r2" :  : "m"(temp));

   __asm volatile ("" :  :  : "memory");
  __nesc_disable_interrupt();
}

# 59 "/opt/tinyos-2.1.0/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP__McuSleep__sleep(void ){
#line 59
  McuSleepC__McuSleep__sleep();
#line 59
}
#line 59
# 67 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
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

# 61 "/opt/tinyos-2.1.0/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__taskLoop(void ){
#line 61
  SchedulerBasicP__Scheduler__taskLoop();
#line 61
}
#line 61
# 88 "/opt/tinyos-2.1.0/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__userId(void ){
#line 88
  unsigned char result;
#line 88

#line 88
  result = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId();
#line 88

#line 88
  return result;
#line 88
}
#line 88
# 270 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA0P.nc"
static inline void HplMsp430UsciA0P__Usci__disableRxIntr(void )
#line 270
{
  HplMsp430UsciA0P__IE2 &= ~(1 << 0);
}

# 161 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA.nc"
inline static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__disableRxIntr(void ){
#line 161
  HplMsp430UsciA0P__Usci__disableRxIntr();
#line 161
}
#line 161
# 132 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
static inline void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciInterrupts__rxDone(uint8_t data)
#line 132
{

  if (/*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_rx_buf) {
    /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_rx_buf[/*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_pos - 1] = data;
    }
  if (/*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_pos < /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_len) {
    /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__continueOp();
    }
  else 
#line 139
    {
      /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__disableRxIntr();
      /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__signalDone();
    }
}

# 89 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data)
#line 89
{
}

# 85 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
inline static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__rxDone(uint8_t arg_0x409701b8, uint8_t data){
#line 85
  switch (arg_0x409701b8) {
#line 85
    case /*RF231SpiWireC.HplRF231SpiC.SpiC.UsciC*/Msp430UsciA0C__0__CLIENT_ID:
#line 85
      /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciInterrupts__rxDone(data);
#line 85
      break;
#line 85
    default:
#line 85
      /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__rxDone(arg_0x409701b8, data);
#line 85
      break;
#line 85
    }
#line 85
}
#line 85
# 80 "/opt/tinyos-2.1.0/tos/interfaces/ArbiterInfo.nc"
inline static bool /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__inUse(void ){
#line 80
  unsigned char result;
#line 80

#line 80
  result = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse();
#line 80

#line 80
  return result;
#line 80
}
#line 80
# 83 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__rxDone(uint8_t data)
#line 83
{
  if (/*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__rxDone(/*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__userId(), data);
    }
}

# 85 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
inline static void HplMsp430UsciA0P__Interrupts__rxDone(uint8_t data){
#line 85
  /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__rxDone(data);
#line 85
}
#line 85
# 127 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA0P.nc"
static inline void HplMsp430UsciA0P__UsciRawInterrupts__rxDone(uint8_t temp)
#line 127
{
  HplMsp430UsciA0P__Interrupts__rxDone(temp);
}

# 85 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciRawInterrupts.nc"
inline static void HplMsp430UsciAB0RawInterruptsP__UsciA__rxDone(uint8_t data){
#line 85
  HplMsp430UsciA0P__UsciRawInterrupts__rxDone(data);
#line 85
}
#line 85
# 81 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciAB0RawInterruptsP.nc"
static inline void HplMsp430UsciAB0RawInterruptsP__UsciB__default__rxDone(uint8_t temp)
#line 81
{
  return;
}

# 85 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciRawInterrupts.nc"
inline static void HplMsp430UsciAB0RawInterruptsP__UsciB__rxDone(uint8_t data){
#line 85
  HplMsp430UsciAB0RawInterruptsP__UsciB__default__rxDone(data);
#line 85
}
#line 85
# 150 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
static inline void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciInterrupts__txDone(void )
#line 150
{
}

# 88 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__txDone(uint8_t id)
#line 88
{
}

# 80 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
inline static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__txDone(uint8_t arg_0x409701b8){
#line 80
  switch (arg_0x409701b8) {
#line 80
    case /*RF231SpiWireC.HplRF231SpiC.SpiC.UsciC*/Msp430UsciA0C__0__CLIENT_ID:
#line 80
      /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__UsciInterrupts__txDone();
#line 80
      break;
#line 80
    default:
#line 80
      /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__txDone(arg_0x409701b8);
#line 80
      break;
#line 80
    }
#line 80
}
#line 80
# 78 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__txDone(void )
#line 78
{
  if (/*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__txDone(/*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__userId());
    }
}

# 80 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
inline static void HplMsp430UsciA0P__Interrupts__txDone(void ){
#line 80
  /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__txDone();
#line 80
}
#line 80
# 131 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA0P.nc"
static inline void HplMsp430UsciA0P__UsciRawInterrupts__txDone(void )
#line 131
{
  HplMsp430UsciA0P__Interrupts__txDone();
}

# 80 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciRawInterrupts.nc"
inline static void HplMsp430UsciAB0RawInterruptsP__UsciA__txDone(void ){
#line 80
  HplMsp430UsciA0P__UsciRawInterrupts__txDone();
#line 80
}
#line 80
# 77 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciAB0RawInterruptsP.nc"
static inline void HplMsp430UsciAB0RawInterruptsP__UsciB__default__txDone(void )
#line 77
{
  return;
}

# 80 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciRawInterrupts.nc"
inline static void HplMsp430UsciAB0RawInterruptsP__UsciB__txDone(void ){
#line 80
  HplMsp430UsciAB0RawInterruptsP__UsciB__default__txDone();
#line 80
}
#line 80
# 146 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__clear(void )
#line 146
{
#line 146
  P1IFG &= ~(1 << 0);
}

#line 122
static inline void HplMsp430InterruptP__Port10__default__fired(void )
#line 122
{
#line 122
  HplMsp430InterruptP__Port10__clear();
}

# 61 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port10__fired(void ){
#line 61
  HplMsp430InterruptP__Port10__default__fired();
#line 61
}
#line 61
# 147 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port11__clear(void )
#line 147
{
#line 147
  P1IFG &= ~(1 << 1);
}

#line 123
static inline void HplMsp430InterruptP__Port11__default__fired(void )
#line 123
{
#line 123
  HplMsp430InterruptP__Port11__clear();
}

# 61 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port11__fired(void ){
#line 61
  HplMsp430InterruptP__Port11__default__fired();
#line 61
}
#line 61
# 148 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port12__clear(void )
#line 148
{
#line 148
  P1IFG &= ~(1 << 2);
}

#line 124
static inline void HplMsp430InterruptP__Port12__default__fired(void )
#line 124
{
#line 124
  HplMsp430InterruptP__Port12__clear();
}

# 61 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port12__fired(void ){
#line 61
  HplMsp430InterruptP__Port12__default__fired();
#line 61
}
#line 61
# 149 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port13__clear(void )
#line 149
{
#line 149
  P1IFG &= ~(1 << 3);
}

#line 125
static inline void HplMsp430InterruptP__Port13__default__fired(void )
#line 125
{
#line 125
  HplMsp430InterruptP__Port13__clear();
}

# 61 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port13__fired(void ){
#line 61
  HplMsp430InterruptP__Port13__default__fired();
#line 61
}
#line 61
# 150 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__clear(void )
#line 150
{
#line 150
  P1IFG &= ~(1 << 4);
}

#line 126
static inline void HplMsp430InterruptP__Port14__default__fired(void )
#line 126
{
#line 126
  HplMsp430InterruptP__Port14__clear();
}

# 61 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port14__fired(void ){
#line 61
  HplMsp430InterruptP__Port14__default__fired();
#line 61
}
#line 61
# 151 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port15__clear(void )
#line 151
{
#line 151
  P1IFG &= ~(1 << 5);
}

#line 127
static inline void HplMsp430InterruptP__Port15__default__fired(void )
#line 127
{
#line 127
  HplMsp430InterruptP__Port15__clear();
}

# 61 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port15__fired(void ){
#line 61
  HplMsp430InterruptP__Port15__default__fired();
#line 61
}
#line 61
# 152 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port16__clear(void )
#line 152
{
#line 152
  P1IFG &= ~(1 << 6);
}

# 41 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port16__clear();
#line 41
}
#line 41
# 144 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port16__disable(void )
#line 144
{
#line 144
  P1IE &= ~(1 << 6);
}

# 36 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__HplInterrupt__disable(void ){
#line 36
  HplMsp430InterruptP__Port16__disable();
#line 36
}
#line 36
# 58 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430InterruptC.nc"
static inline error_t /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__Interrupt__disable(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__HplInterrupt__disable();
    /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__HplInterrupt__clear();
  }
  return SUCCESS;
}

# 50 "/opt/tinyos-2.1.0/tos/interfaces/GpioInterrupt.nc"
inline static error_t RF231driverP__InterruptIRQRFP__disable(void ){
#line 50
  unsigned char result;
#line 50

#line 50
  result = /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__Interrupt__disable();
#line 50

#line 50
  return result;
#line 50
}
#line 50
# 502 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RF231driverP.nc"
static inline void RF231driverP__InterruptIRQRFP__fired(void )
#line 502
{

  RF231driverP__InterruptIRQRFP__disable();
  /* atomic removed: atomic calls only */
#line 505
  RF231driverP__statusRead = FALSE;

  if (RF231driverP__SpiResource__immediateRequest()) {
      /* atomic removed: atomic calls only */
#line 508
      P4OUT &= ~0x0001;
      /* atomic removed: atomic calls only */
#line 509
      RF231driverP__statusRg = RF231driverP__ChipSpiResource__readRegister(0x0f);
      RF231driverP__SpiResource__release();
      /* atomic removed: atomic calls only */
#line 511
      P4OUT |= 0x0001;
      /* atomic removed: atomic calls only */
#line 512
      RF231driverP__statusRead = TRUE;
    }
  else 
    {
      /* atomic removed: atomic calls only */
#line 516
      RF231driverP__internalSpiState = RF231driverP__READINGSTATUS;
      RF231driverP__SpiResource__request();
    }
  if (RF231driverP__statusRead == TRUE) {
    RF231driverP__ISRserviced();
    }
}

# 57 "/opt/tinyos-2.1.0/tos/interfaces/GpioInterrupt.nc"
inline static void /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__Interrupt__fired(void ){
#line 57
  RF231driverP__InterruptIRQRFP__fired();
#line 57
}
#line 57
# 66 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430InterruptC.nc"
static inline void /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__HplInterrupt__fired(void )
#line 66
{
  /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__HplInterrupt__clear();
  /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__Interrupt__fired();
}

# 61 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port16__fired(void ){
#line 61
  /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__HplInterrupt__fired();
#line 61
}
#line 61
# 153 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port17__clear(void )
#line 153
{
#line 153
  P1IFG &= ~(1 << 7);
}

#line 129
static inline void HplMsp430InterruptP__Port17__default__fired(void )
#line 129
{
#line 129
  HplMsp430InterruptP__Port17__clear();
}

# 61 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port17__fired(void ){
#line 61
  HplMsp430InterruptP__Port17__default__fired();
#line 61
}
#line 61
# 273 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port20__clear(void )
#line 273
{
#line 273
  P2IFG &= ~(1 << 0);
}

#line 249
static inline void HplMsp430InterruptP__Port20__default__fired(void )
#line 249
{
#line 249
  HplMsp430InterruptP__Port20__clear();
}

# 61 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port20__fired(void ){
#line 61
  HplMsp430InterruptP__Port20__default__fired();
#line 61
}
#line 61
# 274 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port21__clear(void )
#line 274
{
#line 274
  P2IFG &= ~(1 << 1);
}

#line 250
static inline void HplMsp430InterruptP__Port21__default__fired(void )
#line 250
{
#line 250
  HplMsp430InterruptP__Port21__clear();
}

# 61 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port21__fired(void ){
#line 61
  HplMsp430InterruptP__Port21__default__fired();
#line 61
}
#line 61
# 275 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port22__clear(void )
#line 275
{
#line 275
  P2IFG &= ~(1 << 2);
}

#line 251
static inline void HplMsp430InterruptP__Port22__default__fired(void )
#line 251
{
#line 251
  HplMsp430InterruptP__Port22__clear();
}

# 61 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port22__fired(void ){
#line 61
  HplMsp430InterruptP__Port22__default__fired();
#line 61
}
#line 61
# 276 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port23__clear(void )
#line 276
{
#line 276
  P2IFG &= ~(1 << 3);
}

#line 252
static inline void HplMsp430InterruptP__Port23__default__fired(void )
#line 252
{
#line 252
  HplMsp430InterruptP__Port23__clear();
}

# 61 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port23__fired(void ){
#line 61
  HplMsp430InterruptP__Port23__default__fired();
#line 61
}
#line 61
# 277 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port24__clear(void )
#line 277
{
#line 277
  P2IFG &= ~(1 << 4);
}

#line 253
static inline void HplMsp430InterruptP__Port24__default__fired(void )
#line 253
{
#line 253
  HplMsp430InterruptP__Port24__clear();
}

# 61 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port24__fired(void ){
#line 61
  HplMsp430InterruptP__Port24__default__fired();
#line 61
}
#line 61
# 278 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port25__clear(void )
#line 278
{
#line 278
  P2IFG &= ~(1 << 5);
}

#line 254
static inline void HplMsp430InterruptP__Port25__default__fired(void )
#line 254
{
#line 254
  HplMsp430InterruptP__Port25__clear();
}

# 61 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port25__fired(void ){
#line 61
  HplMsp430InterruptP__Port25__default__fired();
#line 61
}
#line 61
# 279 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port26__clear(void )
#line 279
{
#line 279
  P2IFG &= ~(1 << 6);
}

#line 255
static inline void HplMsp430InterruptP__Port26__default__fired(void )
#line 255
{
#line 255
  HplMsp430InterruptP__Port26__clear();
}

# 61 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port26__fired(void ){
#line 61
  HplMsp430InterruptP__Port26__default__fired();
#line 61
}
#line 61
# 280 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port27__clear(void )
#line 280
{
#line 280
  P2IFG &= ~(1 << 7);
}

#line 256
static inline void HplMsp430InterruptP__Port27__default__fired(void )
#line 256
{
#line 256
  HplMsp430InterruptP__Port27__clear();
}

# 61 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port27__fired(void ){
#line 61
  HplMsp430InterruptP__Port27__default__fired();
#line 61
}
#line 61
# 228 "/opt/tinyos-2.1.0/tos/chips/msp430X/msp430hardware.h"
  __nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = (({
#line 230
    uint16_t __x;

#line 230
     __asm volatile ("mov	r2, %0" : "=r"((uint16_t )__x));__x;
  }
  )
#line 230
   & 0x0008) != 0;

#line 231
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

# 11 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(50)))  void sig_TIMERA0_VECTOR(void )
#line 11
{



  Msp430TimerCommonP__VectorTimerA0__fired();
}

# 169 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
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

# 21 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(48)))  void sig_TIMERA1_VECTOR(void )
#line 21
{



  Msp430TimerCommonP__VectorTimerA1__fired();
}




__attribute((wakeup)) __attribute((interrupt(58)))  void sig_TIMERB0_VECTOR(void )
#line 31
{



  Msp430TimerCommonP__VectorTimerB0__fired();
}

# 135 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n)
{
}

# 28 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(uint8_t arg_0x404d0ce8){
#line 28
  switch (arg_0x404d0ce8) {
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
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(arg_0x404d0ce8);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 159 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
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

# 96 "/opt/tinyos-2.1.0/tos/lib/timer/TransformAlarmC.nc"
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

# 69 "/opt/tinyos-2.1.0/tos/lib/timer/TransformCounterC.nc"
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

# 51 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerP.nc"
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

# 41 "/opt/tinyos-2.1.0/tos/chips/msp430X/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(56)))  void sig_TIMERB1_VECTOR(void )
#line 41
{



  Msp430TimerCommonP__VectorTimerB1__fired();
}

# 52 "/opt/tinyos-2.1.0/tos/system/RealMainP.nc"
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

# 123 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
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

# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0x40441ae8){
#line 64
  switch (arg_0x40441ae8) {
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
    case RF231driverP__taskStartDone:
#line 64
      RF231driverP__taskStartDone__runTask();
#line 64
      break;
#line 64
    case RF231driverP__taskStopDone:
#line 64
      RF231driverP__taskStopDone__runTask();
#line 64
      break;
#line 64
    case RF231driverP__taskSendDone:
#line 64
      RF231driverP__taskSendDone__runTask();
#line 64
      break;
#line 64
    case RF231SpiP__grant:
#line 64
      RF231SpiP__grant__runTask();
#line 64
      break;
#line 64
    case /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__signalDone_task:
#line 64
      /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__signalDone_task__runTask();
#line 64
      break;
#line 64
    case /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask:
#line 64
      /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask();
#line 64
      break;
#line 64
    default:
#line 64
      SchedulerBasicP__TaskBasic__default__runTask(arg_0x40441ae8);
#line 64
      break;
#line 64
    }
#line 64
}
#line 64
# 226 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA0P.nc"
static void HplMsp430UsciA0P__Usci__setModeSpi(msp430_spi_union_config_t *config)
#line 226
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 227
    {
      HplMsp430UsciA0P__Usci__resetUsci(TRUE);
      HplMsp430UsciA0P__configSpi(config);
      HplMsp430UsciA0P__Usci__enableSpi();
      HplMsp430UsciA0P__Usci__resetUsci(FALSE);
      HplMsp430UsciA0P__Usci__clrIntr();
      HplMsp430UsciA0P__Usci__disableIntr();
    }
#line 234
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

# 287 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RF231driverP.nc"
static void RF231driverP__ChipSpiResource__loadPacketDone(error_t error)
#line 287
{
  /* atomic removed: atomic calls only */
  RF231driverP__preparingPacket = TRUE;
  /* atomic removed: atomic calls only */
#line 290
  {
    if (RF231driverP__SpiResource__immediateRequest()) {
        P4OUT &= ~0x0001;
        RF231driverP__ChipSpiResource__changeState(0);
        RF231driverP__SpiResource__release();
        P4OUT |= 0x0001;
      }
    else {
        RF231driverP__internalSpiState = RF231driverP__CHANGINGSTATE;
        RF231driverP__globalNewReg = 0;
        RF231driverP__SpiResource__request();
      }
  }
  /* atomic removed: atomic calls only */
#line 317
  {
    if (RF231driverP__SpiResource__immediateRequest()) {
        P4OUT &= ~0x0001;
        RF231driverP__ChipSpiResource__setChannel(RF231driverP__frequencyChannel);
        RF231driverP__SpiResource__release();
        P4OUT |= 0x0001;
      }
    else {
        RF231driverP__internalSpiState = RF231driverP__CHANGINGFREQ;
        RF231driverP__globalFrequency = RF231driverP__frequencyChannel;
        RF231driverP__SpiResource__request();
      }
  }
}

# 92 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/spi/RF231SpiP.nc"
static error_t RF231SpiP__Resource__immediateRequest(uint8_t id)
#line 92
{
  error_t error;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 95
    {
      if (RF231SpiP__WorkingState__requestState(RF231SpiP__S_BUSY) != SUCCESS) {
          {
            unsigned char __nesc_temp = 
#line 97
            EBUSY;

            {
#line 97
              __nesc_atomic_end(__nesc_atomic); 
#line 97
              return __nesc_temp;
            }
          }
        }
      if (RF231SpiP__SpiResource__isOwner()) {
          RF231SpiP__m_holder = id;
          error = SUCCESS;
        }
      else {
#line 105
        if ((error = RF231SpiP__SpiResource__immediateRequest()) == SUCCESS) {
            RF231SpiP__m_holder = id;
          }
        else {
            RF231SpiP__WorkingState__toIdle();
          }
        }
    }
#line 112
    __nesc_atomic_end(__nesc_atomic); }
#line 112
  return error;
}

# 96 "/opt/tinyos-2.1.0/tos/system/StateImplP.nc"
static error_t StateImplP__State__requestState(uint8_t id, uint8_t reqState)
#line 96
{
  error_t returnVal = 0x0080;

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

# 173 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
static uint8_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id)
#line 173
{
  /* atomic removed: atomic calls only */
#line 174
  {
    if (/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId == id && /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) {
        unsigned char __nesc_temp = 
#line 175
        TRUE;

#line 175
        return __nesc_temp;
      }
    else 
#line 176
      {
        unsigned char __nesc_temp = 
#line 176
        FALSE;

#line 176
        return __nesc_temp;
      }
  }
}

#line 129
static error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void )
#line 129
{
  /* atomic removed: atomic calls only */
#line 130
  {
    if (/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId == /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id) {
        if (/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING) {
            /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
            {
              unsigned char __nesc_temp = 
#line 134
              SUCCESS;

#line 134
              return __nesc_temp;
            }
          }
        else {
#line 136
          if (/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING) {
              /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
              /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
              {
                unsigned char __nesc_temp = 
#line 139
                SUCCESS;

#line 139
                return __nesc_temp;
              }
            }
          }
      }
  }
#line 143
  return 0x0080;
}

# 307 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/spi/RF231SpiP.nc"
static void RF231SpiP__ChipSpiResource__changeState(uint8_t newState)
#line 307
{
  /* atomic removed: atomic calls only */
#line 308
  {
    if (RF231SpiP__WorkingState__isIdle()) {
        return;
      }
  }
  RF231SpiP__SpiByte__write(0xC0 | 0x02);
  RF231SpiP__SpiByte__write(newState);
}

# 61 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
static uint8_t /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__SpiByte__write(uint8_t tx)
#line 61
{
  uint8_t byte;


  /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__tx(tx);
  while (!/*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__isRxIntrPending()) ;
  /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__clrRxIntr();
  byte = /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__rx();

  return byte;
}

# 318 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciA0P.nc"
static uint8_t HplMsp430UsciA0P__Usci__rx(void )
#line 318
{
  uint8_t value;

#line 320
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 320
    value = UCA0RXBUF;
#line 320
    __nesc_atomic_end(__nesc_atomic); }

  return value;
}

# 115 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/spi/RF231SpiP.nc"
static error_t RF231SpiP__Resource__release(uint8_t id)
#line 115
{
  uint8_t i;

#line 117
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 117
    {
      if (RF231SpiP__m_holder != id) {
          {
            unsigned char __nesc_temp = 
#line 119
            0x0080;

            {
#line 119
              __nesc_atomic_end(__nesc_atomic); 
#line 119
              return __nesc_temp;
            }
          }
        }
#line 122
      RF231SpiP__m_holder = RF231SpiP__NO_HOLDER;
      if (!RF231SpiP__m_requests) {
          RF231SpiP__WorkingState__toIdle();
          RF231SpiP__attemptRelease();
        }
      else {
          for (i = RF231SpiP__m_holder + 1; ; i++) {
              i %= RF231SpiP__RESOURCE_COUNT;

              if (RF231SpiP__m_requests & (1 << i)) {
                  RF231SpiP__m_holder = i;
                  RF231SpiP__m_requests &= ~(1 << i);
                  RF231SpiP__grant__postTask();
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
            }
        }
    }
#line 141
    __nesc_atomic_end(__nesc_atomic); }
#line 141
  return SUCCESS;
}

#line 73
static error_t RF231SpiP__Resource__request(uint8_t id)
#line 73
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 75
    {
      if (RF231SpiP__WorkingState__requestState(RF231SpiP__S_BUSY) == SUCCESS) {
          RF231SpiP__m_holder = id;
          if (RF231SpiP__SpiResource__isOwner()) {
              RF231SpiP__grant__postTask();
            }
          else {
              RF231SpiP__SpiResource__request();
            }
        }
      else {
          RF231SpiP__m_requests |= 1 << id;
        }
    }
#line 88
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

#line 327
static void RF231SpiP__ChipSpiResource__setChannel(uint8_t newChannel)
#line 327
{
  /* atomic removed: atomic calls only */
#line 328
  {
    if (RF231SpiP__WorkingState__isIdle()) {
        return;
      }
  }
  RF231SpiP__SpiByte__write(0xC0 | 0x08);
  RF231SpiP__SpiByte__write(0x20 | newChannel);

  RF231SpiP__ChipSpiResource__doneSettingChannel();
}

# 48 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P22*/HplMsp430GeneralIOP__10__IO__toggle(void )
#line 48
{
#line 48
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 48
    * (volatile uint8_t * )41U ^= 0x01 << 2;
#line 48
    __nesc_atomic_end(__nesc_atomic); }
}

# 340 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/spi/RF231SpiP.nc"
static void RF231SpiP__ChipSpiResource__setRegister(uint8_t regAddr, uint8_t newValue)
#line 340
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 341
    {
      if (RF231SpiP__WorkingState__isIdle()) {
          {
#line 343
            __nesc_atomic_end(__nesc_atomic); 
#line 343
            return;
          }
        }
    }
#line 346
    __nesc_atomic_end(__nesc_atomic); }
#line 346
  RF231SpiP__SpiByte__write(0xC0 | regAddr);
  RF231SpiP__SpiByte__write(newValue);
}

#line 317
static uint8_t RF231SpiP__ChipSpiResource__readRegister(uint8_t registerAddress)
#line 317
{
  /* atomic removed: atomic calls only */
#line 318
  {
    if (RF231SpiP__WorkingState__isIdle()) {
        {
          unsigned char __nesc_temp = 
#line 320
          0;

#line 320
          return __nesc_temp;
        }
      }
  }
#line 323
  RF231SpiP__SpiByte__write(0x80 | registerAddress);
  return RF231SpiP__SpiByte__write(0x00);
}

# 524 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RF231driverP.nc"
static void RF231driverP__ISRserviced(void )
#line 524
{
  uint8_t tempstatusRg;

#line 526
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 526
    tempstatusRg = RF231driverP__statusRg;
#line 526
    __nesc_atomic_end(__nesc_atomic); }










  if (tempstatusRg == 0x01) {
#line 560
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 560
        {
          if (RF231driverP__SpiResource__immediateRequest()) {
              P4OUT &= ~0x0001;
              RF231driverP__ChipSpiResource__changeState(0);
              RF231driverP__SpiResource__release();
              P4OUT |= 0x0001;
            }
          else {
              RF231driverP__internalSpiState = RF231driverP__CHANGINGSTATE;
              RF231driverP__globalNewReg = 0;
              RF231driverP__SpiResource__request();
            }
        }
#line 572
        __nesc_atomic_end(__nesc_atomic); }
#line 572
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 572
        RF231driverP__radioState = 9;
#line 572
        __nesc_atomic_end(__nesc_atomic); }
      RF231driverP__taskStartDone__postTask();
    }
  else {
    if (tempstatusRg == 0x04) {
        RF231driverP__receivedSomething();
      }
    else {
      if (tempstatusRg == 0x8F || tempstatusRg == 0x08) {

          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 582
            {
              if (RF231driverP__driverState == RF231driverP__TRANSMITTING) {
#line 583
                RF231driverP__taskSendDone__postTask();
                }
              else {
#line 584
                if (RF231driverP__driverState == RF231driverP__RECEIVING) {
#line 584
                  RF231driverP__unloadPacket();
                  }
                }
            }
#line 587
            __nesc_atomic_end(__nesc_atomic); }
        }
      }
    }
#line 586
  RF231driverP__InterruptIRQRFP__enableRisingEdge();
}

# 41 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/Msp430InterruptC.nc"
static error_t /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__enable(bool rising)
#line 41
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 42
    {
      /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__Interrupt__disable();
      /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__HplInterrupt__edge(rising);
      /*HplRF231InterruptsC.InterruptIRQRFPC*/Msp430InterruptC__0__HplInterrupt__enable();
    }
#line 46
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 369 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/spi/RF231SpiP.nc"
static void RF231SpiP__ChipSpiResource__readPacket(uint8_t *data)
#line 369
{
  uint8_t len = 0;

#line 371
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 371
    {
      if (RF231SpiP__WorkingState__isIdle()) {
          {
#line 373
            __nesc_atomic_end(__nesc_atomic); 
#line 373
            return;
          }
        }
    }
#line 376
    __nesc_atomic_end(__nesc_atomic); }
#line 376
  RF231SpiP__SpiByte__write(0x60);
  len = RF231SpiP__SpiByte__write(0);
  data = malloc(len);
  * data++ = len;
  for (; len; len--) {
      * data++ = RF231SpiP__SpiByte__write(0);
    }
}

#line 350
static void RF231SpiP__ChipSpiResource__loadPacket(uint8_t *data, uint8_t len)
#line 350
{
  uint8_t status = 0;

#line 352
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 352
    {
      if (RF231SpiP__WorkingState__isIdle()) {
          {
#line 354
            __nesc_atomic_end(__nesc_atomic); 
#line 354
            return;
          }
        }
    }
#line 357
    __nesc_atomic_end(__nesc_atomic); }


  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 360
    RF231SpiP__packetOperation = RF231SpiP__WAS_WRITING;
#line 360
    __nesc_atomic_end(__nesc_atomic); }

  status = RF231SpiP__SpiByte__write(0x60);
  RF231SpiP__SpiByte__write(len);
  RF231SpiP__SpiPacket__send(data, (void *)0, len);
}

# 83 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/Msp430SpiNoDmaAP.nc"
static void /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__continueOp(void )
#line 83
{

  uint8_t end;
  uint8_t tmp;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 88
    {
      /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__tx(/*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_tx_buf ? /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_tx_buf[/*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_pos] : 0);

      end = /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_pos + /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__SPI_ATOMIC_SIZE;
      if (end > /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_len) {
        end = /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_len;
        }
      while (++/*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_pos < end) {
          while (!/*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__isRxIntrPending()) ;
          tmp = /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__rx();
          if (/*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_rx_buf) {
            /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_rx_buf[/*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_pos - 1] = tmp;
            }
#line 100
          /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__Usci__tx(/*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_tx_buf ? /*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_tx_buf[/*Msp430SpiNoDmaA0P.SpiP*/Msp430SpiNoDmaAP__0__m_pos] : 0);
        }
    }
#line 102
    __nesc_atomic_end(__nesc_atomic); }
}

# 48 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__toggle(void )
#line 48
{
#line 48
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 48
    * (volatile uint8_t * )41U ^= 0x01 << 1;
#line 48
    __nesc_atomic_end(__nesc_atomic); }
}

# 72 "/opt/tinyos-2.1.0/tos/platforms/gina20/Leds.nc"
static void fabMACP__Leds__led1Toggle(void ){
#line 72
  LedsP__Leds__led1Toggle();
#line 72
}
#line 72
# 48 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__toggle(void )
#line 48
{
#line 48
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 48
    * (volatile uint8_t * )41U ^= 0x01 << 3;
#line 48
    __nesc_atomic_end(__nesc_atomic); }
}

# 62 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
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

# 48 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__toggle(void )
#line 48
{
#line 48
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 48
    * (volatile uint8_t * )41U ^= 0x01 << 0;
#line 48
    __nesc_atomic_end(__nesc_atomic); }
}

# 482 "/opt/tinyos-2.1.0/apps/OpenWSN/01-PHY/rf231/01-PHY/RF231driverP.nc"
static void RF231driverP__shutdown(void )
#line 482
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 484
    P4OUT |= 0x0080;
#line 484
    __nesc_atomic_end(__nesc_atomic); }
  RF231driverP__radioWait__wait(110);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 486
    RF231driverP__radioState = 15;
#line 486
    __nesc_atomic_end(__nesc_atomic); }
  RF231driverP__taskStopDone__postTask();
}

# 52 "/opt/tinyos-2.1.0/tos/lib/timer/BusyWaitCounterC.nc"
static void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__wait(/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {


      /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type t0 = /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get();

      if (dt > /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__HALF_MAX_SIZE_TYPE) 
        {
          dt -= /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__HALF_MAX_SIZE_TYPE;
          while (/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get() - t0 <= dt) ;
          t0 += dt;
          dt = /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__HALF_MAX_SIZE_TYPE;
        }

      while (/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get() - t0 <= dt) ;
    }
#line 69
    __nesc_atomic_end(__nesc_atomic); }
}

# 136 "/opt/tinyos-2.1.0/tos/lib/timer/TransformAlarmC.nc"
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

# 46 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciAB0RawInterruptsP.nc"
__attribute((wakeup)) __attribute((interrupt(46)))  void sig_USCIAB0RX_VECTOR(void )
#line 46
{
  uint8_t temp;

#line 48
  if (IFG2 & (1 << 0)) {
      temp = UCA0RXBUF;
      HplMsp430UsciAB0RawInterruptsP__UsciA__rxDone(temp);
    }
  if (IFG2 & (1 << 2)) {
      temp = UCB0RXBUF;
      HplMsp430UsciAB0RawInterruptsP__UsciB__rxDone(temp);
    }
}

# 149 "/opt/tinyos-2.1.0/tos/system/ArbiterP.nc"
static bool /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void )
#line 149
{
  /* atomic removed: atomic calls only */
#line 150
  {
    if (/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) 
      {
        unsigned char __nesc_temp = 
#line 152
        FALSE;

#line 152
        return __nesc_temp;
      }
  }
#line 154
  return TRUE;
}






static uint8_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void )
#line 162
{
  /* atomic removed: atomic calls only */
#line 163
  {
    if (/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state != /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 165
        /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;

#line 165
        return __nesc_temp;
      }
#line 166
    {
      unsigned char __nesc_temp = 
#line 166
      /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId;

#line 166
      return __nesc_temp;
    }
  }
}

# 58 "/opt/tinyos-2.1.0/tos/chips/msp430X/usci/HplMsp430UsciAB0RawInterruptsP.nc"
__attribute((wakeup)) __attribute((interrupt(44)))  void sig_USCIAB0TX_VECTOR(void )
#line 58
{
  if ((IFG2 & (1 << 1)) | (IFG2 & (1 << 0))) {
      HplMsp430UsciAB0RawInterruptsP__UsciA__txDone();
    }
  if ((IFG2 & (1 << 3)) | (IFG2 & (1 << 2))) {
      HplMsp430UsciAB0RawInterruptsP__UsciB__txDone();
    }
}

# 84 "/opt/tinyos-2.1.0/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
__attribute((wakeup)) __attribute((interrupt(36)))  void sig_PORT1_VECTOR(void )
{
  volatile int n = P1IFG & P1IE;

#line 87
  if (n & (1 << 0)) {
      HplMsp430InterruptP__Port10__fired();
      return;
    }
  if (n & (1 << 1)) {
      HplMsp430InterruptP__Port11__fired();
      return;
    }
  if (n & (1 << 2)) {
      HplMsp430InterruptP__Port12__fired();
      return;
    }
  if (n & (1 << 3)) {
      HplMsp430InterruptP__Port13__fired();
      return;
    }
  if (n & (1 << 4)) {
      HplMsp430InterruptP__Port14__fired();
      return;
    }
  if (n & (1 << 5)) {
      HplMsp430InterruptP__Port15__fired();
      return;
    }
  if (n & (1 << 6)) {
      HplMsp430InterruptP__Port16__fired();
      return;
    }
  if (n & (1 << 7)) {
      HplMsp430InterruptP__Port17__fired();
      return;
    }
}

#line 213
__attribute((wakeup)) __attribute((interrupt(38)))  void sig_PORT2_VECTOR(void )
{
  volatile int n = P2IFG & P2IE;

#line 216
  if (n & (1 << 0)) {
      HplMsp430InterruptP__Port20__fired();
      return;
    }
  if (n & (1 << 1)) {
      HplMsp430InterruptP__Port21__fired();
      return;
    }
  if (n & (1 << 2)) {
      HplMsp430InterruptP__Port22__fired();
      return;
    }
  if (n & (1 << 3)) {
      HplMsp430InterruptP__Port23__fired();
      return;
    }
  if (n & (1 << 4)) {
      HplMsp430InterruptP__Port24__fired();
      return;
    }
  if (n & (1 << 5)) {
      HplMsp430InterruptP__Port25__fired();
      return;
    }
  if (n & (1 << 6)) {
      HplMsp430InterruptP__Port26__fired();
      return;
    }
  if (n & (1 << 7)) {
      HplMsp430InterruptP__Port27__fired();
      return;
    }
}

