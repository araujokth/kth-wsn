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
#line 264
static __inline uint16_t __nesc_ntoh_uint16(const void * source)  ;




static __inline uint16_t __nesc_hton_uint16(void * target, uint16_t value)  ;
#line 281
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
# 40 "/usr/msp430/include/string.h" 3
extern void *memcpy(void *arg_0x4029c200, const void *arg_0x4029c398, size_t arg_0x4029c530);

extern void *memset(void *arg_0x4029f220, int arg_0x4029f378, size_t arg_0x4029f510);
#line 63
extern void *memset(void *arg_0x402ad118, int arg_0x402ad270, size_t arg_0x402ad408);
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

  void (*__cleanup)(struct _reent *arg_0x402c9510);


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


  void (**_sig_func)(int arg_0x402cdb88);




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
#line 144
extern float powf(float arg_0x402f0010, float arg_0x402f0168);
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
# 23 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/tos.h"
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
# 40 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/types/TinyError.h"
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
# 83 "/usr/msp430/include/msp430x16x.h" 3
volatile unsigned char ME1 __asm ("0x0004");





volatile unsigned char ME2 __asm ("0x0005");
# 158 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/msp430hardware.h"
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
# 8 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/platforms/telosb/hardware.h"
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
# 29 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4260 {
#line 29
  int notUsed;
} 
#line 29
TMilli;
typedef struct __nesc_unnamed4261 {
#line 30
  int notUsed;
} 
#line 30
T32khz;
typedef struct __nesc_unnamed4262 {
#line 31
  int notUsed;
} 
#line 31
TMicro;
# 1 "gtsmanagementexample.h"
enum __nesc_unnamed4263 {
  COORDINATOR = 0x00, 
  ROUTER = 0x01, 
  END_DEVICE = 0x02
};
# 29 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/phy/phy_const.h"
#line 23
typedef struct __nesc_unnamed4264 {

  uint8_t phyCurrentChannel;
  uint8_t phyChannelsSupported;
  uint8_t phyTransmitPower;
  uint8_t phyCcaMode;
} phyPIB;
# 11 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/phy/phy_enumerations.h"
enum __nesc_unnamed4265 {
  PHY_BUSY = 0x00, 
  PHY_BUSY_RX = 0x01, 
  PHY_BUSY_TX = 0x02, 
  PHY_FORCE_TRX_OFF = 0x03, 
  PHY_IDLE = 0x04, 
  PHY_INVALID_PARAMETER = 0x05, 
  PHY_RX_ON = 0x06, 
  PHY_SUCCESS = 0x07, 
  PHY_TRX_OFF = 0x08, 
  PHY_TX_ON = 0x09, 
  PHY_UNSUPPORTED_ATTRIBUTE = 0x0a
};


enum __nesc_unnamed4266 {
  PHYCURRENTCHANNEL = 0x00, 
  PHYCHANNELSSUPPORTED = 0X01, 
  PHYTRANSMITPOWER = 0X02, 
  PHYCCAMODE = 0X03
};
# 105 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/mac_const.h"
#line 75
typedef struct __nesc_unnamed4267 {


  uint8_t macAckWaitDuration;
  bool macAssociationPermit;
  bool macAutoRequest;
  bool macBattLifeExt;
  uint8_t macBattLifeExtPeriods;

  uint8_t macBeaconPayload[127 - 75];

  uint8_t macBeaconPayloadLenght;
  uint8_t macBeaconOrder;

  uint32_t macBeaconTxTime;
  uint8_t macBSN;
  uint32_t macCoordExtendedAddress0;
  uint32_t macCoordExtendedAddress1;
  uint16_t macCoordShortAddress;
  uint8_t macDSN;
  bool macGTSPermit;
  uint8_t macMaxCSMABackoffs;
  uint8_t macMinBE;
  uint16_t macPANId;
  bool macPromiscuousMode;
  bool macRxOnWhenIdle;
  uint32_t macShortAddress;
  uint8_t macSuperframeOrder;
  uint32_t macTransactionPersistenceTime;
} 
macPIB;
#line 118
#line 108
typedef struct __nesc_unnamed4268 {

  uint32_t ACLExtendedAddress[2];
  uint16_t ACLShortAddress;
  uint16_t ACLPANId;
  uint8_t ACLSecurityMaterialLength;

  uint8_t ACLSecurityMaterial;
  uint8_t ACLSecuritySuite;
} 
ACLDescriptor;
#line 133
#line 121
typedef struct __nesc_unnamed4269 {


  ACLDescriptor macACLEntryDescriptorSet;
  uint8_t macACLEntryDescriptorSetSize;
  bool macDefaultSecurity;
  uint8_t macDefaultSecurityMaterialLength;

  uint8_t macDefaultSecurityMaterial;
  uint8_t macDefaultSecuritySuite;
  uint8_t macSecurityMode;
} 
macPIBsec;
#line 153
#line 136
typedef struct __nesc_unnamed4270 {


  uint8_t CoordAddrMode;
  uint16_t CoordPANId;
  uint32_t CoordAddress0;
  uint32_t CoordAddress1;
  uint8_t LogicalChannel;

  uint16_t SuperframeSpec;
  bool GTSPermit;
  uint8_t LinkQuality;
  uint32_t TimeStamp;
  bool SecurityUse;
  uint8_t ACLEntry;
  bool SecurityFailure;
} 
PANDescriptor;
#line 165
#line 156
typedef struct __nesc_unnamed4271 {

  uint8_t gts_id;
  uint8_t starting_slot;
  uint8_t length;
  uint8_t direction;
  uint16_t DevAddressType;
  uint8_t expiration;
} 
GTSinfoEntryType;










#line 168
typedef struct __nesc_unnamed4272 {

  uint8_t gts_id;
  uint8_t starting_slot;
  uint8_t length;
  uint16_t DevAddressType;
  uint8_t persistencetime;
} 
GTSinfoEntryType_null;









#line 178
typedef struct __nesc_unnamed4273 {

  uint8_t handler;
  uint16_t transaction_persistent_time;


  uint8_t frame[127];
} 
indirect_transmission_element;








#line 188
typedef struct gts_slot_element {

  uint8_t element_count;
  uint8_t element_in;
  uint8_t element_out;
  uint8_t gts_send_frame_index[3];
} 
gts_slot_element;







#line 198
typedef struct time_stamp32 {


  uint32_t time_stamp;
} 
time_stamp32;






#line 205
typedef struct time_stamp16 {


  uint16_t time_stamp;
} 
time_stamp16;
#line 222
#line 213
typedef struct SCAN_PANDescriptor {


  uint16_t CoordPANId;
  uint16_t CoordAddress;
  uint8_t LogicalChannel;

  uint16_t SuperframeSpec;
  uint8_t lqi;
} SCAN_PANDescriptor;
# 12 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/mac_enumerations.h"
enum __nesc_unnamed4274 {
  MAC_SUCCESS = 0x00, 
  MAC_BEACON_LOSS = 0xE0, 
  MAC_CHANNEL_ACCESS_FAILURE = 0xE1, 
  MAC_DENIED = 0xE2, 

  MAC_DISABLE_TRX_FAILURE = 0xE3, 
  MAC_FAILED_SECURITY_CHECK = 0xE4, 
  MAC_FRAME_TOO_LONG = 0xE5, 
  MAC_INVALID_GTS = 0xE6, 
  MAC_INVALID_HANDLE = 0xE7, 
  MAC_INVALID_PARAMETER = 0xE8, 
  MAC_NO_ACK = 0xE9, 
  MAC_NO_BEACON = 0xEA, 
  MAC_NO_DATA = 0xEB, 
  MAC_NO_SHORT_ADDRESS = 0xEC, 
  MAC_OUT_OF_CAP = 0xED, 
  MAC_PAN_ID_CONFLICT = 0xEE, 
  MAC_REALIGNMENT = 0xEF, 
  MAC_TRANSACTION_EXPIRED = 0xF0, 
  MAC_TRANSACTION_OVERFLOW = 0xF1, 
  MAC_TX_ACTIVE = 0xF2, 
  MAC_UNAVAILABLE_KEY = 0xF3, 
  MAC_UNSUPPORTED_ATTRIBUTE = 0xF4
};




enum __nesc_unnamed4275 {
  MAC_PAN_COORD_LEAVE = 0x01, 
  MAC_PAN_DEVICE_LEAVE = 0x02
};





enum __nesc_unnamed4276 {

  CMD_ASSOCIATION_REQUEST = 0x01, 
  CMD_ASSOCIATION_RESPONSE = 0x02, 
  CMD_DISASSOCIATION_NOTIFICATION = 0x03, 
  CMD_DATA_REQUEST = 0x04, 
  CMD_PANID_CONFLICT = 0x05, 
  CMD_ORPHAN_NOTIFICATION = 0x06, 
  CMD_BEACON_REQUEST = 0x07, 
  CMD_COORDINATOR_REALIGNMENT = 0x08, 
  CMD_GTS_REQUEST = 0x09
};



enum __nesc_unnamed4277 {

  CMD_RESP_ASSOCIATION_SUCCESSFUL = 0x00, 
  CMD_RESP_PAN_CAPACITY = 0x01, 
  CMD_RESP_ACCESS_DENIED = 0x02
};



enum __nesc_unnamed4278 {

  MACACKWAITDURATION = 0x40, 
  MACASSOCIATIONPERMIT = 0x41, 
  MACAUTOREQUEST = 0x42, 
  MACBATTLIFEEXT = 0x43, 
  MACBATTLIFEEXTPERIODS = 0x44, 
  MACBEACONPAYLOAD = 0x45, 
  MACMAXBEACONPAYLOADLENGTH = 0x46, 
  MACBEACONORDER = 0x47, 
  MACBEACONTXTIME = 0x48, 
  MACBSN = 0x49, 
  MACCOORDEXTENDEDADDRESS = 0x4a, 
  MACCOORDSHORTADDRESS = 0x4b, 
  MACDSN = 0x4c, 
  MACGTSPERMIT = 0x4d, 
  MACMAXCSMABACKOFFS = 0x4e, 
  MACMINBE = 0x4f, 
  MACPANID = 0x50, 
  MACPROMISCUOUSMODE = 0x51, 
  MACRXONWHENIDLE = 0x52, 
  MACSHORTADDRESS = 0x53, 
  MACSUPERFRAMEORDER = 0x54, 
  MACTRANSACTIONPERSISTENCETIME = 0x55
};



enum __nesc_unnamed4279 {
  GTS_TX_ONLY = 0x00, 
  GTS_RX_ONLY = 0x01
};


enum __nesc_unnamed4280 {
  ED_SCAN = 0x00, 
  ACTIVE_SCAN = 0x01, 
  PASSIVE_SCAN = 0x02, 
  ORPHAN_SCAN = 0x03
};
# 35 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/includes/mac_func.h"
static uint16_t set_frame_control(uint8_t frame_type, uint8_t security, uint8_t frame_pending, uint8_t ack_request, uint8_t intra_pan, uint8_t dest_addr_mode, uint8_t source_addr_mode);
#line 49
static uint8_t get_fc2_dest_addr(uint8_t frame_control);
#line 68
static uint8_t get_fc2_source_addr(uint8_t frame_control);
#line 96
static inline bool get_fc1_frame_pending(uint8_t frame_control);









static inline bool get_fc1_ack_request(uint8_t frame_control);









static inline bool get_fc1_intra_pan(uint8_t frame_control);
#line 132
static uint16_t set_superframe_specification(uint8_t beacon_order, uint8_t superframe_order, uint8_t final_cap_slot, uint8_t battery_life_extension, uint8_t pan_coordinator, uint8_t association_permit);










static inline uint8_t get_beacon_order(uint16_t superframe);




static inline uint8_t get_superframe_order(uint16_t superframe);
#line 191
static inline uint8_t set_txoptions(uint8_t ack, uint8_t gts, uint8_t indirect_transmission, uint8_t security);




static inline bool get_txoptions_ack(uint8_t txoptions);









static inline bool get_txoptions_gts(uint8_t txoptions);









static inline bool get_txoptions_indirect_transmission(uint8_t txoptions);
#line 255
static inline uint8_t set_pending_address_specification(uint8_t number_short, uint8_t number_extended);




static inline uint8_t get_number_short(uint8_t pending_specification);




static inline uint8_t get_number_extended(uint8_t pending_specification);








static inline uint8_t set_gts_specification(uint8_t gts_descriptor_count, uint8_t gts_permit);
#line 292
static inline uint8_t set_gts_descriptor(uint8_t GTS_starting_slot, uint8_t GTS_length);





static inline uint8_t get_gts_descriptor_len(uint8_t gts_des_part);




static inline uint8_t get_gts_descriptor_ss(uint8_t gts_des_part);









static inline uint8_t set_gts_characteristics(uint8_t gts_length, uint8_t gts_direction, uint8_t characteristic_type);





static inline uint8_t get_gts_length(uint8_t gts_characteristics);




static inline bool get_gts_direction(uint8_t gts_characteristics);







static inline uint8_t get_characteristic_type(uint8_t gts_characteristics);
# 28 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.h"
enum __nesc_unnamed4281 {
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
typedef struct __nesc_unnamed4282 {

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
typedef struct __nesc_unnamed4283 {

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
typedef struct __nesc_unnamed4284 {

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
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/CC2420.h"
typedef uint8_t cc2420_status_t;
#line 93
#line 87
typedef nx_struct security_header_t {
  unsigned char __nesc_filler0[1];


  nx_uint32_t frameCounter;
  nx_uint8_t keyID[1];
} __attribute__((packed)) security_header_t;
#line 113
#line 95
typedef nx_struct cc2420_header_t {
  nxle_uint8_t length;
  nxle_uint16_t fcf;
  nxle_uint8_t dsn;
  nxle_uint16_t destpan;
  nxle_uint16_t dest;
  nxle_uint16_t src;







  nxle_uint8_t network;


  nxle_uint8_t type;
} __attribute__((packed)) cc2420_header_t;





#line 118
typedef nx_struct cc2420_footer_t {
} __attribute__((packed)) cc2420_footer_t;
#line 143
#line 128
typedef nx_struct cc2420_metadata_t {
  nx_uint8_t rssi;
  nx_uint8_t lqi;
  nx_uint8_t tx_power;
  nx_bool crc;
  nx_bool ack;
  nx_bool timesync;
  nx_uint32_t timestamp;
  nx_uint16_t rxInterval;
} __attribute__((packed)) 





cc2420_metadata_t;





#line 146
typedef nx_struct cc2420_packet_t {
  cc2420_header_t packet;
  nx_uint8_t data[];
} __attribute__((packed)) cc2420_packet_t;
#line 179
enum __nesc_unnamed4285 {

  MAC_HEADER_SIZE = sizeof(cc2420_header_t ) - 1, 

  MAC_FOOTER_SIZE = sizeof(uint16_t ), 

  MAC_PACKET_SIZE = MAC_HEADER_SIZE + 28 + MAC_FOOTER_SIZE, 

  CC2420_SIZE = MAC_HEADER_SIZE + MAC_FOOTER_SIZE, 

  AM_OVERHEAD = 2
};

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

enum cc2420_security_enums {
  CC2420_NO_SEC = 0, 
  CC2420_CBC_MAC = 1, 
  CC2420_CTR = 2, 
  CC2420_CCM = 3, 
  NO_SEC = 0, 
  CBC_MAC_4 = 1, 
  CBC_MAC_8 = 2, 
  CBC_MAC_16 = 3, 
  CTR = 4, 
  CCM_4 = 5, 
  CCM_8 = 6, 
  CCM_16 = 7
};


enum __nesc_unnamed4286 {

  CC2420_INVALID_TIMESTAMP = 0x80000000L
};
# 6 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/types/AM.h"
typedef nx_uint8_t nx_am_id_t;
typedef nx_uint8_t nx_am_group_t;
typedef nx_uint16_t nx_am_addr_t;

typedef uint8_t am_id_t;
typedef uint8_t am_group_t;
typedef uint16_t am_addr_t;

enum __nesc_unnamed4287 {
  AM_BROADCAST_ADDR = 0xffff
};









enum __nesc_unnamed4288 {
  TOS_AM_GROUP = 0x22, 
  TOS_AM_ADDRESS = 1
};
# 72 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/Serial.h"
typedef uint8_t uart_id_t;



enum __nesc_unnamed4289 {
  HDLC_FLAG_BYTE = 0x7e, 
  HDLC_CTLESC_BYTE = 0x7d
};



enum __nesc_unnamed4290 {
  TOS_SERIAL_ACTIVE_MESSAGE_ID = 0, 
  TOS_SERIAL_CC1000_ID = 1, 
  TOS_SERIAL_802_15_4_ID = 2, 
  TOS_SERIAL_UNKNOWN_ID = 255
};


enum __nesc_unnamed4291 {
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
# 48 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/platforms/telosa/platform_message.h"
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
# 19 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/types/message.h"
#line 14
typedef nx_struct message_t {
  nx_uint8_t header[sizeof(message_header_t )];
  nx_uint8_t data[28];
  nx_uint8_t footer[sizeof(message_footer_t )];
  nx_uint8_t metadata[sizeof(message_metadata_t )];
} __attribute__((packed)) message_t;
# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/printf/printf.h"
int printfflush();






#line 64
typedef nx_struct printf_msg {
  nx_uint8_t buffer[28];
} __attribute__((packed)) printf_msg_t;

enum __nesc_unnamed4292 {
  AM_PRINTF_MSG = 100
};
# 32 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/types/Leds.h"
enum __nesc_unnamed4293 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 80 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/crc.h"
static uint16_t crcByte(uint16_t crc, uint8_t b);
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/msp430usart.h"
#line 48
typedef enum __nesc_unnamed4294 {

  USART_NONE = 0, 
  USART_UART = 1, 
  USART_UART_TX = 2, 
  USART_UART_RX = 3, 
  USART_SPI = 4, 
  USART_I2C = 5
} msp430_usartmode_t;










#line 58
typedef struct __nesc_unnamed4295 {
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
typedef struct __nesc_unnamed4296 {
  unsigned int txept : 1;
  unsigned int stc : 1;
  unsigned int txwake : 1;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
} __attribute((packed))  msp430_utctl_t;










#line 79
typedef struct __nesc_unnamed4297 {
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
typedef struct __nesc_unnamed4298 {
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
typedef struct __nesc_unnamed4299 {
  uint16_t ubr;
  uint8_t uctl;
  uint8_t utctl;
} msp430_spi_registers_t;




#line 124
typedef union __nesc_unnamed4300 {
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
typedef enum __nesc_unnamed4301 {

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
typedef struct __nesc_unnamed4302 {
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
typedef struct __nesc_unnamed4303 {
  uint16_t ubr;
  uint8_t umctl;
  uint8_t uctl;
  uint8_t utctl;
  uint8_t urctl;
  uint8_t ume;
} msp430_uart_registers_t;




#line 211
typedef union __nesc_unnamed4304 {
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
typedef struct __nesc_unnamed4305 {
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
typedef struct __nesc_unnamed4306 {
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
typedef struct __nesc_unnamed4307 {
  uint8_t uctl;
  uint8_t i2ctctl;
  uint8_t i2cpsc;
  uint8_t i2csclh;
  uint8_t i2cscll;
  uint16_t i2coa;
} msp430_i2c_registers_t;




#line 287
typedef union __nesc_unnamed4308 {
  msp430_i2c_config_t i2cConfig;
  msp430_i2c_registers_t i2cRegisters;
} msp430_i2c_union_config_t;
#line 309
typedef uint8_t uart_speed_t;
typedef uint8_t uart_parity_t;
typedef uint8_t uart_duplex_t;

enum __nesc_unnamed4309 {
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

enum __nesc_unnamed4310 {
  TOS_UART_OFF, 
  TOS_UART_RONLY, 
  TOS_UART_TONLY, 
  TOS_UART_DUPLEX
};

enum __nesc_unnamed4311 {
  TOS_UART_PARITY_NONE, 
  TOS_UART_PARITY_EVEN, 
  TOS_UART_PARITY_ODD
};
# 33 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
# 99 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/includes/printfUART.h"
static inline void printfUART_init();
# 25 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/includes/frame_format.h"
#line 15
typedef struct MPDU {

  uint8_t length;

  uint8_t frame_control1;

  uint8_t frame_control2;

  uint8_t seq_num;
  uint8_t data[120];
} MPDU;
#line 37
#line 27
typedef struct MPDUBuffer {

  uint8_t length;

  uint8_t frame_control1;
  uint8_t frame_control2;
  uint8_t seq_num;
  uint8_t data[120];
  uint8_t retransmission;
  uint8_t indirect;
} MPDUBuffer;
#line 57
#line 51
typedef struct beacon_addr_short {

  uint16_t destination_PAN_identifier;
  uint16_t destination_address;
  uint16_t source_address;
  uint16_t superframe_specification;
} beacon_addr_short;
#line 90
#line 83
typedef struct ACK {

  uint8_t length;
  uint8_t frame_control1;
  uint8_t frame_control2;

  uint8_t seq_num;
} ACK;









#line 96
typedef struct cmd_association_request {

  uint8_t command_frame_identifier;
  uint8_t capability_information;
} cmd_association_request;








#line 102
typedef struct cmd_association_response {

  uint8_t command_frame_identifier;
  uint8_t short_address1;
  uint8_t short_address2;

  uint8_t association_status;
} cmd_association_response;
#line 122
#line 112
typedef struct cmd_disassociation_notification {

  uint16_t destination_PAN_identifier;
  uint32_t destination_address0;
  uint32_t destination_address1;
  uint16_t source_PAN_identifier;
  uint32_t source_address0;
  uint32_t source_address1;
  uint8_t command_frame_identifier;
  uint8_t disassociation_reason;
} cmd_disassociation_notification;







#line 125
typedef struct cmd_beacon_request {

  uint16_t destination_PAN_identifier;
  uint16_t destination_address;
  uint8_t command_frame_identifier;
} cmd_beacon_request;









#line 134
typedef struct cmd_gts_request {

  uint16_t source_PAN_identifier;
  uint16_t source_address;
  uint8_t command_frame_identifier;
  uint8_t gts_characteristics;
} cmd_gts_request;




#line 142
typedef struct cmd_default {

  uint8_t command_frame_identifier;
} cmd_default;
#line 163
#line 149
typedef struct cmd_coord_realignment {

  uint8_t command_frame_identifier;
  uint8_t PAN_identifier0;
  uint8_t PAN_identifier1;
  uint8_t coordinator_short_address0;
  uint8_t coordinator_short_address1;





  uint8_t logical_channel;
  uint16_t short_address;
} cmd_coord_realignment;
#line 183
#line 179
typedef struct dest_short {

  uint16_t destination_PAN_identifier;
  uint16_t destination_address;
} dest_short;






#line 185
typedef struct dest_long {

  uint16_t destination_PAN_identifier;
  uint32_t destination_address0;
  uint32_t destination_address1;
} dest_long;





#line 193
typedef struct intra_pan_source_short {

  uint16_t source_address;
} intra_pan_source_short;





#line 198
typedef struct intra_pan_source_long {

  uint32_t source_address0;
  uint32_t source_address1;
} intra_pan_source_long;






#line 205
typedef struct source_short {

  uint16_t source_PAN_identifier;
  uint16_t source_address;
} source_short;







#line 212
typedef struct source_long {

  uint16_t source_PAN_identifier;
  uint32_t source_address0;
  uint32_t source_address1;
} source_long;
# 38 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/IEEE802154.h"
enum ieee154_fcf_enums {
  IEEE154_FCF_FRAME_TYPE = 0, 
  IEEE154_FCF_SECURITY_ENABLED = 3, 
  IEEE154_FCF_FRAME_PENDING = 4, 
  IEEE154_FCF_ACK_REQ = 5, 
  IEEE154_FCF_INTRAPAN = 6, 
  IEEE154_FCF_DEST_ADDR_MODE = 10, 
  IEEE154_FCF_SRC_ADDR_MODE = 14
};

enum ieee154_fcf_type_enums {
  IEEE154_TYPE_BEACON = 0, 
  IEEE154_TYPE_DATA = 1, 
  IEEE154_TYPE_ACK = 2, 
  IEEE154_TYPE_MAC_CMD = 3
};

enum iee154_fcf_addr_mode_enums {
  IEEE154_ADDR_NONE = 0, 
  IEEE154_ADDR_SHORT = 2, 
  IEEE154_ADDR_EXT = 3
};
enum /*PlatformSerialC.UartC*/Msp430Uart1C__0____nesc_unnamed4312 {
  Msp430Uart1C__0__CLIENT_ID = 0U
};
typedef T32khz /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__precision_tag;
typedef uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__size_type;
typedef T32khz /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag;
typedef /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__precision_tag;
typedef uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type;
enum /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0____nesc_unnamed4313 {
  Msp430Usart1C__0__CLIENT_ID = 0U
};
enum SerialAMQueueP____nesc_unnamed4314 {
  SerialAMQueueP__NUM_CLIENTS = 1U
};
typedef uint8_t /*PrintfC.QueueC*/QueueC__0__queue_t;
typedef /*PrintfC.QueueC*/QueueC__0__queue_t /*PrintfC.QueueC*/QueueC__0__Queue__t;
typedef uint8_t PrintfP__Queue__t;
typedef TMilli GTSManagementExampleP__Timer0__precision_tag;
typedef TMilli GTSManagementExampleP__Timer_Send__precision_tag;
typedef TMilli MacP__T_ackwait__precision_tag;
typedef TMilli MacP__T_ResponseWaitTime__precision_tag;
typedef TMilli MacP__T_ScanDuration__precision_tag;
typedef T32khz CC2420ControlP__StartupTimer__precision_tag;
typedef uint32_t CC2420ControlP__StartupTimer__size_type;
typedef uint16_t CC2420ControlP__ReadRssi__val_t;
enum /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Timer*/Msp430Timer32khzC__0____nesc_unnamed4315 {
  Msp430Timer32khzC__0__ALARM_ID = 0U
};
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__frequency_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__frequency_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__precision_tag;
typedef uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type;
typedef T32khz /*Counter32khz32C.Transform*/TransformCounterC__0__to_precision_tag;
typedef uint32_t /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type;
typedef T32khz /*Counter32khz32C.Transform*/TransformCounterC__0__from_precision_tag;
typedef uint16_t /*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type;
typedef uint16_t /*Counter32khz32C.Transform*/TransformCounterC__0__upper_count_type;
typedef /*Counter32khz32C.Transform*/TransformCounterC__0__from_precision_tag /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__precision_tag;
typedef /*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__size_type;
typedef /*Counter32khz32C.Transform*/TransformCounterC__0__to_precision_tag /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__precision_tag;
typedef /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__size_type;
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_precision_tag;
typedef uint32_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type;
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_precision_tag;
typedef uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__size_type;
enum /*CC2420ControlC.Spi*/CC2420SpiC__0____nesc_unnamed4316 {
  CC2420SpiC__0__CLIENT_ID = 0U
};
enum /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0____nesc_unnamed4317 {
  Msp430Spi0C__0__CLIENT_ID = 0U
};
enum /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0____nesc_unnamed4318 {
  Msp430Usart0C__0__CLIENT_ID = 0U
};
enum /*CC2420ControlC.SyncSpiC*/CC2420SpiC__1____nesc_unnamed4319 {
  CC2420SpiC__1__CLIENT_ID = 1U
};
enum /*CC2420ControlC.RssiResource*/CC2420SpiC__2____nesc_unnamed4320 {
  CC2420SpiC__2__CLIENT_ID = 2U
};
enum /*CC2420TransmitC.Spi*/CC2420SpiC__3____nesc_unnamed4321 {
  CC2420SpiC__3__CLIENT_ID = 3U
};
enum /*CC2420ReceiveC.Spi*/CC2420SpiC__4____nesc_unnamed4322 {
  CC2420SpiC__4__CLIENT_ID = 4U
};
typedef uint16_t RandomMlcgC__SeedInit__parameter;
typedef T32khz TimerAsyncM__AsyncTimer__precision_tag;
typedef uint32_t TimerAsyncM__AsyncTimer__size_type;
enum /*TimerAsyncC.Alarm.AlarmC.Msp430Timer*/Msp430Timer32khzC__1____nesc_unnamed4323 {
  Msp430Timer32khzC__1__ALARM_ID = 1U
};
typedef T32khz /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__frequency_tag;
typedef /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__frequency_tag /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__precision_tag;
typedef uint16_t /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type;
typedef T32khz /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__to_precision_tag;
typedef uint32_t /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__to_size_type;
typedef T32khz /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__from_precision_tag;
typedef uint16_t /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__from_size_type;
typedef /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__to_precision_tag /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Alarm__precision_tag;
typedef /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__to_size_type /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Alarm__size_type;
typedef /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__from_precision_tag /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__precision_tag;
typedef /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__from_size_type /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__size_type;
typedef /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__to_precision_tag /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Counter__precision_tag;
typedef /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__to_size_type /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Counter__size_type;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC__2____nesc_unnamed4324 {
  Msp430Timer32khzC__2__ALARM_ID = 2U
};
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__frequency_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__frequency_tag /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__size_type;
typedef TMilli /*CounterMilli32C.Transform*/TransformCounterC__1__to_precision_tag;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type;
typedef T32khz /*CounterMilli32C.Transform*/TransformCounterC__1__from_precision_tag;
typedef uint16_t /*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__1__upper_count_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__1__from_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__size_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__1__to_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__to_precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__to_size_type;
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__from_precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__from_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__from_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__AlarmFrom__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__from_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__AlarmFrom__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Counter__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Counter__size_type;
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
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Init.nc"
static error_t PlatformP__Init__init(void );
#line 51
static error_t MotePlatformC__Init__init(void );
# 35 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430ClockInit.nc"
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
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Init.nc"
static error_t Msp430ClockP__Init__init(void );
# 28 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(
# 40 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4067ade8);
# 28 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(
# 40 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4067ade8);
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
static bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
# 33 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t time);
# 31 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
# 28 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 33 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t time);
# 31 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
# 28 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 33 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t time);
# 31 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
# 28 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 33 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t time);
# 31 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );
# 28 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );
# 30 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t delta);
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 33 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void );
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(uint8_t cm);
#line 31
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void );
# 28 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 33 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t time);
# 31 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void );
# 28 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );
# 30 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t delta);
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 33 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t time);
# 31 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__enableEvents(void );
#line 36
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__clearPendingInterrupt(void );
# 28 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );
# 30 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEventFromNow(uint16_t delta);
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 33 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t time);
# 31 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
# 28 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 33 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t time);
# 31 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
# 28 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 33 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t time);
# 31 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
# 28 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/McuSleep.nc"
static void McuSleepC__McuSleep__sleep(void );
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x4059bb40);
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x4059bb40);
# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP__Scheduler__init(void );
#line 61
static void SchedulerBasicP__Scheduler__taskLoop(void );
#line 54
static bool SchedulerBasicP__Scheduler__runNextTask(void );
# 89 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Send.nc"
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(
#line 85
message_t * msg, 



error_t error);
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 69 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AMSend.nc"
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(
# 36 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x407758a8, 
# 69 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Packet.nc"
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
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x40781298, 
# 60 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AMPacket.nc"
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
# 83 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SplitControl.nc"
static error_t SerialP__SplitControl__start(void );
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void SerialP__stopDoneTask__runTask(void );
#line 64
static void SerialP__RunTx__runTask(void );
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Init.nc"
static error_t SerialP__Init__init(void );
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialFlush.nc"
static void SerialP__SerialFlush__flushDone(void );
#line 38
static void SerialP__SerialFlush__default__flush(void );
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void SerialP__startDoneTask__runTask(void );
# 83 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialFrameComm.nc"
static void SerialP__SerialFrameComm__dataReceived(uint8_t data);





static void SerialP__SerialFrameComm__putDone(void );
#line 74
static void SerialP__SerialFrameComm__delimiterReceived(void );
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void SerialP__defaultSerialFlushTask__runTask(void );
# 60 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SendBytePacket.nc"
static error_t SerialP__SendBytePacket__completeSend(void );
#line 51
static error_t SerialP__SendBytePacket__startSend(uint8_t first_byte);
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask(void );
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Send.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(
# 40 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40833830, 
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 89
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(
# 40 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40833830, 
# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask(void );
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x408331f0, 
# 60 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 31 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40831398, 
# 31 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40831398);
# 23 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40831398, 
# 23 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t upperLen);
# 70 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SendBytePacket.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte(void );









static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error_t error);
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/ReceiveBytePacket.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket(void );






static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(uint8_t data);










static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(error_t result);
# 79 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/UartStream.nc"
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
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialFrameComm.nc"
static error_t HdlcTranslateC__SerialFrameComm__putDelimiter(void );
#line 68
static void HdlcTranslateC__SerialFrameComm__resetReceive(void );
#line 54
static error_t HdlcTranslateC__SerialFrameComm__putData(uint8_t data);
# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408d0330);
# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__configure(
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408d0330);
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartConfigure.nc"
static msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(
# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408ce978);
# 48 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/UartStream.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__send(
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408d0a90, 
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len);
#line 79
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408d0a90, 
# 79 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/UartStream.nc"
uint8_t byte);
#line 99
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408d0a90, 
# 95 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__sendDone(
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408d0a90, 
# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Counter.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__overflow(void );
# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__release(
# 48 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408cfe50);
# 87 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__immediateRequest(
# 48 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408cfe50);
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__granted(
# 48 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408cfe50);
# 118 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__isOwner(
# 48 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408cfe50);
# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__release(
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408d58c0);
# 87 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__immediateRequest(
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408d58c0);
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__default__granted(
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408d58c0);
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__rxDone(
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408ccc10, 
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__txDone(
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408ccc10);
# 143 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart.nc"
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
# 74 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AsyncStdControl.nc"
static error_t HplMsp430Usart1P__AsyncStdControl__start(void );









static error_t HplMsp430Usart1P__AsyncStdControl__stop(void );
# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void );






static bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void );
#line 64
static void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void );
#line 59
static bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw(void );
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









static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__toggle(void );
#line 71
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );




static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr(void );




static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle(void );
#line 71
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );




static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void );
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Counter.nc"
static /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );






static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Init.nc"
static error_t LedsP__Init__init(void );
# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Leds.nc"
static void LedsP__Leds__led1On(void );










static void LedsP__Leds__led1Toggle(void );
#line 89
static void LedsP__Leds__led2Toggle(void );
#line 66
static void LedsP__Leds__led1Off(void );
#line 83
static void LedsP__Leds__led2Off(void );
#line 78
static void LedsP__Leds__led2On(void );
# 35 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
#line 29
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );

static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle(void );



static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
#line 29
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr(void );
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle(void );



static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
#line 29
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void );
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40ad3f28, 
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40ad3f28);
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Init.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void );
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceQueue.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );
#line 60
static resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(
# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40af5948);
# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(
# 60 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40af4cf8);
# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(
# 60 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40af4cf8);
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40ae1e98);
# 87 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40ae1e98);
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40ae1e98);
# 118 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40ae1e98);
# 80 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
# 52 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/power/PowerDownCleanup.nc"
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup(void );
# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted(void );
#line 81
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested(void );
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartConfigure.nc"
static msp430_uart_union_config_t *TelosSerialP__Msp430UartConfigure__getConfig(void );
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static void TelosSerialP__Resource__granted(void );
# 74 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/StdControl.nc"
static error_t TelosSerialP__StdControl__start(void );









static error_t TelosSerialP__StdControl__stop(void );
# 31 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t SerialPacketInfoActiveMessageP__Info__upperLength(message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t SerialPacketInfoActiveMessageP__Info__offset(void );







static uint8_t SerialPacketInfoActiveMessageP__Info__dataLinkLength(message_t *msg, uint8_t upperLen);
# 69 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AMSend.nc"
static error_t /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 89 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Send.nc"
static void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(
#line 85
message_t * msg, 



error_t error);
# 99 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AMSend.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(
# 40 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/AMQueueImplP.nc"
am_id_t arg_0x40b70ab0, 
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Send.nc"
static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(
# 38 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/AMQueueImplP.nc"
uint8_t arg_0x40b700c8, 
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 89
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(
# 38 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/AMQueueImplP.nc"
uint8_t arg_0x40b700c8, 
# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void );
#line 64
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void );
# 73 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Queue.nc"
static 
#line 71
/*PrintfC.QueueC*/QueueC__0__Queue__t  

/*PrintfC.QueueC*/QueueC__0__Queue__head(void );
#line 90
static error_t /*PrintfC.QueueC*/QueueC__0__Queue__enqueue(
#line 86
/*PrintfC.QueueC*/QueueC__0__Queue__t  newVal);
#line 65
static uint8_t /*PrintfC.QueueC*/QueueC__0__Queue__maxSize(void );
#line 81
static 
#line 79
/*PrintfC.QueueC*/QueueC__0__Queue__t  

/*PrintfC.QueueC*/QueueC__0__Queue__dequeue(void );
#line 50
static bool /*PrintfC.QueueC*/QueueC__0__Queue__empty(void );







static uint8_t /*PrintfC.QueueC*/QueueC__0__Queue__size(void );
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SplitControl.nc"
static void PrintfP__SerialControl__startDone(error_t error);
#line 117
static void PrintfP__SerialControl__stopDone(error_t error);
# 99 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AMSend.nc"
static void PrintfP__AMSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void PrintfP__retrySend__runTask(void );
# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Boot.nc"
static void PrintfP__MainBoot__booted(void );
# 72 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
static void GTSManagementExampleP__Timer0__fired(void );
# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Boot.nc"
static void GTSManagementExampleP__Boot__booted(void );
# 14 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_DISASSOCIATE.nc"
static error_t GTSManagementExampleP__MLME_DISASSOCIATE__indication(uint32_t DeviceAddress[], uint8_t DisassociateReason, uint8_t SecurityUse, uint8_t ACLEntry);
# 15 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_START.nc"
static error_t GTSManagementExampleP__MLME_START__confirm(uint8_t status);
# 14 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_SYNC_LOSS.nc"
static error_t GTSManagementExampleP__MLME_SYNC_LOSS__indication(uint8_t LossReason);
# 17 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_ASSOCIATE.nc"
static error_t GTSManagementExampleP__MLME_ASSOCIATE__confirm(uint16_t AssocShortAddress, uint8_t status);
#line 13
static error_t GTSManagementExampleP__MLME_ASSOCIATE__indication(uint32_t DeviceAddress[], uint8_t CapabilityInformation, bool SecurityUse, uint8_t ACLEntry);
# 72 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
static void GTSManagementExampleP__Timer_Send__fired(void );
# 14 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MCPS_DATA.nc"
static error_t GTSManagementExampleP__MCPS_DATA__confirm(uint8_t msduHandle, uint8_t status);

static error_t GTSManagementExampleP__MCPS_DATA__indication(uint16_t SrcAddrMode, uint16_t SrcPANId, uint32_t SrcAddr[2], uint16_t DstAddrMode, uint16_t DestPANId, uint32_t DstAddr[2], uint16_t msduLength, uint8_t msdu[100], uint16_t mpduLinkQuality, uint16_t SecurityUse, uint16_t ACLEntry);
# 13 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_SET.nc"
static error_t GTSManagementExampleP__MLME_SET__confirm(uint8_t status, uint8_t PIBAttribute);
# 13 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_ORPHAN.nc"
static error_t GTSManagementExampleP__MLME_ORPHAN__indication(uint32_t OrphanAddress[1], uint8_t SecurityUse, uint8_t ACLEntry);
# 12 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_BEACON_NOTIFY.nc"
static error_t GTSManagementExampleP__MLME_BEACON_NOTIFY__indication(uint8_t BSN, PANDescriptor pan_descriptor, uint8_t PenAddrSpec, uint8_t AddrList, uint8_t sduLength, uint8_t sdu[]);
# 15 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_SCAN.nc"
static error_t GTSManagementExampleP__MLME_SCAN__confirm(uint8_t status, uint8_t ScanType, uint32_t UnscannedChannels, uint8_t ResultListSize, uint8_t EnergyDetectList[], SCAN_PANDescriptor PANDescriptorList[]);
# 14 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_GTS.nc"
static error_t GTSManagementExampleP__MLME_GTS__confirm(uint8_t GTSCharacteristics, uint8_t status);

static error_t GTSManagementExampleP__MLME_GTS__indication(uint16_t DevAddress, uint8_t GTSCharacteristics, uint8_t SecurityUse, uint8_t ACLEntry);
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SplitControl.nc"
static void MacP__AMControl__startDone(error_t error);
#line 117
static void MacP__AMControl__stopDone(error_t error);
# 72 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
static void MacP__T_ackwait__fired(void );
# 16 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/phy/PLME_SET.nc"
static error_t MacP__PLME_SET__confirm(uint8_t status, uint8_t PIBAttribute);
# 13 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_START.nc"
static error_t MacP__MLME_START__request(uint32_t PANId, uint8_t LogicalChannel, uint8_t BeaconOrder, uint8_t SuperframeOrder, uint8_t PANCoordinator, uint8_t BatteryLifeExtension, uint8_t CoordRealignment, uint8_t SecurityEnable, uint32_t StartTime);
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void MacP__perform_csma_ca_unslotted__runTask(void );
#line 64
static void MacP__data_indication__runTask(void );
# 72 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
static void MacP__T_ResponseWaitTime__fired(void );
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Init.nc"
static error_t MacP__Init__init(void );
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void MacP__start_coordinator_gts_send__runTask(void );
#line 64
static void MacP__perform_csma_ca_slotted__runTask(void );
#line 64
static void MacP__data_channel_scan_indication__runTask(void );
#line 64
static void MacP__signal_loss__runTask(void );
# 22 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsync.nc"
static error_t MacP__TimerAsync__bi_fired(void );


static error_t MacP__TimerAsync__backoff_fired(void );


static error_t MacP__TimerAsync__time_slot_fired(void );

static error_t MacP__TimerAsync__before_time_slot_fired(void );
#line 20
static error_t MacP__TimerAsync__sd_fired(void );
#line 18
static error_t MacP__TimerAsync__before_bi_fired(void );
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void MacP__send_frame_csma__runTask(void );
# 17 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/phy/PD_DATA.nc"
static error_t MacP__PD_DATA__indication(uint8_t psduLenght, uint8_t *psdu, int8_t ppduLinkQuality);
# 12 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MCPS_DATA.nc"
static error_t MacP__MCPS_DATA__request(uint8_t SrcAddrMode, uint16_t SrcPANId, uint32_t SrcAddr[], uint8_t DstAddrMode, uint16_t DestPANId, uint32_t DstAddr[], uint8_t msduLength, uint8_t msdu[], uint8_t msduHandle, uint8_t TxOptions);
# 11 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_SET.nc"
static error_t MacP__MLME_SET__request(uint8_t PIBAttribute, uint8_t PIBAttributeValue[]);
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void MacP__increment_gts_null__runTask(void );
# 72 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
static void MacP__T_ScanDuration__fired(void );
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void MacP__start_gts_send__runTask(void );
# 12 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_GTS.nc"
static error_t MacP__MLME_GTS__request(uint8_t GTSCharacteristics, uint8_t SecurityEnable);
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void MacP__check_gts_expiration__runTask(void );
#line 64
static void MacP__create_beacon__runTask(void );
# 83 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SplitControl.nc"
static error_t PhyP__SplitControl__start(void );
# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420Config.nc"
static void PhyP__CC2420Config__syncDone(error_t error);
# 14 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/Receiveframe.nc"
static void PhyP__Receiveframe__receive(uint8_t *frame, uint8_t rssi);
# 14 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/phy/PLME_SET.nc"
static error_t PhyP__PLME_SET__request(uint8_t PIBAttribute, uint8_t PIBAttributeValue);
# 21 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/Sendframe.nc"
static void PhyP__Sendframe__sendDone(error_t error);
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Init.nc"
static error_t PhyP__Init__init(void );
# 14 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/phy/PLME_SET_TRX_STATE.nc"
static error_t PhyP__PLME_SET_TRX_STATE__request(uint8_t state);
# 76 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Power.nc"
static void PhyP__CC2420Power__startOscillatorDone(void );
#line 56
static void PhyP__CC2420Power__startVRegDone(void );
# 13 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/phy/PD_DATA.nc"
static error_t PhyP__PD_DATA__request(uint8_t psduLenght, uint8_t *psdu);
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static void PhyP__Resource__granted(void );
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void PhyP__sendDone_task__runTask(void );
#line 64
static void PhyP__stopDone_task__runTask(void );
#line 64
static void PhyP__startDone_task__runTask(void );
# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420Config.nc"
static void CC2420ControlP__CC2420Config__setChannel(uint8_t channel);
#line 52
static error_t CC2420ControlP__CC2420Config__sync(void );
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
static void CC2420ControlP__StartupTimer__fired(void );
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void CC2420ControlP__syncDone__runTask(void );
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Init.nc"
static error_t CC2420ControlP__Init__init(void );
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static void CC2420ControlP__SpiResource__granted(void );
#line 92
static void CC2420ControlP__SyncResource__granted(void );
# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Power.nc"
static error_t CC2420ControlP__CC2420Power__startOscillator(void );
#line 90
static error_t CC2420ControlP__CC2420Power__rxOn(void );
#line 51
static error_t CC2420ControlP__CC2420Power__startVReg(void );
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void CC2420ControlP__sync__runTask(void );
# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t CC2420ControlP__Resource__release(void );
#line 78
static error_t CC2420ControlP__Resource__request(void );
# 57 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GpioInterrupt.nc"
static void CC2420ControlP__InterruptCCA__fired(void );
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static void CC2420ControlP__RssiResource__granted(void );
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type dt);
# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Counter.nc"
static void /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
#line 53
static /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__size_type /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get(void );
# 98 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );
#line 92
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type dt);
#line 55
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type dt);
#line 67
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Counter.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 33 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput(void );
#line 32
static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__get(void );


static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput(void );
#line 29
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set(void );
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr(void );

static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__GeneralIO__get(void );
#line 32
static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__GeneralIO__get(void );


static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput(void );
#line 29
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__set(void );
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__clr(void );


static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput(void );

static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput(void );
#line 29
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__set(void );
# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time);
# 42 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GpioCapture.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge(void );
# 41 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
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
#line 61
static void HplMsp430InterruptP__Port27__default__fired(void );
#line 41
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
#line 61
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired(void );
# 50 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GpioInterrupt.nc"
static error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable(void );
#line 42
static error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void );
# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired(void );
# 50 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GpioInterrupt.nc"
static error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable(void );
#line 43
static error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge(void );
# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SpiPacket.nc"
static void CC2420SpiP__SpiPacket__sendDone(
#line 64
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 62 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static error_t CC2420SpiP__Fifo__continueRead(
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
uint8_t arg_0x41118698, 
# 62 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 91
static void CC2420SpiP__Fifo__default__writeDone(
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
uint8_t arg_0x41118698, 
# 91 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
#line 82
static cc2420_status_t CC2420SpiP__Fifo__write(
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
uint8_t arg_0x41118698, 
# 82 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 51
static cc2420_status_t CC2420SpiP__Fifo__beginRead(
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
uint8_t arg_0x41118698, 
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 71
static void CC2420SpiP__Fifo__default__readDone(
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
uint8_t arg_0x41118698, 
# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static error_t CC2420SpiP__ChipSpiResource__attemptRelease(void );
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static void CC2420SpiP__SpiResource__granted(void );
# 63 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420SpiP__Ram__write(
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
uint16_t arg_0x41116118, 
# 63 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Ram.nc"
uint8_t offset, uint8_t * data, uint8_t length);
# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420SpiP__Reg__read(
# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
uint8_t arg_0x411168c0, 
# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Register.nc"
uint16_t *data);







static cc2420_status_t CC2420SpiP__Reg__write(
# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
uint8_t arg_0x411168c0, 
# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Register.nc"
uint16_t data);
# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__release(
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
uint8_t arg_0x410deb90);
# 87 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__immediateRequest(
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
uint8_t arg_0x410deb90);
# 78 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__request(
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
uint8_t arg_0x410deb90);
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static void CC2420SpiP__Resource__default__granted(
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
uint8_t arg_0x410deb90);
# 118 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static bool CC2420SpiP__Resource__isOwner(
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
uint8_t arg_0x410deb90);
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void CC2420SpiP__grant__runTask(void );
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420SpiP__Strobe__strobe(
# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
uint8_t arg_0x41115088);
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Init.nc"
static error_t StateImplP__Init__init(void );
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/State.nc"
static void StateImplP__State__toIdle(
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/StateImplP.nc"
uint8_t arg_0x4114fd48);
# 66 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/State.nc"
static bool StateImplP__State__isState(
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/StateImplP.nc"
uint8_t arg_0x4114fd48, 
# 66 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/State.nc"
uint8_t myState);
#line 61
static bool StateImplP__State__isIdle(
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/StateImplP.nc"
uint8_t arg_0x4114fd48);
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/State.nc"
static error_t StateImplP__State__requestState(
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/StateImplP.nc"
uint8_t arg_0x4114fd48, 
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/State.nc"
uint8_t reqState);
# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(
# 42 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x41170410);
# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(
# 42 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x41170410);
# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SpiPacket.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x41170ee8, 
# 48 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
#line 71
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x41170ee8, 
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(
# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x4116e1b8);
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SpiByte.nc"
static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(uint8_t tx);
# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(
# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x4116f718);
# 87 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(
# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x4116f718);
# 78 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(
# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x4116f718);
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(
# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x4116f718);
# 118 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(
# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x4116f718);
# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(
# 41 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x41173920);
# 87 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(
# 41 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x41173920);
# 78 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(
# 41 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x41173920);
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(
# 41 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x41173920);
# 118 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(
# 41 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x41173920);
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void );
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void );
# 180 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430Usart0P__Usart__enableRxIntr(void );
#line 197
static void HplMsp430Usart0P__Usart__clrRxIntr(void );
#line 97
static void HplMsp430Usart0P__Usart__resetUsart(bool reset);
#line 179
static void HplMsp430Usart0P__Usart__disableIntr(void );
#line 90
static void HplMsp430Usart0P__Usart__setUmctl(uint8_t umctl);
#line 177
static void HplMsp430Usart0P__Usart__disableRxIntr(void );
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
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__rxDone(
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40ad3f28, 
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__txDone(
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40ad3f28);
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawI2CInterrupts__fired(void );
#line 39
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__I2CInterrupts__default__fired(
# 40 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40acff08);
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__txDone(void );
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Init.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void );
# 69 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(resource_client_id_t id);
#line 43
static bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void );








static bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void );
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(
# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40af5948);
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(
# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40af5948);
# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(
# 60 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40af4cf8);
# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(
# 60 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40af4cf8);
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void );
#line 73
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__requested(void );
#line 46
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__granted(void );
#line 81
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__immediateRequested(void );
# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40ae1e98);
# 87 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40ae1e98);
# 78 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__request(
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40ae1e98);
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40ae1e98);
# 118 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40ae1e98);
# 80 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId(void );
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask(void );
# 7 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430I2C.nc"
static void HplMsp430I2C0P__HplI2C__clearModeI2C(void );
#line 6
static bool HplMsp430I2C0P__HplI2C__isI2C(void );
# 50 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GpioCapture.nc"
static void CC2420TransmitP__CaptureSFD__captured(uint16_t time);
# 16 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/Sendframe.nc"
static error_t CC2420TransmitP__Sendframe__send(uint8_t *frame, uint8_t frame_length);
# 24 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420TransmitP__ChipSpiResource__releasing(void );
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Init.nc"
static error_t CC2420TransmitP__Init__init(void );
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static void CC2420TransmitP__SpiResource__granted(void );
# 74 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/StdControl.nc"
static error_t CC2420TransmitP__StdControl__start(void );
# 91 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420TransmitP__TXFIFO__writeDone(uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420TransmitP__TXFIFO__readDone(uint8_t * data, uint8_t length, error_t error);
# 10 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/AddressFilter.nc"
static error_t CC2420ReceiveP__AddressFilter__set_address(uint16_t mac_short_address, uint32_t mac_extended0, uint32_t mac_extended1);


static error_t CC2420ReceiveP__AddressFilter__set_coord_address(uint16_t mac_coord_address, uint16_t mac_panid);
# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420Config.nc"
static void CC2420ReceiveP__CC2420Config__syncDone(error_t error);
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Init.nc"
static error_t CC2420ReceiveP__Init__init(void );
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static void CC2420ReceiveP__SpiResource__granted(void );
# 91 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420ReceiveP__RXFIFO__writeDone(uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420ReceiveP__RXFIFO__readDone(uint8_t * data, uint8_t length, error_t error);
# 57 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GpioInterrupt.nc"
static void CC2420ReceiveP__InterruptFIFOP__fired(void );
# 74 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/StdControl.nc"
static error_t CC2420ReceiveP__StdControl__start(void );
# 41 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Random.nc"
static uint16_t RandomMlcgC__Random__rand16(void );
#line 35
static uint32_t RandomMlcgC__Random__rand32(void );
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Init.nc"
static error_t RandomMlcgC__Init__init(void );
# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsync.nc"
static uint32_t TimerAsyncM__TimerAsync__get_current_number_backoff_on_time_slot(void );





static uint32_t TimerAsyncM__TimerAsync__get_current_ticks(void );
#line 36
static error_t TimerAsyncM__TimerAsync__set_bi_sd(uint32_t bi_symbols, uint32_t sd_symbols);

static error_t TimerAsyncM__TimerAsync__set_backoff_symbols(uint8_t symbols);









static error_t TimerAsyncM__TimerAsync__set_timers_enable(uint8_t timer);


static uint32_t TimerAsyncM__TimerAsync__get_total_tick_counter(void );
#line 14
static error_t TimerAsyncM__TimerAsync__reset(void );
#line 42
static uint8_t TimerAsyncM__TimerAsync__reset_start(uint32_t start_ticks);
#line 40
static error_t TimerAsyncM__TimerAsync__set_enable_backoffs(bool enable_backoffs);
#line 67
static uint32_t TimerAsyncM__TimerAsync__get_sd_ticks(void );
#line 10
static error_t TimerAsyncM__TimerAsync__start(void );
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
static void TimerAsyncM__AsyncTimer__fired(void );
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
static void /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(/*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type t0, /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type dt);





static /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Alarm__size_type /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Alarm__getNow(void );
#line 92
static void /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Alarm__startAt(/*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Alarm__size_type t0, /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Alarm__size_type dt);
#line 55
static void /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Alarm__start(/*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Alarm__size_type dt);
#line 67
static void /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__fired(void );
# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Counter.nc"
static void /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Counter__overflow(void );
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired(void );
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow(void );
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__size_type dt);
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__stop(void );
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Init__init(void );
# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Counter.nc"
static void /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void );
#line 53
static /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__size_type /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__get(void );
# 98 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__size_type dt);
#line 105
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__AlarmFrom__fired(void );
# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Counter__overflow(void );
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
# 125 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
#line 118
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
# 72 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );
#line 72
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x413eb300);
# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x413eb300, 
# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
uint32_t dt);








static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x413eb300, 
# 62 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
uint32_t dt);




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x413eb300);
# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Init.nc"
static error_t PlatformP__MoteInit__init(void );
#line 51
static error_t PlatformP__MoteClockInit__init(void );
#line 51
static error_t PlatformP__LedsInit__init(void );
# 10 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void );
# 6 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__uwait(uint16_t u);




static __inline void MotePlatformC__TOSH_wait(void );




static void MotePlatformC__TOSH_FLASH_M25P_DP_bit(bool set);










static inline void MotePlatformC__TOSH_FLASH_M25P_DP(void );
#line 56
static inline error_t MotePlatformC__Init__init(void );
# 32 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__initTimerB(void );
#line 31
static void Msp430ClockP__Msp430ClockInit__initTimerA(void );
#line 29
static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__initClocks(void );
# 40 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430ClockP.nc"
static volatile uint8_t Msp430ClockP__IE1 __asm ("0x0000");
static volatile uint16_t Msp430ClockP__TA0CTL __asm ("0x0160");
static volatile uint16_t Msp430ClockP__TA0IV __asm ("0x012E");
static volatile uint16_t Msp430ClockP__TBCTL __asm ("0x0180");
static volatile uint16_t Msp430ClockP__TBIV __asm ("0x011E");

enum Msp430ClockP____nesc_unnamed4325 {

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
# 28 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(
# 40 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4067ade8);
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void );
# 115 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n);
# 28 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(
# 40 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4067ade8);
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void );
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
#line 70
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
#line 115
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );








static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n);
# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time);
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void );
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
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
# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time);
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void );
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
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
# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time);
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void );
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
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
# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time);
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void );
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void );
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t;


static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );
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
# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time);
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void );
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
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
# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time);
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void );
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get(void );
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
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
# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time);
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void );
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__get(void );
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__CC2int(/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__compareControl(void );
#line 74
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__setControlAsCompare(void );
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
# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time);
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void );
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
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
# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time);
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void );
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
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
# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time);
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void );
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
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
# 28 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void Msp430TimerCommonP__VectorTimerB1__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerA0__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerA1__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerB0__fired(void );
# 11 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
void sig_TIMERA0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(12)))  ;
void sig_TIMERA1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(10)))  ;
void sig_TIMERB0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(26)))  ;
void sig_TIMERB1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(24)))  ;
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void );
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/McuSleepC.nc"
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
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Init.nc"
static error_t RealMainP__SoftwareInit__init(void );
# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Boot.nc"
static void RealMainP__Boot__booted(void );
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Init.nc"
static error_t RealMainP__PlatformInit__init(void );
# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Scheduler.nc"
static void RealMainP__Scheduler__init(void );
#line 61
static void RealMainP__Scheduler__taskLoop(void );
#line 54
static bool RealMainP__Scheduler__runNextTask(void );
# 52 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/RealMainP.nc"
int main(void )   ;
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x4059bb40);
# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP__McuSleep__sleep(void );
# 50 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4326 {

  SchedulerBasicP__NUM_TASKS = 31U, 
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
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Send.nc"
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__send(
#line 56
message_t * msg, 







uint8_t len);
# 99 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AMSend.nc"
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__sendDone(
# 36 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x407758a8, 
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__receive(
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x40781298, 
# 60 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialActiveMessageP.nc"
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
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SplitControl.nc"
static void SerialP__SplitControl__startDone(error_t error);
#line 117
static void SerialP__SplitControl__stopDone(error_t error);
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t SerialP__stopDoneTask__postTask(void );
# 74 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/StdControl.nc"
static error_t SerialP__SerialControl__start(void );









static error_t SerialP__SerialControl__stop(void );
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t SerialP__RunTx__postTask(void );
# 38 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialFlush.nc"
static void SerialP__SerialFlush__flush(void );
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t SerialP__startDoneTask__postTask(void );
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialFrameComm.nc"
static error_t SerialP__SerialFrameComm__putDelimiter(void );
#line 68
static void SerialP__SerialFrameComm__resetReceive(void );
#line 54
static error_t SerialP__SerialFrameComm__putData(uint8_t data);
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t SerialP__defaultSerialFlushTask__postTask(void );
# 70 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SendBytePacket.nc"
static uint8_t SerialP__SendBytePacket__nextByte(void );









static void SerialP__SendBytePacket__sendCompleted(error_t error);
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/ReceiveBytePacket.nc"
static error_t SerialP__ReceiveBytePacket__startPacket(void );






static void SerialP__ReceiveBytePacket__byteReceived(uint8_t data);










static void SerialP__ReceiveBytePacket__endPacket(error_t result);
# 189 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialP.nc"
enum SerialP____nesc_unnamed4327 {
#line 189
  SerialP__RunTx = 0U
};
#line 189
typedef int SerialP____nesc_sillytask_RunTx[SerialP__RunTx];
#line 320
enum SerialP____nesc_unnamed4328 {
#line 320
  SerialP__startDoneTask = 1U
};
#line 320
typedef int SerialP____nesc_sillytask_startDoneTask[SerialP__startDoneTask];





enum SerialP____nesc_unnamed4329 {
#line 326
  SerialP__stopDoneTask = 2U
};
#line 326
typedef int SerialP____nesc_sillytask_stopDoneTask[SerialP__stopDoneTask];








enum SerialP____nesc_unnamed4330 {
#line 335
  SerialP__defaultSerialFlushTask = 3U
};
#line 335
typedef int SerialP____nesc_sillytask_defaultSerialFlushTask[SerialP__defaultSerialFlushTask];
#line 79
enum SerialP____nesc_unnamed4331 {
  SerialP__RX_DATA_BUFFER_SIZE = 2, 
  SerialP__TX_DATA_BUFFER_SIZE = 4, 
  SerialP__SERIAL_MTU = 255, 
  SerialP__SERIAL_VERSION = 1, 
  SerialP__ACK_QUEUE_SIZE = 5
};

enum SerialP____nesc_unnamed4332 {
  SerialP__RXSTATE_NOSYNC, 
  SerialP__RXSTATE_PROTO, 
  SerialP__RXSTATE_TOKEN, 
  SerialP__RXSTATE_INFO, 
  SerialP__RXSTATE_INACTIVE
};

enum SerialP____nesc_unnamed4333 {
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
typedef enum SerialP____nesc_unnamed4334 {
  SerialP__BUFFER_AVAILABLE, 
  SerialP__BUFFER_FILLING, 
  SerialP__BUFFER_COMPLETE
} SerialP__tx_data_buffer_states_t;

enum SerialP____nesc_unnamed4335 {
  SerialP__TX_ACK_INDEX = 0, 
  SerialP__TX_DATA_INDEX = 1, 
  SerialP__TX_BUFFER_COUNT = 2
};






#line 122
typedef struct SerialP____nesc_unnamed4336 {
  uint8_t writePtr;
  uint8_t readPtr;
  uint8_t buf[SerialP__RX_DATA_BUFFER_SIZE + 1];
} SerialP__rx_buf_t;




#line 128
typedef struct SerialP____nesc_unnamed4337 {
  uint8_t state;
  uint8_t buf;
} SerialP__tx_buf_t;





#line 133
typedef struct SerialP____nesc_unnamed4338 {
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
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__postTask(void );
# 89 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Send.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__sendDone(
# 40 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40833830, 
# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__postTask(void );
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__receive(
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x408331f0, 
# 60 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 31 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__upperLength(
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40831398, 
# 31 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40831398);
# 23 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__dataLinkLength(
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40831398, 
# 23 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t upperLen);
# 60 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SendBytePacket.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__completeSend(void );
#line 51
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__startSend(uint8_t first_byte);
# 147 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4339 {
#line 147
  SerialDispatcherP__0__signalSendDone = 4U
};
#line 147
typedef int /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_sillytask_signalSendDone[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone];
#line 264
enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4340 {
#line 264
  SerialDispatcherP__0__receiveTask = 5U
};
#line 264
typedef int /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_sillytask_receiveTask[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask];
#line 55
#line 51
typedef enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4341 {
  SerialDispatcherP__0__SEND_STATE_IDLE = 0, 
  SerialDispatcherP__0__SEND_STATE_BEGIN = 1, 
  SerialDispatcherP__0__SEND_STATE_DATA = 2
} /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__send_state_t;

enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4342 {
  SerialDispatcherP__0__RECV_STATE_IDLE = 0, 
  SerialDispatcherP__0__RECV_STATE_BEGIN = 1, 
  SerialDispatcherP__0__RECV_STATE_DATA = 2
};






#line 63
typedef struct /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4343 {
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
# 48 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/UartStream.nc"
static error_t HdlcTranslateC__UartStream__send(
#line 44
uint8_t * buf, 



uint16_t len);
# 83 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialFrameComm.nc"
static void HdlcTranslateC__SerialFrameComm__dataReceived(uint8_t data);





static void HdlcTranslateC__SerialFrameComm__putDone(void );
#line 74
static void HdlcTranslateC__SerialFrameComm__delimiterReceived(void );
# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/HdlcTranslateC.nc"
#line 44
typedef struct HdlcTranslateC____nesc_unnamed4344 {
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
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartConfigure.nc"
static msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__getConfig(
# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408ce978);
# 97 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart.nc"
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
# 79 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/UartStream.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receivedByte(
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408d0a90, 
# 79 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/UartStream.nc"
uint8_t byte);
#line 99
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receiveDone(
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408d0a90, 
# 95 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__sendDone(
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408d0a90, 
# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__release(
# 48 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408cfe50);
# 87 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__immediateRequest(
# 48 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408cfe50);
# 118 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__isOwner(
# 48 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408cfe50);
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__granted(
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x408d58c0);
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
# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart1P__UCLK__selectIOFunc(void );
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void HplMsp430Usart1P__Interrupts__rxDone(uint8_t data);
#line 49
static void HplMsp430Usart1P__Interrupts__txDone(void );
# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart1P__URXD__selectIOFunc(void );
#line 78
static void HplMsp430Usart1P__URXD__selectModuleFunc(void );






static void HplMsp430Usart1P__UTXD__selectIOFunc(void );
#line 78
static void HplMsp430Usart1P__UTXD__selectModuleFunc(void );






static void HplMsp430Usart1P__SOMI__selectIOFunc(void );
#line 85
static void HplMsp430Usart1P__SIMO__selectIOFunc(void );
# 87 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
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
# 48 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void );
#line 48
static inline uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void );
#line 48
static inline uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void );



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
#line 50
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void );
#line 45
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set(void );
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void );






static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void );



static inline void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );






static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr(void );
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__toggle(void );




static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void );
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle(void );




static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void );
static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void );
# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Counter.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void );
# 38 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );




static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );









static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 35 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
static void LedsP__Led0__makeOutput(void );
#line 29
static void LedsP__Led0__set(void );

static void LedsP__Led1__toggle(void );



static void LedsP__Led1__makeOutput(void );
#line 29
static void LedsP__Led1__set(void );
static void LedsP__Led1__clr(void );
static void LedsP__Led2__toggle(void );



static void LedsP__Led2__makeOutput(void );
#line 29
static void LedsP__Led2__set(void );
static void LedsP__Led2__clr(void );
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void );
#line 78
static inline void LedsP__Leds__led1On(void );




static inline void LedsP__Leds__led1Off(void );




static inline void LedsP__Leds__led1Toggle(void );




static inline void LedsP__Leds__led2On(void );




static inline void LedsP__Leds__led2Off(void );




static inline void LedsP__Leds__led2Toggle(void );
# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void );
#line 34
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void );
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );





static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__toggle(void );
#line 71
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void );
#line 34
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void );




static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__clr(void );
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr(void );
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle(void );



static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__toggle(void );
#line 71
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void );
#line 34
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void );




static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr(void );
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void );
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle(void );



static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
# 80 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(void );
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40ad3f28, 
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40ad3f28);









static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );




static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);









static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id);
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data);
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0____nesc_unnamed4345 {
#line 39
  FcfsResourceQueueC__0__NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[1U];
uint8_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
uint8_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

static inline error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void );




static inline bool /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );







static inline resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(
# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40af5948);
# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(
# 60 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40af4cf8);
# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(
# 60 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40af4cf8);
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceQueue.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void );
#line 60
static resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void );
# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void );
#line 81
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void );
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40ae1e98);
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void );
# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4346 {
#line 75
  ArbiterP__0__grantedTask = 6U
};
#line 75
typedef int /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_sillytask_grantedTask[/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask];
#line 67
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4347 {
#line 67
  ArbiterP__0__RES_CONTROLLED, ArbiterP__0__RES_GRANTING, ArbiterP__0__RES_IMM_GRANTING, ArbiterP__0__RES_BUSY
};
#line 68
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4348 {
#line 68
  ArbiterP__0__default_owner_id = 1U
};
#line 69
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4349 {
#line 69
  ArbiterP__0__NO_RES = 0xFF
};
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
#line 93
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id);
#line 111
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id);
#line 133
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
#line 153
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );
#line 166
static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );










static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id);
#line 190
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
#line 202
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id);



static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id);









static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id);

static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id);
# 52 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/power/PowerDownCleanup.nc"
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__cleanup(void );
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__release(void );
# 74 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AsyncStdControl.nc"
static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__start(void );









static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__stop(void );
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested(void );




static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted(void );




static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup(void );
# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t TelosSerialP__Resource__release(void );
#line 87
static error_t TelosSerialP__Resource__immediateRequest(void );
# 8 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/platforms/telosa/TelosSerialP.nc"
msp430_uart_union_config_t TelosSerialP__msp430_uart_telos_config = { { .ubr = UBR_1MHZ_115200, .umctl = UMCTL_1MHZ_115200, .ssel = 0x02, .pena = 0, .pev = 0, .spb = 0, .clen = 1, .listen = 0, .mm = 0, .ckpl = 0, .urxse = 0, .urxeie = 1, .urxwie = 0, .utxe = 1, .urxe = 1 } };

static inline error_t TelosSerialP__StdControl__start(void );


static inline error_t TelosSerialP__StdControl__stop(void );



static inline void TelosSerialP__Resource__granted(void );

static inline msp430_uart_union_config_t *TelosSerialP__Msp430UartConfigure__getConfig(void );
# 40 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__offset(void );


static inline uint8_t SerialPacketInfoActiveMessageP__Info__dataLinkLength(message_t *msg, uint8_t upperLen);


static inline uint8_t SerialPacketInfoActiveMessageP__Info__upperLength(message_t *msg, uint8_t dataLinkLen);
# 99 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AMSend.nc"
static void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Send.nc"
static error_t /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(
#line 56
message_t * msg, 







uint8_t len);
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AMPacket.nc"
static void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 151
static void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/AMQueueEntryP.nc"
static error_t /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len);









static inline void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(message_t *m, error_t err);
# 69 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AMSend.nc"
static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(
# 40 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/AMQueueImplP.nc"
am_id_t arg_0x40b70ab0, 
# 69 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 89 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Send.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(
# 38 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/AMQueueImplP.nc"
uint8_t arg_0x40b700c8, 
# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Packet.nc"
static uint8_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(
#line 63
message_t * msg);
#line 83
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(
#line 79
message_t * msg, 



uint8_t len);
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask(void );
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AMPacket.nc"
static am_addr_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(
#line 63
message_t * amsg);
#line 136
static am_id_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(
#line 132
message_t * amsg);
# 118 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/AMQueueImplP.nc"
enum /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4350 {
#line 118
  AMQueueImplP__0__CancelTask = 7U
};
#line 118
typedef int /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_CancelTask[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask];
#line 161
enum /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4351 {
#line 161
  AMQueueImplP__0__errorTask = 8U
};
#line 161
typedef int /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_errorTask[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask];
#line 49
#line 47
typedef struct /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4352 {
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
# 48 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/QueueC.nc"
/*PrintfC.QueueC*/QueueC__0__queue_t  /*PrintfC.QueueC*/QueueC__0__queue[250];
uint8_t /*PrintfC.QueueC*/QueueC__0__head = 0;
uint8_t /*PrintfC.QueueC*/QueueC__0__tail = 0;
uint8_t /*PrintfC.QueueC*/QueueC__0__size = 0;

static inline bool /*PrintfC.QueueC*/QueueC__0__Queue__empty(void );



static inline uint8_t /*PrintfC.QueueC*/QueueC__0__Queue__size(void );



static inline uint8_t /*PrintfC.QueueC*/QueueC__0__Queue__maxSize(void );



static inline /*PrintfC.QueueC*/QueueC__0__queue_t /*PrintfC.QueueC*/QueueC__0__Queue__head(void );



static inline void /*PrintfC.QueueC*/QueueC__0__printQueue(void );
#line 85
static inline /*PrintfC.QueueC*/QueueC__0__queue_t /*PrintfC.QueueC*/QueueC__0__Queue__dequeue(void );
#line 97
static inline error_t /*PrintfC.QueueC*/QueueC__0__Queue__enqueue(/*PrintfC.QueueC*/QueueC__0__queue_t newVal);
# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Boot.nc"
static void PrintfP__Boot__booted(void );
# 83 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SplitControl.nc"
static error_t PrintfP__SerialControl__start(void );
# 90 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Queue.nc"
static error_t PrintfP__Queue__enqueue(
#line 86
PrintfP__Queue__t  newVal);
#line 81
static 
#line 79
PrintfP__Queue__t  

PrintfP__Queue__dequeue(void );
#line 50
static bool PrintfP__Queue__empty(void );







static uint8_t PrintfP__Queue__size(void );
# 69 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AMSend.nc"
static error_t PrintfP__AMSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 115 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Packet.nc"
static 
#line 112
void * 


PrintfP__Packet__getPayload(
#line 110
message_t * msg, 




uint8_t len);
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t PrintfP__retrySend__postTask(void );
# 127 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/printf/PrintfP.nc"
enum PrintfP____nesc_unnamed4353 {
#line 127
  PrintfP__retrySend = 9U
};
#line 127
typedef int PrintfP____nesc_sillytask_retrySend[PrintfP__retrySend];
#line 100
enum PrintfP____nesc_unnamed4354 {
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
# 62 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
static void GTSManagementExampleP__Timer0__startOneShot(uint32_t dt);
# 13 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_START.nc"
static error_t GTSManagementExampleP__MLME_START__request(uint32_t PANId, uint8_t LogicalChannel, uint8_t BeaconOrder, uint8_t SuperframeOrder, uint8_t PANCoordinator, uint8_t BatteryLifeExtension, uint8_t CoordRealignment, uint8_t SecurityEnable, uint32_t StartTime);
# 72 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Leds.nc"
static void GTSManagementExampleP__Leds__led1Toggle(void );
#line 89
static void GTSManagementExampleP__Leds__led2Toggle(void );
# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
static void GTSManagementExampleP__Timer_Send__startPeriodic(uint32_t dt);
# 12 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MCPS_DATA.nc"
static error_t GTSManagementExampleP__MCPS_DATA__request(uint8_t SrcAddrMode, uint16_t SrcPANId, uint32_t SrcAddr[], uint8_t DstAddrMode, uint16_t DestPANId, uint32_t DstAddr[], uint8_t msduLength, uint8_t msdu[], uint8_t msduHandle, uint8_t TxOptions);
# 11 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_SET.nc"
static error_t GTSManagementExampleP__MLME_SET__request(uint8_t PIBAttribute, uint8_t PIBAttributeValue[]);
# 12 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_GTS.nc"
static error_t GTSManagementExampleP__MLME_GTS__request(uint8_t GTSCharacteristics, uint8_t SecurityEnable);
# 53 "GTSManagementExampleP.nc"
uint8_t GTSManagementExampleP__gts_allocated = 0;

uint8_t GTSManagementExampleP__gts_superframe_count = 0;



uint32_t GTSManagementExampleP__my_short_address = 0x00000000;



static inline void GTSManagementExampleP__Boot__booted(void );
#line 80
static inline void GTSManagementExampleP__Timer0__fired(void );
#line 135
static inline void GTSManagementExampleP__Timer_Send__fired(void );
#line 170
static inline error_t GTSManagementExampleP__MLME_SCAN__confirm(uint8_t status, uint8_t ScanType, uint32_t UnscannedChannels, uint8_t ResultListSize, uint8_t EnergyDetectList[], SCAN_PANDescriptor PANDescriptorList[]);








static inline error_t GTSManagementExampleP__MLME_ORPHAN__indication(uint32_t OrphanAddress[1], uint8_t SecurityUse, uint8_t ACLEntry);
#line 197
static inline error_t GTSManagementExampleP__MLME_SYNC_LOSS__indication(uint8_t LossReason);









static error_t GTSManagementExampleP__MLME_GTS__confirm(uint8_t GTSCharacteristics, uint8_t status);
#line 236
static inline error_t GTSManagementExampleP__MLME_GTS__indication(uint16_t DevAddress, uint8_t GTSCharacteristics, bool SecurityUse, uint8_t ACLEntry);







static inline error_t GTSManagementExampleP__MLME_BEACON_NOTIFY__indication(uint8_t BSN, PANDescriptor pan_descriptor, uint8_t PenAddrSpec, uint8_t AddrList, uint8_t sduLength, uint8_t sdu[]);
#line 259
static inline error_t GTSManagementExampleP__MLME_START__confirm(uint8_t status);









static inline error_t GTSManagementExampleP__MLME_SET__confirm(uint8_t status, uint8_t PIBAttribute);
#line 289
static inline error_t GTSManagementExampleP__MLME_ASSOCIATE__indication(uint32_t DeviceAddress[], uint8_t CapabilityInformation, bool SecurityUse, uint8_t ACLEntry);





static inline error_t GTSManagementExampleP__MLME_ASSOCIATE__confirm(uint16_t AssocShortAddress, uint8_t status);







static inline error_t GTSManagementExampleP__MLME_DISASSOCIATE__indication(uint32_t DeviceAddress[], uint8_t DisassociateReason, bool SecurityUse, uint8_t ACLEntry);
#line 322
static inline error_t GTSManagementExampleP__MCPS_DATA__confirm(uint8_t msduHandle, uint8_t status);




static inline error_t GTSManagementExampleP__MCPS_DATA__indication(uint16_t SrcAddrMode, uint16_t SrcPANId, uint32_t SrcAddr[2], uint16_t DstAddrMode, uint16_t DestPANId, uint32_t DstAddr[2], uint16_t msduLength, uint8_t msdu[100], uint16_t mpduLinkQuality, uint16_t SecurityUse, uint16_t ACLEntry);
# 10 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/AddressFilter.nc"
static error_t MacP__AddressFilter__set_address(uint16_t mac_short_address, uint32_t mac_extended0, uint32_t mac_extended1);


static error_t MacP__AddressFilter__set_coord_address(uint16_t mac_coord_address, uint16_t mac_panid);
# 83 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SplitControl.nc"
static error_t MacP__AMControl__start(void );
# 62 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
static void MacP__T_ackwait__startOneShot(uint32_t dt);




static void MacP__T_ackwait__stop(void );
# 14 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/phy/PLME_SET.nc"
static error_t MacP__PLME_SET__request(uint8_t PIBAttribute, uint8_t PIBAttributeValue);
# 14 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_DISASSOCIATE.nc"
static error_t MacP__MLME_DISASSOCIATE__indication(uint32_t DeviceAddress[], uint8_t DisassociateReason, uint8_t SecurityUse, uint8_t ACLEntry);
# 41 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Random.nc"
static uint16_t MacP__Random__rand16(void );
# 15 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_START.nc"
static error_t MacP__MLME_START__confirm(uint8_t status);
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t MacP__perform_csma_ca_unslotted__postTask(void );
# 14 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_SYNC_LOSS.nc"
static error_t MacP__MLME_SYNC_LOSS__indication(uint8_t LossReason);
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t MacP__data_indication__postTask(void );
# 62 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
static void MacP__T_ResponseWaitTime__startOneShot(uint32_t dt);




static void MacP__T_ResponseWaitTime__stop(void );
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t MacP__start_coordinator_gts_send__postTask(void );
#line 56
static error_t MacP__perform_csma_ca_slotted__postTask(void );
# 32 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
static bool MacP__CCA__get(void );
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t MacP__data_channel_scan_indication__postTask(void );
# 14 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/phy/PLME_SET_TRX_STATE.nc"
static error_t MacP__PLME_SET_TRX_STATE__request(uint8_t state);
# 17 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_ASSOCIATE.nc"
static error_t MacP__MLME_ASSOCIATE__confirm(uint16_t AssocShortAddress, uint8_t status);
#line 13
static error_t MacP__MLME_ASSOCIATE__indication(uint32_t DeviceAddress[], uint8_t CapabilityInformation, bool SecurityUse, uint8_t ACLEntry);
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t MacP__signal_loss__postTask(void );
# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsync.nc"
static uint32_t MacP__TimerAsync__get_current_number_backoff_on_time_slot(void );





static uint32_t MacP__TimerAsync__get_current_ticks(void );
#line 36
static error_t MacP__TimerAsync__set_bi_sd(uint32_t bi_symbols, uint32_t sd_symbols);

static error_t MacP__TimerAsync__set_backoff_symbols(uint8_t symbols);









static error_t MacP__TimerAsync__set_timers_enable(uint8_t timer);


static uint32_t MacP__TimerAsync__get_total_tick_counter(void );
#line 14
static error_t MacP__TimerAsync__reset(void );
#line 42
static uint8_t MacP__TimerAsync__reset_start(uint32_t start_ticks);
#line 40
static error_t MacP__TimerAsync__set_enable_backoffs(bool enable_backoffs);
#line 67
static uint32_t MacP__TimerAsync__get_sd_ticks(void );
#line 10
static error_t MacP__TimerAsync__start(void );
# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Leds.nc"
static void MacP__Leds__led1On(void );




static void MacP__Leds__led1Off(void );
#line 83
static void MacP__Leds__led2Off(void );
#line 78
static void MacP__Leds__led2On(void );
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t MacP__send_frame_csma__postTask(void );
# 13 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/phy/PD_DATA.nc"
static error_t MacP__PD_DATA__request(uint8_t psduLenght, uint8_t *psdu);
# 14 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MCPS_DATA.nc"
static error_t MacP__MCPS_DATA__confirm(uint8_t msduHandle, uint8_t status);

static error_t MacP__MCPS_DATA__indication(uint16_t SrcAddrMode, uint16_t SrcPANId, uint32_t SrcAddr[2], uint16_t DstAddrMode, uint16_t DestPANId, uint32_t DstAddr[2], uint16_t msduLength, uint8_t msdu[100], uint16_t mpduLinkQuality, uint16_t SecurityUse, uint16_t ACLEntry);
# 13 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_SET.nc"
static error_t MacP__MLME_SET__confirm(uint8_t status, uint8_t PIBAttribute);
# 13 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_ORPHAN.nc"
static error_t MacP__MLME_ORPHAN__indication(uint32_t OrphanAddress[1], uint8_t SecurityUse, uint8_t ACLEntry);
# 12 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_BEACON_NOTIFY.nc"
static error_t MacP__MLME_BEACON_NOTIFY__indication(uint8_t BSN, PANDescriptor pan_descriptor, uint8_t PenAddrSpec, uint8_t AddrList, uint8_t sduLength, uint8_t sdu[]);
# 62 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
static void MacP__T_ScanDuration__startOneShot(uint32_t dt);




static void MacP__T_ScanDuration__stop(void );
# 15 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_SCAN.nc"
static error_t MacP__MLME_SCAN__confirm(uint8_t status, uint8_t ScanType, uint32_t UnscannedChannels, uint8_t ResultListSize, uint8_t EnergyDetectList[], SCAN_PANDescriptor PANDescriptorList[]);
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t MacP__start_gts_send__postTask(void );
# 14 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_GTS.nc"
static error_t MacP__MLME_GTS__confirm(uint8_t GTSCharacteristics, uint8_t status);

static error_t MacP__MLME_GTS__indication(uint16_t DevAddress, uint8_t GTSCharacteristics, uint8_t SecurityUse, uint8_t ACLEntry);
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t MacP__create_beacon__postTask(void );
# 120 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
enum MacP____nesc_unnamed4355 {
#line 120
  MacP__signal_loss = 10U
};
#line 120
typedef int MacP____nesc_sillytask_signal_loss[MacP__signal_loss];
#line 252
enum MacP____nesc_unnamed4356 {
#line 252
  MacP__start_coordinator_gts_send = 11U
};
#line 252
typedef int MacP____nesc_sillytask_start_coordinator_gts_send[MacP__start_coordinator_gts_send];








enum MacP____nesc_unnamed4357 {
#line 261
  MacP__increment_gts_null = 12U
};
#line 261
typedef int MacP____nesc_sillytask_increment_gts_null[MacP__increment_gts_null];

enum MacP____nesc_unnamed4358 {
#line 263
  MacP__start_gts_send = 13U
};
#line 263
typedef int MacP____nesc_sillytask_start_gts_send[MacP__start_gts_send];
#line 275
enum MacP____nesc_unnamed4359 {
#line 275
  MacP__check_gts_expiration = 14U
};
#line 275
typedef int MacP____nesc_sillytask_check_gts_expiration[MacP__check_gts_expiration];
#line 299
enum MacP____nesc_unnamed4360 {
#line 299
  MacP__data_channel_scan_indication = 15U
};
#line 299
typedef int MacP____nesc_sillytask_data_channel_scan_indication[MacP__data_channel_scan_indication];
#line 352
enum MacP____nesc_unnamed4361 {
#line 352
  MacP__perform_csma_ca_unslotted = 16U
};
#line 352
typedef int MacP____nesc_sillytask_perform_csma_ca_unslotted[MacP__perform_csma_ca_unslotted];
enum MacP____nesc_unnamed4362 {
#line 353
  MacP__perform_csma_ca_slotted = 17U
};
#line 353
typedef int MacP____nesc_sillytask_perform_csma_ca_slotted[MacP__perform_csma_ca_slotted];
#line 390
enum MacP____nesc_unnamed4363 {
#line 390
  MacP__data_indication = 18U
};
#line 390
typedef int MacP____nesc_sillytask_data_indication[MacP__data_indication];
#line 431
enum MacP____nesc_unnamed4364 {
#line 431
  MacP__send_frame_csma = 19U
};
#line 431
typedef int MacP____nesc_sillytask_send_frame_csma[MacP__send_frame_csma];
#line 453
enum MacP____nesc_unnamed4365 {
#line 453
  MacP__create_beacon = 20U
};
#line 453
typedef int MacP____nesc_sillytask_create_beacon[MacP__create_beacon];
#line 85
uint32_t MacP__aExtendedAddress0;
uint32_t MacP__aExtendedAddress1;

macPIB MacP__mac_PIB;



bool MacP__PANCoordinator = 0;

bool MacP__Beacon_enabled_PAN = 0;
#line 106
uint8_t MacP__trx_status;






static inline void MacP__init_MacPIB(void );

static inline uint8_t MacP__min(uint8_t val1, uint8_t val2);







static void MacP__create_data_request_cmd(void );

static inline void MacP__create_gts_request_cmd(uint8_t gts_characteristics);

static void MacP__build_ack(uint8_t sequence, uint8_t frame_pending);

static void MacP__create_data_frame(uint8_t SrcAddrMode, uint16_t SrcPANId, uint32_t SrcAddr[], uint8_t DstAddrMode, uint16_t DestPANId, uint32_t DstAddr[], uint8_t msduLength, uint8_t msdu[], uint8_t msduHandle, uint8_t TxOptions, uint8_t on_gts_slot, uint8_t pan);








uint8_t MacP__associating = 0;
uint8_t MacP__association_cmd_seq_num = 0;
#line 158
static inline void MacP__process_dissassociation_notification(MPDU *pdu);






bool MacP__TrackBeacon = 0;
bool MacP__beacon_processed = 0;

uint8_t MacP__beacon_loss_reason;


bool MacP__findabeacon = 0;

uint8_t MacP__missed_beacons = 0;

uint8_t MacP__on_sync = 0;









uint8_t MacP__gts_request = 0;
uint8_t MacP__gts_request_seq_num = 0;

bool MacP__gts_confirm;

uint8_t MacP__GTS_specification;


uint8_t MacP__final_CAP_slot = 15;


GTSinfoEntryType MacP__GTS_db[7];
uint8_t MacP__GTS_descriptor_count = 0;
uint8_t MacP__GTS_startslot = 16;
uint8_t MacP__GTS_id = 0x01;



GTSinfoEntryType_null MacP__GTS_null_db[7];

uint8_t MacP__GTS_null_descriptor_count = 0;




uint8_t MacP__s_GTSss = 0;
uint8_t MacP__s_GTS_length = 0;

uint8_t MacP__r_GTSss = 0;
uint8_t MacP__r_GTS_length = 0;


uint8_t MacP__on_s_GTS = 0;

uint8_t MacP__on_r_GTS = 0;


uint8_t MacP__next_on_s_GTS = 0;

uint8_t MacP__next_on_r_GTS = 0;


uint8_t MacP__allow_gts = 1;


gts_slot_element MacP__gts_slot_list[7];
uint8_t MacP__available_gts_index[3];
uint8_t MacP__available_gts_index_count;

uint8_t MacP__coordinator_gts_send_pending_data = 0;
uint8_t MacP__coordinator_gts_send_time_slot = 0;


MPDU MacP__gts_send_buffer[3];



uint8_t MacP__gts_send_buffer_count = 0;
uint8_t MacP__gts_send_buffer_msg_in = 0;
uint8_t MacP__gts_send_buffer_msg_out = 0;
uint8_t MacP__gts_send_pending_data = 0;




static inline void MacP__process_gts_request(MPDU *pdu);
static inline void MacP__init_available_gts_index(void );




static error_t MacP__remove_gts_entry(uint16_t DevAddressType);
static inline error_t MacP__add_gts_entry(uint8_t gts_length, bool direction, uint16_t DevAddressType);
static inline error_t MacP__add_gts_null_entry(uint8_t gts_length, bool direction, uint16_t DevAddressType);









static inline void MacP__init_gts_slot_list(void );
static inline void MacP__init_GTS_null_db(void );

static inline void MacP__init_GTS_db(void );


static inline uint32_t MacP__calculate_gts_expiration(void );







uint8_t MacP__current_channel = 0;




bool MacP__scanning_channels;


uint8_t MacP__current_scanning = 0;

uint8_t MacP__scanned_values[16];
uint8_t MacP__scan_type;

SCAN_PANDescriptor MacP__scan_pans[16];

uint16_t MacP__scan_duration;







uint32_t MacP__response_wait_time;


uint32_t MacP__BI;

uint32_t MacP__SD;


uint32_t MacP__time_slot;
uint32_t MacP__backoff;


uint8_t MacP__number_backoff = 1;
uint8_t MacP__number_time_slot = 0;

bool MacP__csma_slotted = 0;






uint8_t MacP__cca_deference = 0;
uint8_t MacP__backoff_deference = 0;
static uint8_t MacP__check_csma_ca_backoff_send_conditions(uint32_t delay_backoffs);


uint8_t MacP__delay_backoff_period;
bool MacP__csma_delay = 0;

bool MacP__csma_locate_backoff_boundary = 0;

bool MacP__csma_cca_backoff_boundary = 0;



bool MacP__performing_csma_ca = 0;


uint8_t MacP__BE;
uint8_t MacP__CW;
uint8_t MacP__NB;



static void MacP__init_csma_ca(bool slotted);
static inline void MacP__perform_csma_ca(void );









indirect_transmission_element MacP__indirect_trans_queue[2];

uint8_t MacP__indirect_trans_count = 0;




static inline void MacP__init_indirect_trans_buffer(void );

static inline void MacP__send_ind_trans_addr(uint32_t DeviceAddress[]);
#line 383
MPDU MacP__buffer_msg[4];
int MacP__current_msg_in = 0;
int MacP__current_msg_out = 0;
int MacP__buffer_count = 0;





static inline void MacP__indication_cmd(MPDU *pdu, int8_t ppduLinkQuality);
static inline void MacP__indication_ack(MPDU *pdu, int8_t ppduLinkQuality);
static inline void MacP__indication_data(MPDU *pdu, int8_t ppduLinkQuality);







MPDUBuffer MacP__send_buffer[3];
uint8_t MacP__send_buffer_count = 0;
uint8_t MacP__send_buffer_msg_in = 0;
uint8_t MacP__send_buffer_msg_out = 0;


uint8_t MacP__send_ack_check;
uint8_t MacP__retransmit_count;
uint8_t MacP__ack_sequence_number_check;
uint8_t MacP__send_retransmission;
uint8_t MacP__send_indirect_transmission;

uint8_t MacP__pending_request_data = 0;

uint8_t MacP__ackwait_period;

uint8_t MacP__link_quality;

ACK MacP__mac_ack;
ACK *MacP__mac_ack_ptr;

uint32_t MacP__gts_expiration;

uint8_t MacP__I_AM_IN_CAP = 0;
uint8_t MacP__I_AM_IN_CFP = 0;
uint8_t MacP__I_AM_IN_IP = 0;





static uint8_t MacP__check_csma_ca_send_conditions(uint8_t frame_length, uint8_t frame_control1);

static uint8_t MacP__check_gts_send_conditions(uint8_t frame_length);

static inline uint8_t MacP__calculate_ifs(uint8_t pk_length);







MPDU MacP__mac_beacon_txmpdu;
MPDU *MacP__mac_beacon_txmpdu_ptr;

uint8_t *MacP__send_beacon_frame_ptr;
uint8_t MacP__send_beacon_length;





static inline void MacP__process_beacon(MPDU *packet, uint8_t ppduLinkQuality);








static inline void MacP__create_orphan_notification(void );



static void MacP__process_coordinator_realignment(MPDU *pdu);
#line 488
static inline error_t MacP__Init__init(void );
#line 564
static inline void MacP__AMControl__startDone(error_t err);










static inline void MacP__AMControl__stopDone(error_t err);







static inline error_t MacP__TimerAsync__before_bi_fired(void );
#line 612
static inline error_t MacP__TimerAsync__bi_fired(void );
#line 661
static inline error_t MacP__TimerAsync__sd_fired(void );
#line 777
static inline error_t MacP__TimerAsync__before_time_slot_fired(void );
#line 803
static inline error_t MacP__TimerAsync__time_slot_fired(void );
#line 909
static inline error_t MacP__TimerAsync__backoff_fired(void );
#line 967
static inline void MacP__T_ackwait__fired(void );
#line 1027
static inline void MacP__T_ResponseWaitTime__fired(void );
#line 1054
static inline error_t MacP__PD_DATA__indication(uint8_t psduLenght, uint8_t *psdu, int8_t ppduLinkQuality);
#line 1102
static inline void MacP__data_indication__runTask(void );
#line 1204
static inline error_t MacP__PLME_SET__confirm(uint8_t status, uint8_t PIBAttribute);
#line 1231
static inline void MacP__process_beacon(MPDU *packet, uint8_t ppduLinkQuality);
#line 1561
static inline void MacP__process_gts_request(MPDU *pdu);
#line 1592
static inline void MacP__indication_data(MPDU *pdu, int8_t ppduLinkQuality);
#line 1935
static inline void MacP__indication_cmd(MPDU *pdu, int8_t ppduLinkQuality);
#line 2127
static inline void MacP__indication_ack(MPDU *pdu, int8_t ppduLinkQuality);
#line 2220
static inline void MacP__process_dissassociation_notification(MPDU *pdu);
#line 2237
static void MacP__process_coordinator_realignment(MPDU *pdu);
#line 2272
static inline void MacP__create_beacon__runTask(void );
#line 2482
static void MacP__create_data_frame(uint8_t SrcAddrMode, uint16_t SrcPANId, uint32_t SrcAddr[], uint8_t DstAddrMode, uint16_t DestPANId, uint32_t DstAddr[], uint8_t msduLength, uint8_t msdu[], uint8_t msduHandle, uint8_t TxOptions, uint8_t on_gts_slot, uint8_t pan);
#line 2978
static void MacP__create_data_request_cmd(void );
#line 3089
static inline void MacP__create_orphan_notification(void );
#line 3223
static inline void MacP__create_gts_request_cmd(uint8_t gts_characteristics);
#line 3351
static void MacP__build_ack(uint8_t sequence, uint8_t frame_pending);
#line 3382
static inline void MacP__data_channel_scan_indication__runTask(void );
#line 3460
static inline void MacP__T_ScanDuration__fired(void );
#line 3665
static error_t MacP__MLME_GTS__request(uint8_t GTSCharacteristics, bool security_enable);
#line 3687
static inline error_t MacP__MLME_START__request(uint32_t PANId, uint8_t LogicalChannel, uint8_t beacon_order, uint8_t superframe_order, bool pan_coodinator, bool BatteryLifeExtension, bool CoordRealignment, bool securityenable, uint32_t StartTime);
#line 3909
static error_t MacP__MLME_SET__request(uint8_t PIBAttribute, uint8_t PIBAttributeValue[]);
#line 4065
static error_t MacP__MCPS_DATA__request(uint8_t SrcAddrMode, uint16_t SrcPANId, uint32_t SrcAddr[], uint8_t DstAddrMode, uint16_t DestPANId, uint32_t DstAddr[], uint8_t msduLength, uint8_t msdu[], uint8_t msduHandle, uint8_t TxOptions);
#line 4206
static inline void MacP__signal_loss__runTask(void );
#line 4251
static inline void MacP__init_MacPIB(void );
#line 4329
static inline void MacP__send_frame_csma__runTask(void );
#line 4356
static inline void MacP__perform_csma_ca_slotted__runTask(void );
#line 4515
static inline void MacP__perform_csma_ca_unslotted__runTask(void );
#line 4593
static inline void MacP__perform_csma_ca(void );
#line 4644
static inline uint8_t MacP__min(uint8_t val1, uint8_t val2);
#line 4656
static void MacP__init_csma_ca(bool slotted);
#line 4682
static inline uint8_t MacP__calculate_ifs(uint8_t pk_length);







static inline uint32_t MacP__calculate_gts_expiration(void );
#line 4707
static uint8_t MacP__check_csma_ca_send_conditions(uint8_t frame_length, uint8_t frame_control1);
#line 4737
static uint8_t MacP__check_csma_ca_backoff_send_conditions(uint32_t delay_backoffs);
#line 4762
static uint8_t MacP__check_gts_send_conditions(uint8_t frame_length);
#line 4791
static inline void MacP__init_GTS_db(void );
#line 4809
static error_t MacP__remove_gts_entry(uint16_t DevAddressType);
#line 4864
static inline error_t MacP__add_gts_entry(uint8_t gts_length, bool direction, uint16_t DevAddressType);
#line 4926
static inline void MacP__init_GTS_null_db(void );
#line 4945
static inline error_t MacP__add_gts_null_entry(uint8_t gts_length, bool direction, uint16_t DevAddressType);
#line 4984
static inline void MacP__increment_gts_null__runTask(void );
#line 5020
static inline void MacP__check_gts_expiration__runTask(void );
#line 5059
static inline void MacP__init_available_gts_index(void );
#line 5072
static inline void MacP__init_gts_slot_list(void );
#line 5084
static inline void MacP__start_coordinator_gts_send__runTask(void );
#line 5119
static inline void MacP__start_gts_send__runTask(void );
#line 5155
static inline void MacP__init_indirect_trans_buffer(void );
#line 5211
static inline void MacP__send_ind_trans_addr(uint32_t DeviceAddress[]);
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SplitControl.nc"
static void PhyP__SplitControl__startDone(error_t error);
#line 117
static void PhyP__SplitControl__stopDone(error_t error);
# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420Config.nc"
static void PhyP__CC2420Config__setChannel(uint8_t channel);
#line 52
static error_t PhyP__CC2420Config__sync(void );
# 16 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/phy/PLME_SET.nc"
static error_t PhyP__PLME_SET__confirm(uint8_t status, uint8_t PIBAttribute);
# 16 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/Sendframe.nc"
static error_t PhyP__Sendframe__send(uint8_t *frame, uint8_t frame_length);
# 74 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/StdControl.nc"
static error_t PhyP__SubControl__start(void );
# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Power.nc"
static error_t PhyP__CC2420Power__startOscillator(void );
#line 90
static error_t PhyP__CC2420Power__rxOn(void );
#line 51
static error_t PhyP__CC2420Power__startVReg(void );
# 17 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/phy/PD_DATA.nc"
static error_t PhyP__PD_DATA__indication(uint8_t psduLenght, uint8_t *psdu, int8_t ppduLinkQuality);
# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t PhyP__Resource__release(void );
#line 78
static error_t PhyP__Resource__request(void );
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t PhyP__sendDone_task__postTask(void );
#line 56
static error_t PhyP__startDone_task__postTask(void );
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/phy/PhyP.nc"
enum PhyP____nesc_unnamed4366 {
#line 67
  PhyP__startDone_task = 21U
};
#line 67
typedef int PhyP____nesc_sillytask_startDone_task[PhyP__startDone_task];

enum PhyP____nesc_unnamed4367 {
#line 69
  PhyP__stopDone_task = 22U
};
#line 69
typedef int PhyP____nesc_sillytask_stopDone_task[PhyP__stopDone_task];
enum PhyP____nesc_unnamed4368 {
#line 70
  PhyP__sendDone_task = 23U
};
#line 70
typedef int PhyP____nesc_sillytask_sendDone_task[PhyP__sendDone_task];
#line 48
phyPIB PhyP__phy_PIB;







MPDU *PhyP__rxmpdu_ptr;


error_t PhyP__sendErr = SUCCESS;
#line 77
static inline error_t PhyP__Init__init(void );
#line 96
static inline error_t PhyP__SplitControl__start(void );
#line 116
static inline void PhyP__Sendframe__sendDone(error_t error);
#line 128
static inline void PhyP__CC2420Power__startVRegDone(void );




static inline void PhyP__Resource__granted(void );



static inline void PhyP__CC2420Power__startOscillatorDone(void );






static inline void PhyP__sendDone_task__runTask(void );






static inline void PhyP__startDone_task__runTask(void );







static inline void PhyP__stopDone_task__runTask(void );
#line 184
static inline void PhyP__Receiveframe__receive(uint8_t *frame, uint8_t rssi);
#line 210
static inline void PhyP__CC2420Config__syncDone(error_t error);
#line 224
static inline error_t PhyP__PD_DATA__request(uint8_t psduLength, uint8_t *psdu);
#line 291
static error_t PhyP__PLME_SET__request(uint8_t PIBAttribute, uint8_t PIBAttributeValue);
#line 335
static inline error_t PhyP__PLME_SET_TRX_STATE__request(uint8_t state);
# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420Config.nc"
static void CC2420ControlP__CC2420Config__syncDone(error_t error);
# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
static void CC2420ControlP__StartupTimer__start(CC2420ControlP__StartupTimer__size_type dt);
# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__MDMCTRL0__write(uint16_t data);
# 35 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
static void CC2420ControlP__RSTN__makeOutput(void );
#line 29
static void CC2420ControlP__RSTN__set(void );
static void CC2420ControlP__RSTN__clr(void );
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t CC2420ControlP__syncDone__postTask(void );
# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__RSSI__read(uint16_t *data);







static cc2420_status_t CC2420ControlP__IOCFG0__write(uint16_t data);
# 35 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
static void CC2420ControlP__CSN__makeOutput(void );
#line 29
static void CC2420ControlP__CSN__set(void );
static void CC2420ControlP__CSN__clr(void );




static void CC2420ControlP__VREN__makeOutput(void );
#line 29
static void CC2420ControlP__VREN__set(void );
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SXOSCON__strobe(void );
# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t CC2420ControlP__SpiResource__release(void );
#line 78
static error_t CC2420ControlP__SpiResource__request(void );
#line 110
static error_t CC2420ControlP__SyncResource__release(void );
#line 78
static error_t CC2420ControlP__SyncResource__request(void );
# 76 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Power.nc"
static void CC2420ControlP__CC2420Power__startOscillatorDone(void );
#line 56
static void CC2420ControlP__CC2420Power__startVRegDone(void );
# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__IOCFG1__write(uint16_t data);
#line 55
static cc2420_status_t CC2420ControlP__FSCTRL__write(uint16_t data);
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SRXON__strobe(void );
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static void CC2420ControlP__Resource__granted(void );
# 63 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420ControlP__PANID__write(uint8_t offset, uint8_t * data, uint8_t length);
# 50 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GpioInterrupt.nc"
static error_t CC2420ControlP__InterruptCCA__disable(void );
#line 42
static error_t CC2420ControlP__InterruptCCA__enableRisingEdge(void );
# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t CC2420ControlP__RssiResource__release(void );
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SRFOFF__strobe(void );
# 114 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ControlP.nc"
enum CC2420ControlP____nesc_unnamed4369 {
#line 114
  CC2420ControlP__sync = 24U
};
#line 114
typedef int CC2420ControlP____nesc_sillytask_sync[CC2420ControlP__sync];
enum CC2420ControlP____nesc_unnamed4370 {
#line 115
  CC2420ControlP__syncDone = 25U
};
#line 115
typedef int CC2420ControlP____nesc_sillytask_syncDone[CC2420ControlP__syncDone];
#line 88
#line 82
typedef enum CC2420ControlP____nesc_unnamed4371 {
  CC2420ControlP__S_VREG_STOPPED, 
  CC2420ControlP__S_VREG_STARTING, 
  CC2420ControlP__S_VREG_STARTED, 
  CC2420ControlP__S_XOSC_STARTING, 
  CC2420ControlP__S_XOSC_STARTED
} CC2420ControlP__cc2420_control_state_t;

uint8_t CC2420ControlP__m_channel;

uint8_t CC2420ControlP__m_tx_power;

uint16_t CC2420ControlP__m_pan;

uint16_t CC2420ControlP__m_short_addr;

bool CC2420ControlP__m_sync_busy;

bool CC2420ControlP__autoAckEnabled;

bool CC2420ControlP__hwAutoAckDefault;

bool CC2420ControlP__addressRecognition;

CC2420ControlP__cc2420_control_state_t CC2420ControlP__m_state = CC2420ControlP__S_VREG_STOPPED;



static void CC2420ControlP__writeFsctrl(void );
static void CC2420ControlP__writeMdmctrl0(void );
static void CC2420ControlP__writeId(void );





static inline error_t CC2420ControlP__Init__init(void );
#line 161
static inline error_t CC2420ControlP__Resource__request(void );







static inline error_t CC2420ControlP__Resource__release(void );







static error_t CC2420ControlP__CC2420Power__startVReg(void );
#line 197
static inline error_t CC2420ControlP__CC2420Power__startOscillator(void );
#line 259
static inline error_t CC2420ControlP__CC2420Power__rxOn(void );
#line 285
static inline void CC2420ControlP__CC2420Config__setChannel(uint8_t channel);
#line 311
static error_t CC2420ControlP__CC2420Config__sync(void );
#line 374
static inline void CC2420ControlP__SyncResource__granted(void );
#line 392
static inline void CC2420ControlP__SpiResource__granted(void );




static inline void CC2420ControlP__RssiResource__granted(void );
#line 412
static inline void CC2420ControlP__StartupTimer__fired(void );









static inline void CC2420ControlP__InterruptCCA__fired(void );
#line 447
static inline void CC2420ControlP__sync__runTask(void );



static inline void CC2420ControlP__syncDone__runTask(void );









static void CC2420ControlP__writeFsctrl(void );
#line 479
static void CC2420ControlP__writeMdmctrl0(void );
#line 498
static void CC2420ControlP__writeId(void );
# 30 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time);

static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta);
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void );
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void );
# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void );
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void );
#line 33
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );










static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Counter.nc"
static /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__get(void );






static bool /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void );










static void /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__overflow(void );
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/TransformCounterC.nc"
/*Counter32khz32C.Transform*/TransformCounterC__0__upper_count_type /*Counter32khz32C.Transform*/TransformCounterC__0__m_upper;

enum /*Counter32khz32C.Transform*/TransformCounterC__0____nesc_unnamed4372 {

  TransformCounterC__0__LOW_SHIFT_RIGHT = 0, 
  TransformCounterC__0__HIGH_SHIFT_LEFT = 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type ) - /*Counter32khz32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT, 
  TransformCounterC__0__NUM_UPPER_BITS = 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type ) - 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type ) + 0, 



  TransformCounterC__0__OVERFLOW_MASK = /*Counter32khz32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS ? ((/*Counter32khz32C.Transform*/TransformCounterC__0__upper_count_type )2 << (/*Counter32khz32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get(void );
#line 122
static inline void /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__fired(void );
#line 92
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt);
# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Counter.nc"
static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__get(void );
# 66 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/TransformAlarmC.nc"
/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0;
/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt;

enum /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0____nesc_unnamed4373 {

  TransformAlarmC__0__MAX_DELAY_LOG2 = 8 * sizeof(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type ) - 1 - 0, 
  TransformAlarmC__0__MAX_DELAY = (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type )1 << /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY_LOG2
};

static inline /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );
#line 96
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__set_alarm(void );
#line 136
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type dt);









static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type dt);




static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
#line 166
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__makeInput(void );
#line 59
static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__get(void );
# 40 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__get(void );
static inline void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput(void );
# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__makeOutput(void );
#line 34
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__set(void );




static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__clr(void );
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set(void );
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr(void );




static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput(void );
# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__HplGeneralIO__get(void );
# 40 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__GeneralIO__get(void );
# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__get(void );
# 40 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__GeneralIO__get(void );
# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__makeOutput(void );
#line 34
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__set(void );




static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__clr(void );
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__set(void );
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__clr(void );




static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput(void );
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__makeInput(void );
# 41 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput(void );
# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__makeOutput(void );
#line 34
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__set(void );
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__set(void );





static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput(void );
# 57 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow(void );
# 50 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GpioCapture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captured(uint16_t time);
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(uint8_t cm);

static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__enableEvents(void );
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents(void );
#line 33
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 78 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectModuleFunc(void );
# 38 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(uint8_t mode);
#line 50
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge(void );
#line 65
static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time);
# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
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
# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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
static inline void HplMsp430InterruptP__Port27__default__fired(void );
#line 195
static inline void HplMsp430InterruptP__Port20__clear(void );
static inline void HplMsp430InterruptP__Port21__clear(void );
static inline void HplMsp430InterruptP__Port22__clear(void );
static inline void HplMsp430InterruptP__Port23__clear(void );
static inline void HplMsp430InterruptP__Port24__clear(void );
static inline void HplMsp430InterruptP__Port25__clear(void );
static inline void HplMsp430InterruptP__Port26__clear(void );
static inline void HplMsp430InterruptP__Port27__clear(void );
# 41 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear(void );
#line 36
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__disable(void );
#line 56
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__edge(bool low_to_high);
#line 31
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__enable(void );
# 57 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GpioInterrupt.nc"
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__fired(void );
# 41 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__enable(bool rising);








static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void );







static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable(void );







static inline void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired(void );
# 41 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear(void );
#line 36
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__disable(void );
#line 56
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__edge(bool low_to_high);
#line 31
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__enable(void );
# 57 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GpioInterrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__fired(void );
# 41 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__enable(bool rising);
#line 54
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge(void );



static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable(void );







static inline void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired(void );
# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SpiPacket.nc"
static error_t CC2420SpiP__SpiPacket__send(
#line 48
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
# 91 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420SpiP__Fifo__writeDone(
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
uint8_t arg_0x41118698, 
# 91 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420SpiP__Fifo__readDone(
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
uint8_t arg_0x41118698, 
# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
# 24 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420SpiP__ChipSpiResource__releasing(void );
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SpiByte.nc"
static uint8_t CC2420SpiP__SpiByte__write(uint8_t tx);
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/State.nc"
static void CC2420SpiP__WorkingState__toIdle(void );




static bool CC2420SpiP__WorkingState__isIdle(void );
#line 45
static error_t CC2420SpiP__WorkingState__requestState(uint8_t reqState);
# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__SpiResource__release(void );
#line 87
static error_t CC2420SpiP__SpiResource__immediateRequest(void );
#line 78
static error_t CC2420SpiP__SpiResource__request(void );
#line 118
static bool CC2420SpiP__SpiResource__isOwner(void );
#line 92
static void CC2420SpiP__Resource__granted(
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
uint8_t arg_0x410deb90);
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t CC2420SpiP__grant__postTask(void );
# 86 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
enum CC2420SpiP____nesc_unnamed4374 {
#line 86
  CC2420SpiP__grant = 26U
};
#line 86
typedef int CC2420SpiP____nesc_sillytask_grant[CC2420SpiP__grant];
#line 61
enum CC2420SpiP____nesc_unnamed4375 {
  CC2420SpiP__RESOURCE_COUNT = 5U, 
  CC2420SpiP__NO_HOLDER = 0xFF
};


enum CC2420SpiP____nesc_unnamed4376 {
  CC2420SpiP__S_IDLE, 
  CC2420SpiP__S_BUSY
};


uint16_t CC2420SpiP__m_addr;


uint8_t CC2420SpiP__m_requests = 0;


uint8_t CC2420SpiP__m_holder = CC2420SpiP__NO_HOLDER;


bool CC2420SpiP__release;


static error_t CC2420SpiP__attemptRelease(void );
#line 100
static inline error_t CC2420SpiP__ChipSpiResource__attemptRelease(void );




static error_t CC2420SpiP__Resource__request(uint8_t id);
#line 124
static error_t CC2420SpiP__Resource__immediateRequest(uint8_t id);
#line 147
static error_t CC2420SpiP__Resource__release(uint8_t id);
#line 176
static inline uint8_t CC2420SpiP__Resource__isOwner(uint8_t id);





static inline void CC2420SpiP__SpiResource__granted(void );




static inline cc2420_status_t CC2420SpiP__Fifo__beginRead(uint8_t addr, uint8_t *data, 
uint8_t len);
#line 207
static inline error_t CC2420SpiP__Fifo__continueRead(uint8_t addr, uint8_t *data, 
uint8_t len);



static inline cc2420_status_t CC2420SpiP__Fifo__write(uint8_t addr, uint8_t *data, 
uint8_t len);
#line 258
static inline cc2420_status_t CC2420SpiP__Ram__write(uint16_t addr, uint8_t offset, 
uint8_t *data, 
uint8_t len);
#line 283
static inline cc2420_status_t CC2420SpiP__Reg__read(uint8_t addr, uint16_t *data);
#line 301
static cc2420_status_t CC2420SpiP__Reg__write(uint8_t addr, uint16_t data);
#line 314
static cc2420_status_t CC2420SpiP__Strobe__strobe(uint8_t addr);










static void CC2420SpiP__SpiPacket__sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error);








static error_t CC2420SpiP__attemptRelease(void );
#line 356
static inline void CC2420SpiP__grant__runTask(void );








static inline void CC2420SpiP__Resource__default__granted(uint8_t id);


static inline void CC2420SpiP__Fifo__default__readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error);


static inline void CC2420SpiP__Fifo__default__writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error);
# 74 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/StateImplP.nc"
uint8_t StateImplP__state[1U];

enum StateImplP____nesc_unnamed4377 {
  StateImplP__S_IDLE = 0
};


static inline error_t StateImplP__Init__init(void );
#line 96
static error_t StateImplP__State__requestState(uint8_t id, uint8_t reqState);
#line 118
static inline void StateImplP__State__toIdle(uint8_t id);







static inline bool StateImplP__State__isIdle(uint8_t id);






static inline bool StateImplP__State__isState(uint8_t id, uint8_t myState);
# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SpiPacket.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(
# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x41170ee8, 
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(
# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x4116e1b8);
# 180 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__enableRxIntr(void );
#line 197
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__clrRxIntr(void );
#line 97
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(bool reset);
#line 177
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr(void );
#line 224
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(uint8_t data);
#line 168
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 231
static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx(void );
#line 192
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending(void );
#line 158
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableSpi(void );
# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(
# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x4116f718);
# 87 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__immediateRequest(
# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x4116f718);
# 78 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(
# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x4116f718);
# 118 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__isOwner(
# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x4116f718);
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(
# 41 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x41173920);
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__postTask(void );
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
enum /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0____nesc_unnamed4378 {
#line 67
  Msp430SpiNoDmaP__0__signalDone_task = 27U
};
#line 67
typedef int /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0____nesc_sillytask_signalDone_task[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task];
#line 56
enum /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0____nesc_unnamed4379 {
  Msp430SpiNoDmaP__0__SPI_ATOMIC_SIZE = 2
};

uint16_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len;
uint8_t * /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf;
uint8_t * /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf;
uint16_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos;
uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_client;

static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone(void );


static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(uint8_t id);



static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(uint8_t id);



static inline uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(uint8_t id);



static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(uint8_t id);



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(uint8_t id);



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(uint8_t id);





static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(uint8_t id);



static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(uint8_t tx);
#line 111
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(uint8_t id);
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(uint8_t id);
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(uint8_t id);
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(uint8_t id);
static inline msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(uint8_t id);



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(uint8_t id);

static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp(void );
#line 144
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len);
#line 166
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void );



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data);
#line 183
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone(void );




static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void );

static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error);
# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__UCLK__selectIOFunc(void );
#line 78
static void HplMsp430Usart0P__UCLK__selectModuleFunc(void );
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void HplMsp430Usart0P__Interrupts__rxDone(uint8_t data);
#line 49
static void HplMsp430Usart0P__Interrupts__txDone(void );
# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__URXD__selectIOFunc(void );
#line 85
static void HplMsp430Usart0P__UTXD__selectIOFunc(void );
# 7 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430I2C.nc"
static void HplMsp430Usart0P__HplI2C__clearModeI2C(void );
#line 6
static bool HplMsp430Usart0P__HplI2C__isI2C(void );
# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__SOMI__selectIOFunc(void );
#line 78
static void HplMsp430Usart0P__SOMI__selectModuleFunc(void );
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void HplMsp430Usart0P__I2CInterrupts__fired(void );
# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__SIMO__selectIOFunc(void );
#line 78
static void HplMsp430Usart0P__SIMO__selectModuleFunc(void );
# 89 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
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



static inline void HplMsp430Usart0P__Usart__disableRxIntr(void );







static inline void HplMsp430Usart0P__Usart__disableIntr(void );



static inline void HplMsp430Usart0P__Usart__enableRxIntr(void );
#line 382
static inline void HplMsp430Usart0P__Usart__tx(uint8_t data);



static uint8_t HplMsp430Usart0P__Usart__rx(void );
# 80 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__userId(void );
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__rxDone(
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40ad3f28, 
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__txDone(
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40ad3f28);
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__I2CInterrupts__fired(
# 40 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40acff08);








static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__txDone(void );




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__rxDone(uint8_t data);




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawI2CInterrupts__fired(void );




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__txDone(uint8_t id);
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__rxDone(uint8_t id, uint8_t data);
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__I2CInterrupts__default__fired(uint8_t id);
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1____nesc_unnamed4380 {
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
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__requested(
# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40af5948);
# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__immediateRequested(
# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40af5948);
# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(
# 60 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40af4cf8);
# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(
# 60 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40af4cf8);
# 69 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__enqueue(resource_client_id_t id);
#line 43
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__isEmpty(void );
#line 60
static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__dequeue(void );
# 73 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested(void );
#line 46
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted(void );
#line 81
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__immediateRequested(void );
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__granted(
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
uint8_t arg_0x40ae1e98);
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask(void );
# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4381 {
#line 75
  ArbiterP__1__grantedTask = 28U
};
#line 75
typedef int /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_sillytask_grantedTask[/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask];
#line 67
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4382 {
#line 67
  ArbiterP__1__RES_CONTROLLED, ArbiterP__1__RES_GRANTING, ArbiterP__1__RES_IMM_GRANTING, ArbiterP__1__RES_BUSY
};
#line 68
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4383 {
#line 68
  ArbiterP__1__default_owner_id = 1U
};
#line 69
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4384 {
#line 69
  ArbiterP__1__NO_RES = 0xFF
};
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED;
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__default_owner_id;
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId;



static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__request(uint8_t id);
#line 93
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(uint8_t id);
#line 111
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(uint8_t id);
#line 133
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void );
#line 153
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse(void );
#line 166
static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId(void );










static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(uint8_t id);
#line 190
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask(void );
#line 202
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__granted(void );

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__requested(void );


static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__immediateRequested(void );


static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(uint8_t id);
# 97 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430I2C0P__HplUsart__resetUsart(bool reset);
# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static volatile uint8_t HplMsp430I2C0P__U0CTL __asm ("0x0070");





static inline bool HplMsp430I2C0P__HplI2C__isI2C(void );



static inline void HplMsp430I2C0P__HplI2C__clearModeI2C(void );
# 42 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GpioCapture.nc"
static error_t CC2420TransmitP__CaptureSFD__captureRisingEdge(void );
# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420TransmitP__TXCTRL__write(uint16_t data);
# 21 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/Sendframe.nc"
static void CC2420TransmitP__Sendframe__sendDone(error_t error);
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static error_t CC2420TransmitP__ChipSpiResource__attemptRelease(void );
# 35 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP__CSN__makeOutput(void );
#line 29
static void CC2420TransmitP__CSN__set(void );
static void CC2420TransmitP__CSN__clr(void );
# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t CC2420TransmitP__SpiResource__release(void );
#line 87
static error_t CC2420TransmitP__SpiResource__immediateRequest(void );
#line 78
static error_t CC2420TransmitP__SpiResource__request(void );
# 33 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP__CCA__makeInput(void );
#line 33
static void CC2420TransmitP__SFD__makeInput(void );
# 82 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static cc2420_status_t CC2420TransmitP__TXFIFO__write(uint8_t * data, uint8_t length);
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__STXON__strobe(void );
# 77 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420TransmitP.nc"
static void CC2420TransmitP__attemptSend(void );

static inline error_t CC2420TransmitP__acquireSpiResource(void );
static inline error_t CC2420TransmitP__releaseSpiResource(void );
static inline void CC2420TransmitP__signalDone(error_t err);



static inline error_t CC2420TransmitP__Init__init(void );







static inline error_t CC2420TransmitP__StdControl__start(void );
#line 114
static error_t CC2420TransmitP__Sendframe__send(uint8_t *frame, uint8_t frame_length);
#line 158
static inline void CC2420TransmitP__CaptureSFD__captured(uint16_t time);




static inline void CC2420TransmitP__ChipSpiResource__releasing(void );




static inline void CC2420TransmitP__SpiResource__granted(void );










static inline void CC2420TransmitP__TXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error);








static inline void CC2420TransmitP__TXFIFO__readDone(uint8_t *tx_buf, uint8_t tx_len, error_t error);
#line 208
static void CC2420TransmitP__attemptSend(void );
#line 231
static inline error_t CC2420TransmitP__acquireSpiResource(void );









static inline error_t CC2420TransmitP__releaseSpiResource(void );





static inline void CC2420TransmitP__signalDone(error_t err);
# 14 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/Receiveframe.nc"
static void CC2420ReceiveP__Receiveframe__receive(uint8_t *frame, uint8_t rssi);
# 32 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
static bool CC2420ReceiveP__FIFO__get(void );
#line 32
static bool CC2420ReceiveP__FIFOP__get(void );
#line 29
static void CC2420ReceiveP__CSN__set(void );
static void CC2420ReceiveP__CSN__clr(void );
# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
static error_t CC2420ReceiveP__SpiResource__release(void );
#line 87
static error_t CC2420ReceiveP__SpiResource__immediateRequest(void );
#line 78
static error_t CC2420ReceiveP__SpiResource__request(void );
#line 118
static bool CC2420ReceiveP__SpiResource__isOwner(void );
# 62 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static error_t CC2420ReceiveP__RXFIFO__continueRead(uint8_t * data, uint8_t length);
#line 51
static cc2420_status_t CC2420ReceiveP__RXFIFO__beginRead(uint8_t * data, uint8_t length);
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GpioInterrupt.nc"
static error_t CC2420ReceiveP__InterruptFIFOP__enableFallingEdge(void );
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ReceiveP__SFLUSHRX__strobe(void );
# 88 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ReceiveP.nc"
#line 80
typedef enum CC2420ReceiveP____nesc_unnamed4385 {
  CC2420ReceiveP__S_STOPPED = 0, 
  CC2420ReceiveP__S_STARTED = 1, 
  CC2420ReceiveP__S_RX_LENGTH = 2, 
  CC2420ReceiveP__S_RX_FC = 3, 
  CC2420ReceiveP__S_RX_ADDR = 4, 
  CC2420ReceiveP__S_RX_PAYLOAD = 5, 
  CC2420ReceiveP__S_RX_DISCARD = 6
} CC2420ReceiveP__cc2420_receive_state_t;
#line 100
enum CC2420ReceiveP____nesc_unnamed4386 {
  CC2420ReceiveP__RXFIFO_SIZE = 128, 
  CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE = 8, 
  CC2420ReceiveP__SACK_HEADER_LENGTH = 7
};








uint8_t CC2420ReceiveP__m_missed_packets;


bool CC2420ReceiveP__receivingPacket;


uint8_t CC2420ReceiveP__rxFrameLength;


uint8_t CC2420ReceiveP__m_bytes_left;









MPDU CC2420ReceiveP__rxmpdu;
MPDU *CC2420ReceiveP__rxmpdu_ptr;


CC2420ReceiveP__cc2420_receive_state_t CC2420ReceiveP__m_state;


uint8_t CC2420ReceiveP__rssi;
#line 152
uint8_t CC2420ReceiveP__destination_address = 0;


dest_short *CC2420ReceiveP__dest_short_ptr;
dest_long *CC2420ReceiveP__dest_long_ptr;




beacon_addr_short *CC2420ReceiveP__beacon_addr_short_ptr;

uint8_t CC2420ReceiveP__address_decode = 1;


uint16_t CC2420ReceiveP__ver_macCoordShortAddress = 0x0000;
uint16_t CC2420ReceiveP__ver_macShortAddress = 0xffff;

uint32_t CC2420ReceiveP__ver_aExtendedAddress0 = 0x00000000;
uint32_t CC2420ReceiveP__ver_aExtendedAddress1 = 0x00000000;

uint16_t CC2420ReceiveP__ver_macPANId = 0xffff;




static inline void CC2420ReceiveP__reset_state(void );
static void CC2420ReceiveP__beginReceive(void );
static void CC2420ReceiveP__receive(void );
static void CC2420ReceiveP__waitForNextPacket(void );
static void CC2420ReceiveP__flush(void );




static inline error_t CC2420ReceiveP__Init__init(void );










static inline error_t CC2420ReceiveP__StdControl__start(void );
#line 224
static inline error_t CC2420ReceiveP__AddressFilter__set_address(uint16_t mac_short_address, uint32_t mac_extended0, uint32_t mac_extended1);
#line 241
static inline error_t CC2420ReceiveP__AddressFilter__set_coord_address(uint16_t mac_coord_address, uint16_t mac_panid);
#line 319
static inline void CC2420ReceiveP__InterruptFIFOP__fired(void );
#line 351
static inline void CC2420ReceiveP__SpiResource__granted(void );








static inline void CC2420ReceiveP__RXFIFO__readDone(uint8_t *rx_buf, uint8_t rx_len, error_t error);
#line 863
static inline void CC2420ReceiveP__RXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error);
#line 887
static inline void CC2420ReceiveP__CC2420Config__syncDone(error_t error);







static void CC2420ReceiveP__beginReceive(void );
#line 917
static void CC2420ReceiveP__flush(void );
#line 935
static void CC2420ReceiveP__receive(void );
#line 958
static void CC2420ReceiveP__waitForNextPacket(void );
#line 997
static inline void CC2420ReceiveP__reset_state(void );
# 41 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/RandomMlcgC.nc"
uint32_t RandomMlcgC__seed;


static inline error_t RandomMlcgC__Init__init(void );
#line 58
static uint32_t RandomMlcgC__Random__rand32(void );
#line 78
static inline uint16_t RandomMlcgC__Random__rand16(void );
# 22 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsync.nc"
static error_t TimerAsyncM__TimerAsync__bi_fired(void );


static error_t TimerAsyncM__TimerAsync__backoff_fired(void );


static error_t TimerAsyncM__TimerAsync__time_slot_fired(void );

static error_t TimerAsyncM__TimerAsync__before_time_slot_fired(void );
#line 20
static error_t TimerAsyncM__TimerAsync__sd_fired(void );
#line 18
static error_t TimerAsyncM__TimerAsync__before_bi_fired(void );
# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
static void TimerAsyncM__AsyncTimer__start(TimerAsyncM__AsyncTimer__size_type dt);
# 32 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsyncM.nc"
uint32_t TimerAsyncM__ticks_counter;


uint32_t TimerAsyncM__bi_ticks;
uint32_t TimerAsyncM__bi_backoff_periods;
uint32_t TimerAsyncM__before_bi_ticks;
uint32_t TimerAsyncM__sd_ticks;


uint32_t TimerAsyncM__time_slot_backoff_periods;


uint32_t TimerAsyncM__time_slot_ticks;
uint32_t TimerAsyncM__before_time_slot_ticks;
uint32_t TimerAsyncM__time_slot_tick_next_fire;


uint32_t TimerAsyncM__backoff_symbols;


uint32_t TimerAsyncM__backoff_ticks = 5;


uint32_t TimerAsyncM__backoff_ticks_counter = 0;


uint8_t TimerAsyncM__current_time_slot = 0;

uint32_t TimerAsyncM__current_number_backoff_on_time_slot = 0;

uint32_t TimerAsyncM__current_number_backoff = 0;


bool TimerAsyncM__backoffs = 0;
bool TimerAsyncM__enable_backoffs = 0;




uint32_t TimerAsyncM__process_frame_tick_counter = 0;

uint32_t TimerAsyncM__total_tick_counter = 0;

uint8_t TimerAsyncM__timers_enable = 0x01;


static error_t TimerAsyncM__TimerAsync__start(void );
#line 95
static inline error_t TimerAsyncM__TimerAsync__reset(void );






static error_t TimerAsyncM__TimerAsync__set_bi_sd(uint32_t bi_symbols, uint32_t sd_symbols);
#line 133
static error_t TimerAsyncM__TimerAsync__set_backoff_symbols(uint8_t Backoff_Duration_Symbols);
#line 146
static inline error_t TimerAsyncM__TimerAsync__set_enable_backoffs(bool enable);







static inline void TimerAsyncM__AsyncTimer__fired(void );
#line 233
static inline error_t TimerAsyncM__TimerAsync__set_timers_enable(uint8_t timer);
#line 254
static inline uint8_t TimerAsyncM__TimerAsync__reset_start(uint32_t start_ticks);
#line 301
static inline uint32_t TimerAsyncM__TimerAsync__get_current_ticks(void );




static inline uint32_t TimerAsyncM__TimerAsync__get_sd_ticks(void );
#line 343
static inline uint32_t TimerAsyncM__TimerAsync__get_current_number_backoff_on_time_slot(void );






static inline uint32_t TimerAsyncM__TimerAsync__get_total_tick_counter(void );
# 30 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time);

static void /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta);
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void );
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
static void /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void );
# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void );
static void /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void );
#line 33
static void /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void );
# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );










static inline void /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
static void /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Alarm__fired(void );
#line 92
static void /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__startAt(/*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__size_type t0, /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__size_type dt);
# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Counter.nc"
static /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Counter__size_type /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Counter__get(void );
# 66 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/TransformAlarmC.nc"
/*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__to_size_type /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__m_t0;
/*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__to_size_type /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__m_dt;

enum /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1____nesc_unnamed4387 {

  TransformAlarmC__1__MAX_DELAY_LOG2 = 8 * sizeof(/*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__from_size_type ) - 1 - 0, 
  TransformAlarmC__1__MAX_DELAY = (/*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__to_size_type )1 << /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__MAX_DELAY_LOG2
};

static inline /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__to_size_type /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Alarm__getNow(void );
#line 96
static void /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__set_alarm(void );
#line 136
static void /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Alarm__startAt(/*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__to_size_type t0, /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__to_size_type dt);









static inline void /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Alarm__start(/*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__to_size_type dt);




static inline void /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__fired(void );
#line 166
static inline void /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Counter__overflow(void );
# 30 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEvent(uint16_t time);

static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEventFromNow(uint16_t delta);
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__get(void );
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__fired(void );
# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__enableEvents(void );
#line 36
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__setControlAsCompare(void );










static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__disableEvents(void );
#line 33
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__clearPendingInterrupt(void );
# 42 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Init__init(void );
#line 54
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__stop(void );




static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow(void );
# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Counter.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__get(void );






static bool /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__overflow(void );
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/TransformCounterC.nc"
/*CounterMilli32C.Transform*/TransformCounterC__1__upper_count_type /*CounterMilli32C.Transform*/TransformCounterC__1__m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC__1____nesc_unnamed4388 {

  TransformCounterC__1__LOW_SHIFT_RIGHT = 5, 
  TransformCounterC__1__HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC__1__LOW_SHIFT_RIGHT, 
  TransformCounterC__1__NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type ) + 5, 



  TransformCounterC__1__OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC__1__NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC__1__upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC__1__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__get(void );
#line 122
static inline void /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void );
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__fired(void );
#line 92
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__AlarmFrom__size_type dt);
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__AlarmFrom__stop(void );
# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Counter__get(void );
# 66 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2____nesc_unnamed4389 {

  TransformAlarmC__2__MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__from_size_type ) - 1 - 5, 
  TransformAlarmC__2__MAX_DELAY = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__to_size_type )1 << /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__MAX_DELAY_LOG2
};

static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__getNow(void );




static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__getAlarm(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__set_alarm(void );
#line 136
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__to_size_type dt);
#line 151
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__AlarmFrom__fired(void );
#line 166
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Counter__overflow(void );
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void );
# 98 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt);
#line 105
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void );
# 72 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void );
# 63 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_unnamed4390 {
#line 63
  AlarmToTimerC__0__fired = 29U
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
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void );
# 125 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void );
#line 118
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x413eb300);
#line 60
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4391 {
#line 60
  VirtualizeTimerC__0__updateFromTimer = 30U
};
#line 60
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer];
#line 42
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4392 {

  VirtualizeTimerC__0__NUM_TIMERS = 5U, 
  VirtualizeTimerC__0__END_OF_LIST = 255
};








#line 48
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4393 {

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




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);









static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt);




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num);
#line 193
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num);
# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 212 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_enable_interrupt(void )
{
   __asm volatile ("eint");}

# 185 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
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

# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
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
# 126 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow();
}





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n)
{
}

# 28 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(uint8_t arg_0x4067ade8){
#line 28
  switch (arg_0x4067ade8) {
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
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(arg_0x4067ade8);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 115 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(0);
}

# 28 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA0__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired();
#line 28
}
#line 28
# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4394 {
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

# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void )
{
  return * (volatile uint16_t * )370U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void )
{
}

# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired();
#line 34
}
#line 34
# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4395 {
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

# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void )
{
  return * (volatile uint16_t * )372U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void )
{
}

# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired();
#line 34
}
#line 34
# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2____nesc_unnamed4396 {
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

# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void )
{
  return * (volatile uint16_t * )374U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void )
{
}

# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired();
#line 34
}
#line 34
# 120 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )302U;

#line 123
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(n >> 1);
}

# 28 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA1__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired();
#line 28
}
#line 28
# 115 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(0);
}

# 28 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB0__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired();
#line 28
}
#line 28
# 185 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
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

# 208 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__overflow(void )
#line 208
{
}

# 166 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/TransformAlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__overflow(void )
{
}

#line 166
static inline void /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Counter__overflow(void )
{
}

# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Counter.nc"
inline static void /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__overflow(void ){
#line 71
  /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Counter__overflow();
#line 71
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__overflow();
#line 71
}
#line 71
# 122 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/TransformCounterC.nc"
static inline void /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*Counter32khz32C.Transform*/TransformCounterC__0__m_upper++;
    if ((/*Counter32khz32C.Transform*/TransformCounterC__0__m_upper & /*Counter32khz32C.Transform*/TransformCounterC__0__OVERFLOW_MASK) == 0) {
      /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__overflow();
      }
  }
}

# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void )
{
}

# 166 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Counter__overflow(void )
{
}

# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Counter.nc"
inline static void /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__overflow(void ){
#line 71
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Counter__overflow();
#line 71
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow();
#line 71
}
#line 71
# 122 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/TransformCounterC.nc"
static inline void /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMilli32C.Transform*/TransformCounterC__1__m_upper++;
    if ((/*CounterMilli32C.Transform*/TransformCounterC__1__m_upper & /*CounterMilli32C.Transform*/TransformCounterC__1__OVERFLOW_MASK) == 0) {
      /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__overflow();
      }
  }
}

# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Counter.nc"
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void ){
#line 71
  /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__overflow();
#line 71
  /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__overflow();
#line 71
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__overflow();
#line 71
}
#line 71
# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void )
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow();
}

# 103 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow(void )
{
}

#line 103
static inline void /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void )
{
}

#line 103
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void )
{
}

# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void ){
#line 37
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow();
#line 37
  /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow();
#line 37
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow();
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
# 126 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow();
}

# 78 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SpiResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ControlC.Spi*/CC2420SpiC__0__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 161 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ControlP.nc"
static inline error_t CC2420ControlP__Resource__request(void )
#line 161
{
  return CC2420ControlP__SpiResource__request();
}

# 78 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static error_t PhyP__Resource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420ControlP__Resource__request();
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 128 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/phy/PhyP.nc"
static inline void PhyP__CC2420Power__startVRegDone(void )
#line 128
{
  PhyP__Resource__request();
}

# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static void CC2420ControlP__CC2420Power__startVRegDone(void ){
#line 56
  PhyP__CC2420Power__startVRegDone();
#line 56
}
#line 56
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t * )29U |= 0x01 << 6;
}

# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set();
#line 34
}
#line 34
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__set(void )
#line 37
{
#line 37
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__set();
}

# 29 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__set(void ){
#line 29
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__set();
#line 29
}
#line 29
# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void )
#line 46
{
  /* atomic removed: atomic calls only */
#line 46
  * (volatile uint8_t * )29U &= ~(0x01 << 6);
}

# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr();
#line 39
}
#line 39
# 38 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__clr(void )
#line 38
{
#line 38
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__clr();
}

# 30 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__clr(void ){
#line 30
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__clr();
#line 30
}
#line 30
# 412 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ControlP.nc"
static inline void CC2420ControlP__StartupTimer__fired(void )
#line 412
{
  if (CC2420ControlP__m_state == CC2420ControlP__S_VREG_STARTING) {
      CC2420ControlP__m_state = CC2420ControlP__S_VREG_STARTED;
      CC2420ControlP__RSTN__clr();
      CC2420ControlP__RSTN__set();
      CC2420ControlP__CC2420Power__startVRegDone();
    }
}

# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__fired(void ){
#line 67
  CC2420ControlP__StartupTimer__fired();
#line 67
}
#line 67
# 151 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/TransformAlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt == 0) 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__fired();
      }
    else 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__set_alarm();
      }
  }
}

# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void ){
#line 67
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__fired();
#line 67
}
#line 67
# 124 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void )
{
  * (volatile uint16_t * )386U &= ~0x0010;
}

# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents();
#line 47
}
#line 47
# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired();
}

# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void ){
#line 34
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired();
#line 34
}
#line 34
# 139 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void )
{
  return * (volatile uint16_t * )402U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4397 {
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

# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/State.nc"
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
# 111 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(uint8_t id)
#line 111
{
#line 111
  return FAIL;
}

# 118 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__isOwner(uint8_t arg_0x4116f718){
#line 118
  unsigned char __nesc_result;
#line 118

#line 118
  switch (arg_0x4116f718) {
#line 118
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 118
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 118
      break;
#line 118
    default:
#line 118
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(arg_0x4116f718);
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
# 77 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(uint8_t id)
#line 77
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__isOwner(id);
}

# 118 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static bool CC2420SpiP__SpiResource__isOwner(void ){
#line 118
  unsigned char __nesc_result;
#line 118

#line 118
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 118

#line 118
  return __nesc_result;
#line 118
}
#line 118
# 86 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/SchedulerBasicP.nc"
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

# 210 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__requested(void )
#line 210
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release();
}

# 73 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested(void ){
#line 73
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__requested();
#line 73
}
#line 73
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/FcfsResourceQueueC.nc"
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

# 69 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceQueue.nc"
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
# 204 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(uint8_t id)
#line 204
{
}

# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__requested(uint8_t arg_0x40af5948){
#line 43
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(arg_0x40af5948);
#line 43
}
#line 43
# 77 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
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
#line 84
      if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId == id) {
          {
            unsigned char __nesc_temp = 
#line 85
            SUCCESS;

#line 85
            return __nesc_temp;
          }
        }
      else 
#line 87
        {
          unsigned char __nesc_temp = 
#line 87
          /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__enqueue(id);

#line 87
          return __nesc_temp;
        }
      }
  }
#line 89
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested();
  return SUCCESS;
}

# 112 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(uint8_t id)
#line 112
{
#line 112
  return FAIL;
}

# 78 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(uint8_t arg_0x4116f718){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  switch (arg_0x4116f718) {
#line 78
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 78
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__request(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 78
      break;
#line 78
    default:
#line 78
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(arg_0x4116f718);
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
# 73 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(uint8_t id)
#line 73
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(id);
}

# 78 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
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
# 38 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get();
}

# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Counter.nc"
inline static /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__get(void ){
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
# 70 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void )
{
  return * (volatile uint16_t * )384U & 1U;
}

# 35 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
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
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending();
}

# 60 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Counter.nc"
inline static bool /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void ){
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
# 119 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void )
{
  * (volatile uint16_t * )386U |= 0x0010;
}

# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents();
#line 46
}
#line 46
# 84 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )386U &= ~0x0001;
}

# 33 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )402U = x;
}

# 30 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(time);
#line 30
}
#line 30
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
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
# 154 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )402U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get() + x;
}

# 32 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void ){
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
# 70 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 86
          /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 88
    /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt();
    /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents();
  }
}

# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt){
#line 92
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 181 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void )
{
}

# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired();
#line 34
}
#line 34
# 139 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void )
{
  return * (volatile uint16_t * )404U;
}

# 158 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420TransmitP.nc"
static inline void CC2420TransmitP__CaptureSFD__captured(uint16_t time)
#line 158
{
}

# 50 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GpioCapture.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captured(uint16_t time){
#line 50
  CC2420TransmitP__CaptureSFD__captured(time);
#line 50
}
#line 50
# 164 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void )
{
  * (volatile uint16_t * )388U &= ~0x0002;
}

# 57 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow();
#line 57
}
#line 57
# 84 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )388U &= ~0x0001;
}

# 33 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 65 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time)
#line 65
{
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt();
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow();
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captured(time);
}

# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time){
#line 75
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(time);
#line 75
}
#line 75
# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4398 {
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

# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Counter.nc"
inline static /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Counter__size_type /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Counter__get(void ){
#line 53
  unsigned long __nesc_result;
#line 53

#line 53
  __nesc_result = /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/TransformAlarmC.nc"
static inline /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__to_size_type /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Alarm__getNow(void )
{
  return /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Counter__get();
}

#line 146
static inline void /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Alarm__start(/*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__to_size_type dt)
{
  /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Alarm__startAt(/*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Alarm__getNow(), dt);
}

# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
inline static void TimerAsyncM__AsyncTimer__start(TimerAsyncM__AsyncTimer__size_type dt){
#line 55
  /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Alarm__start(dt);
#line 55
}
#line 55
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
inline static error_t MacP__start_gts_send__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(MacP__start_gts_send);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
inline static error_t MacP__start_coordinator_gts_send__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(MacP__start_coordinator_gts_send);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 803 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline error_t MacP__TimerAsync__time_slot_fired(void )
{

  MacP__number_backoff = 0;
  MacP__number_time_slot++;


  if (MacP__PANCoordinator == 1 && MacP__GTS_db[15 - MacP__number_time_slot].direction == 1 && MacP__GTS_db[15 - MacP__number_time_slot].gts_id != 0) 
    {



      MacP__start_coordinator_gts_send__postTask();
    }
  else 

    {

      if (MacP__number_time_slot == MacP__s_GTSss && MacP__gts_send_buffer_count > 0 && MacP__on_sync == 1) 
        {

          MacP__start_gts_send__postTask();
        }
    }

  MacP__next_on_r_GTS = 0;
  MacP__next_on_s_GTS = 0;








  if (MacP__number_time_slot + 1 >= MacP__final_CAP_slot && MacP__number_time_slot + 1 < 16) 
    {
      MacP__I_AM_IN_CAP = 0;
      MacP__I_AM_IN_CFP = 1;
      /* atomic removed: atomic calls only */


      {


        if (MacP__PANCoordinator == 1 && MacP__number_time_slot < 15) 
          {

            if (MacP__GTS_db[14 - MacP__number_time_slot].gts_id != 0x00 && MacP__GTS_db[14 - MacP__number_time_slot].DevAddressType != 0x0000) 
              {
                if (MacP__GTS_db[14 - MacP__number_time_slot].direction == 1) 
                  {
                    MacP__next_on_s_GTS = 1;
                  }
                else 
                  {
                    MacP__next_on_r_GTS = 1;
                  }
              }
          }
        else 
          {

            if (MacP__number_time_slot + 1 == MacP__s_GTSss || MacP__number_time_slot + 1 == MacP__r_GTSss) 
              {

                if (MacP__number_time_slot + 1 == MacP__s_GTSss) 
                  {

                    MacP__next_on_s_GTS = 1;
                    MacP__s_GTS_length--;
                    if (MacP__s_GTS_length != 0) 
                      {
                        MacP__s_GTSss++;
                      }
                  }
                else 
                  {

                    MacP__next_on_r_GTS = 1;
                    MacP__r_GTS_length--;
                    if (MacP__r_GTS_length != 0) 
                      {
                        MacP__r_GTSss++;
                      }
                  }
              }
            else 
              {

                MacP__next_on_s_GTS = 0;
                MacP__next_on_r_GTS = 0;
              }
          }
      }
    }

  return SUCCESS;
}

# 28 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsync.nc"
inline static error_t TimerAsyncM__TimerAsync__time_slot_fired(void ){
#line 28
  unsigned char __nesc_result;
#line 28

#line 28
  __nesc_result = MacP__TimerAsync__time_slot_fired();
#line 28

#line 28
  return __nesc_result;
#line 28
}
#line 28
# 335 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/phy/PhyP.nc"
static inline error_t PhyP__PLME_SET_TRX_STATE__request(uint8_t state)
#line 335
{


  return SUCCESS;
}

# 14 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/phy/PLME_SET_TRX_STATE.nc"
inline static error_t MacP__PLME_SET_TRX_STATE__request(uint8_t state){
#line 14
  unsigned char __nesc_result;
#line 14

#line 14
  __nesc_result = PhyP__PLME_SET_TRX_STATE__request(state);
#line 14

#line 14
  return __nesc_result;
#line 14
}
#line 14
# 777 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline error_t MacP__TimerAsync__before_time_slot_fired(void )
{
  MacP__on_s_GTS = 0;
  MacP__on_r_GTS = 0;

  if (MacP__next_on_s_GTS == 1) 
    {
      MacP__on_s_GTS = 1;
      MacP__next_on_s_GTS = 0;
      MacP__trx_status = PHY_TX_ON;
      MacP__PLME_SET_TRX_STATE__request(PHY_TX_ON);
    }


  if (MacP__next_on_r_GTS == 1) 
    {
      MacP__on_r_GTS = 1;
      MacP__next_on_r_GTS = 0;
      MacP__trx_status = PHY_RX_ON;
      MacP__PLME_SET_TRX_STATE__request(PHY_RX_ON);
    }


  return SUCCESS;
}

# 30 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsync.nc"
inline static error_t TimerAsyncM__TimerAsync__before_time_slot_fired(void ){
#line 30
  unsigned char __nesc_result;
#line 30

#line 30
  __nesc_result = MacP__TimerAsync__before_time_slot_fired();
#line 30

#line 30
  return __nesc_result;
#line 30
}
#line 30
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
inline static error_t MacP__perform_csma_ca_unslotted__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(MacP__perform_csma_ca_unslotted);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
inline static error_t MacP__perform_csma_ca_slotted__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(MacP__perform_csma_ca_slotted);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 78 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/RandomMlcgC.nc"
static inline uint16_t RandomMlcgC__Random__rand16(void )
#line 78
{
  return (uint16_t )RandomMlcgC__Random__rand32();
}

# 41 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Random.nc"
inline static uint16_t MacP__Random__rand16(void ){
#line 41
  unsigned int __nesc_result;
#line 41

#line 41
  __nesc_result = RandomMlcgC__Random__rand16();
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 909 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline error_t MacP__TimerAsync__backoff_fired(void )
{
  /* atomic removed: atomic calls only */
  {

    if (MacP__csma_locate_backoff_boundary == 1) 
      {
        MacP__csma_locate_backoff_boundary = 0;




        if (MacP__backoff_deference == 0) 
          {

            MacP__delay_backoff_period = MacP__Random__rand16() & ((uint8_t )powf(2, MacP__BE) - 1);

            if (MacP__check_csma_ca_backoff_send_conditions((uint32_t )MacP__delay_backoff_period) == 1) 
              {
                MacP__backoff_deference = 1;
              }
          }
        else 

          {
            MacP__backoff_deference = 0;
          }

        MacP__csma_delay = 1;
      }
    if (MacP__csma_cca_backoff_boundary == 1) {
      MacP__perform_csma_ca_slotted__postTask();
      }
  }
  /* atomic removed: atomic calls only */
#line 943
  {
    if (MacP__csma_delay == 1) 
      {
        if (MacP__delay_backoff_period == 0) 
          {
            if (MacP__csma_slotted == 0) 
              {
                MacP__perform_csma_ca_unslotted__postTask();
              }
            else 
              {

                MacP__csma_delay = 0;
                MacP__csma_cca_backoff_boundary = 1;
              }
          }
        MacP__delay_backoff_period--;
      }
  }
  MacP__number_backoff++;
  return SUCCESS;
}

# 25 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsync.nc"
inline static error_t TimerAsyncM__TimerAsync__backoff_fired(void ){
#line 25
  unsigned char __nesc_result;
#line 25

#line 25
  __nesc_result = MacP__TimerAsync__backoff_fired();
#line 25

#line 25
  return __nesc_result;
#line 25
}
#line 25
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t * )49U |= 0x01 << 5;
}

# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set();
#line 34
}
#line 34
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set();
}

# 29 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__set(void ){
#line 29
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set();
#line 29
}
#line 29
# 83 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/LedsP.nc"
static inline void LedsP__Leds__led1Off(void )
#line 83
{
  LedsP__Led1__set();
  ;
#line 85
  ;
}

# 66 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Leds.nc"
inline static void MacP__Leds__led1Off(void ){
#line 66
  LedsP__Leds__led1Off();
#line 66
}
#line 66
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
inline static error_t MacP__signal_loss__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(MacP__signal_loss);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
inline static error_t MacP__create_beacon__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(MacP__create_beacon);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t * )49U |= 0x01 << 6;
}

# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set();
#line 34
}
#line 34
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set();
}

# 29 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__set(void ){
#line 29
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set();
#line 29
}
#line 29
# 98 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/LedsP.nc"
static inline void LedsP__Leds__led2Off(void )
#line 98
{
  LedsP__Led2__set();
  ;
#line 100
  ;
}

# 83 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Leds.nc"
inline static void MacP__Leds__led2Off(void ){
#line 83
  LedsP__Leds__led2Off();
#line 83
}
#line 83
# 661 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline error_t MacP__TimerAsync__sd_fired(void )
{
  MacP__Leds__led2Off();



  MacP__I_AM_IN_CFP = 0;
  MacP__I_AM_IN_IP = 1;


  MacP__number_backoff = 0;
  MacP__number_time_slot = 0;


  if (MacP__PANCoordinator == 0 && COORDINATOR == ROUTER) 
    {
      MacP__trx_status = PHY_RX_ON;

      MacP__PLME_SET_TRX_STATE__request(PHY_RX_ON);
    }
  else 
    {
      MacP__trx_status = PHY_RX_ON;

      MacP__PLME_SET_TRX_STATE__request(PHY_RX_ON);
    }


  if (MacP__mac_PIB.macShortAddress == 0xffff && COORDINATOR == END_DEVICE) 
    {
      MacP__trx_status = PHY_RX_ON;

      MacP__PLME_SET_TRX_STATE__request(PHY_RX_ON);
    }
#line 722
  if (MacP__PANCoordinator == 1) 
    {
      /* atomic removed: atomic calls only */
      {








        MacP__create_beacon__postTask();
      }
    }
  else 


    {


      if (MacP__on_sync == 0) 
        {



          if (MacP__missed_beacons == 4) 
            {



              MacP__signal_loss__postTask();
            }

          MacP__missed_beacons++;
          MacP__Leds__led1Off();
        }
      else 

        {

          MacP__missed_beacons = 0;

          MacP__on_sync = 0;
        }
    }





  return SUCCESS;
}

# 20 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsync.nc"
inline static error_t TimerAsyncM__TimerAsync__sd_fired(void ){
#line 20
  unsigned char __nesc_result;
#line 20

#line 20
  __nesc_result = MacP__TimerAsync__sd_fired();
#line 20

#line 20
  return __nesc_result;
#line 20
}
#line 20
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
inline static error_t MacP__send_frame_csma__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(MacP__send_frame_csma);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 16 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/Sendframe.nc"
inline static error_t PhyP__Sendframe__send(uint8_t *frame, uint8_t frame_length){
#line 16
  unsigned char __nesc_result;
#line 16

#line 16
  __nesc_result = CC2420TransmitP__Sendframe__send(frame, frame_length);
#line 16

#line 16
  return __nesc_result;
#line 16
}
#line 16
# 224 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/phy/PhyP.nc"
static inline error_t PhyP__PD_DATA__request(uint8_t psduLength, uint8_t *psdu)
#line 224
{


  PhyP__Sendframe__send(psdu, psduLength);


  return SUCCESS;
}

# 13 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/phy/PD_DATA.nc"
inline static error_t MacP__PD_DATA__request(uint8_t psduLenght, uint8_t *psdu){
#line 13
  unsigned char __nesc_result;
#line 13

#line 13
  __nesc_result = PhyP__PD_DATA__request(psduLenght, psdu);
#line 13

#line 13
  return __nesc_result;
#line 13
}
#line 13
# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void )
#line 46
{
  /* atomic removed: atomic calls only */
#line 46
  * (volatile uint8_t * )49U &= ~(0x01 << 6);
}

# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr();
#line 39
}
#line 39
# 38 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void )
#line 38
{
#line 38
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr();
}

# 30 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__clr(void ){
#line 30
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr();
#line 30
}
#line 30
# 93 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/LedsP.nc"
static inline void LedsP__Leds__led2On(void )
#line 93
{
  LedsP__Led2__clr();
  ;
#line 95
  ;
}

# 78 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Leds.nc"
inline static void MacP__Leds__led2On(void ){
#line 78
  LedsP__Leds__led2On();
#line 78
}
#line 78
# 612 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline error_t MacP__TimerAsync__bi_fired(void )
{
  MacP__Leds__led2On();


  MacP__I_AM_IN_CAP = 1;
  MacP__I_AM_IN_IP = 0;




  if (MacP__Beacon_enabled_PAN == 1) 
    {

      MacP__PD_DATA__request(MacP__send_beacon_length, MacP__send_beacon_frame_ptr);
    }

  MacP__number_backoff = 0;
  MacP__number_time_slot = 0;





  if (MacP__TrackBeacon == 1) 
    {
      if (MacP__beacon_processed == 1) 
        {
          MacP__beacon_processed = 0;
        }
      else 
        {



          MacP__on_sync = 0;
          MacP__beacon_loss_reason = MAC_BEACON_LOSS;
        }
    }




  MacP__send_frame_csma__postTask();

  return SUCCESS;
}

# 22 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsync.nc"
inline static error_t TimerAsyncM__TimerAsync__bi_fired(void ){
#line 22
  unsigned char __nesc_result;
#line 22

#line 22
  __nesc_result = MacP__TimerAsync__bi_fired();
#line 22

#line 22
  return __nesc_result;
#line 22
}
#line 22
# 583 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline error_t MacP__TimerAsync__before_bi_fired(void )
{


  if (MacP__mac_PIB.macBeaconOrder != MacP__mac_PIB.macSuperframeOrder) 
    {
      if (MacP__Beacon_enabled_PAN == 1) 
        {


          MacP__trx_status = PHY_TX_ON;
          MacP__PLME_SET_TRX_STATE__request(PHY_TX_ON);
        }
      else 
        {


          MacP__trx_status = PHY_RX_ON;
          MacP__PLME_SET_TRX_STATE__request(PHY_RX_ON);
        }
    }


  MacP__findabeacon = 1;

  return SUCCESS;
}

# 18 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsync.nc"
inline static error_t TimerAsyncM__TimerAsync__before_bi_fired(void ){
#line 18
  unsigned char __nesc_result;
#line 18

#line 18
  __nesc_result = MacP__TimerAsync__before_bi_fired();
#line 18

#line 18
  return __nesc_result;
#line 18
}
#line 18
# 154 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsyncM.nc"
static inline void TimerAsyncM__AsyncTimer__fired(void )
#line 154
{
  /* atomic removed: atomic calls only */
  {

    if (TimerAsyncM__timers_enable == 0x01) 
      {

        TimerAsyncM__ticks_counter++;
        TimerAsyncM__process_frame_tick_counter++;

        TimerAsyncM__total_tick_counter++;

        if (TimerAsyncM__ticks_counter == TimerAsyncM__before_bi_ticks) 
          {
            TimerAsyncM__TimerAsync__before_bi_fired();
          }

        if (TimerAsyncM__ticks_counter == TimerAsyncM__bi_ticks) 
          {

            TimerAsyncM__ticks_counter = 0;
            TimerAsyncM__current_time_slot = 0;
            TimerAsyncM__backoff_ticks_counter = 0;
            TimerAsyncM__time_slot_tick_next_fire = TimerAsyncM__time_slot_ticks;
            TimerAsyncM__backoffs = 1;
            TimerAsyncM__enable_backoffs = 1;
            TimerAsyncM__current_number_backoff = 0;
            TimerAsyncM__TimerAsync__bi_fired();
          }

        if (TimerAsyncM__ticks_counter == TimerAsyncM__sd_ticks) 
          {
            TimerAsyncM__backoffs = 0;
            TimerAsyncM__TimerAsync__sd_fired();
          }

        if (TimerAsyncM__enable_backoffs == 1 && TimerAsyncM__backoffs == 1) 
          {
            TimerAsyncM__backoff_ticks_counter++;

            if (TimerAsyncM__backoff_ticks_counter == TimerAsyncM__backoff_ticks) 
              {

                TimerAsyncM__backoff_ticks_counter = 0;
                TimerAsyncM__current_number_backoff++;
                TimerAsyncM__current_number_backoff_on_time_slot++;
                TimerAsyncM__TimerAsync__backoff_fired();
              }


            if (TimerAsyncM__ticks_counter == TimerAsyncM__before_time_slot_ticks) 
              {
                TimerAsyncM__TimerAsync__before_time_slot_fired();
              }


            if (TimerAsyncM__ticks_counter == TimerAsyncM__time_slot_tick_next_fire) 
              {
                TimerAsyncM__time_slot_tick_next_fire = TimerAsyncM__time_slot_tick_next_fire + TimerAsyncM__time_slot_ticks;
                TimerAsyncM__before_time_slot_ticks = TimerAsyncM__time_slot_tick_next_fire - 5;
                TimerAsyncM__backoff_ticks_counter = 0;
                TimerAsyncM__current_number_backoff_on_time_slot = 0;
                TimerAsyncM__current_time_slot++;

                if (TimerAsyncM__current_time_slot > 0 && TimerAsyncM__current_time_slot < 16) {
                  TimerAsyncM__TimerAsync__time_slot_fired();
                  }
              }
          }
      }



    TimerAsyncM__AsyncTimer__start(10);
  }
}

# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
inline static void /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Alarm__fired(void ){
#line 67
  TimerAsyncM__AsyncTimer__fired();
#line 67
}
#line 67
# 151 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/TransformAlarmC.nc"
static inline void /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__m_dt == 0) 
      {
        /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Alarm__fired();
      }
    else 
      {
        /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__set_alarm();
      }
  }
}

# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
inline static void /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void ){
#line 67
  /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__fired();
#line 67
}
#line 67
# 124 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void )
{
  * (volatile uint16_t * )390U &= ~0x0010;
}

# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents();
#line 47
}
#line 47
# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void )
{
  /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
  /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired();
}

# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void ){
#line 34
  /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired();
#line 34
}
#line 34
# 139 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void )
{
  return * (volatile uint16_t * )406U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4399 {
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

# 78 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP__SpiResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID);
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
  __nesc_result = CC2420SpiP__Resource__immediateRequest(/*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID);
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 231 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__acquireSpiResource(void )
#line 231
{
  error_t error = CC2420TransmitP__SpiResource__immediateRequest();


  if (error != SUCCESS) {
      CC2420TransmitP__SpiResource__request();
    }
  return error;
}

# 115 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(uint8_t id)
#line 115
{
  return &msp430_spi_default_config;
}

# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
inline static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(uint8_t arg_0x4116e1b8){
#line 39
  union __nesc_unnamed4300 *__nesc_result;
#line 39

#line 39
    __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(arg_0x4116e1b8);
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 168 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__setModeSpi(msp430_spi_union_config_t *config){
#line 168
  HplMsp430Usart0P__Usart__setModeSpi(config);
#line 168
}
#line 168
# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(uint8_t id)
#line 85
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__setModeSpi(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(id));
}

# 216 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(uint8_t id)
#line 216
{
}

# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(uint8_t arg_0x40af4cf8){
#line 49
  switch (arg_0x40af4cf8) {
#line 49
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 49
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(arg_0x40af4cf8);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 213 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__immediateRequested(void )
#line 213
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release();
}

# 81 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__immediateRequested(void ){
#line 81
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__immediateRequested();
#line 81
}
#line 81
# 206 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(uint8_t id)
#line 206
{
}

# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__immediateRequested(uint8_t arg_0x40af5948){
#line 51
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(arg_0x40af5948);
#line 51
}
#line 51
# 93 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(uint8_t id)
#line 93
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__immediateRequested(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId);
  /* atomic removed: atomic calls only */
#line 95
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED) {
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_IMM_GRANTING;
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId = id;
      }
    else {
        unsigned char __nesc_temp = 
#line 100
        FAIL;

#line 100
        return __nesc_temp;
      }
  }
#line 102
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__immediateRequested();
  if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId == id) {
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId);
      return SUCCESS;
    }
  /* atomic removed: atomic calls only */
#line 107
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED;
  return FAIL;
}

# 113 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(uint8_t id)
#line 113
{
#line 113
  return FAIL;
}

# 87 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__immediateRequest(uint8_t arg_0x4116f718){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  switch (arg_0x4116f718) {
#line 87
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 87
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 87
      break;
#line 87
    default:
#line 87
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(arg_0x4116f718);
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
# 69 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(uint8_t id)
#line 69
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__immediateRequest(id);
}

# 87 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__immediateRequest(void ){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 151 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
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

# 97 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void HplMsp430I2C0P__HplUsart__resetUsart(bool reset){
#line 97
  HplMsp430Usart0P__Usart__resetUsart(reset);
#line 97
}
#line 97
# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
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

# 7 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static void HplMsp430Usart0P__HplI2C__clearModeI2C(void ){
#line 7
  HplMsp430I2C0P__HplI2C__clearModeI2C();
#line 7
}
#line 7
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 5);
}

# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__URXD__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 4);
}

# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UTXD__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc();
#line 85
}
#line 85
# 207 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
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

# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 3;
}

# 78 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UCLK__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc();
#line 78
}
#line 78
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 2;
}

# 78 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SOMI__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc();
#line 78
}
#line 78
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 1;
}

# 78 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SIMO__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc();
#line 78
}
#line 78
# 238 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
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

# 118 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/StateImplP.nc"
static inline void StateImplP__State__toIdle(uint8_t id)
#line 118
{
  /* atomic removed: atomic calls only */
#line 119
  StateImplP__state[id] = StateImplP__S_IDLE;
}

# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/State.nc"
inline static void CC2420SpiP__WorkingState__toIdle(void ){
#line 56
  StateImplP__State__toIdle(0U);
#line 56
}
#line 56
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr();
#line 39
}
#line 39
# 38 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr(void )
#line 38
{
#line 38
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__clr();
}

# 30 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CSN__clr(void ){
#line 30
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr();
#line 30
}
#line 30
# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Register.nc"
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
# 382 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__tx(uint8_t data)
#line 382
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 383
    HplMsp430Usart0P__U0TXBUF = data;
#line 383
    __nesc_atomic_end(__nesc_atomic); }
}

# 224 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(uint8_t data){
#line 224
  HplMsp430Usart0P__Usart__tx(data);
#line 224
}
#line 224
# 330 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline bool HplMsp430Usart0P__Usart__isRxIntrPending(void )
#line 330
{
  if (HplMsp430Usart0P__IFG1 & (1 << 6)) {
      return TRUE;
    }
  return FALSE;
}

# 192 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending(void ){
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
# 341 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__clrRxIntr(void )
#line 341
{
  HplMsp430Usart0P__IFG1 &= ~(1 << 6);
}

# 197 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__clrRxIntr(void ){
#line 197
  HplMsp430Usart0P__Usart__clrRxIntr();
#line 197
}
#line 197
#line 231
inline static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx(void ){
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
# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SpiPacket.nc"
inline static error_t CC2420SpiP__SpiPacket__send(uint8_t * txBuf, uint8_t * rxBuf, uint16_t len){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID, txBuf, rxBuf, len);
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SpiByte.nc"
inline static uint8_t CC2420SpiP__SpiByte__write(uint8_t tx){
#line 34
  unsigned char __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(tx);
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 133 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/StateImplP.nc"
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

# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/State.nc"
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
# 212 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
static inline cc2420_status_t CC2420SpiP__Fifo__write(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 213
{

  uint8_t status = 0;

  /* atomic removed: atomic calls only */
#line 217
  {
    if (CC2420SpiP__WorkingState__isIdle()) {
        {
          unsigned char __nesc_temp = 
#line 219
          status;

#line 219
          return __nesc_temp;
        }
      }
  }
  CC2420SpiP__m_addr = addr;

  status = CC2420SpiP__SpiByte__write(CC2420SpiP__m_addr);
  CC2420SpiP__SpiPacket__send(data, (void *)0, len);

  return status;
}

# 82 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
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
# 361 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__enableRxIntr(void )
#line 361
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 362
    {
      HplMsp430Usart0P__IFG1 &= ~(1 << 6);
      HplMsp430Usart0P__IE1 |= 1 << 6;
    }
#line 365
    __nesc_atomic_end(__nesc_atomic); }
}

# 180 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__enableRxIntr(void ){
#line 180
  HplMsp430Usart0P__Usart__enableRxIntr();
#line 180
}
#line 180
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 306 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsyncM.nc"
static inline uint32_t TimerAsyncM__TimerAsync__get_sd_ticks(void )
{
  return TimerAsyncM__time_slot_ticks * 16;
}

# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsync.nc"
inline static uint32_t MacP__TimerAsync__get_sd_ticks(void ){
#line 67
  unsigned long __nesc_result;
#line 67

#line 67
  __nesc_result = TimerAsyncM__TimerAsync__get_sd_ticks();
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 301 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsyncM.nc"
static inline uint32_t TimerAsyncM__TimerAsync__get_current_ticks(void )
{
  return TimerAsyncM__ticks_counter;
}

# 65 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsync.nc"
inline static uint32_t MacP__TimerAsync__get_current_ticks(void ){
#line 65
  unsigned long __nesc_result;
#line 65

#line 65
  __nesc_result = TimerAsyncM__TimerAsync__get_current_ticks();
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 119 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void )
{
  * (volatile uint16_t * )390U |= 0x0010;
}

# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents();
#line 46
}
#line 46
# 84 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )390U &= ~0x0001;
}

# 33 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )406U = x;
}

# 30 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(time);
#line 30
}
#line 30
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
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
# 154 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )406U = /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get() + x;
}

# 32 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void ){
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
# 70 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 86
          /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 88
    /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt();
    /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents();
  }
}

# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
inline static void /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__startAt(/*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__size_type t0, /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__size_type dt){
#line 92
  /*TimerAsyncC.Alarm.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
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
# 70 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void )
{
#line 71
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask();
}

# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__fired(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired();
#line 67
}
#line 67
# 151 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__m_dt == 0) 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__fired();
      }
    else 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__set_alarm();
      }
  }
}

# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__fired(void ){
#line 67
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__AlarmFrom__fired();
#line 67
}
#line 67
# 124 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__disableEvents(void )
{
  * (volatile uint16_t * )392U &= ~0x0010;
}

# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__disableEvents();
#line 47
}
#line 47
# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__fired();
}

# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void ){
#line 34
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired();
#line 34
}
#line 34
# 139 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void )
{
  return * (volatile uint16_t * )408U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6____nesc_unnamed4400 {
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

# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Counter.nc"
inline static /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__get(void ){
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







inline static bool /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending(void ){
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
# 119 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__enableEvents(void )
{
  * (volatile uint16_t * )392U |= 0x0010;
}

# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__enableEvents();
#line 46
}
#line 46
# 84 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )392U &= ~0x0001;
}

# 33 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )408U = x;
}

# 30 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEvent(time);
#line 30
}
#line 30
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
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
# 154 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )408U = /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__get() + x;
}

# 32 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__get(void ){
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
# 70 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 86
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 88
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__clearPendingInterrupt();
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__enableEvents();
  }
}

# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__AlarmFrom__size_type dt){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 181 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void )
{
}

# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired();
#line 34
}
#line 34
# 139 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void )
{
  return * (volatile uint16_t * )410U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7____nesc_unnamed4401 {
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

# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired();
#line 34
}
#line 34
# 139 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void )
{
  return * (volatile uint16_t * )412U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8____nesc_unnamed4402 {
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

# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired();
#line 34
}
#line 34
# 139 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void )
{
  return * (volatile uint16_t * )414U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9____nesc_unnamed4403 {
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

# 120 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )286U;

#line 123
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(n >> 1);
}

# 28 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB1__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired();
#line 28
}
#line 28
# 113 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__Scheduler__init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP__m_next, SchedulerBasicP__NO_TASK, sizeof SchedulerBasicP__m_next);
    SchedulerBasicP__m_head = SchedulerBasicP__NO_TASK;
    SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
  }
}

# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__init(void ){
#line 46
  SchedulerBasicP__Scheduler__init();
#line 46
}
#line 46
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t * )49U |= 0x01 << 4;
}

# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set();
#line 34
}
#line 34
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set();
}

# 29 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__set(void ){
#line 29
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set();
#line 29
}
#line 29
# 52 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 6;
}

# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput();
}

# 35 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__makeOutput(void ){
#line 35
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 5;
}

# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput();
}

# 35 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__makeOutput(void ){
#line 35
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 4;
}

# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput();
}

# 35 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__makeOutput(void ){
#line 35
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput();
#line 35
}
#line 35
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/LedsP.nc"
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

# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Init.nc"
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
# 36 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/platforms/telosb/hardware.h"
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

# 11 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__TOSH_wait(void )
#line 11
{
   __asm volatile ("nop"); __asm volatile ("nop");}

# 89 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/platforms/telosb/hardware.h"
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

# 27 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/platforms/telosb/MotePlatformC.nc"
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

# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Init.nc"
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
# 152 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430ClockP.nc"
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

# 32 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerB(void ){
#line 32
  Msp430ClockP__Msp430ClockInit__default__initTimerB();
#line 32
}
#line 32
# 89 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430ClockP.nc"
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

# 31 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerA(void ){
#line 31
  Msp430ClockP__Msp430ClockInit__default__initTimerA();
#line 31
}
#line 31
# 68 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430ClockP.nc"
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

# 30 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initClocks(void ){
#line 30
  Msp430ClockP__Msp430ClockInit__default__initClocks();
#line 30
}
#line 30
# 170 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430ClockP.nc"
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

# 29 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void ){
#line 29
  Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate();
#line 29
}
#line 29
# 218 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430ClockP.nc"
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

# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Init.nc"
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
# 10 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void )
#line 10
{
  PlatformP__MoteClockInit__init();
  PlatformP__MoteInit__init();
  PlatformP__LedsInit__init();
  return SUCCESS;
}

# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Init.nc"
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
# 36 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/platforms/telosb/hardware.h"
static inline  void TOSH_CLR_SIMO0_PIN()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x0019");

#line 36
  r &= ~(1 << 1);
}

# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Scheduler.nc"
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
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/AlarmToTimerC.nc"
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

# 118 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt){
#line 118
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(t0, dt);
#line 118
}
#line 118
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__disableEvents();
}

# 62 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__AlarmFrom__stop(void ){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Alarm__stop();
#line 62
}
#line 62
# 91 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__AlarmFrom__stop();
}

# 62 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void ){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__stop();
#line 62
}
#line 62
# 60 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void )
{
#line 61
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop();
}

# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop();
#line 67
}
#line 67
# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Counter.nc"
inline static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Counter__get(void ){
#line 53
  unsigned long __nesc_result;
#line 53

#line 53
  __nesc_result = /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__getNow(void )
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Counter__get();
}

# 98 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void ){
#line 98
  unsigned long __nesc_result;
#line 98

#line 98
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__getNow();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void )
{
#line 86
  return /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow();
}

# 125 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
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
# 89 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/VirtualizeTimerC.nc"
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

# 967 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline void MacP__T_ackwait__fired(void )
#line 967
{





  if (MacP__send_ack_check == 1) 
    {
      MacP__retransmit_count++;

      if (MacP__retransmit_count == 1 || MacP__send_indirect_transmission > 0) 
        {










          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 989
            {


              MacP__send_buffer_count--;
              MacP__send_buffer_msg_out++;


              if (MacP__send_buffer_count > 3) 
                {

                  MacP__send_buffer_count = 0;
                  MacP__send_buffer_msg_out = 0;
                  MacP__send_buffer_msg_in = 0;
                }



              if (MacP__send_buffer_msg_out == 3) {
                MacP__send_buffer_msg_out = 0;
                }
              if (MacP__send_buffer_count > 0) {
                MacP__send_frame_csma__postTask();
                }
              MacP__send_ack_check = 0;
              MacP__retransmit_count = 0;
              MacP__ack_sequence_number_check = 0;
            }
#line 1015
            __nesc_atomic_end(__nesc_atomic); }
        }




      MacP__send_frame_csma__postTask();
    }
}

# 295 "GTSManagementExampleP.nc"
static inline error_t GTSManagementExampleP__MLME_ASSOCIATE__confirm(uint16_t AssocShortAddress, uint8_t status)
{

  return SUCCESS;
}

# 17 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_ASSOCIATE.nc"
inline static error_t MacP__MLME_ASSOCIATE__confirm(uint16_t AssocShortAddress, uint8_t status){
#line 17
  unsigned char __nesc_result;
#line 17

#line 17
  __nesc_result = GTSManagementExampleP__MLME_ASSOCIATE__confirm(AssocShortAddress, status);
#line 17

#line 17
  return __nesc_result;
#line 17
}
#line 17
# 1027 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline void MacP__T_ResponseWaitTime__fired(void )
#line 1027
{



  if (MacP__associating == 1) 
    {

      MacP__associating = 0;
      MacP__MLME_ASSOCIATE__confirm(0x0000, MAC_NO_DATA);
    }
}

# 62 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
inline static void MacP__T_ScanDuration__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(2U, dt);
#line 62
}
#line 62
# 3089 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline void MacP__create_orphan_notification(void )
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 3092
    {

      cmd_default *cmd_orphan_notification = 0;

      dest_short *dest_short_ptr = 0;
      source_long *source_long_ptr = 0;

      MPDU *frame_pkt = 0;

      uint16_t frame_control = 0;

      if (MacP__send_buffer_msg_in == 3) {
        MacP__send_buffer_msg_in = 0;
        }
      frame_pkt = (MPDU *)&MacP__send_buffer[MacP__send_buffer_msg_in];

      frame_pkt->length = 20;

      cmd_orphan_notification = (cmd_default *)&MacP__send_buffer[MacP__send_buffer_msg_in].data[4 + 10];


      dest_short_ptr = (dest_short *)&frame_pkt->data[0];
      source_long_ptr = (source_long *)&frame_pkt->data[4];


      frame_control = set_frame_control(3, 0, 0, 0, 0, 2, 3);
      frame_pkt->frame_control1 = (uint8_t )frame_control;
      frame_pkt->frame_control2 = (uint8_t )(frame_control >> 8);


      frame_pkt->seq_num = MacP__mac_PIB.macDSN;

      MacP__mac_PIB.macDSN++;


      dest_short_ptr->destination_PAN_identifier = 0xffff;

      dest_short_ptr->destination_address = 0xffff;

      source_long_ptr->source_PAN_identifier = 0xffff;

      source_long_ptr->source_address0 = MacP__aExtendedAddress0;
      source_long_ptr->source_address1 = MacP__aExtendedAddress1;


      cmd_orphan_notification->command_frame_identifier = CMD_ORPHAN_NOTIFICATION;


      MacP__send_buffer_count++;
      MacP__send_buffer_msg_in++;

      MacP__send_frame_csma__postTask();
    }
#line 3144
    __nesc_atomic_end(__nesc_atomic); }

  return;
}

# 170 "GTSManagementExampleP.nc"
static inline error_t GTSManagementExampleP__MLME_SCAN__confirm(uint8_t status, uint8_t ScanType, uint32_t UnscannedChannels, uint8_t ResultListSize, uint8_t EnergyDetectList[], SCAN_PANDescriptor PANDescriptorList[])
{

  return SUCCESS;
}

# 15 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_SCAN.nc"
inline static error_t MacP__MLME_SCAN__confirm(uint8_t status, uint8_t ScanType, uint32_t UnscannedChannels, uint8_t ResultListSize, uint8_t EnergyDetectList[], SCAN_PANDescriptor PANDescriptorList[]){
#line 15
  unsigned char __nesc_result;
#line 15

#line 15
  __nesc_result = GTSManagementExampleP__MLME_SCAN__confirm(status, ScanType, UnscannedChannels, ResultListSize, EnergyDetectList, PANDescriptorList);
#line 15

#line 15
  return __nesc_result;
#line 15
}
#line 15
# 14 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/phy/PLME_SET.nc"
inline static error_t MacP__PLME_SET__request(uint8_t PIBAttribute, uint8_t PIBAttributeValue){
#line 14
  unsigned char __nesc_result;
#line 14

#line 14
  __nesc_result = PhyP__PLME_SET__request(PIBAttribute, PIBAttributeValue);
#line 14

#line 14
  return __nesc_result;
#line 14
}
#line 14
# 3460 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline void MacP__T_ScanDuration__fired(void )
#line 3460
{

  MacP__current_scanning++;

  ;

  MacP__PLME_SET__request(PHYCURRENTCHANNEL, 0x0A + MacP__current_scanning);

  MacP__current_channel = 0x0A + MacP__current_scanning;


  if (MacP__current_scanning == 16) 
    {


      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 3475
        MacP__scanning_channels = 0;
#line 3475
        __nesc_atomic_end(__nesc_atomic); }

      switch (MacP__scan_type) 
        {
          case ED_SCAN: 
            MacP__MLME_SCAN__confirm(MAC_SUCCESS, MacP__scan_type, 0x00, 16, MacP__scanned_values, 0x00);
          break;

          case ACTIVE_SCAN: break;

          case PASSIVE_SCAN: 

            MacP__MLME_SCAN__confirm(MAC_SUCCESS, MacP__scan_type, 0x00, 16, 0x00, MacP__scan_pans);
          break;

          case ORPHAN_SCAN: 



            MacP__MLME_SCAN__confirm(MAC_SUCCESS, MacP__scan_type, 0x00, 16, 0x00, MacP__scan_pans);

          break;
        }
    }
  else 
    {
      switch (MacP__scan_type) 
        {
          case ORPHAN_SCAN: 
            MacP__create_orphan_notification();
          break;
        }

      MacP__T_ScanDuration__startOneShot(MacP__scan_duration);
    }
}

# 143 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, FALSE);
}

# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
inline static void GTSManagementExampleP__Timer_Send__startPeriodic(uint32_t dt){
#line 53
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(4U, dt);
#line 53
}
#line 53
# 313 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/includes/mac_func.h"
static inline uint8_t set_gts_characteristics(uint8_t gts_length, uint8_t gts_direction, uint8_t characteristic_type)
{
  return ((gts_length << 0) | (gts_direction << 4)) | (characteristic_type << 5);
}

# 12 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_GTS.nc"
inline static error_t GTSManagementExampleP__MLME_GTS__request(uint8_t GTSCharacteristics, uint8_t SecurityEnable){
#line 12
  unsigned char __nesc_result;
#line 12

#line 12
  __nesc_result = MacP__MLME_GTS__request(GTSCharacteristics, SecurityEnable);
#line 12

#line 12
  return __nesc_result;
#line 12
}
#line 12
# 11 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_SET.nc"
inline static error_t GTSManagementExampleP__MLME_SET__request(uint8_t PIBAttribute, uint8_t PIBAttributeValue[]){
#line 11
  unsigned char __nesc_result;
#line 11

#line 11
  __nesc_result = MacP__MLME_SET__request(PIBAttribute, PIBAttributeValue);
#line 11

#line 11
  return __nesc_result;
#line 11
}
#line 11
# 259 "GTSManagementExampleP.nc"
static inline error_t GTSManagementExampleP__MLME_START__confirm(uint8_t status)
{


  return SUCCESS;
}

# 15 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_START.nc"
inline static error_t MacP__MLME_START__confirm(uint8_t status){
#line 15
  unsigned char __nesc_result;
#line 15

#line 15
  __nesc_result = GTSManagementExampleP__MLME_START__confirm(status);
#line 15

#line 15
  return __nesc_result;
#line 15
}
#line 15
# 95 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsyncM.nc"
static inline error_t TimerAsyncM__TimerAsync__reset(void )
{
  /* atomic removed: atomic calls only */
#line 97
  TimerAsyncM__ticks_counter = 0;
  TimerAsyncM__AsyncTimer__start(10);
  return SUCCESS;
}

# 14 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsync.nc"
inline static error_t MacP__TimerAsync__reset(void ){
#line 14
  unsigned char __nesc_result;
#line 14

#line 14
  __nesc_result = TimerAsyncM__TimerAsync__reset();
#line 14

#line 14
  return __nesc_result;
#line 14
}
#line 14
# 233 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsyncM.nc"
static inline error_t TimerAsyncM__TimerAsync__set_timers_enable(uint8_t timer)
{
  /* atomic removed: atomic calls only */
  TimerAsyncM__timers_enable = timer;



  return SUCCESS;
}

# 48 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsync.nc"
inline static error_t MacP__TimerAsync__set_timers_enable(uint8_t timer){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = TimerAsyncM__TimerAsync__set_timers_enable(timer);
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
#line 36
inline static error_t MacP__TimerAsync__set_bi_sd(uint32_t bi_symbols, uint32_t sd_symbols){
#line 36
  unsigned char __nesc_result;
#line 36

#line 36
  __nesc_result = TimerAsyncM__TimerAsync__set_bi_sd(bi_symbols, sd_symbols);
#line 36

#line 36
  return __nesc_result;
#line 36
}
#line 36


inline static error_t MacP__TimerAsync__set_backoff_symbols(uint8_t symbols){
#line 38
  unsigned char __nesc_result;
#line 38

#line 38
  __nesc_result = TimerAsyncM__TimerAsync__set_backoff_symbols(symbols);
#line 38

#line 38
  return __nesc_result;
#line 38
}
#line 38
# 3687 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline error_t MacP__MLME_START__request(uint32_t PANId, uint8_t LogicalChannel, uint8_t beacon_order, uint8_t superframe_order, bool pan_coodinator, bool BatteryLifeExtension, bool CoordRealignment, bool securityenable, uint32_t StartTime)
{

  uint32_t BO_EXPONENT;
  uint32_t SO_EXPONENT;



  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 3695
    {
      MacP__PANCoordinator = 1;
      MacP__Beacon_enabled_PAN = 1;




      if (MacP__mac_PIB.macShortAddress == 0xffff) 
        {

          MacP__MLME_START__confirm(MAC_NO_SHORT_ADDRESS);
          {
            unsigned char __nesc_temp = 
#line 3706
            SUCCESS;

            {
#line 3706
              __nesc_atomic_end(__nesc_atomic); 
#line 3706
              return __nesc_temp;
            }
          }
        }
      else 
#line 3709
        {
          MacP__mac_PIB.macBeaconOrder = beacon_order;

          if (beacon_order == 15) {
            MacP__mac_PIB.macSuperframeOrder = 15;
            }
          else {
#line 3715
            MacP__mac_PIB.macSuperframeOrder = superframe_order;
            }


          if (pan_coodinator == 1) 
            {
              MacP__mac_PIB.macPANId = PANId;
              MacP__PLME_SET__request(PHYCURRENTCHANNEL, LogicalChannel);
            }
          if (CoordRealignment == 1) 
            {
            }

          if (securityenable == 1) 
            {
            }
        }


      if (MacP__mac_PIB.macSuperframeOrder == 0) {
        SO_EXPONENT = 1;
        }
      else {
          SO_EXPONENT = powf(2, MacP__mac_PIB.macSuperframeOrder);
        }

      if (MacP__mac_PIB.macBeaconOrder == 0) {
        BO_EXPONENT = 1;
        }
      else {
          BO_EXPONENT = powf(2, MacP__mac_PIB.macBeaconOrder);
        }
    }
#line 3747
    __nesc_atomic_end(__nesc_atomic); }


  MacP__BI = 960 * BO_EXPONENT;

  MacP__SD = 960 * SO_EXPONENT;

  MacP__backoff = 20;


  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 3757
    MacP__time_slot = MacP__SD / 16;
#line 3757
    __nesc_atomic_end(__nesc_atomic); }

  MacP__TimerAsync__set_backoff_symbols(MacP__backoff);

  MacP__TimerAsync__set_bi_sd(MacP__BI, MacP__SD);

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 3763
    {

      MacP__TimerAsync__set_timers_enable(0x01);

      MacP__TimerAsync__reset();
    }
#line 3768
    __nesc_atomic_end(__nesc_atomic); }


  MacP__MLME_START__confirm(MAC_SUCCESS);

  return SUCCESS;
}

# 13 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_START.nc"
inline static error_t GTSManagementExampleP__MLME_START__request(uint32_t PANId, uint8_t LogicalChannel, uint8_t BeaconOrder, uint8_t SuperframeOrder, uint8_t PANCoordinator, uint8_t BatteryLifeExtension, uint8_t CoordRealignment, uint8_t SecurityEnable, uint32_t StartTime){
#line 13
  unsigned char __nesc_result;
#line 13

#line 13
  __nesc_result = MacP__MLME_START__request(PANId, LogicalChannel, BeaconOrder, SuperframeOrder, PANCoordinator, BatteryLifeExtension, CoordRealignment, SecurityEnable, StartTime);
#line 13

#line 13
  return __nesc_result;
#line 13
}
#line 13
# 80 "GTSManagementExampleP.nc"
static inline void GTSManagementExampleP__Timer0__fired(void )
#line 80
{

  uint8_t v_temp[2];



  if (COORDINATOR == COORDINATOR) 
    {


      v_temp[0] = (uint8_t )(GTSManagementExampleP__my_short_address >> 8);
      v_temp[1] = (uint8_t )GTSManagementExampleP__my_short_address;

      GTSManagementExampleP__MLME_SET__request(MACSHORTADDRESS, v_temp);


      v_temp[0] = (uint8_t )(0x1234 >> 8);
      v_temp[1] = (uint8_t )0x1234;

      GTSManagementExampleP__MLME_SET__request(MACPANID, v_temp);


      GTSManagementExampleP__MLME_START__request(0x1234, 0x19, 6, 6, 1, 0, 0, 0, 0);
    }
  else 


    {
      GTSManagementExampleP__my_short_address = TOS_NODE_ID;
      v_temp[0] = (uint8_t )(GTSManagementExampleP__my_short_address >> 8);
      v_temp[1] = (uint8_t )GTSManagementExampleP__my_short_address;

      GTSManagementExampleP__MLME_SET__request(MACSHORTADDRESS, v_temp);


      GTSManagementExampleP__gts_superframe_count = 0;


      printf("GTS req: %i\n", COORDINATOR);



      GTSManagementExampleP__MLME_GTS__request(set_gts_characteristics(1, GTS_TX_ONLY, 1), 0x00);


      GTSManagementExampleP__MLME_GTS__request(set_gts_characteristics(1, GTS_RX_ONLY, 1), 0x00);



      GTSManagementExampleP__Timer_Send__startPeriodic(1000);
    }
}

# 191 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/includes/mac_func.h"
static inline uint8_t set_txoptions(uint8_t ack, uint8_t gts, uint8_t indirect_transmission, uint8_t security)
{
  return (((ack << 0) | (gts << 1)) | (indirect_transmission << 2)) | (security << 3);
}

# 12 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MCPS_DATA.nc"
inline static error_t GTSManagementExampleP__MCPS_DATA__request(uint8_t SrcAddrMode, uint16_t SrcPANId, uint32_t SrcAddr[], uint8_t DstAddrMode, uint16_t DestPANId, uint32_t DstAddr[], uint8_t msduLength, uint8_t msdu[], uint8_t msduHandle, uint8_t TxOptions){
#line 12
  unsigned char __nesc_result;
#line 12

#line 12
  __nesc_result = MacP__MCPS_DATA__request(SrcAddrMode, SrcPANId, SrcAddr, DstAddrMode, DestPANId, DstAddr, msduLength, msdu, msduHandle, TxOptions);
#line 12

#line 12
  return __nesc_result;
#line 12
}
#line 12
# 135 "GTSManagementExampleP.nc"
static inline void GTSManagementExampleP__Timer_Send__fired(void )
#line 135
{


  uint32_t SrcAddr[2];
  uint32_t DstAddr[2];
  uint8_t msdu_payload[4];

#line 141
  printfflush();
  if (COORDINATOR == COORDINATOR) 
    {
      SrcAddr[0] = 0x00000000;
      SrcAddr[1] = TOS_NODE_ID;

      DstAddr[0] = 0x00000000;
      DstAddr[1] = 0x00000002;

      GTSManagementExampleP__MCPS_DATA__request(2, 0x1234, SrcAddr, 2, 0x1234, DstAddr, 4, msdu_payload, 1, set_txoptions(1, 1, 0, 0));
    }
  else 
    {

      SrcAddr[0] = 0x00000000;
      SrcAddr[1] = TOS_NODE_ID;

      DstAddr[0] = 0x00000000;
      DstAddr[1] = 0x00000000;

      GTSManagementExampleP__MCPS_DATA__request(2, 0x1234, SrcAddr, 2, 0x1234, DstAddr, 4, msdu_payload, 1, set_txoptions(1, 1, 0, 0));
    }
}

# 193 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num)
{
}

# 72 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(uint8_t arg_0x413eb300){
#line 72
  switch (arg_0x413eb300) {
#line 72
    case 0U:
#line 72
      MacP__T_ackwait__fired();
#line 72
      break;
#line 72
    case 1U:
#line 72
      MacP__T_ResponseWaitTime__fired();
#line 72
      break;
#line 72
    case 2U:
#line 72
      MacP__T_ScanDuration__fired();
#line 72
      break;
#line 72
    case 3U:
#line 72
      GTSManagementExampleP__Timer0__fired();
#line 72
      break;
#line 72
    case 4U:
#line 72
      GTSManagementExampleP__Timer_Send__fired();
#line 72
      break;
#line 72
    default:
#line 72
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(arg_0x413eb300);
#line 72
      break;
#line 72
    }
#line 72
}
#line 72
# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/QueueC.nc"
static inline bool /*PrintfC.QueueC*/QueueC__0__Queue__empty(void )
#line 53
{
  return /*PrintfC.QueueC*/QueueC__0__size == 0;
}

# 50 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Queue.nc"
inline static bool PrintfP__Queue__empty(void ){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = /*PrintfC.QueueC*/QueueC__0__Queue__empty();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 115 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Packet.nc"
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
# 120 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialActiveMessageP.nc"
static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength(void )
#line 120
{
  return 28;
}

# 57 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/QueueC.nc"
static inline uint8_t /*PrintfC.QueueC*/QueueC__0__Queue__size(void )
#line 57
{
  return /*PrintfC.QueueC*/QueueC__0__size;
}

# 58 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Queue.nc"
inline static uint8_t PrintfP__Queue__size(void ){
#line 58
  unsigned char __nesc_result;
#line 58

#line 58
  __nesc_result = /*PrintfC.QueueC*/QueueC__0__Queue__size();
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 69 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/QueueC.nc"
static inline void /*PrintfC.QueueC*/QueueC__0__printQueue(void )
#line 69
{
}

#line 65
static inline /*PrintfC.QueueC*/QueueC__0__queue_t /*PrintfC.QueueC*/QueueC__0__Queue__head(void )
#line 65
{
  return /*PrintfC.QueueC*/QueueC__0__queue[/*PrintfC.QueueC*/QueueC__0__head];
}

#line 85
static inline /*PrintfC.QueueC*/QueueC__0__queue_t /*PrintfC.QueueC*/QueueC__0__Queue__dequeue(void )
#line 85
{
  /*PrintfC.QueueC*/QueueC__0__queue_t t = /*PrintfC.QueueC*/QueueC__0__Queue__head();

#line 87
  ;
  if (!/*PrintfC.QueueC*/QueueC__0__Queue__empty()) {
      /*PrintfC.QueueC*/QueueC__0__head++;
      if (/*PrintfC.QueueC*/QueueC__0__head == 250) {
#line 90
        /*PrintfC.QueueC*/QueueC__0__head = 0;
        }
#line 91
      /*PrintfC.QueueC*/QueueC__0__size--;
      /*PrintfC.QueueC*/QueueC__0__printQueue();
    }
  return t;
}

# 81 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Queue.nc"
inline static PrintfP__Queue__t  PrintfP__Queue__dequeue(void ){
#line 81
  unsigned char __nesc_result;
#line 81

#line 81
  __nesc_result = /*PrintfC.QueueC*/QueueC__0__Queue__dequeue();
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

# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialActiveMessageP.nc"
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

# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AMPacket.nc"
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

# 166 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setType(message_t *amsg, am_id_t type)
#line 166
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(amsg);

#line 168
  __nesc_hton_uint8(header->type.data, type);
}

# 151 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AMPacket.nc"
inline static void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(message_t * amsg, am_id_t t){
#line 151
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setType(amsg, t);
#line 151
}
#line 151
# 69 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AMSend.nc"
inline static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(am_id_t arg_0x40b70ab0, am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(arg_0x40b70ab0, addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AMPacket.nc"
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
# 116 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__setPayloadLength(message_t *msg, uint8_t len)
#line 116
{
  __nesc_hton_uint8(/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(msg)->length.data, len);
}

# 83 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Packet.nc"
inline static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(message_t * msg, uint8_t len){
#line 83
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__setPayloadLength(msg, len);
#line 83
}
#line 83
# 82 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/AMQueueImplP.nc"
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

# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Send.nc"
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

# 522 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialP.nc"
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

# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SendBytePacket.nc"
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
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__dataLinkLength(message_t *msg, uint8_t upperLen)
#line 43
{
  return upperLen + sizeof(serial_header_t );
}

# 350 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(uart_id_t id, message_t *msg, 
uint8_t upperLen)
#line 351
{
  return 0;
}

# 23 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__dataLinkLength(uart_id_t arg_0x40831398, message_t *msg, uint8_t upperLen){
#line 23
  unsigned char __nesc_result;
#line 23

#line 23
  switch (arg_0x40831398) {
#line 23
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 23
      __nesc_result = SerialPacketInfoActiveMessageP__Info__dataLinkLength(msg, upperLen);
#line 23
      break;
#line 23
    default:
#line 23
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(arg_0x40831398, msg, upperLen);
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
# 40 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__offset(void )
#line 40
{
  return (uint8_t )(sizeof(message_header_t ) - sizeof(serial_header_t ));
}

# 347 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(uart_id_t id)
#line 347
{
  return 0;
}

# 15 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(uart_id_t arg_0x40831398){
#line 15
  unsigned char __nesc_result;
#line 15

#line 15
  switch (arg_0x40831398) {
#line 15
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 15
      __nesc_result = SerialPacketInfoActiveMessageP__Info__offset();
#line 15
      break;
#line 15
    default:
#line 15
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(arg_0x40831398);
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
# 100 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
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

# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Send.nc"
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
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
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
# 206 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/includes/mac_func.h"
static inline bool get_txoptions_gts(uint8_t txoptions)
{

  if ((txoptions & 0x2) == 0x2) {
    return 1;
    }
  else {
#line 212
    return 0;
    }
}

# 322 "GTSManagementExampleP.nc"
static inline error_t GTSManagementExampleP__MCPS_DATA__confirm(uint8_t msduHandle, uint8_t status)
{

  return SUCCESS;
}

# 14 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MCPS_DATA.nc"
inline static error_t MacP__MCPS_DATA__confirm(uint8_t msduHandle, uint8_t status){
#line 14
  unsigned char __nesc_result;
#line 14

#line 14
  __nesc_result = GTSManagementExampleP__MCPS_DATA__confirm(msduHandle, status);
#line 14

#line 14
  return __nesc_result;
#line 14
}
#line 14
# 216 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/includes/mac_func.h"
static inline bool get_txoptions_indirect_transmission(uint8_t txoptions)
{

  if ((txoptions & 0x4) == 0x4) {
    return 1;
    }
  else {
#line 222
    return 0;
    }
}

#line 196
static inline bool get_txoptions_ack(uint8_t txoptions)
{

  if ((txoptions & 0x1) == 0x1) {
    return 1;
    }
  else {
#line 202
    return 0;
    }
}

# 350 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsyncM.nc"
static inline uint32_t TimerAsyncM__TimerAsync__get_total_tick_counter(void )
{
  return TimerAsyncM__total_tick_counter;
}

# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsync.nc"
inline static uint32_t MacP__TimerAsync__get_total_tick_counter(void ){
#line 51
  unsigned long __nesc_result;
#line 51

#line 51
  __nesc_result = TimerAsyncM__TimerAsync__get_total_tick_counter();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 269 "GTSManagementExampleP.nc"
static inline error_t GTSManagementExampleP__MLME_SET__confirm(uint8_t status, uint8_t PIBAttribute)
{


  return SUCCESS;
}

# 13 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_SET.nc"
inline static error_t MacP__MLME_SET__confirm(uint8_t status, uint8_t PIBAttribute){
#line 13
  unsigned char __nesc_result;
#line 13

#line 13
  __nesc_result = GTSManagementExampleP__MLME_SET__confirm(status, PIBAttribute);
#line 13

#line 13
  return __nesc_result;
#line 13
}
#line 13
# 285 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ControlP.nc"
static inline void CC2420ControlP__CC2420Config__setChannel(uint8_t channel)
#line 285
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 286
    CC2420ControlP__m_channel = channel;
#line 286
    __nesc_atomic_end(__nesc_atomic); }
}

# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420Config.nc"
inline static void PhyP__CC2420Config__setChannel(uint8_t channel){
#line 59
  CC2420ControlP__CC2420Config__setChannel(channel);
#line 59
}
#line 59
#line 52
inline static error_t PhyP__CC2420Config__sync(void ){
#line 52
  unsigned char __nesc_result;
#line 52

#line 52
  __nesc_result = CC2420ControlP__CC2420Config__sync();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 78 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SyncResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ControlC.SyncSpiC*/CC2420SpiC__1__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 1204 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline error_t MacP__PLME_SET__confirm(uint8_t status, uint8_t PIBAttribute)
#line 1204
{
  return SUCCESS;
}

# 16 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/phy/PLME_SET.nc"
inline static error_t PhyP__PLME_SET__confirm(uint8_t status, uint8_t PIBAttribute){
#line 16
  unsigned char __nesc_result;
#line 16

#line 16
  __nesc_result = MacP__PLME_SET__confirm(status, PIBAttribute);
#line 16

#line 16
  return __nesc_result;
#line 16
}
#line 16
# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__toggle(void )
#line 47
{
#line 47
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 47
    * (volatile uint8_t * )49U ^= 0x01 << 5;
#line 47
    __nesc_atomic_end(__nesc_atomic); }
}

# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__toggle(void ){
#line 44
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__toggle();
#line 44
}
#line 44
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle(void )
#line 39
{
#line 39
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__toggle();
}

# 31 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__toggle(void ){
#line 31
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle();
#line 31
}
#line 31
# 88 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/LedsP.nc"
static inline void LedsP__Leds__led1Toggle(void )
#line 88
{
  LedsP__Led1__toggle();
  ;
#line 90
  ;
}

# 72 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Leds.nc"
inline static void GTSManagementExampleP__Leds__led1Toggle(void ){
#line 72
  LedsP__Leds__led1Toggle();
#line 72
}
#line 72
# 332 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/includes/mac_func.h"
static inline uint8_t get_characteristic_type(uint8_t gts_characteristics)
{
  if ((gts_characteristics & 0x20) == 0x20) {
    return 1;
    }
  else {
#line 337
    return 0;
    }
}

# 3223 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline void MacP__create_gts_request_cmd(uint8_t gts_characteristics)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 3225
    {
      cmd_gts_request *mac_gts_request;

      MPDU *frame_pkt;

      uint16_t frame_control;



      if (MacP__send_buffer_msg_in == 3) {
        MacP__send_buffer_msg_in = 0;
        }
      frame_pkt = (MPDU *)&MacP__send_buffer[MacP__send_buffer_msg_in];

      mac_gts_request = (cmd_gts_request *)& MacP__send_buffer[MacP__send_buffer_msg_in].data;

      frame_pkt->length = 11;

      if (get_characteristic_type(gts_characteristics) != 0) 
        {


          frame_control = set_frame_control(3, 0, 0, 1, 0, 0, 2);
          frame_pkt->frame_control1 = (uint8_t )frame_control;
          frame_pkt->frame_control2 = (uint8_t )(frame_control >> 8);
        }
      else 
        {


          frame_control = set_frame_control(3, 0, 0, 1, 1, 0, 2);
          frame_pkt->frame_control1 = (uint8_t )frame_control;
          frame_pkt->frame_control2 = (uint8_t )(frame_control >> 8);
        }

      frame_pkt->seq_num = MacP__mac_PIB.macDSN;

      MacP__gts_request_seq_num = frame_pkt->seq_num;

      MacP__mac_PIB.macDSN++;


      MacP__send_buffer[MacP__send_buffer_msg_in].retransmission = 1;
      MacP__send_buffer[MacP__send_buffer_msg_in].indirect = 0;


      mac_gts_request->source_PAN_identifier = MacP__mac_PIB.macPANId;

      mac_gts_request->source_address = MacP__mac_PIB.macShortAddress;

      mac_gts_request->command_frame_identifier = CMD_GTS_REQUEST;


      mac_gts_request->gts_characteristics = gts_characteristics;


      MacP__send_buffer_count++;
      MacP__send_buffer_msg_in++;

      MacP__send_frame_csma__postTask();
    }
#line 3285
    __nesc_atomic_end(__nesc_atomic); }


  return;
}

# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
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
# 128 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void )
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow());
}

# 72 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void ){
#line 72
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired();
#line 72
}
#line 72
# 80 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__getAlarm(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 82
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__to_size_type __nesc_temp = 
#line 82
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__m_dt;

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

# 105 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void ){
#line 105
  unsigned long __nesc_result;
#line 105

#line 105
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__getAlarm();
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 63 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt, FALSE);
    }
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired();
}

# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
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
# 182 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
static inline void CC2420SpiP__SpiResource__granted(void )
#line 182
{
  CC2420SpiP__grant__postTask();
}

# 119 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(uint8_t id)
#line 119
{
}

# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(uint8_t arg_0x41173920){
#line 92
  switch (arg_0x41173920) {
#line 92
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 92
      CC2420SpiP__SpiResource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(arg_0x41173920);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 95 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(uint8_t id)
#line 95
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(id);
}

# 202 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(uint8_t id)
#line 202
{
}

# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__granted(uint8_t arg_0x40ae1e98){
#line 92
  switch (arg_0x40ae1e98) {
#line 92
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 92
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(arg_0x40ae1e98);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 190 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask(void )
#line 190
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 191
    {
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId;
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY;
    }
#line 194
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId);
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__granted(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId);
}

# 190 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error)
#line 190
{
}

# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SpiPacket.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(uint8_t arg_0x41170ee8, uint8_t * txBuf, uint8_t * rxBuf, uint16_t len, error_t error){
#line 71
  switch (arg_0x41170ee8) {
#line 71
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 71
      CC2420SpiP__SpiPacket__sendDone(txBuf, rxBuf, len, error);
#line 71
      break;
#line 71
    default:
#line 71
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(arg_0x41170ee8, txBuf, rxBuf, len, error);
#line 71
      break;
#line 71
    }
#line 71
}
#line 71
# 183 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone(void )
#line 183
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_client, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len, 
  SUCCESS);
}

#line 166
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void )
#line 166
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 167
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone();
#line 167
    __nesc_atomic_end(__nesc_atomic); }
}

# 188 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420TransmitP.nc"
static inline void CC2420TransmitP__TXFIFO__readDone(uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 188
{
}

# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static error_t CC2420ReceiveP__SpiResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set();
#line 34
}
#line 34
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set(void )
#line 37
{
#line 37
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__set();
}

# 29 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void CC2420ReceiveP__CSN__set(void ){
#line 29
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set();
#line 29
}
#line 29
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
inline static error_t MacP__data_indication__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(MacP__data_indication);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
inline static error_t MacP__data_channel_scan_indication__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(MacP__data_channel_scan_indication);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 1054 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline error_t MacP__PD_DATA__indication(uint8_t psduLenght, uint8_t *psdu, int8_t ppduLinkQuality)
#line 1054
{



  if (MacP__buffer_count > 4) 
    {
    }
  else 


    {

      memcpy(&MacP__buffer_msg[MacP__current_msg_in], psdu, sizeof(MPDU ));
      /* atomic removed: atomic calls only */
      {
        MacP__current_msg_in++;

        if (MacP__current_msg_in == 4) {
          MacP__current_msg_in = 0;
          }
        MacP__buffer_count++;
      }

      MacP__link_quality = ppduLinkQuality;

      if (MacP__scanning_channels == 1) 
        {

          MacP__data_channel_scan_indication__postTask();
        }
      else 

        {

          MacP__data_indication__postTask();
        }
    }






  return SUCCESS;
}

# 17 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/phy/PD_DATA.nc"
inline static error_t PhyP__PD_DATA__indication(uint8_t psduLenght, uint8_t *psdu, int8_t ppduLinkQuality){
#line 17
  unsigned char __nesc_result;
#line 17

#line 17
  __nesc_result = MacP__PD_DATA__indication(psduLenght, psdu, ppduLinkQuality);
#line 17

#line 17
  return __nesc_result;
#line 17
}
#line 17
# 184 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/phy/PhyP.nc"
static inline void PhyP__Receiveframe__receive(uint8_t *frame, uint8_t rssi)
{

  PhyP__rxmpdu_ptr = (MPDU *)frame;

  PhyP__PD_DATA__indication(PhyP__rxmpdu_ptr->length, (uint8_t *)PhyP__rxmpdu_ptr, rssi);
}

# 14 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/Receiveframe.nc"
inline static void CC2420ReceiveP__Receiveframe__receive(uint8_t *frame, uint8_t rssi){
#line 14
  PhyP__Receiveframe__receive(frame, rssi);
#line 14
}
#line 14
# 207 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
static inline error_t CC2420SpiP__Fifo__continueRead(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 208
{
  return CC2420SpiP__SpiPacket__send((void *)0, data, len);
}

# 62 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
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
# 48 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
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

# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__get(void ){
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
# 40 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__GeneralIO__get(void )
#line 40
{
#line 40
  return /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__get();
}

# 32 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static bool CC2420ReceiveP__FIFOP__get(void ){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__GeneralIO__get();
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 48 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
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

# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__HplGeneralIO__get(void ){
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
# 40 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__GeneralIO__get(void )
#line 40
{
#line 40
  return /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__HplGeneralIO__get();
}

# 32 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static bool CC2420ReceiveP__FIFO__get(void ){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__GeneralIO__get();
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 360 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__RXFIFO__readDone(uint8_t *rx_buf, uint8_t rx_len, error_t error)
#line 360
{
  /* atomic removed: atomic calls only */
#line 403
  {

    switch (CC2420ReceiveP__m_state) {

        case CC2420ReceiveP__S_RX_LENGTH: 

          CC2420ReceiveP__rxFrameLength = rx_buf[0];

        CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_FC;






        if (CC2420ReceiveP__rxFrameLength + 1 > CC2420ReceiveP__m_bytes_left) 
          {


            CC2420ReceiveP__flush();
          }
        else 

          {
            if (!CC2420ReceiveP__FIFO__get() && !CC2420ReceiveP__FIFOP__get()) 
              {


                CC2420ReceiveP__m_bytes_left -= CC2420ReceiveP__rxFrameLength + 1;
              }



            if (CC2420ReceiveP__rxFrameLength > 0) 
              {

                if (CC2420ReceiveP__rxFrameLength > 2) {





                    CC2420ReceiveP__RXFIFO__continueRead((uint8_t *)CC2420ReceiveP__rxmpdu_ptr + 1, 3);
                  }
                else 
                  {



                    CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_DISCARD;


                    CC2420ReceiveP__RXFIFO__continueRead((uint8_t *)CC2420ReceiveP__rxmpdu_ptr + 1, CC2420ReceiveP__rxFrameLength);
                    return;
                  }
              }
            else 
              {

                CC2420ReceiveP__receivingPacket = FALSE;
                CC2420ReceiveP__CSN__set();
                CC2420ReceiveP__SpiResource__release();
                CC2420ReceiveP__waitForNextPacket();
              }
          }










        break;
        case CC2420ReceiveP__S_RX_FC: 




          if ((CC2420ReceiveP__rxmpdu_ptr->frame_control1 & 0x7) == 2) 
            {
              CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_PAYLOAD;


              CC2420ReceiveP__RXFIFO__continueRead((uint8_t *)CC2420ReceiveP__rxmpdu_ptr + 4, 2);
              return;
            }


        if (CC2420ReceiveP__address_decode == 1) 
          {
            CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_ADDR;

            CC2420ReceiveP__destination_address = get_fc2_dest_addr(CC2420ReceiveP__rxmpdu_ptr->frame_control2);

            if (CC2420ReceiveP__destination_address > 1) 
              {
                switch (CC2420ReceiveP__destination_address) 
                  {
                    case 2: 


                      CC2420ReceiveP__RXFIFO__continueRead((uint8_t *)CC2420ReceiveP__rxmpdu_ptr + 4, 6);
                    break;

                    case 3: 

                      CC2420ReceiveP__RXFIFO__continueRead((uint8_t *)CC2420ReceiveP__rxmpdu_ptr + 4, 12);
                    break;
                  }
              }
            else 
              {

                CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_PAYLOAD;


                CC2420ReceiveP__RXFIFO__continueRead((uint8_t *)CC2420ReceiveP__rxmpdu_ptr + 4, CC2420ReceiveP__rxmpdu_ptr->length - 3);
              }
          }
        else 

          {

            CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_PAYLOAD;
            CC2420ReceiveP__RXFIFO__continueRead((uint8_t *)CC2420ReceiveP__rxmpdu_ptr + 4, CC2420ReceiveP__rxmpdu_ptr->length - 3);
          }


        break;
        case CC2420ReceiveP__S_RX_ADDR: 
          CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_PAYLOAD;


        switch (CC2420ReceiveP__rxmpdu_ptr->frame_control1 & 0x7) 
          {
            case 0: 


              CC2420ReceiveP__beacon_addr_short_ptr = (beacon_addr_short *)&CC2420ReceiveP__rxmpdu_ptr->data[0];
#line 559
            if (CC2420ReceiveP__beacon_addr_short_ptr->source_address != CC2420ReceiveP__ver_macCoordShortAddress) 
              {


                CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_DISCARD;
                CC2420ReceiveP__RXFIFO__continueRead((uint8_t *)CC2420ReceiveP__rxmpdu_ptr + 10, CC2420ReceiveP__rxmpdu_ptr->length - 9);
                return;
              }
#line 580
            break;
            case 1: 
              case 3: 


                if (CC2420ReceiveP__destination_address > 1) 
                  {
                    switch (CC2420ReceiveP__destination_address) 
                      {
                        case 2: 
                          CC2420ReceiveP__dest_short_ptr = (dest_short *)&CC2420ReceiveP__rxmpdu_ptr->data[0];

                        if (CC2420ReceiveP__dest_short_ptr->destination_address != 0xffff && CC2420ReceiveP__dest_short_ptr->destination_address != CC2420ReceiveP__ver_macShortAddress) 
                          {


                            CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_DISCARD;
                            CC2420ReceiveP__RXFIFO__continueRead((uint8_t *)CC2420ReceiveP__rxmpdu_ptr + 10, CC2420ReceiveP__rxmpdu_ptr->length - 9);
                            return;
                          }



                        if (CC2420ReceiveP__dest_short_ptr->destination_PAN_identifier != 0xffff && CC2420ReceiveP__dest_short_ptr->destination_PAN_identifier != CC2420ReceiveP__ver_macPANId) 
                          {


                            CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_DISCARD;
                            CC2420ReceiveP__RXFIFO__continueRead((uint8_t *)CC2420ReceiveP__rxmpdu_ptr + 10, CC2420ReceiveP__rxmpdu_ptr->length - 9);
                            return;
                          }

                        break;

                        case 3: 

                          CC2420ReceiveP__dest_long_ptr = (dest_long *)&CC2420ReceiveP__rxmpdu_ptr->data[0];
#line 639
                        break;
                      }
                  }

            break;

            case 2: 




              return;

            break;
          }




        switch (CC2420ReceiveP__destination_address) 
          {
            case 2: 


              CC2420ReceiveP__RXFIFO__continueRead((uint8_t *)CC2420ReceiveP__rxmpdu_ptr + 10, CC2420ReceiveP__rxmpdu_ptr->length - 9);
            break;

            case 3: 

              CC2420ReceiveP__RXFIFO__continueRead((uint8_t *)CC2420ReceiveP__rxmpdu_ptr + 16, CC2420ReceiveP__rxmpdu_ptr->length - 15);
            break;
          }

        break;


        case CC2420ReceiveP__S_RX_PAYLOAD: 
          CC2420ReceiveP__CSN__set();
#line 689
        if (!CC2420ReceiveP__m_missed_packets) {

            CC2420ReceiveP__SpiResource__release();
          }

        CC2420ReceiveP__rssi = 255 - CC2420ReceiveP__rxmpdu_ptr->data[CC2420ReceiveP__rxmpdu_ptr->length - 4];



        CC2420ReceiveP__Receiveframe__receive((uint8_t *)CC2420ReceiveP__rxmpdu_ptr, CC2420ReceiveP__rssi);


        if (CC2420ReceiveP__m_missed_packets == 0) 
          {
            CC2420ReceiveP__flush();
          }
        else 
          {
            CC2420ReceiveP__waitForNextPacket();
          }

        break;


        case CC2420ReceiveP__S_RX_DISCARD: 
          CC2420ReceiveP__receivingPacket = FALSE;
        CC2420ReceiveP__CSN__set();
        CC2420ReceiveP__SpiResource__release();
        if (CC2420ReceiveP__m_missed_packets == 0) 
          {
            CC2420ReceiveP__flush();
          }
        else 
          {
            CC2420ReceiveP__waitForNextPacket();
          }

        break;

        default: 
          CC2420ReceiveP__receivingPacket = FALSE;
        CC2420ReceiveP__CSN__set();
        CC2420ReceiveP__SpiResource__release();
        break;
      }
  }
}

# 368 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
static inline void CC2420SpiP__Fifo__default__readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error)
#line 368
{
}

# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static void CC2420SpiP__Fifo__readDone(uint8_t arg_0x41118698, uint8_t * data, uint8_t length, error_t error){
#line 71
  switch (arg_0x41118698) {
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
      CC2420SpiP__Fifo__default__readDone(arg_0x41118698, data, length, error);
#line 71
      break;
#line 71
    }
#line 71
}
#line 71
# 30 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void CC2420ReceiveP__CSN__clr(void ){
#line 30
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr();
#line 30
}
#line 30
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
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
# 163 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420TransmitP.nc"
static inline void CC2420TransmitP__ChipSpiResource__releasing(void )
#line 163
{
}

# 24 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
inline static void CC2420SpiP__ChipSpiResource__releasing(void ){
#line 24
  CC2420TransmitP__ChipSpiResource__releasing();
#line 24
}
#line 24
# 208 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__granted(void )
#line 208
{
}

# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted(void ){
#line 46
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__granted();
#line 46
}
#line 46
# 97 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(bool reset){
#line 97
  HplMsp430Usart0P__Usart__resetUsart(reset);
#line 97
}
#line 97
#line 158
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableSpi(void ){
#line 158
  HplMsp430Usart0P__Usart__disableSpi();
#line 158
}
#line 158
# 89 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(uint8_t id)
#line 89
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(TRUE);
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableSpi();
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(FALSE);
}

# 218 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(uint8_t id)
#line 218
{
}

# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(uint8_t arg_0x40af4cf8){
#line 55
  switch (arg_0x40af4cf8) {
#line 55
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 55
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 55
      break;
#line 55
    default:
#line 55
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(arg_0x40af4cf8);
#line 55
      break;
#line 55
    }
#line 55
}
#line 55
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
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
# 58 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/FcfsResourceQueueC.nc"
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

# 60 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceQueue.nc"
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
# 50 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/FcfsResourceQueueC.nc"
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

# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceQueue.nc"
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
# 111 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(uint8_t id)
#line 111
{
  /* atomic removed: atomic calls only */
#line 112
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
#line 127
          SUCCESS;

#line 127
          return __nesc_temp;
        }
      }
  }
#line 130
  return FAIL;
}

# 114 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(uint8_t id)
#line 114
{
#line 114
  return FAIL;
}

# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(uint8_t arg_0x4116f718){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  switch (arg_0x4116f718) {
#line 110
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 110
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 110
      break;
#line 110
    default:
#line 110
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(arg_0x4116f718);
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
# 81 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(uint8_t id)
#line 81
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(id);
}

# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 1);
}

# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SIMO__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 2);
}

# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SOMI__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 3);
}

# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UCLK__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc();
#line 85
}
#line 85
# 176 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
static inline uint8_t CC2420SpiP__Resource__isOwner(uint8_t id)
#line 176
{
  /* atomic removed: atomic calls only */
#line 177
  {
    unsigned char __nesc_temp = 
#line 177
    CC2420SpiP__m_holder == id;

#line 177
    return __nesc_temp;
  }
}

# 118 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static bool CC2420ReceiveP__SpiResource__isOwner(void ){
#line 118
  unsigned char __nesc_result;
#line 118

#line 118
  __nesc_result = CC2420SpiP__Resource__isOwner(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 118

#line 118
  return __nesc_result;
#line 118
}
#line 118
# 187 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
static inline cc2420_status_t CC2420SpiP__Fifo__beginRead(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 188
{

  cc2420_status_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 192
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 194
            status;

            {
#line 194
              __nesc_atomic_end(__nesc_atomic); 
#line 194
              return __nesc_temp;
            }
          }
        }
    }
#line 198
    __nesc_atomic_end(__nesc_atomic); }
#line 198
  CC2420SpiP__m_addr = addr | 0x40;

  status = CC2420SpiP__SpiByte__write(CC2420SpiP__m_addr);
  CC2420SpiP__Fifo__continueRead(addr, data, len);

  return status;
}

# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
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
# 87 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static error_t CC2420ReceiveP__SpiResource__immediateRequest(void ){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = CC2420SpiP__Resource__immediateRequest(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
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
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 29 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CSN__set(void ){
#line 29
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set();
#line 29
}
#line 29
# 179 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420TransmitP.nc"
static inline void CC2420TransmitP__TXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error)
{

  CC2420TransmitP__CSN__set();
  CC2420TransmitP__attemptSend();
}

# 863 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__RXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 863
{
}

# 371 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
static inline void CC2420SpiP__Fifo__default__writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 371
{
}

# 91 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static void CC2420SpiP__Fifo__writeDone(uint8_t arg_0x41118698, uint8_t * data, uint8_t length, error_t error){
#line 91
  switch (arg_0x41118698) {
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
      CC2420SpiP__Fifo__default__writeDone(arg_0x41118698, data, length, error);
#line 91
      break;
#line 91
    }
#line 91
}
#line 91
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
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
# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP__SpiResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 241 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__releaseSpiResource(void )
#line 241
{
  CC2420TransmitP__SpiResource__release();
  return SUCCESS;
}

# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
inline static error_t PhyP__sendDone_task__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(PhyP__sendDone_task);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 116 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/phy/PhyP.nc"
static inline void PhyP__Sendframe__sendDone(error_t error)
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 119
    PhyP__sendErr = error;
#line 119
    __nesc_atomic_end(__nesc_atomic); }
  PhyP__sendDone_task__postTask();
}

# 21 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/Sendframe.nc"
inline static void CC2420TransmitP__Sendframe__sendDone(error_t error){
#line 21
  PhyP__Sendframe__sendDone(error);
#line 21
}
#line 21
# 100 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
static inline error_t CC2420SpiP__ChipSpiResource__attemptRelease(void )
#line 100
{
  return CC2420SpiP__attemptRelease();
}

# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
inline static error_t CC2420TransmitP__ChipSpiResource__attemptRelease(void ){
#line 39
  unsigned char __nesc_result;
#line 39

#line 39
  __nesc_result = CC2420SpiP__ChipSpiResource__attemptRelease();
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 247 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420TransmitP.nc"
static inline void CC2420TransmitP__signalDone(error_t err)
#line 247
{

  CC2420TransmitP__ChipSpiResource__attemptRelease();
  CC2420TransmitP__Sendframe__sendDone(err);
}

# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Register.nc"
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
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
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
# 79 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__enable(void )
#line 79
{
#line 79
  P1IE |= 1 << 4;
}

# 31 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__enable(void ){
#line 31
  HplMsp430InterruptP__Port14__enable();
#line 31
}
#line 31
# 131 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__edge(bool low_to_high){
#line 56
  HplMsp430InterruptP__Port14__edge(low_to_high);
#line 56
}
#line 56
# 95 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__clear(void )
#line 95
{
#line 95
  P1IFG &= ~(1 << 4);
}

# 41 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port14__clear();
#line 41
}
#line 41
# 87 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__disable(void )
#line 87
{
#line 87
  P1IE &= ~(1 << 4);
}

# 36 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__disable(void ){
#line 36
  HplMsp430InterruptP__Port14__disable();
#line 36
}
#line 36
# 58 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__disable();
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear();
  }
  return SUCCESS;
}

#line 41
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__enable(bool rising)
#line 41
{
  /* atomic removed: atomic calls only */
#line 42
  {
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable();
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__edge(rising);
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__enable();
  }
  return SUCCESS;
}

static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void )
#line 50
{
  return /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__enable(TRUE);
}

# 42 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ControlP__InterruptCCA__enableRisingEdge(void ){
#line 42
  unsigned char __nesc_result;
#line 42

#line 42
  __nesc_result = /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge();
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Register.nc"
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
# 197 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Power__startOscillator(void )
#line 197
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 198
    {










      CC2420ControlP__IOCFG1__write(0x0018);

      CC2420ControlP__InterruptCCA__enableRisingEdge();
      CC2420ControlP__SXOSCON__strobe();




      CC2420ControlP__IOCFG0__write(0x027F);


      CC2420ControlP__writeFsctrl();







      CC2420ControlP__MDMCTRL0__write(0x02E2);


      CC2420ControlP__writeMdmctrl0();
    }
#line 232
    __nesc_atomic_end(__nesc_atomic); }
#line 244
  return SUCCESS;
}

# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t PhyP__CC2420Power__startOscillator(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = CC2420ControlP__CC2420Power__startOscillator();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 133 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/phy/PhyP.nc"
static inline void PhyP__Resource__granted(void )
#line 133
{
  PhyP__CC2420Power__startOscillator();
}

# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static void CC2420ControlP__Resource__granted(void ){
#line 92
  PhyP__Resource__granted();
#line 92
}
#line 92
# 30 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__clr(void ){
#line 30
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr();
#line 30
}
#line 30
# 392 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ControlP.nc"
static inline void CC2420ControlP__SpiResource__granted(void )
#line 392
{
  CC2420ControlP__CSN__clr();
  CC2420ControlP__Resource__granted();
}

# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420ControlP__syncDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420ControlP__syncDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SyncResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ControlC.SyncSpiC*/CC2420SpiC__1__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 29 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__set(void ){
#line 29
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set();
#line 29
}
#line 29
# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
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
# 374 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ControlP.nc"
static inline void CC2420ControlP__SyncResource__granted(void )
#line 374
{


  CC2420ControlP__CSN__clr();
  CC2420ControlP__SRFOFF__strobe();

  CC2420ControlP__writeFsctrl();

  CC2420ControlP__writeMdmctrl0();
  CC2420ControlP__writeId();
  CC2420ControlP__CSN__set();
  CC2420ControlP__CSN__clr();
  CC2420ControlP__SRXON__strobe();
  CC2420ControlP__CSN__set();
  CC2420ControlP__SyncResource__release();
  CC2420ControlP__syncDone__postTask();
}

# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__RssiResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ControlC.RssiResource*/CC2420SpiC__2__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 283 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
static inline cc2420_status_t CC2420SpiP__Reg__read(uint8_t addr, uint16_t *data)
#line 283
{

  cc2420_status_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 287
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 289
            status;

            {
#line 289
              __nesc_atomic_end(__nesc_atomic); 
#line 289
              return __nesc_temp;
            }
          }
        }
    }
#line 293
    __nesc_atomic_end(__nesc_atomic); }
#line 293
  status = CC2420SpiP__SpiByte__write(addr | 0x40);
  *data = (uint16_t )CC2420SpiP__SpiByte__write(0) << 8;
  *data |= CC2420SpiP__SpiByte__write(0);

  return status;
}

# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__RSSI__read(uint16_t *data){
#line 47
  unsigned char __nesc_result;
#line 47

#line 47
  __nesc_result = CC2420SpiP__Reg__read(CC2420_RSSI, data);
#line 47

#line 47
  return __nesc_result;
#line 47
}
#line 47
# 397 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ControlP.nc"
static inline void CC2420ControlP__RssiResource__granted(void )
#line 397
{
  uint16_t data;

#line 399
  CC2420ControlP__CSN__clr();
  CC2420ControlP__RSSI__read(&data);
  CC2420ControlP__CSN__set();

  CC2420ControlP__RssiResource__release();
  data += 0x7f;
  data &= 0x00ff;
}

# 168 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420TransmitP.nc"
static inline void CC2420TransmitP__SpiResource__granted(void )
#line 168
{

  CC2420TransmitP__attemptSend();
}

# 351 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__SpiResource__granted(void )
#line 351
{
  CC2420ReceiveP__receive();
}

# 365 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
static inline void CC2420SpiP__Resource__default__granted(uint8_t id)
#line 365
{
}

# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static void CC2420SpiP__Resource__granted(uint8_t arg_0x410deb90){
#line 92
  switch (arg_0x410deb90) {
#line 92
    case /*CC2420ControlC.Spi*/CC2420SpiC__0__CLIENT_ID:
#line 92
      CC2420ControlP__SpiResource__granted();
#line 92
      break;
#line 92
    case /*CC2420ControlC.SyncSpiC*/CC2420SpiC__1__CLIENT_ID:
#line 92
      CC2420ControlP__SyncResource__granted();
#line 92
      break;
#line 92
    case /*CC2420ControlC.RssiResource*/CC2420SpiC__2__CLIENT_ID:
#line 92
      CC2420ControlP__RssiResource__granted();
#line 92
      break;
#line 92
    case /*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID:
#line 92
      CC2420TransmitP__SpiResource__granted();
#line 92
      break;
#line 92
    case /*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID:
#line 92
      CC2420ReceiveP__SpiResource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      CC2420SpiP__Resource__default__granted(arg_0x410deb90);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 356 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
static inline void CC2420SpiP__grant__runTask(void )
#line 356
{
  uint8_t holder;

#line 358
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 358
    {
      holder = CC2420SpiP__m_holder;
    }
#line 360
    __nesc_atomic_end(__nesc_atomic); }
  CC2420SpiP__Resource__granted(holder);
}

# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Register.nc"
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
# 258 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
static inline cc2420_status_t CC2420SpiP__Ram__write(uint16_t addr, uint8_t offset, 
uint8_t *data, 
uint8_t len)
#line 260
{

  cc2420_status_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 264
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 266
            status;

            {
#line 266
              __nesc_atomic_end(__nesc_atomic); 
#line 266
              return __nesc_temp;
            }
          }
        }
    }
#line 270
    __nesc_atomic_end(__nesc_atomic); }
#line 270
  addr += offset;

  CC2420SpiP__SpiByte__write(addr | 0x80);
  CC2420SpiP__SpiByte__write((addr >> 1) & 0xc0);
  for (; len; len--) {
      status = CC2420SpiP__SpiByte__write(* data++);
    }

  return status;
}

# 63 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Ram.nc"
inline static cc2420_status_t CC2420ControlP__PANID__write(uint8_t offset, uint8_t * data, uint8_t length){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Ram__write(CC2420_RAM_PANID, offset, data, length);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
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

# 210 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/phy/PhyP.nc"
static inline void PhyP__CC2420Config__syncDone(error_t error)
{



  return;
}

# 887 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__CC2420Config__syncDone(error_t error)
#line 887
{
}

# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420Config.nc"
inline static void CC2420ControlP__CC2420Config__syncDone(error_t error){
#line 53
  CC2420ReceiveP__CC2420Config__syncDone(error);
#line 53
  PhyP__CC2420Config__syncDone(error);
#line 53
}
#line 53
# 451 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ControlP.nc"
static inline void CC2420ControlP__syncDone__runTask(void )
#line 451
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 452
    CC2420ControlP__m_sync_busy = FALSE;
#line 452
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP__CC2420Config__syncDone(SUCCESS);
}

#line 447
static inline void CC2420ControlP__sync__runTask(void )
#line 447
{
  CC2420ControlP__CC2420Config__sync();
}

# 144 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/phy/PhyP.nc"
static inline void PhyP__sendDone_task__runTask(void )
#line 144
{
  error_t packetErr;

#line 146
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 146
    packetErr = PhyP__sendErr;
#line 146
    __nesc_atomic_end(__nesc_atomic); }
}

# 575 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline void MacP__AMControl__stopDone(error_t err)
#line 575
{
}

# 117 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SplitControl.nc"
inline static void PhyP__SplitControl__stopDone(error_t error){
#line 117
  MacP__AMControl__stopDone(error);
#line 117
}
#line 117
# 159 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/phy/PhyP.nc"
static inline void PhyP__stopDone_task__runTask(void )
#line 159
{

  PhyP__SplitControl__stopDone(SUCCESS);
}

# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t PhyP__CC2420Power__startVReg(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = CC2420ControlP__CC2420Power__startVReg();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 96 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/phy/PhyP.nc"
static inline error_t PhyP__SplitControl__start(void )
#line 96
{



  PhyP__CC2420Power__startVReg();


  return SUCCESS;
}

# 83 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SplitControl.nc"
inline static error_t MacP__AMControl__start(void ){
#line 83
  unsigned char __nesc_result;
#line 83

#line 83
  __nesc_result = PhyP__SplitControl__start();
#line 83

#line 83
  return __nesc_result;
#line 83
}
#line 83
# 10 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsync.nc"
inline static error_t MacP__TimerAsync__start(void ){
#line 10
  unsigned char __nesc_result;
#line 10

#line 10
  __nesc_result = TimerAsyncM__TimerAsync__start();
#line 10

#line 10
  return __nesc_result;
#line 10
}
#line 10
# 564 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline void MacP__AMControl__startDone(error_t err)
#line 564
{
  if (err == SUCCESS) {

      MacP__TimerAsync__start();
    }
  else 
    {
      MacP__AMControl__start();
    }
}

# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SplitControl.nc"
inline static void PhyP__SplitControl__startDone(error_t error){
#line 92
  MacP__AMControl__startDone(error);
#line 92
}
#line 92
# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SpiResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ControlC.Spi*/CC2420SpiC__0__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 169 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ControlP.nc"
static inline error_t CC2420ControlP__Resource__release(void )
#line 169
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 170
    {
      CC2420ControlP__CSN__set();
      {
        unsigned char __nesc_temp = 
#line 172
        CC2420ControlP__SpiResource__release();

        {
#line 172
          __nesc_atomic_end(__nesc_atomic); 
#line 172
          return __nesc_temp;
        }
      }
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
}

# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static error_t PhyP__Resource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420ControlP__Resource__release();
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 259 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Power__rxOn(void )
#line 259
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 260
    {
      if (CC2420ControlP__m_state != CC2420ControlP__S_XOSC_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 262
            FAIL;

            {
#line 262
              __nesc_atomic_end(__nesc_atomic); 
#line 262
              return __nesc_temp;
            }
          }
        }
#line 264
      CC2420ControlP__SRXON__strobe();
    }
#line 265
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 90 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t PhyP__CC2420Power__rxOn(void ){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = CC2420ControlP__CC2420Power__rxOn();
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 58 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__enable(void )
#line 75
{
#line 75
  P1IE |= 1 << 0;
}

# 31 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__enable(void ){
#line 31
  HplMsp430InterruptP__Port10__enable();
#line 31
}
#line 31
# 107 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__edge(bool low_to_high){
#line 56
  HplMsp430InterruptP__Port10__edge(low_to_high);
#line 56
}
#line 56
# 91 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__clear(void )
#line 91
{
#line 91
  P1IFG &= ~(1 << 0);
}

# 41 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port10__clear();
#line 41
}
#line 41
# 83 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__disable(void )
#line 83
{
#line 83
  P1IE &= ~(1 << 0);
}

# 36 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__disable(void ){
#line 36
  HplMsp430InterruptP__Port10__disable();
#line 36
}
#line 36
# 58 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__disable();
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear();
  }
  return SUCCESS;
}

#line 41
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__enable(bool rising)
#line 41
{
  /* atomic removed: atomic calls only */
#line 42
  {
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable();
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__edge(rising);
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__enable();
  }
  return SUCCESS;
}





static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge(void )
#line 54
{
  return /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__enable(FALSE);
}

# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ReceiveP__InterruptFIFOP__enableFallingEdge(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 997 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__reset_state(void )
#line 997
{
  CC2420ReceiveP__m_bytes_left = CC2420ReceiveP__RXFIFO_SIZE;
  /* atomic removed: atomic calls only */
#line 999
  CC2420ReceiveP__receivingPacket = FALSE;


  CC2420ReceiveP__m_missed_packets = 0;
}

#line 197
static inline error_t CC2420ReceiveP__StdControl__start(void )
#line 197
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 199
    {
      CC2420ReceiveP__reset_state();
      CC2420ReceiveP__m_state = CC2420ReceiveP__S_STARTED;
      CC2420ReceiveP__receivingPacket = FALSE;
      CC2420ReceiveP__InterruptFIFOP__enableFallingEdge();
    }
#line 204
    __nesc_atomic_end(__nesc_atomic); }



  return SUCCESS;
}

# 119 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void )
{
  * (volatile uint16_t * )388U |= 0x0010;
}

# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents();
#line 46
}
#line 46
# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4404 {
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

# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(uint8_t cm){
#line 44
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(cm);
#line 44
}
#line 44
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )31U |= 0x01 << 1;
}

# 78 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc();
#line 78
}
#line 78
# 124 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void )
{
  * (volatile uint16_t * )388U &= ~0x0010;
}

# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents();
#line 47
}
#line 47
# 38 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(uint8_t mode)
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

static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge(void )
#line 50
{
  return /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(MSP430TIMER_CM_RISING);
}

# 42 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GpioCapture.nc"
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
# 93 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__StdControl__start(void )
#line 93
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 94
    {
      CC2420TransmitP__CaptureSFD__captureRisingEdge();
    }
#line 96
    __nesc_atomic_end(__nesc_atomic); }

  return SUCCESS;
}

# 74 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/StdControl.nc"
inline static error_t PhyP__SubControl__start(void ){
#line 74
  unsigned char __nesc_result;
#line 74

#line 74
  __nesc_result = CC2420TransmitP__StdControl__start();
#line 74
  __nesc_result = ecombine(__nesc_result, CC2420ReceiveP__StdControl__start());
#line 74

#line 74
  return __nesc_result;
#line 74
}
#line 74
# 151 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/phy/PhyP.nc"
static inline void PhyP__startDone_task__runTask(void )
#line 151
{
  PhyP__SubControl__start();
  PhyP__CC2420Power__rxOn();
  PhyP__Resource__release();

  PhyP__SplitControl__startDone(SUCCESS);
}

# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
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

# 34 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set();
#line 34
}
#line 34
# 37 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__set(void )
#line 37
{
#line 37
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__set();
}

# 29 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__VREN__set(void ){
#line 29
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__set();
#line 29
}
#line 29
# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Counter.nc"
inline static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__get(void ){
#line 53
  unsigned long __nesc_result;
#line 53

#line 53
  __nesc_result = /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 75 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/TransformAlarmC.nc"
static inline /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow(void )
{
  return /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__get();
}

#line 136
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 = t0;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt = dt;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__set_alarm();
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type dt)
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow(), dt);
}

# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Alarm.nc"
inline static void CC2420ControlP__StartupTimer__start(CC2420ControlP__StartupTimer__size_type dt){
#line 55
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(dt);
#line 55
}
#line 55
# 255 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/includes/mac_func.h"
static inline uint8_t set_pending_address_specification(uint8_t number_short, uint8_t number_extended)
{
  return (number_extended << 4) | (number_short << 0);
}

#line 274
static inline uint8_t set_gts_specification(uint8_t gts_descriptor_count, uint8_t gts_permit)
{
  return (gts_descriptor_count << 0) | (gts_permit << 7);
}

#line 292
static inline uint8_t set_gts_descriptor(uint8_t GTS_starting_slot, uint8_t GTS_length)
{

  return (GTS_starting_slot << 0) | (GTS_length << 4);
}

# 2272 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline void MacP__create_beacon__runTask(void )
{
  int i = 0;
  uint8_t packet_length = 25;
  int data_count = 0;
  int pending_data_index = 0;
  MPDU *pkt_ptr = 0;

  uint8_t short_addr_pending = 0;
  uint8_t long_addr_pending = 0;

  uint8_t gts_directions = 0x00;

  uint16_t frame_control;


  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 2288
    {

      beacon_addr_short *mac_beacon_addr_short_ptr;

      mac_beacon_addr_short_ptr = (beacon_addr_short *)&MacP__mac_beacon_txmpdu.data[0];


      MacP__mac_beacon_txmpdu_ptr->length = 15;

      frame_control = set_frame_control(0, 0, 0, 0, 1, 2, 2);

      MacP__mac_beacon_txmpdu_ptr->frame_control1 = (uint8_t )frame_control;

      MacP__mac_beacon_txmpdu_ptr->frame_control2 = (uint8_t )(frame_control >> 8);


      MacP__mac_beacon_txmpdu_ptr->seq_num = MacP__mac_PIB.macBSN;
      MacP__mac_PIB.macBSN++;



      mac_beacon_addr_short_ptr->destination_PAN_identifier = MacP__mac_PIB.macPANId;

      mac_beacon_addr_short_ptr->destination_address = 0xffff;

      mac_beacon_addr_short_ptr->source_address = MacP__mac_PIB.macShortAddress;
      if (MacP__mac_PIB.macShortAddress == 0x0000) 
        {
          mac_beacon_addr_short_ptr->superframe_specification = set_superframe_specification(MacP__mac_PIB.macBeaconOrder, (uint8_t )MacP__mac_PIB.macSuperframeOrder, MacP__final_CAP_slot, 0, 1, MacP__mac_PIB.macAssociationPermit);
        }
      else 
        {
          mac_beacon_addr_short_ptr->superframe_specification = set_superframe_specification(MacP__mac_PIB.macBeaconOrder, (uint8_t )MacP__mac_PIB.macSuperframeOrder, MacP__final_CAP_slot, 0, 1, MacP__mac_PIB.macAssociationPermit);
        }

      MacP__mac_beacon_txmpdu_ptr->data[8] = set_gts_specification(MacP__GTS_descriptor_count, MacP__mac_PIB.macGTSPermit);

      MacP__mac_beacon_txmpdu_ptr->data[9] = set_pending_address_specification(short_addr_pending, long_addr_pending);

      data_count = 9;
      packet_length = 15;



      if (MacP__GTS_descriptor_count + MacP__GTS_null_descriptor_count > 0) 
        {
          data_count++;

          for (i = 0; i < 7; i++) 
            {
              if (MacP__GTS_db[i].gts_id != 0x00 && MacP__GTS_db[i].DevAddressType != 0x0000) 
                {

                  MacP__mac_beacon_txmpdu_ptr->data[data_count] = MacP__GTS_db[i].DevAddressType;


                  data_count++;
                  MacP__mac_beacon_txmpdu_ptr->data[data_count] = MacP__GTS_db[i].DevAddressType >> 8;


                  data_count++;

                  MacP__mac_beacon_txmpdu_ptr->data[data_count] = set_gts_descriptor(15 - i, MacP__GTS_db[i].length);
                  data_count++;


                  packet_length = packet_length + 3;

                  if (MacP__GTS_db[i].direction == 1) 
                    {
                      gts_directions = gts_directions | (1 << i);
                    }
                  else 
                    {
                      gts_directions = gts_directions | (0 << i);
                    }
                }
            }

          MacP__mac_beacon_txmpdu_ptr->data[9] = gts_directions;

          packet_length++;

          if (MacP__GTS_null_descriptor_count > 0) 
            {
              for (i = 0; i < 7; i++) 
                {
                  if (MacP__GTS_null_db[i].DevAddressType != 0x0000) 
                    {
                      MacP__mac_beacon_txmpdu_ptr->data[data_count] = MacP__GTS_null_db[i].DevAddressType;

                      data_count++;
                      MacP__mac_beacon_txmpdu_ptr->data[data_count] = MacP__GTS_null_db[i].DevAddressType >> 8;

                      data_count++;
                      MacP__mac_beacon_txmpdu_ptr->data[data_count] = 0x00;
                      data_count++;

                      packet_length = packet_length + 3;
                    }
                }
            }

          MacP__mac_beacon_txmpdu_ptr->data[8] = set_gts_specification(MacP__GTS_descriptor_count + MacP__GTS_null_descriptor_count, MacP__mac_PIB.macGTSPermit);
        }



      pending_data_index = data_count;
      data_count++;




      if (MacP__indirect_trans_count > 0) 
        {


          for (i = 0; i < 2; i++) 
            {
              if (MacP__indirect_trans_queue[i].handler > 0x00) 
                {
                  pkt_ptr = (MPDU *)& MacP__indirect_trans_queue[i].frame;

                  if (get_fc2_dest_addr(pkt_ptr->frame_control2) == 2) 
                    {
                      short_addr_pending++;
                      packet_length = packet_length + 2;
                      MacP__mac_beacon_txmpdu_ptr->data[data_count] = pkt_ptr->data[2];
                      data_count++;
                      MacP__mac_beacon_txmpdu_ptr->data[data_count] = pkt_ptr->data[3];
                      data_count++;
                    }
                }
            }
          for (i = 0; i < 2; i++) 
            {
              if (MacP__indirect_trans_queue[i].handler > 0x00) 
                {
                  if (get_fc2_dest_addr(pkt_ptr->frame_control2) == 3) 
                    {
                      long_addr_pending++;
                      packet_length = packet_length + 8;

                      MacP__mac_beacon_txmpdu_ptr->data[data_count] = pkt_ptr->data[0];
                      data_count++;
                      MacP__mac_beacon_txmpdu_ptr->data[data_count] = pkt_ptr->data[1];
                      data_count++;
                      MacP__mac_beacon_txmpdu_ptr->data[data_count] = pkt_ptr->data[2];
                      data_count++;
                      MacP__mac_beacon_txmpdu_ptr->data[data_count] = pkt_ptr->data[3];
                      data_count++;
                      MacP__mac_beacon_txmpdu_ptr->data[data_count] = pkt_ptr->data[4];
                      data_count++;
                      MacP__mac_beacon_txmpdu_ptr->data[data_count] = pkt_ptr->data[5];
                      data_count++;
                      MacP__mac_beacon_txmpdu_ptr->data[data_count] = pkt_ptr->data[6];
                      data_count++;
                      MacP__mac_beacon_txmpdu_ptr->data[data_count] = pkt_ptr->data[7];
                      data_count++;
                    }
                }
            }
        }


      MacP__mac_beacon_txmpdu_ptr->data[pending_data_index] = set_pending_address_specification(short_addr_pending, long_addr_pending);



      if (MacP__mac_PIB.macBeaconPayloadLenght > 0) 
        {
          for (i = 0; i < MacP__mac_PIB.macBeaconPayloadLenght; i++) 
            {
              MacP__mac_beacon_txmpdu_ptr->data[data_count] = MacP__mac_PIB.macBeaconPayload[i];
              data_count++;
              packet_length++;
            }
        }






      MacP__mac_beacon_txmpdu_ptr->length = packet_length;

      MacP__send_beacon_length = packet_length;

      MacP__send_beacon_frame_ptr = (uint8_t *)MacP__mac_beacon_txmpdu_ptr;
    }
#line 2478
    __nesc_atomic_end(__nesc_atomic); }
}

#line 4644
static inline uint8_t MacP__min(uint8_t val1, uint8_t val2)
{
  if (val1 < val2) 
    {
      return val1;
    }
  else 
    {
      return val2;
    }
}

#line 4593
static inline void MacP__perform_csma_ca(void )
{
  uint8_t random_interval;

#line 4596
  MacP__csma_slotted = 1;

  if (MacP__csma_slotted == 0) 
    {
      /* atomic removed: atomic calls only */
#line 4600
      {

        MacP__init_csma_ca(MacP__csma_slotted);

        random_interval = powf(2, MacP__BE) - 1;
        MacP__delay_backoff_period = MacP__Random__rand16() & random_interval;

        MacP__csma_delay = 1;
      }

      return;
    }
  else 
    {
      /* atomic removed: atomic calls only */
      {

        if (MacP__cca_deference == 0) 
          {
            MacP__init_csma_ca(MacP__csma_slotted);
            if (MacP__mac_PIB.macBattLifeExt == 1) 
              {
                MacP__BE = MacP__min(2, MacP__mac_PIB.macMinBE);
              }
            else 
              {
                MacP__BE = MacP__mac_PIB.macMinBE;
              }
            MacP__csma_locate_backoff_boundary = 1;
          }
        else 
          {
            MacP__cca_deference = 0;
            MacP__csma_delay = 0;
            MacP__csma_locate_backoff_boundary = 0;
            MacP__csma_cca_backoff_boundary = 1;
          }
      }

      return;
    }
}

#line 4329
static inline void MacP__send_frame_csma__runTask(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 4331
    {




      if ((MacP__send_buffer_count > 0 && MacP__send_buffer_count <= 3 && MacP__performing_csma_ca == 0) || MacP__I_AM_IN_IP != 0) 
        {



          MacP__performing_csma_ca = 1;

          MacP__perform_csma_ca();
        }
      else 
        {
        }
    }
#line 4348
    __nesc_atomic_end(__nesc_atomic); }
}

# 47 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle(void )
#line 47
{
  /* atomic removed: atomic calls only */
#line 47
  * (volatile uint8_t * )49U ^= 0x01 << 6;
}

# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__toggle(void ){
#line 44
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle();
#line 44
}
#line 44
# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle(void )
#line 39
{
#line 39
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__toggle();
}

# 31 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__toggle(void ){
#line 31
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle();
#line 31
}
#line 31
# 103 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/LedsP.nc"
static inline void LedsP__Leds__led2Toggle(void )
#line 103
{
  LedsP__Led2__toggle();
  ;
#line 105
  ;
}

# 89 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Leds.nc"
inline static void GTSManagementExampleP__Leds__led2Toggle(void ){
#line 89
  LedsP__Leds__led2Toggle();
#line 89
}
#line 89
# 244 "GTSManagementExampleP.nc"
static inline error_t GTSManagementExampleP__MLME_BEACON_NOTIFY__indication(uint8_t BSN, PANDescriptor pan_descriptor, uint8_t PenAddrSpec, uint8_t AddrList, uint8_t sduLength, uint8_t sdu[])
{
  GTSManagementExampleP__Leds__led2Toggle();

  GTSManagementExampleP__gts_superframe_count++;
  if (GTSManagementExampleP__gts_superframe_count == 30) 
    {

      GTSManagementExampleP__MLME_GTS__request(set_gts_characteristics(1, GTS_TX_ONLY, 0), 0x00);
    }
  return SUCCESS;
}

# 12 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_BEACON_NOTIFY.nc"
inline static error_t MacP__MLME_BEACON_NOTIFY__indication(uint8_t BSN, PANDescriptor pan_descriptor, uint8_t PenAddrSpec, uint8_t AddrList, uint8_t sduLength, uint8_t sdu[]){
#line 12
  unsigned char __nesc_result;
#line 12

#line 12
  __nesc_result = GTSManagementExampleP__MLME_BEACON_NOTIFY__indication(BSN, pan_descriptor, PenAddrSpec, AddrList, sduLength, sdu);
#line 12

#line 12
  return __nesc_result;
#line 12
}
#line 12
# 254 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsyncM.nc"
static inline uint8_t TimerAsyncM__TimerAsync__reset_start(uint32_t start_ticks)
{



  TimerAsyncM__current_time_slot = start_ticks / TimerAsyncM__time_slot_ticks;

  if (TimerAsyncM__current_time_slot == 0) 
    {
      TimerAsyncM__time_slot_tick_next_fire = TimerAsyncM__time_slot_ticks;
      TimerAsyncM__current_number_backoff = start_ticks / TimerAsyncM__backoff_ticks;
      TimerAsyncM__current_number_backoff_on_time_slot = TimerAsyncM__current_number_backoff;
    }
  else 
    {
      TimerAsyncM__time_slot_tick_next_fire = (TimerAsyncM__current_time_slot + 1) * TimerAsyncM__time_slot_ticks;
      TimerAsyncM__current_number_backoff = start_ticks / TimerAsyncM__backoff_ticks;
      TimerAsyncM__current_number_backoff_on_time_slot = TimerAsyncM__current_number_backoff - TimerAsyncM__current_time_slot * TimerAsyncM__time_slot_backoff_periods;
    }

  TimerAsyncM__backoff_ticks_counter = 0;
  TimerAsyncM__backoffs = 1;


  TimerAsyncM__total_tick_counter = TimerAsyncM__total_tick_counter + start_ticks;
  TimerAsyncM__ticks_counter = start_ticks;
#line 292
  return TimerAsyncM__current_time_slot;
}

# 42 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsync.nc"
inline static uint8_t MacP__TimerAsync__reset_start(uint32_t start_ticks){
#line 42
  unsigned char __nesc_result;
#line 42

#line 42
  __nesc_result = TimerAsyncM__TimerAsync__reset_start(start_ticks);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr(void )
#line 46
{
  /* atomic removed: atomic calls only */
#line 46
  * (volatile uint8_t * )49U &= ~(0x01 << 5);
}

# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr();
#line 39
}
#line 39
# 38 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr(void )
#line 38
{
#line 38
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__clr();
}

# 30 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__clr(void ){
#line 30
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr();
#line 30
}
#line 30
# 78 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/LedsP.nc"
static inline void LedsP__Leds__led1On(void )
#line 78
{
  LedsP__Led1__clr();
  ;
#line 80
  ;
}

# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Leds.nc"
inline static void MacP__Leds__led1On(void ){
#line 61
  LedsP__Leds__led1On();
#line 61
}
#line 61
# 265 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/includes/mac_func.h"
static inline uint8_t get_number_extended(uint8_t pending_specification)
{
  return (pending_specification >> 4) & 0x07;
}

#line 260
static inline uint8_t get_number_short(uint8_t pending_specification)
{
  return pending_specification & 0x07;
}

# 14 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_GTS.nc"
inline static error_t MacP__MLME_GTS__confirm(uint8_t GTSCharacteristics, uint8_t status){
#line 14
  unsigned char __nesc_result;
#line 14

#line 14
  __nesc_result = GTSManagementExampleP__MLME_GTS__confirm(GTSCharacteristics, status);
#line 14

#line 14
  return __nesc_result;
#line 14
}
#line 14
# 298 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/includes/mac_func.h"
static inline uint8_t get_gts_descriptor_len(uint8_t gts_des_part)
{
  return (gts_des_part & 0xf0) >> 4;
}

static inline uint8_t get_gts_descriptor_ss(uint8_t gts_des_part)
{
  return gts_des_part & 0x0f;
}

#line 148
static inline uint8_t get_superframe_order(uint16_t superframe)
{
  return ((uint8_t )superframe >> 4) & 0xF;
}

#line 143
static inline uint8_t get_beacon_order(uint16_t superframe)
{
  return (uint8_t )superframe & 0xF;
}

# 1231 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline void MacP__process_beacon(MPDU *packet, uint8_t ppduLinkQuality)
{
#line 1245
  uint32_t SO_EXPONENT;
  uint32_t BO_EXPONENT;
  int i = 0;
  uint16_t gts_descriptor_addr;
  uint8_t data_count;

  uint8_t gts_directions;
  uint8_t gts_des_count;

  uint8_t gts_ss;
  uint8_t gts_l;
  uint8_t dir;
  uint8_t dir_mask;




  beacon_addr_short *beacon_ptr;

  PANDescriptor pan_descriptor;




  uint8_t short_addr_pending = 0;
  uint8_t long_addr_pending = 0;








  MacP__beacon_processed = 1;
  MacP__missed_beacons = 0;


  beacon_ptr = (beacon_addr_short *)packet->data;
  /* atomic removed: atomic calls only */


  MacP__buffer_count--;





  if (beacon_ptr->source_address != MacP__mac_PIB.macCoordShortAddress) 
    {

      return;
    }






  if (MacP__PANCoordinator == 0) 
    {
      MacP__mac_PIB.macBeaconOrder = get_beacon_order(beacon_ptr->superframe_specification);
      MacP__mac_PIB.macSuperframeOrder = get_superframe_order(beacon_ptr->superframe_specification);








      if (MacP__mac_PIB.macSuperframeOrder == 0) 
        {
          SO_EXPONENT = 1;
        }
      else 
        {
          SO_EXPONENT = powf(2, MacP__mac_PIB.macSuperframeOrder);
        }

      if (MacP__mac_PIB.macBeaconOrder == 0) 
        {
          BO_EXPONENT = 1;
        }
      else 
        {
          BO_EXPONENT = powf(2, MacP__mac_PIB.macBeaconOrder);
        }
      MacP__BI = 960 * BO_EXPONENT;
      MacP__SD = 960 * SO_EXPONENT;


      MacP__backoff = 20;
      MacP__time_slot = MacP__SD / 16;

      MacP__TimerAsync__set_bi_sd(MacP__BI, MacP__SD);
    }




  MacP__allow_gts = 1;


  MacP__s_GTSss = 0;
  MacP__s_GTS_length = 0;

  MacP__r_GTSss = 0;
  MacP__r_GTS_length = 0;







  MacP__final_CAP_slot = 15;


  gts_des_count = packet->data[8] & 0x0f;

  data_count = 9;

  MacP__final_CAP_slot = 15 - gts_des_count;

  if (gts_des_count > 0) 
    {
      data_count = 10;


      gts_directions = packet->data[9];



      for (i = 0; i < gts_des_count; i++) 
        {
          gts_descriptor_addr = (uint16_t )packet->data[data_count];



          data_count = data_count + 2;

          if (gts_descriptor_addr == MacP__mac_PIB.macShortAddress) 
            {



              gts_ss = get_gts_descriptor_ss(packet->data[data_count]);
              gts_l = get_gts_descriptor_len(packet->data[data_count]);

              if (i == 0) 
                {
                  dir_mask = 1;
                }
              else 
                {

                  dir_mask = powf(2, i);
                }

              dir = gts_directions & dir_mask;
              if (dir == 0) 
                {
                  MacP__s_GTSss = gts_ss;
                  MacP__s_GTS_length = gts_l;
                }
              else 
                {

                  MacP__r_GTSss = gts_ss;
                  MacP__r_GTS_length = gts_l;
                }




              if (gts_l == 0) 
                {
                  MacP__allow_gts = 0;
                }

              if (MacP__gts_confirm == 1 && gts_l != 0) 
                {


                  MacP__gts_confirm = 0;
                  MacP__MLME_GTS__confirm(MacP__GTS_specification, MAC_SUCCESS);
                }
              else 
                {


                  MacP__gts_confirm = 0;
                  MacP__MLME_GTS__confirm(MacP__GTS_specification, MAC_DENIED);
                }
            }

          data_count++;
        }
    }







  short_addr_pending = get_number_short(packet->data[data_count]);
  long_addr_pending = get_number_extended(packet->data[data_count]);



  data_count++;

  if (short_addr_pending > 0) 
    {
      for (i = 0; i < short_addr_pending; i++) 
        {



          if ((uint16_t )packet->data[data_count] == MacP__mac_PIB.macShortAddress) 
            {

              MacP__create_data_request_cmd();
            }
          data_count = data_count + 2;
        }
    }
  if (long_addr_pending > 0) 
    {
      for (i = 0; i < long_addr_pending; i++) 
        {
          if ((uint32_t )packet->data[data_count] == MacP__aExtendedAddress0 && (uint32_t )packet->data[data_count + 4] == MacP__aExtendedAddress1) 
            {

              data_count = data_count + 8;
            }
        }
    }
#line 1496
  pan_descriptor.CoordAddrMode = 2;
  pan_descriptor.CoordPANId = 0x0000;
  pan_descriptor.CoordAddress0 = 0x00000000;
  pan_descriptor.CoordAddress1 = MacP__mac_PIB.macCoordShortAddress;
  pan_descriptor.LogicalChannel = MacP__current_channel;

  pan_descriptor.SuperframeSpec = beacon_ptr->superframe_specification;

  pan_descriptor.GTSPermit = MacP__mac_PIB.macGTSPermit;
  pan_descriptor.LinkQuality = 0x00;
  pan_descriptor.TimeStamp = 0x000000;
  pan_descriptor.SecurityUse = 0;
  pan_descriptor.ACLEntry = 0x00;
  pan_descriptor.SecurityFailure = 0x00;
#line 1524
  if (MacP__PANCoordinator == 0) 
    {
      MacP__I_AM_IN_CAP = 1;
      MacP__I_AM_IN_IP = 0;


      MacP__Leds__led2On();

      MacP__Leds__led1On();

      if (MacP__findabeacon == 1) 
        {

          MacP__TimerAsync__set_timers_enable(1);
          MacP__findabeacon = 0;
        }







      MacP__number_time_slot = MacP__TimerAsync__reset_start(75);



      MacP__on_sync = 1;
    }


  MacP__MLME_BEACON_NOTIFY__indication((uint8_t )packet->seq_num, pan_descriptor, 0, 0, MacP__mac_PIB.macBeaconPayloadLenght, packet->data);

  return;
}

# 236 "GTSManagementExampleP.nc"
static inline error_t GTSManagementExampleP__MLME_GTS__indication(uint16_t DevAddress, uint8_t GTSCharacteristics, bool SecurityUse, uint8_t ACLEntry)
{
  return SUCCESS;
}

# 16 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_GTS.nc"
inline static error_t MacP__MLME_GTS__indication(uint16_t DevAddress, uint8_t GTSCharacteristics, uint8_t SecurityUse, uint8_t ACLEntry){
#line 16
  unsigned char __nesc_result;
#line 16

#line 16
  __nesc_result = GTSManagementExampleP__MLME_GTS__indication(DevAddress, GTSCharacteristics, SecurityUse, ACLEntry);
#line 16

#line 16
  return __nesc_result;
#line 16
}
#line 16
# 324 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/includes/mac_func.h"
static inline bool get_gts_direction(uint8_t gts_characteristics)
{
  if ((gts_characteristics & 0x10) == 0x10) {
    return 1;
    }
  else {
#line 329
    return 0;
    }
}

#line 319
static inline uint8_t get_gts_length(uint8_t gts_characteristics)
{
  return gts_characteristics & 0xF;
}

# 4864 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline error_t MacP__add_gts_entry(uint8_t gts_length, bool direction, uint16_t DevAddressType)
{
  int i;





  if (MacP__GTS_startslot - gts_length < 5) 
    {
    }




  if (MacP__GTS_startslot - gts_length < 9) 
    {
      return FAIL;
    }


  for (i = 0; i < 7; i++) 
    {
      if (MacP__GTS_db[i].DevAddressType == DevAddressType && MacP__GTS_db[i].direction == direction && MacP__GTS_db[i].gts_id > 0) 
        {

          return FAIL;
        }
      if (MacP__GTS_null_db[i].DevAddressType == DevAddressType && MacP__GTS_null_db[i].gts_id > 0) 
        {

          return FAIL;
        }
    }
  /* atomic removed: atomic calls only */


  {


    MacP__GTS_startslot = MacP__GTS_startslot - gts_length;

    MacP__GTS_db[15 - MacP__GTS_startslot].gts_id = MacP__GTS_id;
    MacP__GTS_db[15 - MacP__GTS_startslot].starting_slot = MacP__GTS_startslot;
    MacP__GTS_db[15 - MacP__GTS_startslot].length = gts_length;
    MacP__GTS_db[15 - MacP__GTS_startslot].direction = direction;
    MacP__GTS_db[15 - MacP__GTS_startslot].DevAddressType = DevAddressType;
    MacP__GTS_db[15 - MacP__GTS_startslot].expiration = 0x00;



    MacP__GTS_id++;
    MacP__GTS_descriptor_count++;

    MacP__final_CAP_slot = MacP__final_CAP_slot - gts_length;
  }

  return SUCCESS;
}

#line 1561
static inline void MacP__process_gts_request(MPDU *pdu)
{
  error_t status;
  cmd_gts_request *mac_gts_request;

  mac_gts_request = (cmd_gts_request *)& pdu->data;
  /* atomic removed: atomic calls only */
  {
    if (get_characteristic_type(mac_gts_request->gts_characteristics) == 1) 
      {



        status = MacP__add_gts_entry(get_gts_length(mac_gts_request->gts_characteristics), get_gts_direction(mac_gts_request->gts_characteristics), mac_gts_request->source_address);
      }
    else 

      {


        status = MacP__remove_gts_entry(mac_gts_request->source_address);
      }

    MacP__MLME_GTS__indication(mac_gts_request->source_address, mac_gts_request->gts_characteristics, 0, 0);
  }


  return;
}

# 106 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/includes/mac_func.h"
static inline bool get_fc1_ack_request(uint8_t frame_control)
{

  if ((frame_control & 0x20) == 0x20) {
    return 1;
    }
  else {
#line 112
    return 0;
    }
}

# 179 "GTSManagementExampleP.nc"
static inline error_t GTSManagementExampleP__MLME_ORPHAN__indication(uint32_t OrphanAddress[1], uint8_t SecurityUse, uint8_t ACLEntry)
{

  return SUCCESS;
}

# 13 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_ORPHAN.nc"
inline static error_t MacP__MLME_ORPHAN__indication(uint32_t OrphanAddress[1], uint8_t SecurityUse, uint8_t ACLEntry){
#line 13
  unsigned char __nesc_result;
#line 13

#line 13
  __nesc_result = GTSManagementExampleP__MLME_ORPHAN__indication(OrphanAddress, SecurityUse, ACLEntry);
#line 13

#line 13
  return __nesc_result;
#line 13
}
#line 13
# 5211 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline void MacP__send_ind_trans_addr(uint32_t DeviceAddress[])
{

  uint8_t destination_address = 0;

  dest_short *dest_short_ptr = 0;
  dest_long *dest_long_ptr = 0;

  int i = 0;
  MPDU *frame_ptr = 0;





  for (i = 0; i < 2; i++) 
    {
      if (MacP__indirect_trans_queue[i].handler > 0x00) 
        {
          frame_ptr = (MPDU *)MacP__indirect_trans_queue[i].frame;
          destination_address = get_fc2_dest_addr(frame_ptr->frame_control2);

          switch (destination_address) 
            {
              case 3: dest_long_ptr = (dest_long *)frame_ptr->data;
              break;
              case 2: dest_short_ptr = (dest_short *)frame_ptr->data;
              break;
            }



          if ((dest_long_ptr->destination_address0 == DeviceAddress[1] && dest_long_ptr->destination_address1 == DeviceAddress[0]) || dest_short_ptr->destination_address == (uint16_t )DeviceAddress[0]) 
            {

              if (MacP__send_buffer_msg_in == 3) {
                MacP__send_buffer_msg_in = 0;
                }
              memcpy(&MacP__send_buffer[MacP__send_buffer_msg_in], (MPDU *)& MacP__indirect_trans_queue[i].frame, sizeof(MPDU ));


              MacP__send_buffer[MacP__send_buffer_msg_in].retransmission = 0;
              MacP__send_buffer[MacP__send_buffer_msg_in].indirect = i + 1;


              MacP__indirect_trans_queue[i].handler = 0x00;


              MacP__indirect_trans_count--;
              if (MacP__indirect_trans_count > 2) 
                {
                  MacP__indirect_trans_count = 0;
                }
              /* atomic removed: atomic calls only */
              MacP__send_buffer_count++;
              /* atomic removed: atomic calls only */
#line 5266
              MacP__send_buffer_msg_in++;

              MacP__send_frame_csma__postTask();



              return;
            }
        }
    }



  return;
}

# 303 "GTSManagementExampleP.nc"
static inline error_t GTSManagementExampleP__MLME_DISASSOCIATE__indication(uint32_t DeviceAddress[], uint8_t DisassociateReason, bool SecurityUse, uint8_t ACLEntry)
{
  return SUCCESS;
}

# 14 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_DISASSOCIATE.nc"
inline static error_t MacP__MLME_DISASSOCIATE__indication(uint32_t DeviceAddress[], uint8_t DisassociateReason, uint8_t SecurityUse, uint8_t ACLEntry){
#line 14
  unsigned char __nesc_result;
#line 14

#line 14
  __nesc_result = GTSManagementExampleP__MLME_DISASSOCIATE__indication(DeviceAddress, DisassociateReason, SecurityUse, ACLEntry);
#line 14

#line 14
  return __nesc_result;
#line 14
}
#line 14
# 2220 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline void MacP__process_dissassociation_notification(MPDU *pdu)
{
  /* atomic removed: atomic calls only */
#line 2222
  {
    cmd_disassociation_notification *mac_disassociation_notification;


    mac_disassociation_notification = (cmd_disassociation_notification *)pdu->data;

    MacP__MLME_DISASSOCIATE__indication(& mac_disassociation_notification->source_address0, mac_disassociation_notification->disassociation_reason, 0, 0);
  }

  return;
}

# 153 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num].isrunning = FALSE;
}

# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
inline static void MacP__T_ResponseWaitTime__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(1U);
#line 67
}
#line 67
# 289 "GTSManagementExampleP.nc"
static inline error_t GTSManagementExampleP__MLME_ASSOCIATE__indication(uint32_t DeviceAddress[], uint8_t CapabilityInformation, bool SecurityUse, uint8_t ACLEntry)
{

  return SUCCESS;
}

# 13 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_ASSOCIATE.nc"
inline static error_t MacP__MLME_ASSOCIATE__indication(uint32_t DeviceAddress[], uint8_t CapabilityInformation, bool SecurityUse, uint8_t ACLEntry){
#line 13
  unsigned char __nesc_result;
#line 13

#line 13
  __nesc_result = GTSManagementExampleP__MLME_ASSOCIATE__indication(DeviceAddress, CapabilityInformation, SecurityUse, ACLEntry);
#line 13

#line 13
  return __nesc_result;
#line 13
}
#line 13
# 1935 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline void MacP__indication_cmd(MPDU *pdu, int8_t ppduLinkQuality)
{
  uint8_t cmd_type;

  uint8_t addressing_fields_length = 0;

  uint32_t SrcAddr[2];



  uint8_t source_address = 0;
  uint8_t destination_address = 0;






  source_long *source_long_ptr;

  dest_short *dest_short_ptr;
  dest_long *dest_long_ptr;





  destination_address = get_fc2_dest_addr(pdu->frame_control2);
  source_address = get_fc2_source_addr(pdu->frame_control2);
  /* atomic removed: atomic calls only */

  MacP__buffer_count--;

  switch (destination_address) 
    {
      case 3: addressing_fields_length = 10;
      dest_long_ptr = (dest_long *)&pdu->data[0];
      if (dest_long_ptr->destination_address0 != MacP__aExtendedAddress0 && dest_long_ptr->destination_address1 != MacP__aExtendedAddress1) 
        {

          return;
        }

      break;
      case 2: addressing_fields_length = 4;
      dest_short_ptr = (dest_short *)&pdu->data[0];

      if (dest_short_ptr->destination_address != MacP__mac_PIB.macShortAddress && dest_short_ptr->destination_address != 0xffff) 
        {


          return;
        }
      break;
    }
  switch (source_address) 
    {
      case 3: addressing_fields_length = addressing_fields_length + 10;
      break;
      case 2: addressing_fields_length = addressing_fields_length + 4;
      break;
    }

  cmd_type = pdu->data[addressing_fields_length];


  switch (cmd_type) 
    {

      case CMD_ASSOCIATION_REQUEST: 





        if (MacP__mac_PIB.macAssociationPermit == 0) 
          {

            if (get_fc1_ack_request(pdu->frame_control1) == 1) 
              {
                MacP__build_ack(pdu->seq_num, 0);
              }
            return;
          }

      if (MacP__PANCoordinator == 0) 
        {

          return;
        }
      /* atomic removed: atomic calls only */
#line 2025
      {
        source_long_ptr = (source_long *)&pdu->data[4];

        SrcAddr[1] = source_long_ptr->source_address0;
        SrcAddr[0] = source_long_ptr->source_address1;


        MacP__MLME_ASSOCIATE__indication(SrcAddr, pdu->data[addressing_fields_length + 1], 0, 0);
      }


      if (get_fc1_ack_request(pdu->frame_control1) == 1) 
        {
          MacP__build_ack(pdu->seq_num, 1);
        }


      break;

      case CMD_ASSOCIATION_RESPONSE: /* atomic removed: atomic calls only */
#line 2044
        {


          MacP__associating = 0;
          MacP__T_ResponseWaitTime__stop();

          if (get_fc1_ack_request(pdu->frame_control1) == 1) 
            {
              MacP__build_ack(pdu->seq_num, 0);
            }

          MacP__MLME_ASSOCIATE__confirm((uint16_t )(pdu->data[addressing_fields_length + 1] + (pdu->data[addressing_fields_length + 2] << 8)), pdu->data[addressing_fields_length + 3]);
        }
      break;

      case CMD_DISASSOCIATION_NOTIFICATION: 

        if (get_fc1_ack_request(pdu->frame_control1) == 1) 
          {
            MacP__build_ack(pdu->seq_num, 0);
          }

      MacP__process_dissassociation_notification(pdu);
      break;
      case CMD_DATA_REQUEST: 


        if (get_fc1_ack_request(pdu->frame_control1) == 1) 
          {
          }







      source_long_ptr = (source_long *)&pdu->data[0];

      SrcAddr[1] = source_long_ptr->source_address0;
      SrcAddr[0] = source_long_ptr->source_address1;

      MacP__send_ind_trans_addr(SrcAddr);

      break;
      case CMD_PANID_CONFLICT: 
        break;

      case CMD_ORPHAN_NOTIFICATION: 


        source_long_ptr = (source_long *)&pdu->data[4];

      SrcAddr[1] = source_long_ptr->source_address0;
      SrcAddr[0] = source_long_ptr->source_address1;

      MacP__MLME_ORPHAN__indication(SrcAddr, 0x00, 0x00);


      break;
      case CMD_BEACON_REQUEST: 
        break;
      case CMD_COORDINATOR_REALIGNMENT: 


        MacP__process_coordinator_realignment(pdu);

      break;
      case CMD_GTS_REQUEST: 

        if (get_fc1_ack_request(pdu->frame_control1) == 1) 
          {
            MacP__build_ack(pdu->seq_num, 0);
          }
      MacP__process_gts_request(pdu);
      break;
      default: break;
    }


  return;
}

# 96 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/includes/mac_func.h"
static inline bool get_fc1_frame_pending(uint8_t frame_control)
{

  if ((frame_control & 0x10) == 0x10) {
    return 1;
    }
  else {
#line 102
    return 0;
    }
}

# 62 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
inline static void MacP__T_ResponseWaitTime__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(1U, dt);
#line 62
}
#line 62





inline static void MacP__T_ackwait__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(0U);
#line 67
}
#line 67
# 2127 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline void MacP__indication_ack(MPDU *pdu, int8_t ppduLinkQuality)
{
  /* atomic removed: atomic calls only */

  MacP__buffer_count--;
  /* atomic removed: atomic calls only */


  {
    if (MacP__send_ack_check == 1 && MacP__ack_sequence_number_check == pdu->seq_num) 
      {

        MacP__T_ackwait__stop();

        MacP__send_buffer_count--;
        MacP__send_buffer_msg_out++;


        if (MacP__send_buffer_count > 3) 
          {
            MacP__send_buffer_count = 0;
            MacP__send_buffer_msg_out = 0;
            MacP__send_buffer_msg_in = 0;
          }

        if (MacP__send_buffer_msg_out == 3) {
          MacP__send_buffer_msg_out = 0;
          }

        if (MacP__associating == 1 && MacP__association_cmd_seq_num == pdu->seq_num) 
          {

            MacP__T_ResponseWaitTime__startOneShot(MacP__response_wait_time);
          }


        if (MacP__gts_request == 1 && MacP__gts_request_seq_num == pdu->seq_num) 
          {

            MacP__T_ResponseWaitTime__startOneShot(MacP__response_wait_time);
          }




        if (MacP__send_indirect_transmission > 0) 
          {

            MacP__indirect_trans_queue[MacP__send_indirect_transmission - 1].handler = 0x00;
            MacP__indirect_trans_count--;
          }


        MacP__send_ack_check = 0;
        MacP__retransmit_count = 0;
        MacP__ack_sequence_number_check = 0;


        if (MacP__send_buffer_count > 0) {
          MacP__send_frame_csma__postTask();
          }
      }
  }



  if (get_fc1_frame_pending(pdu->frame_control1) == 1 && MacP__pending_request_data == 1) 
    {

      MacP__pending_request_data = 0;
      MacP__create_data_request_cmd();
    }
#line 2210
  if (MacP__gts_send_pending_data == 1) {
    MacP__start_gts_send__postTask();
    }
  if (MacP__coordinator_gts_send_pending_data == 1 && MacP__coordinator_gts_send_time_slot == MacP__number_time_slot) {
    MacP__start_coordinator_gts_send__postTask();
    }
  return;
}

# 327 "GTSManagementExampleP.nc"
static inline error_t GTSManagementExampleP__MCPS_DATA__indication(uint16_t SrcAddrMode, uint16_t SrcPANId, uint32_t SrcAddr[2], uint16_t DstAddrMode, uint16_t DestPANId, uint32_t DstAddr[2], uint16_t msduLength, uint8_t msdu[100], uint16_t mpduLinkQuality, uint16_t SecurityUse, uint16_t ACLEntry)
{


  return SUCCESS;
}

# 16 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MCPS_DATA.nc"
inline static error_t MacP__MCPS_DATA__indication(uint16_t SrcAddrMode, uint16_t SrcPANId, uint32_t SrcAddr[2], uint16_t DstAddrMode, uint16_t DestPANId, uint32_t DstAddr[2], uint16_t msduLength, uint8_t msdu[100], uint16_t mpduLinkQuality, uint16_t SecurityUse, uint16_t ACLEntry){
#line 16
  unsigned char __nesc_result;
#line 16

#line 16
  __nesc_result = GTSManagementExampleP__MCPS_DATA__indication(SrcAddrMode, SrcPANId, SrcAddr, DstAddrMode, DestPANId, DstAddr, msduLength, msdu, mpduLinkQuality, SecurityUse, ACLEntry);
#line 16

#line 16
  return __nesc_result;
#line 16
}
#line 16
# 116 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/includes/mac_func.h"
static inline bool get_fc1_intra_pan(uint8_t frame_control)
{

  if ((frame_control & 0x40) == 0x40) {
    return 1;
    }
  else {
#line 122
    return 0;
    }
}

# 1592 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline void MacP__indication_data(MPDU *pdu, int8_t ppduLinkQuality)
{
  uint8_t data_len;

  uint8_t payload[80];
  uint8_t msdu_length = 0;



  uint32_t SrcAddr[2];
  uint32_t DstAddr[2];



  uint8_t source_address = 0;
  uint8_t destination_address = 0;


  dest_short *dest_short_ptr;
  dest_long *dest_long_ptr;

  source_short *source_short_ptr;
  source_long *source_long_ptr;






  source_address = get_fc2_source_addr(pdu->frame_control2);
  destination_address = get_fc2_dest_addr(pdu->frame_control2);
  /* atomic removed: atomic calls only */

  MacP__buffer_count--;

  SrcAddr[0] = 0x00000000;
  SrcAddr[1] = 0x00000000;
  DstAddr[0] = 0x00000000;
  DstAddr[1] = 0x00000000;





  if (get_fc1_intra_pan(pdu->frame_control1) == 0) 
    {

      if (destination_address > 1 && source_address > 1) 
        {

          if (destination_address == 3 && source_address == 3) 
            {
              dest_long_ptr = (dest_long *)&pdu->data[0];
              source_long_ptr = (source_long *)&pdu->data[10];




              if (dest_long_ptr->destination_address0 != MacP__aExtendedAddress0 && dest_long_ptr->destination_address1 != MacP__aExtendedAddress1) 
                {

                  return;
                }


              if (dest_long_ptr->destination_PAN_identifier != 0xffff && dest_long_ptr->destination_PAN_identifier != MacP__mac_PIB.macPANId) 
                {

                  return;
                }
              data_len = 20;


              DstAddr[1] = dest_long_ptr->destination_address0;
              DstAddr[0] = dest_long_ptr->destination_address1;

              SrcAddr[1] = source_long_ptr->source_address0;
              SrcAddr[0] = source_long_ptr->source_address1;

              msdu_length = pdu->length - data_len;

              memcpy(&payload, &pdu->data[data_len], msdu_length * sizeof(uint8_t ));

              MacP__MCPS_DATA__indication((uint16_t )source_address, (uint16_t )source_long_ptr->source_PAN_identifier, SrcAddr, (uint16_t )destination_address, (uint16_t )dest_long_ptr->destination_PAN_identifier, DstAddr, (uint16_t )msdu_length, payload, (uint16_t )ppduLinkQuality, 0x0000, 0x0000);
            }



          if (destination_address == 2 && source_address == 3) 
            {
              dest_short_ptr = (dest_short *)&pdu->data[0];
              source_long_ptr = (source_long *)&pdu->data[4];




              if (dest_short_ptr->destination_address != 0xffff && dest_short_ptr->destination_address != MacP__mac_PIB.macShortAddress) 
                {

                  return;
                }


              if (dest_short_ptr->destination_PAN_identifier != 0xffff && dest_short_ptr->destination_PAN_identifier != MacP__mac_PIB.macPANId) 
                {

                  return;
                }

              data_len = 14;

              DstAddr[0] = dest_short_ptr->destination_address;

              SrcAddr[1] = source_long_ptr->source_address0;
              SrcAddr[0] = source_long_ptr->source_address1;

              msdu_length = pdu->length - data_len;

              memcpy(&payload, &pdu->data[data_len], msdu_length * sizeof(uint8_t ));

              MacP__MCPS_DATA__indication((uint16_t )source_address, (uint16_t )source_long_ptr->source_PAN_identifier, SrcAddr, (uint16_t )destination_address, (uint16_t )dest_short_ptr->destination_PAN_identifier, DstAddr, (uint16_t )msdu_length, payload, (uint16_t )ppduLinkQuality, 0x0000, 0x0000);
            }


          if (destination_address == 3 && source_address == 2) 
            {
              dest_long_ptr = (dest_long *)&pdu->data[0];
              source_short_ptr = (source_short *)&pdu->data[10];




              if (dest_long_ptr->destination_address0 != MacP__aExtendedAddress0 && dest_long_ptr->destination_address1 != MacP__aExtendedAddress1) 
                {

                  return;
                }


              if (dest_long_ptr->destination_PAN_identifier != 0xffff && dest_long_ptr->destination_PAN_identifier != MacP__mac_PIB.macPANId) 
                {

                  return;
                }

              data_len = 14;

              DstAddr[1] = dest_long_ptr->destination_address0;
              DstAddr[0] = dest_long_ptr->destination_address1;


              SrcAddr[0] = source_short_ptr->source_address;

              msdu_length = pdu->length - data_len;

              memcpy(&payload, &pdu->data[data_len], msdu_length * sizeof(uint8_t ));

              MacP__MCPS_DATA__indication((uint16_t )source_address, (uint16_t )source_short_ptr->source_PAN_identifier, SrcAddr, (uint16_t )destination_address, (uint16_t )dest_long_ptr->destination_PAN_identifier, DstAddr, (uint16_t )msdu_length, payload, (uint16_t )ppduLinkQuality, 0x0000, 0x0000);
            }




          if (destination_address == 2 && source_address == 2) 
            {
              dest_short_ptr = (dest_short *)&pdu->data[0];
              source_short_ptr = (source_short *)&pdu->data[4];




              if (dest_short_ptr->destination_address != 0xffff && dest_short_ptr->destination_address != MacP__mac_PIB.macShortAddress) 
                {

                  return;
                }


              if (dest_short_ptr->destination_PAN_identifier != 0xffff && dest_short_ptr->destination_PAN_identifier != MacP__mac_PIB.macPANId) 
                {

                  return;
                }

              data_len = 8;

              if (get_fc1_ack_request(pdu->frame_control1) == 1) 
                {
                  MacP__build_ack(pdu->seq_num, 0);
                }

              DstAddr[0] = dest_short_ptr->destination_address;

              SrcAddr[0] = source_short_ptr->source_address;

              msdu_length = pdu->length - 5 - data_len;


              memcpy(&payload, &pdu->data[data_len], msdu_length * sizeof(uint8_t ));

              MacP__MCPS_DATA__indication((uint16_t )source_address, (uint16_t )source_short_ptr->source_PAN_identifier, SrcAddr, (uint16_t )destination_address, (uint16_t )dest_short_ptr->destination_PAN_identifier, DstAddr, (uint16_t )msdu_length, payload, (uint16_t )ppduLinkQuality, 0x0000, 0x0000);
            }
        }




      if (destination_address == 0 && source_address > 1) 
        {

          if (source_address == 3) 
            {
              source_long_ptr = (source_long *)&pdu->data[0];



              if (MacP__PANCoordinator == 0 || source_long_ptr->source_PAN_identifier != MacP__mac_PIB.macPANId) 
                {

                  return;
                }

              data_len = 10;

              SrcAddr[1] = source_long_ptr->source_address0;
              SrcAddr[0] = source_long_ptr->source_address1;

              msdu_length = pdu->length - data_len;

              memcpy(&payload, &pdu->data[data_len], msdu_length * sizeof(uint8_t ));

              MacP__MCPS_DATA__indication((uint16_t )source_address, (uint16_t )source_long_ptr->source_PAN_identifier, SrcAddr, (uint16_t )destination_address, 0x0000, DstAddr, (uint16_t )msdu_length, payload, (uint16_t )ppduLinkQuality, 0x0000, 0x0000);
            }
          else 

            {

              source_short_ptr = (source_short *)&pdu->data[0];


              if (MacP__PANCoordinator == 0 || source_short_ptr->source_PAN_identifier != MacP__mac_PIB.macPANId) 
                {

                  return;
                }

              data_len = 4;


              SrcAddr[0] = source_short_ptr->source_address;

              msdu_length = pdu->length - data_len;

              memcpy(&payload, &pdu->data[data_len], msdu_length * sizeof(uint8_t ));

              MacP__MCPS_DATA__indication((uint16_t )source_address, (uint16_t )source_short_ptr->source_PAN_identifier, SrcAddr, (uint16_t )destination_address, 0x0000, DstAddr, (uint16_t )msdu_length, payload, (uint16_t )ppduLinkQuality, 0x0000, 0x0000);
            }
        }



      if (destination_address > 1 && source_address == 0) 
        {
          if (destination_address == 3) 
            {
              dest_long_ptr = (dest_long *)&pdu->data[0];




              if (dest_long_ptr->destination_address0 != MacP__aExtendedAddress0 && dest_long_ptr->destination_address1 != MacP__aExtendedAddress1) 
                {

                  return;
                }


              if (dest_long_ptr->destination_PAN_identifier != 0xffff && dest_long_ptr->destination_PAN_identifier != MacP__mac_PIB.macPANId) 
                {

                  return;
                }

              data_len = 10;

              DstAddr[1] = dest_long_ptr->destination_address0;
              DstAddr[0] = dest_long_ptr->destination_address1;

              msdu_length = pdu->length - data_len;

              memcpy(&payload, &pdu->data[data_len], msdu_length * sizeof(uint8_t ));

              MacP__MCPS_DATA__indication((uint16_t )source_address, 0x0000, SrcAddr, (uint16_t )destination_address, (uint16_t )dest_long_ptr->destination_PAN_identifier, DstAddr, (uint16_t )msdu_length, payload, (uint16_t )ppduLinkQuality, 0x0000, 0x0000);
            }
          else 

            {
              dest_short_ptr = (dest_short *)&pdu->data[0];




              if (dest_short_ptr->destination_address != 0xffff && dest_short_ptr->destination_address != MacP__mac_PIB.macShortAddress) 
                {

                  return;
                }


              if (dest_short_ptr->destination_PAN_identifier != 0xffff && dest_short_ptr->destination_PAN_identifier != MacP__mac_PIB.macPANId) 
                {

                  return;
                }

              data_len = 4;

              DstAddr[0] = dest_short_ptr->destination_address;

              msdu_length = pdu->length - data_len;

              memcpy(&payload, &pdu->data[data_len], msdu_length * sizeof(uint8_t ));


              MacP__MCPS_DATA__indication((uint16_t )source_address, 0x0000, SrcAddr, (uint16_t )destination_address, (uint16_t )dest_short_ptr->destination_PAN_identifier, DstAddr, (uint16_t )msdu_length, payload, (uint16_t )ppduLinkQuality, 0x0000, 0x0000);

              data_len = 4;
            }
        }
    }
  else 

    {
    }






  return;
}

#line 1102
static inline void MacP__data_indication__runTask(void )
{



  uint8_t link_qual;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 1109
    link_qual = MacP__link_quality;
#line 1109
    __nesc_atomic_end(__nesc_atomic); }







  if (MacP__performing_csma_ca == 1) 
    {

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 1120
        {
          MacP__buffer_count--;

          MacP__current_msg_out++;
          if (MacP__current_msg_out == 4) {
            MacP__current_msg_out = 0;
            }
        }
#line 1127
        __nesc_atomic_end(__nesc_atomic); }
      return;
    }


  if (MacP__scanning_channels == 1) 
    {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 1134
        {
          MacP__buffer_count--;

          MacP__current_msg_out++;
          if (MacP__current_msg_out == 4) {
            MacP__current_msg_out = 0;
            }
        }
#line 1141
        __nesc_atomic_end(__nesc_atomic); }
#line 1141
      return;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 1143
    {




      switch (MacP__buffer_msg[MacP__current_msg_out].frame_control1 & 0x7) 
        {

          case 1: 
            MacP__indication_data(&MacP__buffer_msg[MacP__current_msg_out], link_qual);
          break;

          case 2: 

            MacP__indication_ack(&MacP__buffer_msg[MacP__current_msg_out], link_qual);

          break;

          case 3: 
            MacP__indication_cmd(&MacP__buffer_msg[MacP__current_msg_out], link_qual);
          break;

          case 0: 


            if (MacP__mac_PIB.macShortAddress == 0x0000) 
              {
                MacP__buffer_count--;
              }
            else 
              {
                MacP__process_beacon(&MacP__buffer_msg[MacP__current_msg_out], link_qual);
              }


          break;
          default: 
            MacP__buffer_count--;


          break;
        }
      {
        MacP__current_msg_out++;
        if (MacP__current_msg_out == 4) {
          MacP__current_msg_out = 0;
          }
      }
    }
#line 1191
    __nesc_atomic_end(__nesc_atomic); }
#line 1191
  return;
}

# 62 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
inline static void MacP__T_ackwait__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(0U, dt);
#line 62
}
#line 62
# 48 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw(void )
#line 48
{
#line 48
  return * (volatile uint8_t * )32U & (0x01 << 4);
}

#line 49
static inline bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get(void )
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw() != 0;
}

# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__get(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 40 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__get(void )
#line 40
{
#line 40
  return /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__get();
}

# 32 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static bool MacP__CCA__get(void ){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__get();
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 4356 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline void MacP__perform_csma_ca_slotted__runTask(void )
{
  uint8_t random_interval;




  if (MacP__check_csma_ca_send_conditions(MacP__send_buffer[MacP__send_buffer_msg_out].length, MacP__send_buffer[MacP__send_buffer_msg_out].frame_control1) == 1) 
    {
      MacP__cca_deference = 0;
    }
  else 
    {

      MacP__cca_deference = 1;
      return;
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 4374
    {


      if (MacP__CCA__get() == 1) 
        {


          MacP__CW--;
          if (MacP__CW == 0) 
            {

              MacP__csma_cca_backoff_boundary = 0;




              if (get_fc1_ack_request(MacP__send_buffer[MacP__send_buffer_msg_out].frame_control1) == 1) 
                {
                  MacP__send_ack_check = 1;
                  MacP__ack_sequence_number_check = MacP__send_buffer[MacP__send_buffer_msg_out].seq_num;

                  MacP__send_retransmission = MacP__send_buffer[MacP__send_buffer_msg_out].retransmission;

                  MacP__send_indirect_transmission = MacP__send_buffer[MacP__send_buffer_msg_out].indirect;

                  MacP__PD_DATA__request(MacP__send_buffer[MacP__send_buffer_msg_out].length, (uint8_t *)&MacP__send_buffer[MacP__send_buffer_msg_out]);




                  MacP__T_ackwait__startOneShot(MacP__ackwait_period);
                }
              else 
                {

                  MacP__PD_DATA__request(MacP__send_buffer[MacP__send_buffer_msg_out].length, (uint8_t *)&MacP__send_buffer[MacP__send_buffer_msg_out]);

                  MacP__send_buffer_count--;
                  MacP__send_buffer_msg_out++;


                  if (MacP__send_buffer_count > 3) 
                    {
                      MacP__send_buffer_count = 0;
                      MacP__send_buffer_msg_out = 0;
                      MacP__send_buffer_msg_in = 0;
                    }

                  if (MacP__send_buffer_msg_out == 3) {
                    MacP__send_buffer_msg_out = 0;
                    }
                  if (MacP__send_buffer_count > 0 && MacP__send_buffer_count <= 3) {
                    MacP__send_frame_csma__postTask();
                    }
                }




              MacP__performing_csma_ca = 0;
            }
        }
      else 
        {




          if (MacP__NB < MacP__mac_PIB.macMaxCSMABackoffs) 
            {


              MacP__CW = 2;
              MacP__NB++;
              MacP__BE = MacP__min(MacP__BE + 1, 5);







              if (MacP__backoff_deference == 0) 
                {
                  random_interval = powf(2, MacP__BE) - 1;
                  MacP__delay_backoff_period = MacP__Random__rand16() & random_interval;

                  if (MacP__check_csma_ca_backoff_send_conditions((uint32_t )MacP__delay_backoff_period) == 1) 
                    {
                      MacP__backoff_deference = 1;
                    }
                }
              else 
                {
                  MacP__backoff_deference = 0;
                }





              MacP__csma_delay = 1;
            }
          else 
            {

              MacP__csma_delay = 0;
              MacP__csma_cca_backoff_boundary = 0;

              MacP__send_buffer_count--;
              MacP__send_buffer_msg_out++;


              if (MacP__send_buffer_count > 3) 
                {
                  MacP__send_buffer_count = 0;
                  MacP__send_buffer_msg_out = 0;
                  MacP__send_buffer_msg_in = 0;
                }

              if (MacP__send_buffer_msg_out == 3) {
                MacP__send_buffer_msg_out = 0;
                }
              if (MacP__send_buffer_count > 0 && MacP__send_buffer_count <= 3) {
                MacP__send_frame_csma__postTask();
                }
              MacP__performing_csma_ca = 0;
            }
        }
    }
#line 4503
    __nesc_atomic_end(__nesc_atomic); }








  return;
}

#line 4682
static inline uint8_t MacP__calculate_ifs(uint8_t pk_length)
{
  if (pk_length > 18) {
    return 40;
    }
  else {
#line 4687
    return 12;
    }
}

# 343 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsyncM.nc"
static inline uint32_t TimerAsyncM__TimerAsync__get_current_number_backoff_on_time_slot(void )
{

  return TimerAsyncM__current_number_backoff_on_time_slot;
}

# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsync.nc"
inline static uint32_t MacP__TimerAsync__get_current_number_backoff_on_time_slot(void ){
#line 59
  unsigned long __nesc_result;
#line 59

#line 59
  __nesc_result = TimerAsyncM__TimerAsync__get_current_number_backoff_on_time_slot();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 4515 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline void MacP__perform_csma_ca_unslotted__runTask(void )
{
  uint8_t random_interval;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 4519
    {
      if (MacP__NB < MacP__mac_PIB.macMaxCSMABackoffs) 
        {





          if (MacP__CCA__get() == 1) 
            {


              {
                MacP__csma_delay = 0;



                if (MacP__check_csma_ca_send_conditions(MacP__send_buffer[MacP__send_buffer_msg_out].length, MacP__send_buffer[MacP__send_buffer_msg_out].frame_control1) == 1) 
                  {
                    MacP__PD_DATA__request(MacP__send_buffer[MacP__send_buffer_msg_out].length, (uint8_t *)&MacP__send_buffer[MacP__send_buffer_msg_out]);

                    MacP__send_buffer_count--;
                    MacP__send_buffer_msg_out++;


                    if (MacP__send_buffer_count > 3) 
                      {
                        MacP__send_buffer_count = 0;
                        MacP__send_buffer_msg_out = 0;
                        MacP__send_buffer_msg_in = 0;
                      }

                    if (MacP__send_buffer_msg_out == 3) {
                      MacP__send_buffer_msg_out = 0;
                      }
                  }

                MacP__performing_csma_ca = 0;
              }
              {
                __nesc_atomic_end(__nesc_atomic); 
#line 4559
                return;
              }
            }




          MacP__NB++;
          MacP__BE = MacP__min(MacP__BE + 1, 5);





          random_interval = powf(2, MacP__BE) - 1;



          MacP__delay_backoff_period = MacP__Random__rand16() & random_interval;

          MacP__csma_delay = 1;
        }
      else 


        {
          MacP__csma_delay = 0;
        }
    }
#line 4587
    __nesc_atomic_end(__nesc_atomic); }

  return;
}

# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
inline static void MacP__T_ScanDuration__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(2U);
#line 67
}
#line 67
# 3382 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline void MacP__data_channel_scan_indication__runTask(void )
{
  uint8_t link_qual;

  beacon_addr_short *beacon_ptr;



  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 3390
    link_qual = MacP__link_quality;
#line 3390
    __nesc_atomic_end(__nesc_atomic); }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 3392
    MacP__buffer_count--;
#line 3392
    __nesc_atomic_end(__nesc_atomic); }

  switch (MacP__scan_type) 
    {
      case ED_SCAN: 
        if (MacP__scanned_values[MacP__current_scanning - 1] < link_qual) {
          MacP__scanned_values[MacP__current_scanning - 1] = link_qual;
          }
#line 3399
      break;

      case ACTIVE_SCAN: break;

      case PASSIVE_SCAN: 
        switch (MacP__buffer_msg[MacP__current_msg_out].frame_control1 & 0x7) 
          {
            case 0: 
              ;
            beacon_ptr = (beacon_addr_short *)& MacP__buffer_msg[MacP__current_msg_out].data;




            MacP__scan_pans[MacP__current_scanning - 1].CoordPANId = beacon_ptr->destination_PAN_identifier;
            MacP__scan_pans[MacP__current_scanning - 1].CoordAddress = beacon_ptr->source_address;
            MacP__scan_pans[MacP__current_scanning - 1].LogicalChannel = MacP__current_channel;

            MacP__scan_pans[MacP__current_scanning - 1].SuperframeSpec = beacon_ptr->superframe_specification;

            if (MacP__scan_pans[MacP__current_scanning - 1].lqi < link_qual) {
              MacP__scan_pans[MacP__current_scanning - 1].lqi = link_qual;
              }
            break;

            default: break;
          }



      break;
      case ORPHAN_SCAN: 

        switch (MacP__buffer_msg[MacP__current_msg_out].frame_control1 & 0x7) 
          {
            case 3: 

              if (MacP__buffer_msg[MacP__current_msg_out].data[4 + 10] == CMD_COORDINATOR_REALIGNMENT) 
                {

                  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 3439
                    MacP__scanning_channels = 0;
#line 3439
                    __nesc_atomic_end(__nesc_atomic); }
                  MacP__T_ScanDuration__stop();
                  MacP__process_coordinator_realignment(&MacP__buffer_msg[MacP__current_msg_out]);
                }


            break;
            default: break;
          }
      break;
    }


  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 3452
    {
      MacP__current_msg_out++;
      if (MacP__current_msg_out == 4) {
        MacP__current_msg_out = 0;
        }
    }
#line 3457
    __nesc_atomic_end(__nesc_atomic); }
#line 3457
  return;
}

#line 4945
static inline error_t MacP__add_gts_null_entry(uint8_t gts_length, bool direction, uint16_t DevAddressType)
{
  int i;


  for (i = 0; i < 7; i++) 
    {
      if (MacP__GTS_null_db[i].DevAddressType == DevAddressType && MacP__GTS_null_db[i].gts_id > 0) 
        {

          return FAIL;
        }
    }

  for (i = 0; i < 7; i++) 
    {
      if (MacP__GTS_null_db[i].DevAddressType == 0x0000 && MacP__GTS_null_db[i].gts_id == 0x00) 
        {
          MacP__GTS_null_db[i].gts_id = MacP__GTS_id;
          MacP__GTS_null_db[i].starting_slot = 0x00;
          MacP__GTS_null_db[i].length = 0x00;

          MacP__GTS_null_db[i].DevAddressType = DevAddressType;
          MacP__GTS_null_db[i].persistencetime = 0x00;




          MacP__GTS_id++;
          MacP__GTS_null_descriptor_count++;

          return SUCCESS;
        }
    }


  return FAIL;
}

#line 4690
static inline uint32_t MacP__calculate_gts_expiration(void )
{
  uint32_t exp_res;

#line 4693
  if (MacP__mac_PIB.macBeaconOrder > 9) {
    exp_res = 1;
    }
  else {

      exp_res = powf(2, 8 - MacP__mac_PIB.macBeaconOrder);
    }




  return exp_res;
}

#line 5020
static inline void MacP__check_gts_expiration__runTask(void )
{
  int i;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 5024
    {
      MacP__gts_expiration = MacP__calculate_gts_expiration();

      MacP__gts_expiration = 2;


      for (i = 0; i < 7; i++) 
        {

          if (MacP__GTS_db[i].DevAddressType != 0x0000 && MacP__GTS_db[i].gts_id != 0x00) 
            {
              if (MacP__GTS_db[i].expiration == MacP__gts_expiration + 1 && MacP__GTS_db[i].direction == 0x00) 
                {


                  {

                    MacP__add_gts_null_entry(MacP__GTS_db[i].length, MacP__GTS_db[i].direction, MacP__GTS_db[i].DevAddressType);

                    MacP__remove_gts_entry(MacP__GTS_db[i].DevAddressType);
                  }
                }
              else 
                {
                  MacP__GTS_db[i].expiration++;
                }
            }
        }
    }
#line 5052
    __nesc_atomic_end(__nesc_atomic); }



  return;
}

#line 5119
static inline void MacP__start_gts_send__runTask(void )
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 5122
    {
      MacP__gts_send_pending_data = 0;

      if (MacP__gts_send_buffer_count > 0) 
        {
          if (MacP__check_gts_send_conditions(MacP__gts_send_buffer[MacP__gts_send_buffer_msg_out].length) == 1) 
            {

              MacP__gts_send_buffer[MacP__gts_send_buffer_msg_out].length = MacP__gts_send_buffer[MacP__gts_send_buffer_msg_out].length - 2;

              MacP__PD_DATA__request(MacP__gts_send_buffer[MacP__gts_send_buffer_msg_out].length, (uint8_t *)&MacP__gts_send_buffer[MacP__gts_send_buffer_msg_out]);

              MacP__gts_send_buffer_count--;
              MacP__gts_send_buffer_msg_out++;

              if (MacP__gts_send_buffer_msg_out == 3) {
                MacP__gts_send_buffer_msg_out = 0;
                }


              if (MacP__gts_send_buffer_count > 0) {
                MacP__gts_send_pending_data = 1;
                }
            }
        }
    }
#line 5147
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

#line 4984
static inline void MacP__increment_gts_null__runTask(void )
{
  int i;


  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 4989
    {
      for (i = 0; i < 7; i++) 
        {
          if (MacP__GTS_null_db[i].DevAddressType != 0x0000 && MacP__GTS_null_db[i].gts_id != 0x00) 
            {

              MacP__GTS_null_db[i].persistencetime++;
            }


          if (MacP__GTS_null_db[i].persistencetime > 4 - 1) 
            {
              MacP__GTS_null_db[i].gts_id = 0x00;
              MacP__GTS_null_db[i].starting_slot = 0x00;
              MacP__GTS_null_db[i].length = 0x00;

              MacP__GTS_null_db[i].DevAddressType = 0x0000;
              MacP__GTS_null_db[i].persistencetime = 0x00;



              MacP__GTS_null_descriptor_count--;
            }
        }
    }
#line 5013
    __nesc_atomic_end(__nesc_atomic); }



  return;
}

#line 5084
static inline void MacP__start_coordinator_gts_send__runTask(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 5086
    {

      MacP__coordinator_gts_send_pending_data = 0;

      if (MacP__gts_slot_list[15 - MacP__number_time_slot].element_count > 0) 
        {
          if (MacP__check_gts_send_conditions(MacP__gts_send_buffer[MacP__gts_slot_list[15 - MacP__number_time_slot].gts_send_frame_index[MacP__gts_slot_list[15 - MacP__number_time_slot].element_out]].length) == 1) 
            {

              MacP__gts_send_buffer[MacP__gts_slot_list[15 - MacP__number_time_slot].gts_send_frame_index[MacP__gts_slot_list[15 - MacP__number_time_slot].element_out]].length = MacP__gts_send_buffer[MacP__gts_slot_list[15 - MacP__number_time_slot].gts_send_frame_index[MacP__gts_slot_list[15 - MacP__number_time_slot].element_out]].length - 2;

              MacP__PD_DATA__request(MacP__gts_send_buffer[MacP__gts_slot_list[15 - MacP__number_time_slot].gts_send_frame_index[MacP__gts_slot_list[15 - MacP__number_time_slot].element_out]].length, (uint8_t *)&MacP__gts_send_buffer[MacP__gts_slot_list[15 - MacP__number_time_slot].gts_send_frame_index[MacP__gts_slot_list[15 - MacP__number_time_slot].element_out]]);

              MacP__available_gts_index_count++;
              MacP__available_gts_index[MacP__available_gts_index_count] = MacP__gts_slot_list[15 - MacP__number_time_slot].gts_send_frame_index[MacP__gts_slot_list[15 - MacP__number_time_slot].element_out];

              MacP__gts_slot_list[15 - MacP__number_time_slot].element_count--;
              MacP__gts_slot_list[15 - MacP__number_time_slot].element_out++;

              if (MacP__gts_slot_list[15 - MacP__number_time_slot].element_out == 3) {
                MacP__gts_slot_list[15 - MacP__number_time_slot].element_out = 0;
                }
              if (MacP__gts_slot_list[15 - MacP__number_time_slot].element_count > 0) 
                {
                  MacP__coordinator_gts_send_pending_data = 1;
                  MacP__coordinator_gts_send_time_slot = MacP__number_time_slot;
                }
            }
        }
    }
#line 5115
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

# 197 "GTSManagementExampleP.nc"
static inline error_t GTSManagementExampleP__MLME_SYNC_LOSS__indication(uint8_t LossReason)
{

  return SUCCESS;
}

# 14 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/interfaces/mac/MLME_SYNC_LOSS.nc"
inline static error_t MacP__MLME_SYNC_LOSS__indication(uint8_t LossReason){
#line 14
  unsigned char __nesc_result;
#line 14

#line 14
  __nesc_result = GTSManagementExampleP__MLME_SYNC_LOSS__indication(LossReason);
#line 14

#line 14
  return __nesc_result;
#line 14
}
#line 14
# 4206 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline void MacP__signal_loss__runTask(void )
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 4209
    MacP__MLME_SYNC_LOSS__indication(MacP__beacon_loss_reason);
#line 4209
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
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
# 69 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AMSend.nc"
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
# 127 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/printf/PrintfP.nc"
static inline void PrintfP__retrySend__runTask(void )
#line 127
{
  if (PrintfP__AMSend__send(AM_BROADCAST_ADDR, &PrintfP__printfMsg, sizeof(printf_msg_t )) != SUCCESS) {
    PrintfP__retrySend__postTask();
    }
}

# 99 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AMSend.nc"
inline static void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(message_t * msg, error_t error){
#line 99
  PrintfP__AMSend__sendDone(msg, error);
#line 99
}
#line 99
# 57 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/AMQueueEntryP.nc"
static inline void /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(message_t *m, error_t err)
#line 57
{
  /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(m, err);
}

# 207 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/AMQueueImplP.nc"
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(uint8_t id, message_t *msg, error_t err)
#line 207
{
}

# 89 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Send.nc"
inline static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(uint8_t arg_0x40b700c8, message_t * msg, error_t error){
#line 89
  switch (arg_0x40b700c8) {
#line 89
    case 0U:
#line 89
      /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(msg, error);
#line 89
      break;
#line 89
    default:
#line 89
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(arg_0x40b700c8, msg, error);
#line 89
      break;
#line 89
    }
#line 89
}
#line 89
# 118 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/AMQueueImplP.nc"
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

# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
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

# 111 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialActiveMessageP.nc"
static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__payloadLength(message_t *msg)
#line 111
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(msg);

#line 113
  return __nesc_ntoh_uint8(header->length.data);
}

# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Packet.nc"
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
# 57 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/AMQueueImplP.nc"
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

# 17 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/platforms/telosa/TelosSerialP.nc"
static inline void TelosSerialP__Resource__granted(void )
#line 17
{
}

# 218 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__default__granted(uint8_t id)
#line 218
{
}

# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__granted(uint8_t arg_0x408d58c0){
#line 92
  switch (arg_0x408d58c0) {
#line 92
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 92
      TelosSerialP__Resource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__default__granted(arg_0x408d58c0);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 101 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__granted(uint8_t id)
#line 101
{
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__granted(id);
}

# 202 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id)
#line 202
{
}

# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(uint8_t arg_0x40ae1e98){
#line 92
  switch (arg_0x40ae1e98) {
#line 92
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 92
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__granted(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(arg_0x40ae1e98);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 216 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id)
#line 216
{
}

# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(uint8_t arg_0x40af4cf8){
#line 49
  switch (arg_0x40af4cf8) {
#line 49
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 49
      /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__configure(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(arg_0x40af4cf8);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 190 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void )
#line 190
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 191
    {
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
    }
#line 194
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId);
}

# 19 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/platforms/telosa/TelosSerialP.nc"
static inline msp430_uart_union_config_t *TelosSerialP__Msp430UartConfigure__getConfig(void )
#line 19
{
  return &TelosSerialP__msp430_uart_telos_config;
}

# 214 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
static inline msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(uint8_t id)
#line 214
{
  return &msp430_uart_default_config;
}

# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartConfigure.nc"
inline static msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__getConfig(uint8_t arg_0x408ce978){
#line 39
  union __nesc_unnamed4304 *__nesc_result;
#line 39

#line 39
  switch (arg_0x408ce978) {
#line 39
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 39
      __nesc_result = TelosSerialP__Msp430UartConfigure__getConfig();
#line 39
      break;
#line 39
    default:
#line 39
      __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(arg_0x408ce978);
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
# 359 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
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

# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 6;
}

# 78 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__UTXD__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectModuleFunc();
#line 78
}
#line 78
# 220 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__enableUartTx(void )
#line 220
{
  HplMsp430Usart1P__UTXD__selectModuleFunc();
  HplMsp430Usart1P__ME2 |= 1 << 5;
}

# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 7);
}

# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__URXD__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc();
#line 85
}
#line 85
# 236 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__disableUartRx(void )
#line 236
{
  HplMsp430Usart1P__ME2 &= ~(1 << 4);
  HplMsp430Usart1P__URXD__selectIOFunc();
}

# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 7;
}

# 78 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__URXD__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectModuleFunc();
#line 78
}
#line 78
# 231 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__enableUartRx(void )
#line 231
{
  HplMsp430Usart1P__URXD__selectModuleFunc();
  HplMsp430Usart1P__ME2 |= 1 << 4;
}

# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 6);
}

# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__UTXD__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc();
#line 85
}
#line 85
# 225 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
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

# 174 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__setModeUart(msp430_uart_union_config_t *config){
#line 174
  HplMsp430Usart1P__Usart__setModeUart(config);
#line 174
}
#line 174
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )51U &= ~(0x01 << 1);
}

# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__SIMO__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )51U &= ~(0x01 << 2);
}

# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__SOMI__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )51U &= ~(0x01 << 3);
}

# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__UCLK__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc();
#line 85
}
#line 85
# 377 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
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

# 182 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__enableIntr(void ){
#line 182
  HplMsp430Usart1P__Usart__enableIntr();
#line 182
}
#line 182
# 181 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/AMQueueImplP.nc"
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

# 99 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AMSend.nc"
inline static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__sendDone(am_id_t arg_0x407758a8, message_t * msg, error_t error){
#line 99
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(arg_0x407758a8, msg, error);
#line 99
}
#line 99
# 90 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(message_t *msg, error_t result)
#line 90
{
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__sendDone(/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(msg), msg, result);
}

# 365 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(uart_id_t idxxx, message_t *msg, error_t error)
#line 365
{
  return;
}

# 89 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Send.nc"
inline static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__sendDone(uart_id_t arg_0x40833830, message_t * msg, error_t error){
#line 89
  switch (arg_0x40833830) {
#line 89
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 89
      /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(msg, error);
#line 89
      break;
#line 89
    default:
#line 89
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(arg_0x40833830, msg, error);
#line 89
      break;
#line 89
    }
#line 89
}
#line 89
# 147 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
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

# 98 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialActiveMessageP.nc"
static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(uint8_t id, message_t *msg, void *payload, uint8_t len)
#line 98
{
  return msg;
}

# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Receive.nc"
inline static message_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__receive(am_id_t arg_0x40781298, message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
    __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(arg_0x40781298, msg, payload, len);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 102 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialActiveMessageP.nc"
static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 102
{
  return /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__receive(/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(msg), msg, msg->data, len);
}

# 360 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
static inline message_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(uart_id_t idxxx, message_t *msg, 
void *payload, 
uint8_t len)
#line 362
{
  return msg;
}

# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Receive.nc"
inline static message_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__receive(uart_id_t arg_0x408331f0, message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  switch (arg_0x408331f0) {
#line 67
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 67
      __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(msg, payload, len);
#line 67
      break;
#line 67
    default:
#line 67
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(arg_0x408331f0, msg, payload, len);
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
# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__upperLength(message_t *msg, uint8_t dataLinkLen)
#line 46
{
  return dataLinkLen - sizeof(serial_header_t );
}

# 354 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(uart_id_t id, message_t *msg, 
uint8_t dataLinkLen)
#line 355
{
  return 0;
}

# 31 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__upperLength(uart_id_t arg_0x40831398, message_t *msg, uint8_t dataLinkLen){
#line 31
  unsigned char __nesc_result;
#line 31

#line 31
  switch (arg_0x40831398) {
#line 31
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 31
      __nesc_result = SerialPacketInfoActiveMessageP__Info__upperLength(msg, dataLinkLen);
#line 31
      break;
#line 31
    default:
#line 31
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(arg_0x40831398, msg, dataLinkLen);
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
# 264 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
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

# 123 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/printf/PrintfP.nc"
static inline void PrintfP__SerialControl__stopDone(error_t error)
#line 123
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 124
    PrintfP__state = PrintfP__S_STOPPED;
#line 124
    __nesc_atomic_end(__nesc_atomic); }
}

# 117 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SplitControl.nc"
inline static void SerialP__SplitControl__stopDone(error_t error){
#line 117
  PrintfP__SerialControl__stopDone(error);
#line 117
}
#line 117
# 109 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline error_t HplMsp430Usart1P__AsyncStdControl__stop(void )
#line 109
{
  HplMsp430Usart1P__Usart__disableSpi();
  HplMsp430Usart1P__Usart__disableUart();
  return SUCCESS;
}

# 84 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AsyncStdControl.nc"
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
# 74 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup(void )
#line 74
{
}

# 52 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/power/PowerDownCleanup.nc"
inline static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__cleanup(void ){
#line 52
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup();
#line 52
}
#line 52
# 69 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted(void )
#line 69
{
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__cleanup();
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__stop();
}

# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void ){
#line 46
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted();
#line 46
}
#line 46
# 128 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart.nc"
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
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(uint8_t id)
#line 92
{
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__resetUsart(TRUE);
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableIntr();
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableUart();
}

# 218 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id)
#line 218
{
}

# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(uint8_t arg_0x40af4cf8){
#line 55
  switch (arg_0x40af4cf8) {
#line 55
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 55
      /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 55
      break;
#line 55
    default:
#line 55
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(arg_0x40af4cf8);
#line 55
      break;
#line 55
    }
#line 55
}
#line 55
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
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
# 58 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/FcfsResourceQueueC.nc"
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

# 60 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceQueue.nc"
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
# 50 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/FcfsResourceQueueC.nc"
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

# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceQueue.nc"
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
# 111 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id)
#line 111
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 112
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
#line 127
            SUCCESS;

            {
#line 127
              __nesc_atomic_end(__nesc_atomic); 
#line 127
              return __nesc_temp;
            }
          }
        }
    }
#line 131
    __nesc_atomic_end(__nesc_atomic); }
#line 130
  return FAIL;
}

# 213 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__release(uint8_t id)
#line 213
{
#line 213
  return FAIL;
}

# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__release(uint8_t arg_0x408cfe50){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  switch (arg_0x408cfe50) {
#line 110
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 110
      __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(/*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID);
#line 110
      break;
#line 110
    default:
#line 110
      __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__release(arg_0x408cfe50);
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
# 210 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__isOwner(uint8_t id)
#line 210
{
#line 210
  return FAIL;
}

# 118 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__isOwner(uint8_t arg_0x408cfe50){
#line 118
  unsigned char __nesc_result;
#line 118

#line 118
  switch (arg_0x408cfe50) {
#line 118
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 118
      __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(/*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID);
#line 118
      break;
#line 118
    default:
#line 118
      __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__isOwner(arg_0x408cfe50);
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
# 77 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
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

# 110 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
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
# 13 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/platforms/telosa/TelosSerialP.nc"
static inline error_t TelosSerialP__StdControl__stop(void )
#line 13
{
  TelosSerialP__Resource__release();
  return SUCCESS;
}

# 84 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/StdControl.nc"
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
# 330 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialP.nc"
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

# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
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
# 338 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFlush__default__flush(void )
#line 338
{
  SerialP__defaultSerialFlushTask__postTask();
}

# 38 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialFlush.nc"
inline static void SerialP__SerialFlush__flush(void ){
#line 38
  SerialP__SerialFlush__default__flush();
#line 38
}
#line 38
# 326 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialP.nc"
static inline void SerialP__stopDoneTask__runTask(void )
#line 326
{
  SerialP__SerialFlush__flush();
}

# 62 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/Timer.nc"
inline static void GTSManagementExampleP__Timer0__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(3U, dt);
#line 62
}
#line 62
# 63 "GTSManagementExampleP.nc"
static inline void GTSManagementExampleP__Boot__booted(void )
#line 63
{

  if (COORDINATOR == COORDINATOR) 
    {

      GTSManagementExampleP__my_short_address = 0x0000;
      GTSManagementExampleP__Timer0__startOneShot(5000);
    }
  else 
    {
      GTSManagementExampleP__Timer0__startOneShot(8000);
    }
}

# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Boot.nc"
inline static void PrintfP__Boot__booted(void ){
#line 49
  GTSManagementExampleP__Boot__booted();
#line 49
}
#line 49
# 113 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/printf/PrintfP.nc"
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

# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SplitControl.nc"
inline static void SerialP__SplitControl__startDone(error_t error){
#line 92
  PrintfP__SerialControl__startDone(error);
#line 92
}
#line 92
# 133 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void )
#line 133
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 134
    {
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id) {
          if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING) {
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
              {
                unsigned char __nesc_temp = 
#line 138
                SUCCESS;

                {
#line 138
                  __nesc_atomic_end(__nesc_atomic); 
#line 138
                  return __nesc_temp;
                }
              }
            }
          else {
#line 140
            if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING) {
                /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
                /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
                {
                  unsigned char __nesc_temp = 
#line 143
                  SUCCESS;

                  {
#line 143
                    __nesc_atomic_end(__nesc_atomic); 
#line 143
                    return __nesc_temp;
                  }
                }
              }
            }
        }
    }
#line 149
    __nesc_atomic_end(__nesc_atomic); }
#line 147
  return FAIL;
}

# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceDefaultOwner.nc"
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
# 105 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline error_t HplMsp430Usart1P__AsyncStdControl__start(void )
#line 105
{
  return SUCCESS;
}

# 74 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/AsyncStdControl.nc"
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
# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested(void )
#line 64
{
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__start();
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__release();
}

# 81 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void ){
#line 81
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested();
#line 81
}
#line 81
# 206 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id)
#line 206
{
}

# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(uint8_t arg_0x40af5948){
#line 51
    /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(arg_0x40af5948);
#line 51
}
#line 51
# 93 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id)
#line 93
{
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 95
    {
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) {
          /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING;
          /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = id;
        }
      else {
          unsigned char __nesc_temp = 
#line 100
          FAIL;

          {
#line 100
            __nesc_atomic_end(__nesc_atomic); 
#line 100
            return __nesc_temp;
          }
        }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
#line 102
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested();
  if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId == id) {
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId);
      return SUCCESS;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 107
    /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
#line 107
    __nesc_atomic_end(__nesc_atomic); }
  return FAIL;
}

# 212 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__immediateRequest(uint8_t id)
#line 212
{
#line 212
  return FAIL;
}

# 87 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
inline static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__immediateRequest(uint8_t arg_0x408cfe50){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  switch (arg_0x408cfe50) {
#line 87
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 87
      __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(/*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID);
#line 87
      break;
#line 87
    default:
#line 87
      __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__immediateRequest(arg_0x408cfe50);
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
# 65 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__immediateRequest(uint8_t id)
#line 65
{
  return /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__immediateRequest(id);
}

# 87 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Resource.nc"
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
# 10 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/platforms/telosa/TelosSerialP.nc"
static inline error_t TelosSerialP__StdControl__start(void )
#line 10
{
  return TelosSerialP__Resource__immediateRequest();
}

# 74 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/StdControl.nc"
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
# 320 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialP.nc"
static inline void SerialP__startDoneTask__runTask(void )
#line 320
{
  SerialP__SerialControl__start();
  SerialP__SplitControl__startDone(SUCCESS);
}

# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialFrameComm.nc"
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
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
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
# 183 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
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

# 80 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SendBytePacket.nc"
inline static void SerialP__SendBytePacket__sendCompleted(error_t error){
#line 80
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error);
#line 80
}
#line 80
# 242 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialP.nc"
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

# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
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
# 48 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/UartStream.nc"
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
# 214 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialP.nc"
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

# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void )
#line 45
{
  memset(/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ, /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY, sizeof /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ);
  return SUCCESS;
}

# 99 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/includes/printfUART.h"
static inline void printfUART_init()
#line 99
{
}

# 146 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsyncM.nc"
static inline error_t TimerAsyncM__TimerAsync__set_enable_backoffs(bool enable)
{
  /* atomic removed: atomic calls only */
#line 148
  TimerAsyncM__enable_backoffs = enable;
  return SUCCESS;
}

# 40 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsync.nc"
inline static error_t MacP__TimerAsync__set_enable_backoffs(bool enable_backoffs){
#line 40
  unsigned char __nesc_result;
#line 40

#line 40
  __nesc_result = TimerAsyncM__TimerAsync__set_enable_backoffs(enable_backoffs);
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 5155 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline void MacP__init_indirect_trans_buffer(void )
{
  int i;

#line 5158
  for (i = 0; i < 2; i++) 
    {
      MacP__indirect_trans_queue[i].handler = 0x00;
      MacP__indirect_trans_count = 0;
    }

  return;
}

# 241 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ReceiveP.nc"
static inline error_t CC2420ReceiveP__AddressFilter__set_coord_address(uint16_t mac_coord_address, uint16_t mac_panid)
{

  CC2420ReceiveP__ver_macCoordShortAddress = mac_coord_address;
  CC2420ReceiveP__ver_macPANId = mac_panid;



  return SUCCESS;
}

# 13 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/AddressFilter.nc"
inline static error_t MacP__AddressFilter__set_coord_address(uint16_t mac_coord_address, uint16_t mac_panid){
#line 13
  unsigned char __nesc_result;
#line 13

#line 13
  __nesc_result = CC2420ReceiveP__AddressFilter__set_coord_address(mac_coord_address, mac_panid);
#line 13

#line 13
  return __nesc_result;
#line 13
}
#line 13
# 224 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ReceiveP.nc"
static inline error_t CC2420ReceiveP__AddressFilter__set_address(uint16_t mac_short_address, uint32_t mac_extended0, uint32_t mac_extended1)
{

  CC2420ReceiveP__ver_macShortAddress = mac_short_address;

  CC2420ReceiveP__ver_aExtendedAddress0 = mac_extended0;
  CC2420ReceiveP__ver_aExtendedAddress1 = mac_extended1;

  CC2420ReceiveP__address_decode = 1;




  return SUCCESS;
}

# 10 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/AddressFilter.nc"
inline static error_t MacP__AddressFilter__set_address(uint16_t mac_short_address, uint32_t mac_extended0, uint32_t mac_extended1){
#line 10
  unsigned char __nesc_result;
#line 10

#line 10
  __nesc_result = CC2420ReceiveP__AddressFilter__set_address(mac_short_address, mac_extended0, mac_extended1);
#line 10

#line 10
  return __nesc_result;
#line 10
}
#line 10
# 5059 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static inline void MacP__init_available_gts_index(void )
{
  int i = 0;

  /* atomic removed: atomic calls only */
#line 5062
  {
    MacP__available_gts_index_count = 3;
    for (i = 0; i < 3; i++) 
      {
        MacP__available_gts_index[i] = i;
      }
  }
  return;
}

static inline void MacP__init_gts_slot_list(void )
{
  int i = 0;

#line 5075
  for (i = 0; i < 7; i++) 
    {
      MacP__gts_slot_list[i].element_count = 0x00;
      MacP__gts_slot_list[i].element_in = 0x00;
      MacP__gts_slot_list[i].element_out = 0x00;
    }
}

#line 4926
static inline void MacP__init_GTS_null_db(void )
{

  int i;

  /* atomic removed: atomic calls only */
#line 4930
  {
    for (i = 0; i < 7; i++) 
      {
        MacP__GTS_null_db[i].gts_id = 0x00;
        MacP__GTS_null_db[i].starting_slot = 0x00;
        MacP__GTS_null_db[i].length = 0x00;

        MacP__GTS_null_db[i].DevAddressType = 0x0000;
        MacP__GTS_null_db[i].persistencetime = 0x00;
      }
  }
  return;
}

#line 4791
static inline void MacP__init_GTS_db(void )
{

  int i;

  /* atomic removed: atomic calls only */
#line 4795
  {
    for (i = 0; i < 7; i++) 
      {
        MacP__GTS_db[i].gts_id = 0x00;
        MacP__GTS_db[i].starting_slot = 0x00;
        MacP__GTS_db[i].length = 0x00;
        MacP__GTS_db[i].direction = 0x00;
        MacP__GTS_db[i].DevAddressType = 0x0000;
      }
  }

  return;
}

#line 4251
static inline void MacP__init_MacPIB(void )
{
  /* atomic removed: atomic calls only */
  {



    MacP__mac_PIB.macAckWaitDuration = 65;

    MacP__mac_PIB.macAssociationPermit = 1;
    MacP__mac_PIB.macAutoRequest = 1;

    MacP__mac_PIB.macBattLifeExt = 0;

    MacP__mac_PIB.macBattLifeExtPeriods = 6;

    MacP__mac_PIB.macBeaconPayloadLenght = 0;

    MacP__mac_PIB.macBeaconTxTime = 0xffffff << 24;


    MacP__mac_PIB.macBSN = MacP__Random__rand16();


    MacP__mac_PIB.macCoordExtendedAddress0 = 0x00000000;
    MacP__mac_PIB.macCoordExtendedAddress1 = 0x00000000;

    MacP__mac_PIB.macCoordShortAddress = 0x0000;
#line 4291
    MacP__mac_PIB.macDSN = MacP__Random__rand16();


    MacP__mac_PIB.macGTSPermit = 1;


    MacP__mac_PIB.macMaxCSMABackoffs = 4;
    MacP__mac_PIB.macMinBE = 0;


    MacP__mac_PIB.macPANId = 0x1234;

    MacP__mac_PIB.macPromiscuousMode = 0;
    MacP__mac_PIB.macRxOnWhenIdle = 0;


    MacP__mac_PIB.macShortAddress = 0xffff;


    MacP__mac_PIB.macBeaconOrder = 7;
    MacP__mac_PIB.macSuperframeOrder = 3;


    MacP__mac_PIB.macTransactionPersistenceTime = 0x0010;
  }
}

#line 488
static inline error_t MacP__Init__init(void )
#line 488
{

  MacP__AMControl__start();



  MacP__mac_beacon_txmpdu_ptr = &MacP__mac_beacon_txmpdu;
  /* atomic removed: atomic calls only */


  {

    MacP__init_MacPIB();

    MacP__init_GTS_db();

    MacP__init_GTS_null_db();

    MacP__init_gts_slot_list();

    MacP__init_available_gts_index();

    MacP__aExtendedAddress0 = TOS_NODE_ID;
    MacP__aExtendedAddress1 = TOS_NODE_ID;


    MacP__AddressFilter__set_address(MacP__mac_PIB.macShortAddress, MacP__aExtendedAddress0, MacP__aExtendedAddress1);

    MacP__AddressFilter__set_coord_address(MacP__mac_PIB.macCoordShortAddress, MacP__mac_PIB.macPANId);




    MacP__init_indirect_trans_buffer();
  }




  MacP__mac_beacon_txmpdu_ptr = &MacP__mac_beacon_txmpdu;


  MacP__mac_ack_ptr = &MacP__mac_ack;


  MacP__ackwait_period = MacP__mac_PIB.macAckWaitDuration * 4.0 / 250.0 * 3;

  MacP__response_wait_time = 32 * 960 * 4.0 / 250.0 * 2;
  /* atomic removed: atomic calls only */
  {


    MacP__BI = 960 * powf(2, MacP__mac_PIB.macBeaconOrder);
    MacP__SD = 960 * powf(2, MacP__mac_PIB.macSuperframeOrder);



    MacP__backoff = 20;


    MacP__time_slot = MacP__SD / 16;

    MacP__TimerAsync__set_enable_backoffs(1);
    MacP__TimerAsync__set_backoff_symbols(MacP__backoff);

    MacP__TimerAsync__set_bi_sd(MacP__BI, MacP__SD);

    MacP__TimerAsync__start();
  }


  printfUART_init();

  return SUCCESS;
}

# 77 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/phy/PhyP.nc"
static inline error_t PhyP__Init__init(void )
#line 77
{
#line 91
  return SUCCESS;
}

# 52 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )30U |= 0x01 << 5;
}

# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__makeOutput();
}

# 35 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__VREN__makeOutput(void ){
#line 35
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )30U |= 0x01 << 6;
}

# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__makeOutput();
}

# 35 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__makeOutput(void ){
#line 35
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )30U |= 0x01 << 2;
}

# 71 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__makeOutput();
}

# 35 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__makeOutput(void ){
#line 35
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput();
#line 35
}
#line 35
# 118 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ControlP.nc"
static inline error_t CC2420ControlP__Init__init(void )
#line 118
{
  CC2420ControlP__CSN__makeOutput();
  CC2420ControlP__RSTN__makeOutput();
  CC2420ControlP__VREN__makeOutput();



  CC2420ControlP__m_tx_power = 31;


  CC2420ControlP__m_channel = 0x19;



  CC2420ControlP__autoAckEnabled = FALSE;







  CC2420ControlP__hwAutoAckDefault = FALSE;



  CC2420ControlP__addressRecognition = FALSE;




  return SUCCESS;
}

# 81 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/StateImplP.nc"
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

# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void )
#line 45
{
  memset(/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ, /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY, sizeof /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ);
  return SUCCESS;
}

# 50 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )30U &= ~(0x01 << 1);
}

# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput();
#line 64
}
#line 64
# 41 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput(void )
#line 41
{
#line 41
  /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__makeInput();
}

# 33 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__SFD__makeInput(void ){
#line 33
  /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput();
#line 33
}
#line 33


inline static void CC2420TransmitP__CSN__makeOutput(void ){
#line 35
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput();
#line 35
}
#line 35
# 50 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )34U &= ~(0x01 << 4);
}

# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput();
#line 64
}
#line 64
# 41 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput(void )
#line 41
{
#line 41
  /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__makeInput();
}

# 33 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CCA__makeInput(void ){
#line 33
  /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput();
#line 33
}
#line 33
# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__Init__init(void )
#line 85
{
  CC2420TransmitP__CCA__makeInput();
  CC2420TransmitP__CSN__makeOutput();
  CC2420TransmitP__SFD__makeInput();
  return SUCCESS;
}

# 186 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ReceiveP.nc"
static inline error_t CC2420ReceiveP__Init__init(void )
#line 186
{


  CC2420ReceiveP__rxmpdu_ptr = &CC2420ReceiveP__rxmpdu;

  printfUART_init();

  return SUCCESS;
}

# 44 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/RandomMlcgC.nc"
static inline error_t RandomMlcgC__Init__init(void )
#line 44
{
  /* atomic removed: atomic calls only */
#line 45
  RandomMlcgC__seed = (uint32_t )(TOS_NODE_ID + 1);

  return SUCCESS;
}

# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__CC2int(/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6____nesc_unnamed4405 {
#line 46
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__compareControl(void )
{
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__CC2int(x);
}

#line 94
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__setControlAsCompare(void )
{
  * (volatile uint16_t * )392U = /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__compareControl();
}

# 36 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__setControlAsCompare(void ){
#line 36
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__setControlAsCompare();
#line 36
}
#line 36
# 42 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Init__init(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Init.nc"
inline static error_t RealMainP__SoftwareInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__2__Init__init();
#line 51
  __nesc_result = ecombine(__nesc_result, RandomMlcgC__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, CC2420ReceiveP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, CC2420TransmitP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, StateImplP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, CC2420ControlP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, PhyP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, MacP__Init__init());
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
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
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
# 342 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialP.nc"
static inline error_t SerialP__SplitControl__start(void )
#line 342
{
  SerialP__startDoneTask__postTask();
  return SUCCESS;
}

# 83 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/SplitControl.nc"
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
# 109 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/printf/PrintfP.nc"
static inline void PrintfP__MainBoot__booted(void )
#line 109
{
  PrintfP__SerialControl__start();
}

# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Boot.nc"
inline static void RealMainP__Boot__booted(void ){
#line 49
  PrintfP__MainBoot__booted();
#line 49
}
#line 49
# 206 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_disable_interrupt(void )
{
   __asm volatile ("dint");
   __asm volatile ("nop");}

# 52 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void )
#line 52
{
  return MSP430_POWER_LPM3;
}

# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/McuPowerOverride.nc"
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
# 66 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/McuSleepC.nc"
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

# 194 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/msp430hardware.h"
static inline  mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 194
{
  return m1 < m2 ? m1 : m2;
}

# 104 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/McuSleepC.nc"
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

# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP__McuSleep__sleep(void ){
#line 59
  McuSleepC__McuSleep__sleep();
#line 59
}
#line 59
# 67 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/SchedulerBasicP.nc"
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

# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__taskLoop(void ){
#line 61
  SchedulerBasicP__Scheduler__taskLoop();
#line 61
}
#line 61
# 88 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ArbiterInfo.nc"
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
# 387 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFrameComm__dataReceived(uint8_t data)
#line 387
{
  SerialP__rx_state_machine(FALSE, data);
}

# 83 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC__SerialFrameComm__dataReceived(uint8_t data){
#line 83
  SerialP__SerialFrameComm__dataReceived(data);
#line 83
}
#line 83
# 384 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFrameComm__delimiterReceived(void )
#line 384
{
  SerialP__rx_state_machine(TRUE, 0);
}

# 74 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC__SerialFrameComm__delimiterReceived(void ){
#line 74
  SerialP__SerialFrameComm__delimiterReceived();
#line 74
}
#line 74
# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/HdlcTranslateC.nc"
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

# 221 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(uint8_t id, uint8_t byte)
#line 221
{
}

# 79 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receivedByte(uint8_t arg_0x408d0a90, uint8_t byte){
#line 79
  switch (arg_0x408d0a90) {
#line 79
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 79
      HdlcTranslateC__UartStream__receivedByte(byte);
#line 79
      break;
#line 79
    default:
#line 79
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(arg_0x408d0a90, byte);
#line 79
      break;
#line 79
    }
#line 79
}
#line 79
# 116 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC__UartStream__receiveDone(uint8_t *buf, uint16_t len, error_t error)
#line 116
{
}

# 222 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error)
#line 222
{
}

# 99 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receiveDone(uint8_t arg_0x408d0a90, uint8_t * buf, uint16_t len, error_t error){
#line 99
  switch (arg_0x408d0a90) {
#line 99
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 99
      HdlcTranslateC__UartStream__receiveDone(buf, len, error);
#line 99
      break;
#line 99
    default:
#line 99
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(arg_0x408d0a90, buf, len, error);
#line 99
      break;
#line 99
    }
#line 99
}
#line 99
# 134 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
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

# 65 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data)
#line 65
{
}

# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(uint8_t arg_0x40ad3f28, uint8_t data){
#line 54
  switch (arg_0x40ad3f28) {
#line 54
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 54
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__rxDone(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID, data);
#line 54
      break;
#line 54
    default:
#line 54
      /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(arg_0x40ad3f28, data);
#line 54
      break;
#line 54
    }
#line 54
}
#line 54
# 80 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ArbiterInfo.nc"
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
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data)
#line 54
{
  if (/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(), data);
    }
}

# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart1P__Interrupts__rxDone(uint8_t data){
#line 54
  /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(data);
#line 54
}
#line 54
# 391 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialP.nc"
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

# 192 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
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

# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/ReceiveBytePacket.nc"
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
# 309 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialP.nc"
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

# 69 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/ReceiveBytePacket.nc"
inline static void SerialP__ReceiveBytePacket__endPacket(error_t result){
#line 69
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(result);
#line 69
}
#line 69
# 210 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBufferSwap(void )
#line 210
{
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which ? 0 : 1;
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer = (uint8_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messagePtrs[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which];
}

# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
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
# 232 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialP.nc"
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

# 233 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
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

# 58 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/ReceiveBytePacket.nc"
inline static void SerialP__ReceiveBytePacket__byteReceived(uint8_t data){
#line 58
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(data);
#line 58
}
#line 58
# 299 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialP.nc"
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

# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC__SerialFrameComm__resetReceive(void )
#line 55
{
  HdlcTranslateC__state.receiveEscape = 0;
}

# 68 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialFrameComm.nc"
inline static void SerialP__SerialFrameComm__resetReceive(void ){
#line 68
  HdlcTranslateC__SerialFrameComm__resetReceive();
#line 68
}
#line 68
# 220 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__sendDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error)
#line 220
{
}

# 57 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__sendDone(uint8_t arg_0x408d0a90, uint8_t * buf, uint16_t len, error_t error){
#line 57
  switch (arg_0x408d0a90) {
#line 57
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 57
      HdlcTranslateC__UartStream__sendDone(buf, len, error);
#line 57
      break;
#line 57
    default:
#line 57
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__sendDone(arg_0x408d0a90, buf, len, error);
#line 57
      break;
#line 57
    }
#line 57
}
#line 57
# 384 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__tx(uint8_t data)
#line 384
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 385
    HplMsp430Usart1P__U1TXBUF = data;
#line 385
    __nesc_atomic_end(__nesc_atomic); }
}

# 224 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__tx(uint8_t data){
#line 224
  HplMsp430Usart1P__Usart__tx(data);
#line 224
}
#line 224
# 162 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
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

# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id)
#line 64
{
}

# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(uint8_t arg_0x40ad3f28){
#line 49
  switch (arg_0x40ad3f28) {
#line 49
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 49
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__txDone(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(arg_0x40ad3f28);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void )
#line 49
{
  if (/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId());
    }
}

# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart1P__Interrupts__txDone(void ){
#line 49
  /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone();
#line 49
}
#line 49
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialFrameComm.nc"
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
# 513 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialP.nc"
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

# 60 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SendBytePacket.nc"
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
# 167 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
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

# 70 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SendBytePacket.nc"
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
# 642 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialP.nc"
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

# 89 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC__SerialFrameComm__putDone(void ){
#line 89
  SerialP__SerialFrameComm__putDone();
#line 89
}
#line 89
# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/QueueC.nc"
static inline uint8_t /*PrintfC.QueueC*/QueueC__0__Queue__maxSize(void )
#line 61
{
  return 250;
}

#line 97
static inline error_t /*PrintfC.QueueC*/QueueC__0__Queue__enqueue(/*PrintfC.QueueC*/QueueC__0__queue_t newVal)
#line 97
{
  if (/*PrintfC.QueueC*/QueueC__0__Queue__size() < /*PrintfC.QueueC*/QueueC__0__Queue__maxSize()) {
      ;
      /*PrintfC.QueueC*/QueueC__0__queue[/*PrintfC.QueueC*/QueueC__0__tail] = newVal;
      /*PrintfC.QueueC*/QueueC__0__tail++;
      if (/*PrintfC.QueueC*/QueueC__0__tail == 250) {
#line 102
        /*PrintfC.QueueC*/QueueC__0__tail = 0;
        }
#line 103
      /*PrintfC.QueueC*/QueueC__0__size++;
      /*PrintfC.QueueC*/QueueC__0__printQueue();
      return SUCCESS;
    }
  else {
      return FAIL;
    }
}

# 90 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/Queue.nc"
inline static error_t PrintfP__Queue__enqueue(PrintfP__Queue__t  newVal){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = /*PrintfC.QueueC*/QueueC__0__Queue__enqueue(newVal);
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 319 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__InterruptFIFOP__fired(void )
#line 319
{






  if (CC2420ReceiveP__m_state == CC2420ReceiveP__S_STARTED) {


      CC2420ReceiveP__beginReceive();
    }
  else 
#line 343
    {
      CC2420ReceiveP__m_missed_packets++;
    }
}

# 57 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GpioInterrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__fired(void ){
#line 57
  CC2420ReceiveP__InterruptFIFOP__fired();
#line 57
}
#line 57
# 66 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired(void )
#line 66
{
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear();
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__fired();
}

# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port10__fired(void ){
#line 61
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired();
#line 61
}
#line 61
# 92 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port11__fired(void ){
#line 61
  HplMsp430InterruptP__Port11__default__fired();
#line 61
}
#line 61
# 93 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port12__fired(void ){
#line 61
  HplMsp430InterruptP__Port12__default__fired();
#line 61
}
#line 61
# 94 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port13__fired(void ){
#line 61
  HplMsp430InterruptP__Port13__default__fired();
#line 61
}
#line 61
# 56 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
inline static error_t PhyP__startDone_task__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(PhyP__startDone_task);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 137 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/phy/PhyP.nc"
static inline void PhyP__CC2420Power__startOscillatorDone(void )
#line 137
{
  PhyP__startDone_task__postTask();
}

# 76 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static void CC2420ControlP__CC2420Power__startOscillatorDone(void ){
#line 76
  PhyP__CC2420Power__startOscillatorDone();
#line 76
}
#line 76
# 50 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ControlP__InterruptCCA__disable(void ){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 422 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ControlP.nc"
static inline void CC2420ControlP__InterruptCCA__fired(void )
#line 422
{
  CC2420ControlP__m_state = CC2420ControlP__S_XOSC_STARTED;
  CC2420ControlP__InterruptCCA__disable();
  CC2420ControlP__IOCFG1__write(0);
  CC2420ControlP__writeId();
  CC2420ControlP__CSN__set();
  CC2420ControlP__CSN__clr();
  CC2420ControlP__CC2420Power__startOscillatorDone();
}

# 57 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/GpioInterrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__fired(void ){
#line 57
  CC2420ControlP__InterruptCCA__fired();
#line 57
}
#line 57
# 66 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired(void )
#line 66
{
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear();
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__fired();
}

# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port14__fired(void ){
#line 61
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired();
#line 61
}
#line 61
# 96 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port15__fired(void ){
#line 61
  HplMsp430InterruptP__Port15__default__fired();
#line 61
}
#line 61
# 97 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port16__fired(void ){
#line 61
  HplMsp430InterruptP__Port16__default__fired();
#line 61
}
#line 61
# 98 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port17__fired(void ){
#line 61
  HplMsp430InterruptP__Port17__default__fired();
#line 61
}
#line 61
# 195 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port20__fired(void ){
#line 61
  HplMsp430InterruptP__Port20__default__fired();
#line 61
}
#line 61
# 196 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port21__fired(void ){
#line 61
  HplMsp430InterruptP__Port21__default__fired();
#line 61
}
#line 61
# 197 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port22__fired(void ){
#line 61
  HplMsp430InterruptP__Port22__default__fired();
#line 61
}
#line 61
# 198 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port23__fired(void ){
#line 61
  HplMsp430InterruptP__Port23__default__fired();
#line 61
}
#line 61
# 199 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port24__fired(void ){
#line 61
  HplMsp430InterruptP__Port24__default__fired();
#line 61
}
#line 61
# 200 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port25__fired(void ){
#line 61
  HplMsp430InterruptP__Port25__default__fired();
#line 61
}
#line 61
# 201 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port26__fired(void ){
#line 61
  HplMsp430InterruptP__Port26__default__fired();
#line 61
}
#line 61
# 202 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port27__clear(void )
#line 202
{
#line 202
  P2IFG &= ~(1 << 7);
}

#line 178
static inline void HplMsp430InterruptP__Port27__default__fired(void )
#line 178
{
#line 178
  HplMsp430InterruptP__Port27__clear();
}

# 61 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port27__fired(void ){
#line 61
  HplMsp430InterruptP__Port27__default__fired();
#line 61
}
#line 61
# 88 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ArbiterInfo.nc"
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
# 349 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__disableRxIntr(void )
#line 349
{
  HplMsp430Usart0P__IE1 &= ~(1 << 6);
}

# 177 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr(void ){
#line 177
  HplMsp430Usart0P__Usart__disableRxIntr();
#line 177
}
#line 177
# 170 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data)
#line 170
{

  if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf) {
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos - 1] = data;
    }
  if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos < /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len) {
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp();
    }
  else 
#line 177
    {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr();
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone();
    }
}

# 65 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__rxDone(uint8_t id, uint8_t data)
#line 65
{
}

# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__rxDone(uint8_t arg_0x40ad3f28, uint8_t data){
#line 54
  switch (arg_0x40ad3f28) {
#line 54
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 54
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(data);
#line 54
      break;
#line 54
    default:
#line 54
      /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__rxDone(arg_0x40ad3f28, data);
#line 54
      break;
#line 54
    }
#line 54
}
#line 54
# 80 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/ArbiterInfo.nc"
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
# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__rxDone(uint8_t data)
#line 54
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__rxDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__userId(), data);
    }
}

# 54 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart0P__Interrupts__rxDone(uint8_t data){
#line 54
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__rxDone(data);
#line 54
}
#line 54
# 55 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
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

# 6 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430I2C.nc"
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
# 66 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__I2CInterrupts__default__fired(uint8_t id)
#line 66
{
}

# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__I2CInterrupts__fired(uint8_t arg_0x40acff08){
#line 39
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__I2CInterrupts__default__fired(arg_0x40acff08);
#line 39
}
#line 39
# 59 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawI2CInterrupts__fired(void )
#line 59
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__I2CInterrupts__fired(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__userId());
    }
}

# 39 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
inline static void HplMsp430Usart0P__I2CInterrupts__fired(void ){
#line 39
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawI2CInterrupts__fired();
#line 39
}
#line 39
# 188 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void )
#line 188
{
}

# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__txDone(uint8_t id)
#line 64
{
}

# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__txDone(uint8_t arg_0x40ad3f28){
#line 49
  switch (arg_0x40ad3f28) {
#line 49
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 49
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone();
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__txDone(arg_0x40ad3f28);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__txDone(void )
#line 49
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__Interrupts__txDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__userId());
    }
}

# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart0P__Interrupts__txDone(void ){
#line 49
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__txDone();
#line 49
}
#line 49
# 226 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/msp430hardware.h"
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

# 11 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(12)))  void sig_TIMERA0_VECTOR(void )
#line 11
{
#line 11
  Msp430TimerCommonP__VectorTimerA0__fired();
}

# 169 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
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

# 12 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
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

# 135 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n)
{
}

# 28 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(uint8_t arg_0x4067ade8){
#line 28
  switch (arg_0x4067ade8) {
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
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(arg_0x4067ade8);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 105 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
static error_t CC2420SpiP__Resource__request(uint8_t id)
#line 105
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

# 96 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/StateImplP.nc"
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

# 177 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(uint8_t id)
#line 177
{
  /* atomic removed: atomic calls only */
#line 178
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId == id && /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY) {
        unsigned char __nesc_temp = 
#line 179
        TRUE;

#line 179
        return __nesc_temp;
      }
    else 
#line 180
      {
        unsigned char __nesc_temp = 
#line 180
        FALSE;

#line 180
        return __nesc_temp;
      }
  }
}

# 159 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/SchedulerBasicP.nc"
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

# 133 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void )
#line 133
{
  /* atomic removed: atomic calls only */
#line 134
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__default_owner_id) {
        if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_GRANTING) {
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask();
            {
              unsigned char __nesc_temp = 
#line 138
              SUCCESS;

#line 138
              return __nesc_temp;
            }
          }
        else {
#line 140
          if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_IMM_GRANTING) {
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId;
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY;
              {
                unsigned char __nesc_temp = 
#line 143
                SUCCESS;

#line 143
                return __nesc_temp;
              }
            }
          }
      }
  }
#line 147
  return FAIL;
}

# 96 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/TransformAlarmC.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__set_alarm(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type now = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__get();
#line 98
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type expires;
#line 98
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type remaining;




  expires = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 + /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt;


  remaining = (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type )(expires - now);


  if (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 <= now) 
    {
      if (expires >= /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY) 
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 = now + /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt = remaining - /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      remaining = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY;
    }
  else 
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 += /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt = 0;
    }
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt((/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type )now << 0, 
  (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type )remaining << 0);
}

# 69 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/TransformCounterC.nc"
static /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get(void )
{
  /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type rv = 0;

#line 72
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*Counter32khz32C.Transform*/TransformCounterC__0__upper_count_type high = /*Counter32khz32C.Transform*/TransformCounterC__0__m_upper;
      /*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type low = /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__get();

#line 76
      if (/*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__get();
        }
      {
        /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type high_to = high;
        /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type low_to = low >> /*Counter32khz32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT;

#line 90
        rv = (high_to << /*Counter32khz32C.Transform*/TransformCounterC__0__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 92
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 51 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerP.nc"
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

# 114 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420TransmitP.nc"
static error_t CC2420TransmitP__Sendframe__send(uint8_t *frame, uint8_t frame_length)
{


  if (CC2420TransmitP__acquireSpiResource() == SUCCESS) {
      CC2420TransmitP__CSN__clr();

      CC2420TransmitP__TXCTRL__write((((2 << CC2420_TXCTRL_TXMIXBUF_CUR) | (
      3 << CC2420_TXCTRL_PA_CURRENT)) | (
      1 << CC2420_TXCTRL_RESERVED)) | ((
      31 & 0x1F) << CC2420_TXCTRL_PA_LEVEL));


      CC2420TransmitP__TXFIFO__write((uint8_t *)frame, frame_length);
    }

  return SUCCESS;
}

# 124 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
static error_t CC2420SpiP__Resource__immediateRequest(uint8_t id)
#line 124
{
  error_t error;

  /* atomic removed: atomic calls only */
#line 127
  {
    if (CC2420SpiP__WorkingState__requestState(CC2420SpiP__S_BUSY) != SUCCESS) {
        {
          unsigned char __nesc_temp = 
#line 129
          EBUSY;

#line 129
          return __nesc_temp;
        }
      }

    if (CC2420SpiP__SpiResource__isOwner()) {
        CC2420SpiP__m_holder = id;
        error = SUCCESS;
      }
    else {
#line 137
      if ((error = CC2420SpiP__SpiResource__immediateRequest()) == SUCCESS) {
          CC2420SpiP__m_holder = id;
        }
      else {
          CC2420SpiP__WorkingState__toIdle();
        }
      }
  }
#line 144
  return error;
}

# 265 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
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

# 46 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
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

# 301 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Reg__write(uint8_t addr, uint16_t data)
#line 301
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 302
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 304
            0;

            {
#line 304
              __nesc_atomic_end(__nesc_atomic); 
#line 304
              return __nesc_temp;
            }
          }
        }
    }
#line 308
    __nesc_atomic_end(__nesc_atomic); }
#line 307
  CC2420SpiP__SpiByte__write(addr);
  CC2420SpiP__SpiByte__write(data >> 8);
  return CC2420SpiP__SpiByte__write(data & 0xff);
}

# 99 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(uint8_t tx)
#line 99
{
  uint8_t byte;


  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(tx);
  while (!/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending()) ;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__clrRxIntr();
  byte = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx();

  return byte;
}

# 386 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static uint8_t HplMsp430Usart0P__Usart__rx(void )
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

# 144 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len)
#line 146
{

  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_client = id;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf = tx_buf;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf = rx_buf;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len = len;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos = 0;

  if (len) {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__enableRxIntr();
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp();
    }
  else {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__postTask();
    }

  return SUCCESS;
}

#line 121
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp(void )
#line 121
{

  uint8_t end;
  uint8_t tmp;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 126
    {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf ? /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos] : 0);

      end = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos + /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SPI_ATOMIC_SIZE;
      if (end > /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len) {
        end = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len;
        }
      while (++/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos < end) {
          while (!/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending()) ;
          tmp = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx();
          if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf) {
            /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos - 1] = tmp;
            }
#line 138
          /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf ? /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos] : 0);
        }
    }
#line 140
    __nesc_atomic_end(__nesc_atomic); }
}

# 58 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/RandomMlcgC.nc"
static uint32_t RandomMlcgC__Random__rand32(void )
#line 58
{
  uint32_t mlcg;
#line 59
  uint32_t p;
#line 59
  uint32_t q;
  uint64_t tmpseed;

  /* atomic removed: atomic calls only */
#line 62
  {
    tmpseed = (uint64_t )33614U * (uint64_t )RandomMlcgC__seed;
    q = tmpseed;
    q = q >> 1;
    p = tmpseed >> 32;
    mlcg = p + q;
    if (mlcg & 0x80000000) {
        mlcg = mlcg & 0x7FFFFFFF;
        mlcg++;
      }
    RandomMlcgC__seed = mlcg;
  }
  return mlcg;
}

# 4737 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static uint8_t MacP__check_csma_ca_backoff_send_conditions(uint32_t delay_backoffs)
{

  uint32_t number_of_sd_ticks = 0;
  uint32_t current_ticks = 0;
  uint32_t ticks_remaining = 0;
  uint32_t number_of_backoffs_remaining = 0;

  number_of_sd_ticks = MacP__TimerAsync__get_sd_ticks();

  current_ticks = MacP__TimerAsync__get_current_ticks();

  ticks_remaining = number_of_sd_ticks - current_ticks;

  number_of_backoffs_remaining = ticks_remaining / 5;

  if (number_of_backoffs_remaining > delay_backoffs) {
    return 0;
    }
  else {
#line 4756
    return 1;
    }
}

# 136 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/TransformAlarmC.nc"
static void /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Alarm__startAt(/*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__to_size_type t0, /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__m_t0 = t0;
      /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__m_dt = dt;
      /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__set_alarm();
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

#line 96
static void /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__set_alarm(void )
{
  /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__to_size_type now = /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__Counter__get();
#line 98
  /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__to_size_type expires;
#line 98
  /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__to_size_type remaining;




  expires = /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__m_t0 + /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__m_dt;


  remaining = (/*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__to_size_type )(expires - now);


  if (/*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__m_t0 <= now) 
    {
      if (expires >= /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__MAX_DELAY) 
    {
      /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__m_t0 = now + /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__MAX_DELAY;
      /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__m_dt = remaining - /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__MAX_DELAY;
      remaining = /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__MAX_DELAY;
    }
  else 
    {
      /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__m_t0 += /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__m_dt;
      /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__m_dt = 0;
    }
  /*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__startAt((/*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__from_size_type )now << 0, 
  (/*TimerAsyncC.Alarm.Transform*/TransformAlarmC__1__from_size_type )remaining << 0);
}

#line 96
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__set_alarm(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__to_size_type now = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Counter__get();
#line 98
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__to_size_type expires;
#line 98
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__to_size_type remaining;




  expires = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__m_dt;


  remaining = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__to_size_type )(expires - now);


  if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__m_t0 <= now) 
    {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__MAX_DELAY) 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__m_t0 = now + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__MAX_DELAY;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__m_dt = remaining - /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__MAX_DELAY;
      remaining = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__MAX_DELAY;
    }
  else 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__m_t0 += /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__m_dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__m_dt = 0;
    }
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__AlarmFrom__startAt((/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__from_size_type )now << 5, 
  (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__from_size_type )remaining << 5);
}

# 69 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/TransformCounterC.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__get(void )
{
  /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type rv = 0;

#line 72
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMilli32C.Transform*/TransformCounterC__1__upper_count_type high = /*CounterMilli32C.Transform*/TransformCounterC__1__m_upper;
      /*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type low = /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__get();

#line 76
      if (/*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__get();
        }
      {
        /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type high_to = high;
        /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type low_to = low >> /*CounterMilli32C.Transform*/TransformCounterC__1__LOW_SHIFT_RIGHT;

#line 90
        rv = (high_to << /*CounterMilli32C.Transform*/TransformCounterC__1__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 92
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 14 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(24)))  void sig_TIMERB1_VECTOR(void )
#line 14
{
#line 14
  Msp430TimerCommonP__VectorTimerB1__fired();
}

# 52 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/RealMainP.nc"
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

# 164 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/timer/Msp430ClockP.nc"
static void Msp430ClockP__set_dco_calib(int calib)
{
  BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  DCOCTL = calib & 0xff;
}

# 16 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/platforms/telosb/MotePlatformC.nc"
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

# 123 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/SchedulerBasicP.nc"
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

# 64 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0x4059bb40){
#line 64
  switch (arg_0x4059bb40) {
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
    case MacP__signal_loss:
#line 64
      MacP__signal_loss__runTask();
#line 64
      break;
#line 64
    case MacP__start_coordinator_gts_send:
#line 64
      MacP__start_coordinator_gts_send__runTask();
#line 64
      break;
#line 64
    case MacP__increment_gts_null:
#line 64
      MacP__increment_gts_null__runTask();
#line 64
      break;
#line 64
    case MacP__start_gts_send:
#line 64
      MacP__start_gts_send__runTask();
#line 64
      break;
#line 64
    case MacP__check_gts_expiration:
#line 64
      MacP__check_gts_expiration__runTask();
#line 64
      break;
#line 64
    case MacP__data_channel_scan_indication:
#line 64
      MacP__data_channel_scan_indication__runTask();
#line 64
      break;
#line 64
    case MacP__perform_csma_ca_unslotted:
#line 64
      MacP__perform_csma_ca_unslotted__runTask();
#line 64
      break;
#line 64
    case MacP__perform_csma_ca_slotted:
#line 64
      MacP__perform_csma_ca_slotted__runTask();
#line 64
      break;
#line 64
    case MacP__data_indication:
#line 64
      MacP__data_indication__runTask();
#line 64
      break;
#line 64
    case MacP__send_frame_csma:
#line 64
      MacP__send_frame_csma__runTask();
#line 64
      break;
#line 64
    case MacP__create_beacon:
#line 64
      MacP__create_beacon__runTask();
#line 64
      break;
#line 64
    case PhyP__startDone_task:
#line 64
      PhyP__startDone_task__runTask();
#line 64
      break;
#line 64
    case PhyP__stopDone_task:
#line 64
      PhyP__stopDone_task__runTask();
#line 64
      break;
#line 64
    case PhyP__sendDone_task:
#line 64
      PhyP__sendDone_task__runTask();
#line 64
      break;
#line 64
    case CC2420ControlP__sync:
#line 64
      CC2420ControlP__sync__runTask();
#line 64
      break;
#line 64
    case CC2420ControlP__syncDone:
#line 64
      CC2420ControlP__syncDone__runTask();
#line 64
      break;
#line 64
    case CC2420SpiP__grant:
#line 64
      CC2420SpiP__grant__runTask();
#line 64
      break;
#line 64
    case /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task:
#line 64
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask();
#line 64
      break;
#line 64
    case /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask:
#line 64
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask();
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
    default:
#line 64
      SchedulerBasicP__TaskBasic__default__runTask(arg_0x4059bb40);
#line 64
      break;
#line 64
    }
#line 64
}
#line 64
# 62 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/VirtualizeTimerC.nc"
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

# 143 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/printf/PrintfP.nc"
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

# 124 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialActiveMessageP.nc"
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

# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/AMQueueEntryP.nc"
static error_t /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 47
{
  /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(msg, dest);
  /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(msg, 100);
  return /*PrintfC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(msg, len);
}

# 161 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialActiveMessageP.nc"
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

# 502 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialP.nc"
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

# 4065 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static error_t MacP__MCPS_DATA__request(uint8_t SrcAddrMode, uint16_t SrcPANId, uint32_t SrcAddr[], uint8_t DstAddrMode, uint16_t DestPANId, uint32_t DstAddr[], uint8_t msduLength, uint8_t msdu[], uint8_t msduHandle, uint8_t TxOptions)
{
  int i;

  uint32_t total_ticks;

#line 4090
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 4090
    {

      if (MacP__mac_PIB.macShortAddress == 0xffff) 
        {
          unsigned char __nesc_temp = 
#line 4093
          FAIL;

          {
#line 4093
            __nesc_atomic_end(__nesc_atomic); 
#line 4093
            return __nesc_temp;
          }
        }
    }
#line 4096
    __nesc_atomic_end(__nesc_atomic); }
#line 4096
  if (MacP__PANCoordinator == 1) 
    {



      if (get_txoptions_gts(TxOptions) == 1) 
        {

          for (i = 0; i < 7; i++) 
            {

              if (MacP__GTS_db[i].DevAddressType == (uint16_t )DstAddr[1] && MacP__GTS_db[i].direction == 1 && MacP__GTS_db[i].gts_id != 0) 
                {



                  MacP__create_data_frame(SrcAddrMode, SrcPANId, SrcAddr, DstAddrMode, DestPANId, DstAddr, msduLength, msdu, msduHandle, TxOptions, MacP__GTS_db[i].starting_slot, 1);

                  return SUCCESS;
                  break;
                }
            }
          MacP__MCPS_DATA__confirm(msduHandle, MAC_INVALID_GTS);
          return FAIL;
        }
      else 
        {
#line 4141
          MacP__create_data_frame(SrcAddrMode, SrcPANId, SrcAddr, DstAddrMode, DestPANId, DstAddr, msduLength, msdu, msduHandle, TxOptions, 0, 0);
        }
    }
  else 

    {

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 4148
        {



          if (get_txoptions_gts(TxOptions) == 1) 
            {

              if (MacP__s_GTSss == 0x00) 
                {

                  MacP__MCPS_DATA__confirm(msduHandle, MAC_INVALID_GTS);
                }
              else 
                {
                  total_ticks = MacP__TimerAsync__get_total_tick_counter();
                  msdu[0] = (uint8_t )(total_ticks >> 0);
                  msdu[1] = (uint8_t )(total_ticks >> 8);
                  msdu[2] = (uint8_t )(total_ticks >> 16);
                  msdu[3] = (uint8_t )(total_ticks >> 24);

                  if (MacP__on_sync == 1 && MacP__s_GTSss > 0) {
                    MacP__create_data_frame(SrcAddrMode, SrcPANId, SrcAddr, DstAddrMode, DestPANId, DstAddr, msduLength, msdu, msduHandle, TxOptions, MacP__s_GTSss, 0);
                    }
                }
            }
          else {


              MacP__create_data_frame(SrcAddrMode, SrcPANId, SrcAddr, DstAddrMode, DestPANId, DstAddr, msduLength, msdu, msduHandle, TxOptions, 0, 0);
            }
        }
#line 4178
        __nesc_atomic_end(__nesc_atomic); }
    }
  return SUCCESS;
}

#line 2482
static void MacP__create_data_frame(uint8_t SrcAddrMode, uint16_t SrcPANId, uint32_t SrcAddr[], uint8_t DstAddrMode, uint16_t DestPANId, uint32_t DstAddr[], uint8_t msduLength, uint8_t msdu[], uint8_t msduHandle, uint8_t TxOptions, uint8_t on_gts_slot, uint8_t pan)
{

  int i_indirect_trans = 0;

  dest_short *dest_short_ptr;
  dest_long *dest_long_ptr;

  source_short *source_short_ptr;
  source_long *source_long_ptr;





  uint8_t intra_pan = 0;
  uint8_t data_len = 0;

  uint8_t current_gts_element_count = 0;

  MPDU *frame_pkt = 0;

  uint16_t frame_control;




  if (on_gts_slot > 0) 
    {

      if (MacP__PANCoordinator == 1) 
        {



          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 2517
            current_gts_element_count = MacP__gts_slot_list[15 - on_gts_slot].element_count;
#line 2517
            __nesc_atomic_end(__nesc_atomic); }



          if (current_gts_element_count == 3 || MacP__available_gts_index_count == 0) 
            {

              MacP__MCPS_DATA__confirm(0x00, MAC_TRANSACTION_OVERFLOW);
              return;
            }
          else 
            {
              frame_pkt = (MPDU *)&MacP__gts_send_buffer[MacP__available_gts_index[MacP__available_gts_index_count]];
            }
        }
      else 

        {



          if (MacP__gts_send_buffer_count == 3) 
            {
              MacP__MCPS_DATA__confirm(0x00, MAC_TRANSACTION_OVERFLOW);
              return;
            }
          if (MacP__gts_send_buffer_msg_in == 3) {
            MacP__gts_send_buffer_msg_in = 0;
            }
          frame_pkt = (MPDU *)&MacP__gts_send_buffer[MacP__gts_send_buffer_msg_in];
        }
    }
  else 

    {

      if (get_txoptions_indirect_transmission(TxOptions) == 1) 
        {



          if (MacP__indirect_trans_count == 2) 
            {
              MacP__MCPS_DATA__confirm(0x00, MAC_TRANSACTION_OVERFLOW);

              return;
            }

          for (i_indirect_trans = 0; i_indirect_trans < 2; i_indirect_trans++) 
            {
              if (MacP__indirect_trans_queue[i_indirect_trans].handler == 0x00) 
                {
                  frame_pkt = (MPDU *)& MacP__indirect_trans_queue[i_indirect_trans].frame;
                  break;
                }
            }
        }
      else 


        {


          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 2580
            {
              if (MacP__send_buffer_count + 1 > 3) {
                {
#line 2582
                  __nesc_atomic_end(__nesc_atomic); 
#line 2582
                  return;
                }
                }
#line 2584
              if (MacP__send_buffer_msg_in == 3) {
                MacP__send_buffer_msg_in = 0;
                }
              frame_pkt = (MPDU *)&MacP__send_buffer[MacP__send_buffer_msg_in];
            }
#line 2588
            __nesc_atomic_end(__nesc_atomic); }
        }
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 2592
    {

      if (intra_pan == 0) 
        {

          if (DstAddrMode > 1 && SrcAddrMode > 1) 
            {

              if (DstAddrMode == 3 && SrcAddrMode == 3) 
                {
                  dest_long_ptr = (dest_long *)&frame_pkt->data[0];
                  source_long_ptr = (source_long *)&frame_pkt->data[10];

                  dest_long_ptr->destination_PAN_identifier = DestPANId;
                  dest_long_ptr->destination_address0 = DstAddr[1];
                  dest_long_ptr->destination_address1 = DstAddr[0];

                  source_long_ptr->source_PAN_identifier = SrcPANId;
                  source_long_ptr->source_address0 = SrcAddr[1];
                  source_long_ptr->source_address1 = SrcAddr[0];

                  data_len = 20;
                }


              if (DstAddrMode == 2 && SrcAddrMode == 3) 
                {
                  dest_short_ptr = (dest_short *)&frame_pkt->data[0];
                  source_long_ptr = (source_long *)&frame_pkt->data[4];

                  dest_short_ptr->destination_PAN_identifier = DestPANId;
                  dest_short_ptr->destination_address = (uint16_t )DstAddr[1];

                  source_long_ptr->source_PAN_identifier = SrcPANId;
                  source_long_ptr->source_address0 = SrcAddr[1];
                  source_long_ptr->source_address1 = SrcAddr[0];

                  data_len = 14;
                }

              if (DstAddrMode == 3 && SrcAddrMode == 2) 
                {
                  dest_long_ptr = (dest_long *)&frame_pkt->data[0];
                  source_short_ptr = (source_short *)&frame_pkt->data[10];

                  dest_long_ptr->destination_PAN_identifier = DestPANId;
                  dest_long_ptr->destination_address0 = DstAddr[1];
                  dest_long_ptr->destination_address1 = DstAddr[0];

                  source_short_ptr->source_PAN_identifier = SrcPANId;
                  source_short_ptr->source_address = (uint16_t )SrcAddr[1];

                  data_len = 14;
                }



              if (DstAddrMode == 2 && SrcAddrMode == 2) 
                {
                  dest_short_ptr = (dest_short *)&frame_pkt->data[0];
                  source_short_ptr = (source_short *)&frame_pkt->data[4];

                  dest_short_ptr->destination_PAN_identifier = DestPANId;
                  dest_short_ptr->destination_address = (uint16_t )DstAddr[1];

                  source_short_ptr->source_PAN_identifier = SrcPANId;
                  source_short_ptr->source_address = (uint16_t )SrcAddr[1];

                  data_len = 8;
                }
            }

          if (DstAddrMode == 0 && SrcAddrMode > 1) 
            {

              if (SrcAddrMode == 3) 
                {
                  source_long_ptr = (source_long *)&frame_pkt->data[0];

                  source_long_ptr->source_PAN_identifier = SrcPANId;
                  source_long_ptr->source_address0 = SrcAddr[1];
                  source_long_ptr->source_address1 = SrcAddr[0];

                  data_len = 10;
                }
              else 
                {

                  source_short_ptr = (source_short *)&frame_pkt->data[0];

                  source_short_ptr->source_PAN_identifier = SrcPANId;
                  source_short_ptr->source_address = (uint16_t )SrcAddr[1];

                  data_len = 4;
                }
            }

          if (DstAddrMode > 1 && SrcAddrMode == 0) 
            {
              if (DstAddrMode == 3) 
                {
                  dest_long_ptr = (dest_long *)&frame_pkt->data[0];

                  dest_long_ptr->destination_PAN_identifier = DestPANId;
                  dest_long_ptr->destination_address0 = DstAddr[1];
                  dest_long_ptr->destination_address1 = DstAddr[0];

                  data_len = 10;
                }
              else 
                {
                  dest_short_ptr = (dest_short *)&frame_pkt->data[0];

                  dest_short_ptr->destination_PAN_identifier = DestPANId;
                  dest_short_ptr->destination_address = (uint16_t )DstAddr[1];

                  data_len = 4;
                }
            }
        }
      else 
        {
        }



      memcpy(&frame_pkt->data[data_len], &msdu[0], msduLength * sizeof(uint8_t ));

      if (on_gts_slot > 0) 
        {



          frame_pkt->length = data_len + msduLength + 5;

          frame_control = set_frame_control(1, 0, 0, 1, intra_pan, DstAddrMode, SrcAddrMode);
          frame_pkt->frame_control1 = (uint8_t )frame_control;
          frame_pkt->frame_control2 = (uint8_t )(frame_control >> 8);

          frame_pkt->seq_num = MacP__mac_PIB.macDSN;
          MacP__mac_PIB.macDSN++;


          {
            if (MacP__PANCoordinator == 1) 
              {
                MacP__gts_slot_list[15 - on_gts_slot].element_count++;
                MacP__gts_slot_list[15 - on_gts_slot].gts_send_frame_index[MacP__gts_slot_list[15 - on_gts_slot].element_in] = MacP__available_gts_index[MacP__available_gts_index_count];


                MacP__gts_slot_list[15 - on_gts_slot].element_in++;

                if (MacP__gts_slot_list[15 - on_gts_slot].element_in == 3) {
                  MacP__gts_slot_list[15 - on_gts_slot].element_in = 0;
                  }
                MacP__available_gts_index_count--;
              }
            else 


              {
                MacP__gts_send_buffer_count++;
                MacP__gts_send_buffer_msg_in++;
              }
          }
        }
      else 

        {

          frame_pkt->length = data_len + msduLength + 5;


          frame_control = set_frame_control(1, 0, 0, get_txoptions_ack(TxOptions), intra_pan, DstAddrMode, SrcAddrMode);
          frame_pkt->frame_control1 = (uint8_t )frame_control;
          frame_pkt->frame_control2 = (uint8_t )(frame_control >> 8);

          frame_pkt->seq_num = MacP__mac_PIB.macDSN;



          MacP__mac_PIB.macDSN++;

          if (get_txoptions_indirect_transmission(TxOptions) == 1) 
            {
              MacP__indirect_trans_queue[i_indirect_trans].handler = MacP__indirect_trans_count + 1;
              MacP__indirect_trans_queue[i_indirect_trans].transaction_persistent_time = 0x0000;

              MacP__indirect_trans_count++;
            }
          else 


            {

              MacP__send_buffer[MacP__send_buffer_msg_in].retransmission = 1;
              MacP__send_buffer[MacP__send_buffer_msg_in].indirect = 0;

              MacP__send_buffer_count++;

              MacP__send_buffer_msg_in++;

              MacP__send_frame_csma__postTask();
            }
        }
    }
#line 2797
    __nesc_atomic_end(__nesc_atomic); }


  return;
}

# 35 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/includes/mac_func.h"
static uint16_t set_frame_control(uint8_t frame_type, uint8_t security, uint8_t frame_pending, uint8_t ack_request, uint8_t intra_pan, uint8_t dest_addr_mode, uint8_t source_addr_mode)
{
  uint8_t fc_b1 = 0;
  uint8_t fc_b2 = 0;

#line 39
  fc_b1 = ((((intra_pan << 6) | (ack_request << 5)) | (frame_pending << 4)) | (
  security << 3)) | (frame_type << 0);
  fc_b2 = (source_addr_mode << 6) | (dest_addr_mode << 2);
  return (fc_b2 << 8) | (fc_b1 << 0);
}

# 3909 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static error_t MacP__MLME_SET__request(uint8_t PIBAttribute, uint8_t PIBAttributeValue[])
{





  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 3916
    {

      switch (PIBAttribute) 
        {


          case MACACKWAITDURATION: MacP__mac_PIB.macAckWaitDuration = PIBAttributeValue[0];

          MacP__MLME_SET__confirm(MAC_SUCCESS, PIBAttribute);
          break;

          case MACASSOCIATIONPERMIT: if ((uint8_t )PIBAttributeValue[1] == 0x00) 
              {
                MacP__mac_PIB.macAssociationPermit = 0x00;
              }
            else 
              {
                MacP__mac_PIB.macAssociationPermit = 0x01;
              }

          MacP__MLME_SET__confirm(MAC_SUCCESS, PIBAttribute);
          break;

          case MACAUTOREQUEST: MacP__mac_PIB.macAutoRequest = PIBAttributeValue[0];

          MacP__MLME_SET__confirm(MAC_SUCCESS, PIBAttribute);
          break;

          case MACBATTLIFEEXT: MacP__mac_PIB.macBattLifeExt = PIBAttributeValue[0];

          MacP__MLME_SET__confirm(MAC_SUCCESS, PIBAttribute);
          break;
          case MACBATTLIFEEXTPERIODS: MacP__mac_PIB.macBattLifeExtPeriods = PIBAttributeValue[0];

          MacP__MLME_SET__confirm(MAC_SUCCESS, PIBAttribute);
          break;

          case MACBEACONPAYLOAD: 




            memcpy(&PIBAttributeValue[0], &MacP__mac_PIB.macBeaconPayload[0], MacP__mac_PIB.macBeaconPayloadLenght * sizeof(uint8_t ));

          MacP__MLME_SET__confirm(MAC_SUCCESS, PIBAttribute);
          break;

          case MACMAXBEACONPAYLOADLENGTH: MacP__mac_PIB.macBeaconPayloadLenght = PIBAttributeValue[0];

          MacP__MLME_SET__confirm(MAC_SUCCESS, PIBAttribute);
          break;

          case MACBEACONORDER: MacP__mac_PIB.macBeaconOrder = PIBAttributeValue[0];

          MacP__MLME_SET__confirm(MAC_SUCCESS, PIBAttribute);
          break;
          case MACBEACONTXTIME: MacP__mac_PIB.macBeaconTxTime = PIBAttributeValue[0];

          MacP__MLME_SET__confirm(MAC_SUCCESS, PIBAttribute);
          break;
          case MACBSN: MacP__mac_PIB.macBSN = PIBAttributeValue[0];

          MacP__MLME_SET__confirm(MAC_SUCCESS, PIBAttribute);
          break;

          case MACCOORDEXTENDEDADDRESS: 




            MacP__MLME_SET__confirm(MAC_SUCCESS, PIBAttribute);
          break;


          case MACCOORDSHORTADDRESS: MacP__mac_PIB.macCoordShortAddress = (PIBAttributeValue[0] << 8) | PIBAttributeValue[1];

          MacP__MLME_SET__confirm(MAC_SUCCESS, PIBAttribute);
          break;

          case MACDSN: MacP__mac_PIB.macDSN = PIBAttributeValue[0];

          MacP__MLME_SET__confirm(MAC_SUCCESS, PIBAttribute);
          break;
          case MACGTSPERMIT: MacP__mac_PIB.macGTSPermit = PIBAttributeValue[0];

          MacP__MLME_SET__confirm(MAC_SUCCESS, PIBAttribute);
          break;
          case MACMAXCSMABACKOFFS: MacP__mac_PIB.macMaxCSMABackoffs = PIBAttributeValue[0];

          MacP__MLME_SET__confirm(MAC_SUCCESS, PIBAttribute);
          break;
          case MACMINBE: MacP__mac_PIB.macMinBE = PIBAttributeValue[0];

          MacP__MLME_SET__confirm(MAC_SUCCESS, PIBAttribute);
          break;

          case MACPANID: MacP__mac_PIB.macPANId = (PIBAttributeValue[0] << 8) | PIBAttributeValue[1];



          MacP__AddressFilter__set_coord_address(MacP__mac_PIB.macCoordShortAddress, MacP__mac_PIB.macPANId);


          MacP__MLME_SET__confirm(MAC_SUCCESS, PIBAttribute);
          break;

          case MACPROMISCUOUSMODE: MacP__mac_PIB.macPromiscuousMode = PIBAttributeValue[0];

          MacP__MLME_SET__confirm(MAC_SUCCESS, PIBAttribute);
          break;
          case MACRXONWHENIDLE: MacP__mac_PIB.macRxOnWhenIdle = PIBAttributeValue[0];

          MacP__MLME_SET__confirm(MAC_SUCCESS, PIBAttribute);
          break;


          case MACSHORTADDRESS: MacP__mac_PIB.macShortAddress = (PIBAttributeValue[0] << 8) | PIBAttributeValue[1];


          MacP__AddressFilter__set_address(MacP__mac_PIB.macShortAddress, MacP__aExtendedAddress0, MacP__aExtendedAddress0);


          MacP__MLME_SET__confirm(MAC_SUCCESS, PIBAttribute);
          break;

          case MACSUPERFRAMEORDER: MacP__mac_PIB.macSuperframeOrder = PIBAttributeValue[0];

          MacP__MLME_SET__confirm(MAC_SUCCESS, PIBAttribute);
          break;

          case MACTRANSACTIONPERSISTENCETIME: MacP__mac_PIB.macTransactionPersistenceTime = PIBAttributeValue[0];

          MacP__MLME_SET__confirm(MAC_SUCCESS, PIBAttribute);
          break;

          default: MacP__MLME_SET__confirm(MAC_UNSUPPORTED_ATTRIBUTE, PIBAttribute);
          break;
        }
    }
#line 4054
    __nesc_atomic_end(__nesc_atomic); }




  return SUCCESS;
}

# 291 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/phy/PhyP.nc"
static error_t PhyP__PLME_SET__request(uint8_t PIBAttribute, uint8_t PIBAttributeValue)
#line 291
{



  switch (PIBAttribute) 
    {
      case PHYCURRENTCHANNEL: 

        PhyP__phy_PIB.phyCurrentChannel = PIBAttributeValue;

      PhyP__CC2420Config__setChannel(PhyP__phy_PIB.phyCurrentChannel);

      PhyP__CC2420Config__sync();


      PhyP__PLME_SET__confirm(PHY_SUCCESS, PIBAttribute);
      break;

      case PHYCHANNELSSUPPORTED: 
        PhyP__phy_PIB.phyChannelsSupported = PIBAttributeValue;
      PhyP__PLME_SET__confirm(PHY_SUCCESS, PIBAttribute);
      break;

      case PHYTRANSMITPOWER: 
        PhyP__phy_PIB.phyTransmitPower = PIBAttributeValue;

      PhyP__PLME_SET__confirm(PHY_SUCCESS, PIBAttribute);
      break;
      case PHYCCAMODE: 
        PhyP__phy_PIB.phyCcaMode = PIBAttributeValue;
      PhyP__PLME_SET__confirm(PHY_SUCCESS, PIBAttribute);
      break;
      default: 
        PhyP__PLME_SET__confirm(PHY_UNSUPPORTED_ATTRIBUTE, PIBAttribute);
      break;
    }
  return SUCCESS;
}

# 311 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ControlP.nc"
static error_t CC2420ControlP__CC2420Config__sync(void )
#line 311
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 312
    {
      if (CC2420ControlP__m_sync_busy) {
          {
            unsigned char __nesc_temp = 
#line 314
            FAIL;

            {
#line 314
              __nesc_atomic_end(__nesc_atomic); 
#line 314
              return __nesc_temp;
            }
          }
        }
#line 317
      CC2420ControlP__m_sync_busy = TRUE;
      if (CC2420ControlP__m_state == CC2420ControlP__S_XOSC_STARTED) {
          CC2420ControlP__SyncResource__request();
        }
      else 
#line 320
        {
          CC2420ControlP__syncDone__postTask();
        }
    }
#line 323
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 133 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsyncM.nc"
static error_t TimerAsyncM__TimerAsync__set_backoff_symbols(uint8_t Backoff_Duration_Symbols)
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      TimerAsyncM__backoff_symbols = Backoff_Duration_Symbols;
      TimerAsyncM__backoff_ticks = 1;
    }
#line 140
    __nesc_atomic_end(__nesc_atomic); }

  return SUCCESS;
}

#line 102
static error_t TimerAsyncM__TimerAsync__set_bi_sd(uint32_t bi_symbols, uint32_t sd_symbols)
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 105
    {
      TimerAsyncM__time_slot_backoff_periods = sd_symbols / 16 / TimerAsyncM__backoff_symbols;
      TimerAsyncM__time_slot_ticks = TimerAsyncM__time_slot_backoff_periods * TimerAsyncM__backoff_ticks;
      TimerAsyncM__time_slot_tick_next_fire = TimerAsyncM__time_slot_ticks;
      TimerAsyncM__before_time_slot_ticks = TimerAsyncM__time_slot_ticks - 5;
      TimerAsyncM__sd_ticks = TimerAsyncM__time_slot_ticks * 16;

      if (bi_symbols == sd_symbols) 
        {

          TimerAsyncM__sd_ticks = TimerAsyncM__sd_ticks - 2;
        }

      TimerAsyncM__bi_backoff_periods = bi_symbols / TimerAsyncM__backoff_symbols;
      TimerAsyncM__bi_ticks = TimerAsyncM__bi_backoff_periods * TimerAsyncM__backoff_ticks;

      TimerAsyncM__before_bi_ticks = TimerAsyncM__bi_ticks - 100;
    }
#line 122
    __nesc_atomic_end(__nesc_atomic); }






  return SUCCESS;
}

# 3665 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static error_t MacP__MLME_GTS__request(uint8_t GTSCharacteristics, bool security_enable)
{



  if (MacP__mac_PIB.macShortAddress == 0xffff) {
    MacP__MLME_GTS__confirm(GTSCharacteristics, MAC_NO_SHORT_ADDRESS);
    }


  MacP__gts_request = 1;

  MacP__create_gts_request_cmd(GTSCharacteristics);

  return SUCCESS;
}

# 207 "GTSManagementExampleP.nc"
static error_t GTSManagementExampleP__MLME_GTS__confirm(uint8_t GTSCharacteristics, uint8_t status)
{
  switch (status) 
    {
      case MAC_SUCCESS: GTSManagementExampleP__gts_allocated = 1;
      GTSManagementExampleP__Leds__led1Toggle();
      break;

      case MAC_DENIED: GTSManagementExampleP__gts_allocated = 0;
      break;

      case MAC_NO_SHORT_ADDRESS: GTSManagementExampleP__gts_allocated = 0;
      break;

      case MAC_CHANNEL_ACCESS_FAILURE: GTSManagementExampleP__gts_allocated = 0;
      break;

      case MAC_NO_ACK: GTSManagementExampleP__gts_allocated = 0;
#line 224
      break;

      case MAC_NO_DATA: GTSManagementExampleP__gts_allocated = 0;
#line 226
      break;


      default: break;
    }


  return SUCCESS;
}

# 133 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

#line 136
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}






static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, TRUE);
}

# 136 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__m_t0 = t0;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__m_dt = dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__2__set_alarm();
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

# 325 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
static void CC2420SpiP__SpiPacket__sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error)
#line 326
{
  if (CC2420SpiP__m_addr & 0x40) {
      CC2420SpiP__Fifo__readDone(CC2420SpiP__m_addr & ~0x40, rx_buf, len, error);
    }
  else 
#line 329
    {
      CC2420SpiP__Fifo__writeDone(CC2420SpiP__m_addr, tx_buf, len, error);
    }
}

# 917 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ReceiveP.nc"
static void CC2420ReceiveP__flush(void )
#line 917
{

  CC2420ReceiveP__reset_state();
  CC2420ReceiveP__CSN__set();
  CC2420ReceiveP__CSN__clr();
  CC2420ReceiveP__SFLUSHRX__strobe();
  CC2420ReceiveP__SFLUSHRX__strobe();
  CC2420ReceiveP__CSN__set();
  CC2420ReceiveP__SpiResource__release();
  CC2420ReceiveP__waitForNextPacket();
}

# 45 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
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

# 314 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Strobe__strobe(uint8_t addr)
#line 314
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 315
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 317
            0;

            {
#line 317
              __nesc_atomic_end(__nesc_atomic); 
#line 317
              return __nesc_temp;
            }
          }
        }
    }
#line 321
    __nesc_atomic_end(__nesc_atomic); }
#line 321
  return CC2420SpiP__SpiByte__write(addr);
}

#line 147
static error_t CC2420SpiP__Resource__release(uint8_t id)
#line 147
{
  uint8_t i;

#line 149
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 149
    {
      if (CC2420SpiP__m_holder != id) {
          {
            unsigned char __nesc_temp = 
#line 151
            FAIL;

            {
#line 151
              __nesc_atomic_end(__nesc_atomic); 
#line 151
              return __nesc_temp;
            }
          }
        }
#line 154
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
#line 167
                    SUCCESS;

                    {
#line 167
                      __nesc_atomic_end(__nesc_atomic); 
#line 167
                      return __nesc_temp;
                    }
                  }
                }
            }
        }
    }
#line 173
    __nesc_atomic_end(__nesc_atomic); }
#line 173
  return SUCCESS;
}

#line 335
static error_t CC2420SpiP__attemptRelease(void )
#line 335
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 337
    {


      if ((
#line 338
      CC2420SpiP__m_requests > 0
       || CC2420SpiP__m_holder != CC2420SpiP__NO_HOLDER)
       || !CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 341
            FAIL;

            {
#line 341
              __nesc_atomic_end(__nesc_atomic); 
#line 341
              return __nesc_temp;
            }
          }
        }
    }
#line 345
    __nesc_atomic_end(__nesc_atomic); }
#line 344
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 344
    CC2420SpiP__release = TRUE;
#line 344
    __nesc_atomic_end(__nesc_atomic); }
  CC2420SpiP__ChipSpiResource__releasing();
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 346
    {
      if (CC2420SpiP__release) {
          CC2420SpiP__SpiResource__release();
          {
            unsigned char __nesc_temp = 
#line 349
            SUCCESS;

            {
#line 349
              __nesc_atomic_end(__nesc_atomic); 
#line 349
              return __nesc_temp;
            }
          }
        }
    }
#line 353
    __nesc_atomic_end(__nesc_atomic); }
#line 353
  return EBUSY;
}

# 247 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
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

# 958 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ReceiveP.nc"
static void CC2420ReceiveP__waitForNextPacket(void )
#line 958
{
  /* atomic removed: atomic calls only */
#line 959
  {
    if (CC2420ReceiveP__m_state == CC2420ReceiveP__S_STOPPED) {
        CC2420ReceiveP__SpiResource__release();
        return;
      }




    CC2420ReceiveP__receivingPacket = FALSE;

    if ((CC2420ReceiveP__m_missed_packets && CC2420ReceiveP__FIFO__get()) || !CC2420ReceiveP__FIFOP__get()) {

        if (CC2420ReceiveP__m_missed_packets) {
            CC2420ReceiveP__m_missed_packets--;
          }

        CC2420ReceiveP__beginReceive();
      }
    else {




        CC2420ReceiveP__m_state = CC2420ReceiveP__S_STARTED;



        CC2420ReceiveP__m_missed_packets = 0;
        CC2420ReceiveP__SpiResource__release();
      }
  }
}

#line 895
static void CC2420ReceiveP__beginReceive(void )
#line 895
{

  CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_LENGTH;
  /* atomic removed: atomic calls only */
  CC2420ReceiveP__receivingPacket = TRUE;



  if (CC2420ReceiveP__SpiResource__isOwner()) {
      CC2420ReceiveP__receive();
    }
  else {
#line 906
    if (CC2420ReceiveP__SpiResource__immediateRequest() == SUCCESS) {
        CC2420ReceiveP__receive();
      }
    else {
        CC2420ReceiveP__SpiResource__request();
      }
    }
}

#line 935
static void CC2420ReceiveP__receive(void )
#line 935
{


  CC2420ReceiveP__CSN__clr();






  CC2420ReceiveP__RXFIFO__beginRead((uint8_t *)CC2420ReceiveP__rxmpdu_ptr, 1);
}

# 49 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/includes/mac_func.h"
static uint8_t get_fc2_dest_addr(uint8_t frame_control)
{
  switch (frame_control & 0xC) 
    {
      case 0x4: return 1;
      break;
      case 0x8: return 2;
      break;
      case 0xC: return 3;
      break;
      default: 
        return 0;
      break;
    }
}

# 208 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420TransmitP.nc"
static void CC2420TransmitP__attemptSend(void )
#line 208
{





  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 214
    {


      CC2420TransmitP__CSN__clr();

      CC2420TransmitP__STXON__strobe();

      CC2420TransmitP__CSN__set();

      CC2420TransmitP__releaseSpiResource();
    }
#line 224
    __nesc_atomic_end(__nesc_atomic); }


  CC2420TransmitP__signalDone(SUCCESS);
}

# 461 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ControlP.nc"
static void CC2420ControlP__writeFsctrl(void )
#line 461
{
  uint8_t channel;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 464
    {
      channel = CC2420ControlP__m_channel;
    }
#line 466
    __nesc_atomic_end(__nesc_atomic); }

  CC2420ControlP__FSCTRL__write((1 << CC2420_FSCTRL_LOCK_THR) | (((
  channel - 11) * 5 + 357) << CC2420_FSCTRL_FREQ));
}








static void CC2420ControlP__writeMdmctrl0(void )
#line 479
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 480
    {
      CC2420ControlP__MDMCTRL0__write((((((((1 << CC2420_MDMCTRL0_RESERVED_FRAME_MODE) | (
      CC2420ControlP__addressRecognition << CC2420_MDMCTRL0_ADR_DECODE)) | (
      2 << CC2420_MDMCTRL0_CCA_HYST)) | (
      3 << CC2420_MDMCTRL0_CCA_MOD)) | (
      1 << CC2420_MDMCTRL0_AUTOCRC)) | ((
      CC2420ControlP__autoAckEnabled && CC2420ControlP__hwAutoAckDefault) << CC2420_MDMCTRL0_AUTOACK)) | (
      0 << CC2420_MDMCTRL0_AUTOACK)) | (
      2 << CC2420_MDMCTRL0_PREAMBLE_LENGTH));
    }
#line 489
    __nesc_atomic_end(__nesc_atomic); }
}







static void CC2420ControlP__writeId(void )
#line 498
{
  nxle_uint16_t id[2];

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 501
    {
      __nesc_hton_leuint16(id[0].data, CC2420ControlP__m_pan);
      __nesc_hton_leuint16(id[1].data, CC2420ControlP__m_short_addr);
    }
#line 504
    __nesc_atomic_end(__nesc_atomic); }

  CC2420ControlP__PANID__write(0, (uint8_t *)&id, sizeof id);
}

# 78 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/timerasync/TimerAsyncM.nc"
static error_t TimerAsyncM__TimerAsync__start(void )
{

  TimerAsyncM__AsyncTimer__start(10);

  return SUCCESS;
}

# 177 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/cc2420/CC2420ControlP.nc"
static error_t CC2420ControlP__CC2420Power__startVReg(void )
#line 177
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 178
    {
      if (CC2420ControlP__m_state != CC2420ControlP__S_VREG_STOPPED) {
          {
            unsigned char __nesc_temp = 
#line 180
            FAIL;

            {
#line 180
              __nesc_atomic_end(__nesc_atomic); 
#line 180
              return __nesc_temp;
            }
          }
        }
#line 182
      CC2420ControlP__m_state = CC2420ControlP__S_VREG_STARTING;
    }
#line 183
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP__VREN__set();
  CC2420ControlP__StartupTimer__start(CC2420_TIME_VREN);
  return SUCCESS;
}

# 132 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/includes/mac_func.h"
static uint16_t set_superframe_specification(uint8_t beacon_order, uint8_t superframe_order, uint8_t final_cap_slot, uint8_t battery_life_extension, uint8_t pan_coordinator, uint8_t association_permit)
{
  uint8_t sf_b1 = 0;
  uint8_t sf_b2 = 0;

#line 136
  sf_b1 = (superframe_order << 4) | (beacon_order << 0);
  sf_b2 = (((association_permit << 7) | (pan_coordinator << 6)) | (
  battery_life_extension << 4)) | (final_cap_slot << 0);
  return (sf_b2 << 8) | (sf_b1 << 0);
}

# 4656 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static void MacP__init_csma_ca(bool slotted)
{




  MacP__csma_delay = 0;

  if (slotted == 0) 
    {
      MacP__NB = 0;
      MacP__BE = MacP__mac_PIB.macMinBE;
    }
  else 
    {
      MacP__NB = 0;
      MacP__CW = 2;

      MacP__csma_cca_backoff_boundary = 0;
      MacP__csma_locate_backoff_boundary = 0;
    }

  return;
}

# 68 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/includes/mac_func.h"
static uint8_t get_fc2_source_addr(uint8_t frame_control)
{
  switch (frame_control & 0xC0) 
    {
      case 0x40: return 1;
      break;
      case 0x80: return 2;
      break;
      case 0xC0: return 3;
      break;
      default: 
        return 0;
      break;
    }
}

# 3351 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/net/zigbee/ieee802154/mac/MacP.nc"
static void MacP__build_ack(uint8_t sequence, uint8_t frame_pending)
{
  uint16_t frame_control;

  /* atomic removed: atomic calls only */
#line 3354
  {
    MacP__mac_ack_ptr->length = 5;


    frame_control = set_frame_control(2, 0, frame_pending, 0, 0, 0, 0);
    MacP__mac_ack_ptr->frame_control1 = (uint8_t )frame_control;
    MacP__mac_ack_ptr->frame_control2 = (uint8_t )(frame_control >> 8);

    MacP__mac_ack_ptr->seq_num = sequence;

    MacP__PD_DATA__request(MacP__mac_ack_ptr->length, (uint8_t *)MacP__mac_ack_ptr);
  }
}

#line 2978
static void MacP__create_data_request_cmd(void )
{
  /* atomic removed: atomic calls only */

  {

    source_long *source_long_ptr;


    MPDU *frame_pkt;

    uint16_t frame_control;

    if (MacP__send_buffer_msg_in == 3) {
      MacP__send_buffer_msg_in = 0;
      }
    frame_pkt = (MPDU *)&MacP__send_buffer[MacP__send_buffer_msg_in];


    source_long_ptr = (source_long *)&MacP__send_buffer[MacP__send_buffer_msg_in].data[0];



    source_long_ptr = (source_long *)&frame_pkt->data[0];


    frame_pkt->length = 16;


    frame_control = set_frame_control(3, 0, 0, 1, 1, 0, 3);
    frame_pkt->frame_control1 = (uint8_t )frame_control;
    frame_pkt->frame_control2 = (uint8_t )(frame_control >> 8);

    frame_pkt->seq_num = MacP__mac_PIB.macDSN;

    MacP__mac_PIB.macDSN++;


    MacP__send_buffer[MacP__send_buffer_msg_in].retransmission = 1;
    MacP__send_buffer[MacP__send_buffer_msg_in].indirect = 0;


    source_long_ptr->source_PAN_identifier = MacP__mac_PIB.macPANId;

    source_long_ptr->source_address0 = MacP__aExtendedAddress0;
    source_long_ptr->source_address1 = MacP__aExtendedAddress1;


    frame_pkt->data[10] = CMD_DATA_REQUEST;


    MacP__send_buffer_count++;
    MacP__send_buffer_msg_in++;

    MacP__send_frame_csma__postTask();
  }


  return;
}

#line 2237
static void MacP__process_coordinator_realignment(MPDU *pdu)
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 2240
    {
      cmd_coord_realignment *cmd_realignment = 0;

      dest_long *dest_long_ptr = 0;
      source_short *source_short_ptr = 0;

      cmd_realignment = (cmd_coord_realignment *)&pdu->data[10 + 4];


      dest_long_ptr = (dest_long *)&pdu->data[0];
      source_short_ptr = (source_short *)&pdu->data[10];

      MacP__mac_PIB.macCoordShortAddress = (cmd_realignment->coordinator_short_address0 << 8) | cmd_realignment->coordinator_short_address0;
      MacP__mac_PIB.macShortAddress = cmd_realignment->short_address;
    }
#line 2254
    __nesc_atomic_end(__nesc_atomic); }




  return;
}

#line 4809
static error_t MacP__remove_gts_entry(uint16_t DevAddressType)
{
  uint8_t r_lenght = 0;

  int i;

  /* atomic removed: atomic calls only */
#line 4815
  {
    for (i = 0; i < 7; i++) 
      {
        if (MacP__GTS_db[i].DevAddressType == DevAddressType) 
          {

            r_lenght = MacP__GTS_db[i].length;


            MacP__GTS_db[i].gts_id = 0x00;
            MacP__GTS_db[i].starting_slot = 0x00;
            MacP__GTS_db[i].length = 0x00;
            MacP__GTS_db[i].direction = 0x00;
            MacP__GTS_db[i].DevAddressType = 0x0000;
            MacP__GTS_db[i].expiration = 0x00;


            MacP__GTS_startslot = MacP__GTS_startslot + r_lenght;
            MacP__GTS_descriptor_count--;
            MacP__final_CAP_slot = MacP__final_CAP_slot + r_lenght;
          }

        if (r_lenght > 0) 
          {
            if (MacP__GTS_db[i].gts_id != 0x00 && MacP__GTS_db[i].DevAddressType != 0x0000) 
              {
                MacP__GTS_db[i - r_lenght].gts_id = MacP__GTS_db[i].gts_id;
                MacP__GTS_db[i - r_lenght].starting_slot = i - r_lenght;
                MacP__GTS_db[i - r_lenght].length = MacP__GTS_db[i].length;
                MacP__GTS_db[i - r_lenght].direction = MacP__GTS_db[i].direction;
                MacP__GTS_db[i - r_lenght].DevAddressType = MacP__GTS_db[i].DevAddressType;
                MacP__GTS_db[i - r_lenght].expiration = MacP__GTS_db[i].expiration;


                MacP__GTS_db[i].gts_id = 0x00;
                MacP__GTS_db[i].starting_slot = 0x00;
                MacP__GTS_db[i].length = 0x00;
                MacP__GTS_db[i].direction = 0x00;
                MacP__GTS_db[i].DevAddressType = 0x0000;
                MacP__GTS_db[i].expiration = 0x00;
              }
          }
      }
  }


  return SUCCESS;
}

#line 4707
static uint8_t MacP__check_csma_ca_send_conditions(uint8_t frame_length, uint8_t frame_control1)
{
  uint8_t ifs_symbols;
  uint32_t frame_tx_time;
  uint32_t remaining_gts_duration;


  ifs_symbols = MacP__calculate_ifs(frame_length);





  if (get_fc1_ack_request(frame_control1) == 1) {
    frame_tx_time = frame_length + 5 + 12 + ifs_symbols;
    }
  else {
#line 4723
    frame_tx_time = frame_length + ifs_symbols;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 4725
    remaining_gts_duration = MacP__time_slot - MacP__TimerAsync__get_current_number_backoff_on_time_slot() * 20;
#line 4725
    __nesc_atomic_end(__nesc_atomic); }



  if (frame_tx_time < remaining_gts_duration) {
    return 1;
    }
  else {
#line 4732
    return 0;
    }
}

#line 4762
static uint8_t MacP__check_gts_send_conditions(uint8_t frame_length)
{
  uint8_t ifs_symbols;
  uint32_t frame_tx_time;
  uint32_t remaining_gts_duration;


  ifs_symbols = MacP__calculate_ifs(frame_length);





  frame_tx_time = frame_length + 5 + 12 + ifs_symbols;

  remaining_gts_duration = MacP__time_slot - MacP__TimerAsync__get_current_number_backoff_on_time_slot() * 20;



  if (frame_tx_time < remaining_gts_duration) {
    return 1;
    }
  else {
#line 4784
    return 0;
    }
}

# 155 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/printf/PrintfP.nc"
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

# 155 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/AMQueueImplP.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(uint8_t last, message_t * msg, error_t err)
#line 155
{
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg = (void *)0;
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend();
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(last, msg, err);
}

# 85 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__configure(uint8_t id)
#line 85
{
  msp430_uart_union_config_t *config = /*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__getConfig(id);

#line 87
  /*Msp430Uart1P.UartP*/Msp430UartP__0__m_byte_time = config->uartConfig.ubr / 2;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__setModeUart(config);
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__enableIntr();
}

# 251 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
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

# 177 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id)
#line 177
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 178
    {
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId == id && /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) {
          unsigned char __nesc_temp = 
#line 179
          TRUE;

          {
#line 179
            __nesc_atomic_end(__nesc_atomic); 
#line 179
            return __nesc_temp;
          }
        }
      else 
#line 180
        {
          unsigned char __nesc_temp = 
#line 180
          FALSE;

          {
#line 180
            __nesc_atomic_end(__nesc_atomic); 
#line 180
            return __nesc_temp;
          }
        }
    }
#line 183
    __nesc_atomic_end(__nesc_atomic); }
}

# 347 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialP.nc"
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

# 86 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/HdlcTranslateC.nc"
static error_t HdlcTranslateC__SerialFrameComm__putDelimiter(void )
#line 86
{
  HdlcTranslateC__state.sendEscape = 0;
  HdlcTranslateC__m_data = HDLC_FLAG_BYTE;
  return HdlcTranslateC__UartStream__send(&HdlcTranslateC__m_data, 1);
}

# 147 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/Msp430UartP.nc"
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

# 96 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
__attribute((wakeup)) __attribute((interrupt(6)))  void sig_UART1RX_VECTOR(void )
#line 96
{
  uint8_t temp = U1RXBUF;

#line 98
  HplMsp430Usart1P__Interrupts__rxDone(temp);
}

# 153 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void )
#line 153
{
  /* atomic removed: atomic calls only */
#line 154
  {
    if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) 
      {
        unsigned char __nesc_temp = 
#line 156
        FALSE;

#line 156
        return __nesc_temp;
      }
  }
#line 158
  return TRUE;
}

# 402 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialP.nc"
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

# 80 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/crc.h"
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

# 285 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/SerialDispatcherP.nc"
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

# 166 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void )
#line 166
{
  /* atomic removed: atomic calls only */
#line 167
  {
    if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state != /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 169
        /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;

#line 169
        return __nesc_temp;
      }
#line 170
    {
      unsigned char __nesc_temp = 
#line 170
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId;

#line 170
      return __nesc_temp;
    }
  }
}

# 101 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
__attribute((wakeup)) __attribute((interrupt(4)))  void sig_UART1TX_VECTOR(void )
#line 101
{
  HplMsp430Usart1P__Interrupts__txDone();
}

# 104 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/serial/HdlcTranslateC.nc"
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

# 165 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/lib/printf/PrintfP.nc"
__attribute((noinline))   int putchar(int c)
#line 165
{
#line 177
  if (PrintfP__state == PrintfP__S_STARTED && PrintfP__Queue__size() >= 250 / 2) {
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

# 53 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 96 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
__attribute((wakeup)) __attribute((interrupt(18)))  void sig_UART0RX_VECTOR(void )
#line 96
{
  uint8_t temp = U0RXBUF;

#line 98
  HplMsp430Usart0P__Interrupts__rxDone(temp);
}

# 153 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/system/ArbiterP.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse(void )
#line 153
{
  /* atomic removed: atomic calls only */
#line 154
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED) 
      {
        unsigned char __nesc_temp = 
#line 156
        FALSE;

#line 156
        return __nesc_temp;
      }
  }
#line 158
  return TRUE;
}






static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId(void )
#line 166
{
  /* atomic removed: atomic calls only */
#line 167
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state != /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 169
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__NO_RES;

#line 169
        return __nesc_temp;
      }
#line 170
    {
      unsigned char __nesc_temp = 
#line 170
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId;

#line 170
      return __nesc_temp;
    }
  }
}

# 101 "/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
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

