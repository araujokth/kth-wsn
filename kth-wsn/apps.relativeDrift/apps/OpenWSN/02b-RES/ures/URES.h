#ifndef RESERVATION_H
#define RESERVATION_H

typedef struct ongoingRes_t {
   uint8_t         todo;   //possibilities listed below
   cellType_t      type;   //OFF,RXDATA,TXDATA,RXRES,TXRES,ADV
   shortnodeid_t   neighbor;
   uint8_t         retries;
   bool            randomCell;
   slotOffset_t    slotOffset;
   channelOffset_t channelOffset;
   bool            role;
   error_t         outcome;
   timervalue_t    timestamp;
   message_t*      pkt;
} ongoingRes_t;

typedef struct debugOngoingRes_t {
   uint8_t         row;
   ongoingRes_t    ongoingRes;
} debugOngoingRes_t;

enum {//todo values
   UNUSED                         =0,
   LOCAL_RES                      =1,
   DISTANT_RES                    =2,
   WAIT_FOR_SENT                  =3,
   WAIT_FOR_RESPONSE              =4,
   DISTANT_FAILED_RESTART_LOCAL   =5,
   INFORM_REQUESTER               =6
};

//add values
#define ADD TRUE
#define REMOVE FALSE

//role values
#define REQUESTER TRUE
#define REPLIER FALSE

//randomCell values
#define PICKRANDOM TRUE
#define NORANDOM FALSE

#endif
