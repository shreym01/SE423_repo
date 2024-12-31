#define MAXDSPCON 8
#warn Dan Block changed from 256 to 100000 for picture
#define MAXDSPPKTLEN 100000
#define MAXDSPPACKETS 32

// One for each TCP dsp connection
typedef struct dspcon {
  int con_num;
  int data_in_p;
  int data_out_p;
  char data_in[MAXDSPPKTLEN * MAXDSPPACKETS];
} dspcon_t;

// One for all UDP packets
typedef struct udpcons {
  int numcons;
} udocons_t;

typedef struct dspcomms {
   
} dspcomms_t;
