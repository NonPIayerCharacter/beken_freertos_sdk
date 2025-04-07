#ifndef _IPCHKSUM_PUB_H_
#define _IPCHKSUM_PUB_H_

#define IPCHKSUM_DEV_NAME           "ipchksum"

extern void ipchksum_init(void);
extern void ipchksum_exit(void);
extern void ipchksum_isr(void);

#endif // _IPCHKSUM_PUB_H_
// eof

