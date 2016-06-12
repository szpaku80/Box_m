#ifndef _BLECOMMS_H_
#define _BLECOMMS_H_

char initHM10 (void);
void receiveBLEMessage (char *message, unsigned int messageLen);

#endif