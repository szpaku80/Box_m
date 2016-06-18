#ifndef _DATAPROCESSING_H_
#define _DATAPROCESSING_H_

void processMessage (char *message, unsigned int length, char dataErr);
void processInfo (void);
void updateCalFactor (void);
void updateHistoryData (void);
float getISIGfromRAW (long raw, char adjValue);

#endif