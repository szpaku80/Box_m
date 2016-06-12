#include "ioCC1110.h"
#include "globals.h"
#include "timingController.h"

void initTimingTable (void)
{
  char i,j,k;
 
  for (i=0;i<3;i++) for (j=0;j<8;j++) {
    missesTable[i][j] = 0;
    for (k=0;k<2;k++) {
      timingTable     [i][j][k] = 0;
      fiveMinAdjTable [i][j] = 0;
    }
  }
}

void timingSanityCheck (void)
{
  char i,j;
  
  // Remove non-valid timings
  for (i=0;i<3;i++) for (j=0;j<8;j++) {
    if ( (timingTable[i][j][0] < ((2*60 + 0) * 10)) ||
        (timingTable[i][j][0] > ((7*60 + 0) * 10)) ) { 
      timingTable[i][j][0] = 0;
    }
    if ( (timingTable[i][j][1] < ((0*60 + 8) * 10)) ||
        (timingTable[i][j][1] > ((0*60 + 25) * 10)) ) { 
      timingTable[i][j][1] = 0;
    }
  }
  
  // Adjust timings to the closest second
  for (i=0;i<3;i++) for (j=0;j<8;j++) {
    if ((timingTable[i][j][0] % 10) > 5) {
      timingTable[i][j][0] = timingTable[i][j][0] + (10 -
        (timingTable[i][j][0] % 10));
    } else {
      timingTable[i][j][0] = timingTable[i][j][0] -
        (timingTable[i][j][0] % 10);
    }
    if ((timingTable[i][j][1] % 10) > 5) {
      timingTable[i][j][1] = timingTable[i][j][1] + (10 -
        (timingTable[i][j][1] % 10));
    } else {
      timingTable[i][j][1] = timingTable[i][j][1] -
        (timingTable[i][j][1] % 10);
    }
  }
}

unsigned int getTimeForNumSeq ( char seqNumQuery )
{
  char id, seqNum, subSeqNum;
  
  seqNum           = (seqNumQuery >> 4) & 0x07;
  subSeqNum        =  seqNumQuery & 0x01;
  
  if      ((lastMinilinkID[0] == minilinkID1[0]) &&
           (lastMinilinkID[1] == minilinkID1[1]) &&
           (lastMinilinkID[2] == minilinkID1[2])) id = 0;
  else if ((lastMinilinkID[0] == minilinkID2[0]) &&
           (lastMinilinkID[1] == minilinkID2[1]) &&
           (lastMinilinkID[2] == minilinkID2[2])) id = 1;
  else if ((lastMinilinkID[0] == minilinkID3[0]) &&
           (lastMinilinkID[1] == minilinkID3[1]) &&
           (lastMinilinkID[2] == minilinkID3[2])) id = 2;
  else id = 3;
  
  if (id < 3) {
    return (timingTable[id][seqNum][subSeqNum]);
  } else {
    return (0);
  }
}

void updateTimingTable ( char newSeqNum, unsigned char lastSeqNum, unsigned int time )
{
  char id, seqNum, subSeqNum, seqNumCurrent, subSeqNumCurrent;
  
  seqNum           = (lastSeqNum >> 4) & 0x07;
  subSeqNum        =  lastSeqNum & 0x01;
  seqNumCurrent    = ( newSeqNum >> 4) & 0x07;
  subSeqNumCurrent =   newSeqNum & 0x01;
  
  if      ((lastMinilinkID[0] == minilinkID1[0]) &&
           (lastMinilinkID[1] == minilinkID1[1]) &&
           (lastMinilinkID[2] == minilinkID1[2])) id = 0;
  else if ((lastMinilinkID[0] == minilinkID2[0]) &&
           (lastMinilinkID[1] == minilinkID2[1]) &&
           (lastMinilinkID[2] == minilinkID2[2])) id = 1;
  else if ((lastMinilinkID[0] == minilinkID3[0]) &&
           (lastMinilinkID[1] == minilinkID3[1]) &&
           (lastMinilinkID[2] == minilinkID3[2])) id = 2;
  else id = 3;
  
  if (id < 3) {
    
    // If we received two following sequence numbers...
    if ((seqNumCurrent == ((seqNum + 1) % 8)) && (seqNum < 0x71) &&
        (time < ((8*60 + 0) * 10)) ) {
          
      // ... and both are the first tranmissions ...
      if ( (subSeqNum == 0) && (subSeqNumCurrent == 0) ) {
        if (timingTable[id][seqNum][0] == 0) {
          timingTable[id][seqNum][0] = time;
        } else {
          timingTable[id][seqNum][0] += time;
          timingTable[id][seqNum][0] /= 2;
        }
      }
    
      // ... and we have the first transmission for the previous
      //   sequence number and the second for the current one ...
      else if ( (subSeqNum == 0) && (subSeqNumCurrent == 1) ) {
        
        if ((timingTable[id][seqNumCurrent][1]) != 0 ) {
          if (timingTable[id][seqNum][0] == 0) {
            timingTable[id][seqNum][0]  = (time - timingTable[id][seqNumCurrent][1]);
          } else {
            timingTable[id][seqNum][0] += (time - timingTable[id][seqNumCurrent][1]);
            timingTable[id][seqNum][0] /= 2;
          }
        }
    
        if ((timingTable[id][seqNum][0]) != 0 ) {
          if (timingTable[id][seqNumCurrent][1] == 0) {
            timingTable[id][seqNumCurrent][1]  = (time - timingTable[id][seqNum][0]);
          } else {
            timingTable[id][seqNumCurrent][1] += (time - timingTable[id][seqNum][0]);
            timingTable[id][seqNumCurrent][1] /= 2;
          }
        }
        
      }
      
      else if ( (subSeqNum == 1) && (subSeqNumCurrent == 0) ) {
        if (timingTable[id][seqNum][0] == 0) {
          timingTable[id][seqNum][0]  = time;
        } else {
          timingTable[id][seqNum][0] += time;
          timingTable[id][seqNum][0] /= 2;
        }
      }   
    } else if ((seqNumCurrent == seqNum) && (time < ((1*60 + 0) * 10))) {
      if (timingTable[id][seqNum][1] == 0) {
        timingTable[id][seqNum][1]  = time;
      } else {
        timingTable[id][seqNum][1] += time;
        timingTable[id][seqNum][1] /= 2;
      }
    }
    
    timingSanityCheck();
  
    if (timingCorrect(id)) {
      calculateFiveMinAdjustment(id);
    }
    
  }
}

char timingCorrect (char id)
{
  unsigned long timeSum;
  unsigned char i;

  timingTableCorrect[id] = 0;
  
  if (id > 2) return(0);
  
  timeSum = 0;
  for (i=0;i<8;i++) timeSum += timingTable[id][i][0];
  
  if ((timeSum > 8*((5*60 +  2) * 10)) ||
      (timeSum < 8*((4*60 + 58) * 10))) return(0);
  
  for (i=0;i<8;i++) 
    if ((timingTable[id][i][1] < ((0*60 +  5) * 10)) ||
        (timingTable[id][i][1] > ((0*60 + 25) * 10))) return(0);
  
  timingTableCorrect[id] = 1;
  
  return(1);
}

void calculateFiveMinAdjustment(char id)
{
  unsigned int tempValue;
  int maxTime, timePeriod, timeAccum;
  char i;
  
  timePeriod = 0;
  timeAccum = 0;
  maxTime = -((20*60 + 0) * 10);
  for (i=0; i<8; i++) {
    timePeriod = ((5*60 + 0) * 10)*(i+1);
    timeAccum += timingTable[id][i][0];
    fiveMinAdjTable [id][i] = timePeriod - timeAccum;
    if (fiveMinAdjTable [id][i] > maxTime) 
      maxTime = fiveMinAdjTable [id][i];
  }
  
  tempValue = fiveMinAdjTable [id][7];
  for (i=7; i>0; i--) {
    fiveMinAdjTable [id][i] = maxTime - fiveMinAdjTable [id][i-1];
  }
  fiveMinAdjTable[id][0] = maxTime - tempValue;
  
}

void recalculateTiming(void)
{
  unsigned int nextActivation, timeIncrease;
  unsigned int i;
  unsigned char nextSeqNum, undetFlag;
  
  timeCounterOn  = 0;
  timeCounterOff = 0;
  undetFlag      = 0;
  
  for (i=0; timeCounter > timeCounterOff; i++) {
    nextSeqNum = ((((((lastMinilinkSeqNum >> 4) & 0x07) + i) % 8) << 4) & 0xF0);
    nextActivation = getTimeForNumSeq (nextSeqNum);
    
    if ((nextActivation == 0) || (undetFlag == 1)) {
      undetFlag = 1;
      timeIncrease  += (5*60 + 30)*10;
      timeCounterOn  = timeIncrease - (2*60 + 30)*10;
      timeCounterOff = timeIncrease + (2*60 + 30)*10;
    } else {
      timeIncrease  += nextActivation;
      timeCounterOn  = timeIncrease - (0*60 + (1+i))*10;
      nextActivation = getTimeForNumSeq (nextSeqNum | 0x01);
      if (nextActivation == 0) {
        timeCounterOff = timeIncrease + (0*60 + 30)*10;
      } else {
        timeCounterOff = timeIncrease + nextActivation + (0*60 + (1+i))*10;
      }
    }
  }
  
}
