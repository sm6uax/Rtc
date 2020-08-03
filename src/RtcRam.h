#ifndef __RTCRAM_H__
#define __RTCRAM_H__


class RtcRAM
{
    public:
     
       void writeRTC(byte addr, byte *values, byte nBytes);
       void readRTC(byte addr, byte *values, byte nBytes);
    protected:
        int8_t  _result;  

};

#endif // __RTCRAME_H__