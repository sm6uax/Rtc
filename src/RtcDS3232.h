

#ifndef __RTCDS3232_H__
#define __RTCDS3232_H__

#include <Arduino.h>

#include "RtcDateTime.h"
#include "RtcTemperature.h"

#include "RtcUtility.h"


//I2C Slave Address  
const uint8_t   DS3232_ADDRESS = 0x68;

//DS3232 Register Addresses
const uint8_t DS3232_REG_TIMEDATE  = 0x00;
const uint8_t DS3232_REG_ALARMONE  = 0x07;
const uint8_t DS3232_REG_ALARMTWO  = 0x0B;
                                         
const uint8_t DS3232_REG_CONTROL   = 0x0E;
const uint8_t DS3232_REG_STATUS    = 0x0F;
const uint8_t DS3232_REG_AGING     = 0x10;
                                         
const uint8_t DS3232_REG_TEMP      = 0x11;

//DS3232 Register Data Size if not just 1
const uint8_t DS3232_REG_TIMEDATE_SIZE = 7;
const uint8_t DS3232_REG_ALARMONE_SIZE = 4;
const uint8_t DS3232_REG_ALARMTWO_SIZE = 3;

const uint8_t DS3232_REG_TEMP_SIZE = 2;

// DS3232 Control Register Bits
const uint8_t DS3232_A1IE  = 0;
const uint8_t DS3232_A2IE  = 1;
const uint8_t DS3232_INTCN = 2;
const uint8_t DS3232_RS1   = 3;
const uint8_t DS3232_RS2   = 4;
const uint8_t DS3232_CONV  = 5;
const uint8_t DS3232_BBSQW = 6;
const uint8_t DS3232_EOSC  = 7;
const uint8_t DS3232_AIEMASK = (_BV(DS3232_A1IE) | _BV(DS3232_A2IE));
const uint8_t DS3232_RSMASK = (_BV(DS3232_RS1) | _BV(DS3232_RS2));

// DS3232 Status Register Bits
const uint8_t DS3232_A1F      = 0;
const uint8_t DS3232_A2F      = 1;
const uint8_t DS3232_BSY      = 2;
const uint8_t DS3232_EN32KHZ  = 3;
const uint8_t DS3232_OSF      = 7;
const uint8_t DS3232_AIFMASK = (_BV(DS3232_A1F) | _BV(DS3232_A2F));


// seconds accuracy
enum DS3232AlarmOneControl
{
    // bit order:  A1M4  DY/DT  A1M3  A1M2  A1M1
    DS3232AlarmOneControl_HoursMinutesSecondsDayOfMonthMatch = 0x00,
    DS3232AlarmOneControl_OncePerSecond = 0x17,
    DS3232AlarmOneControl_SecondsMatch = 0x16,
    DS3232AlarmOneControl_MinutesSecondsMatch = 0x14,
    DS3232AlarmOneControl_HoursMinutesSecondsMatch = 0x10,
    DS3232AlarmOneControl_HoursMinutesSecondsDayOfWeekMatch = 0x08,
};
template<class T_WIRE_METHOD> class DS3232RAM
{
public:
    DS3232RAM(T_WIRE_METHOD& wire) :
        _address(DS3232_ADDRESS ),
        _wire(wire),
        _lastError(0)
    {
    }

    void Begin()
    {
        _wire.begin();
    }

    uint8_t LastError()
    {
        return _lastError;
    }

    void SetMemory(uint16_t memoryAddress, uint8_t value)
    {
        SetMemory(memoryAddress, &value, 1);
    }

    uint8_t GetMemory(uint16_t memoryAddress)
    {
        uint8_t value;

        GetMemory(memoryAddress, &value, 1);
       
        return value;
    }

    // note: this method will write within a single page of eeprom.
    // Pages are 32 bytes (5 bits), so writing past a page boundary will
    // just wrap within the page of the starting memory address.  
    // 
    // xxxppppp pppaaaaa => p = page #, a = address within the page
    //
    // NOTE: hardware WIRE libraries often have a limit of a 32 byte send buffer.  The 
    // effect of this is that only 30 bytes can be sent, 2 bytes for the address to write to,
    // and then 30 bytes of the actual data. 
    uint8_t SetMemory(uint16_t memoryAddress, const uint8_t* pValue, uint8_t countBytes)
    {
        uint8_t countWritten = 0;
        _wire.beginTransmission(_address);
        _wire.write(memoryAddress);

        while (countBytes > 0)
        {
            _wire.write(*pValue++);
            delay(10); // per spec, memory writes

            countBytes--;
            countWritten++;
        }

        _lastError = _wire.endTransmission();
        
        return countWritten;
    }

    // reading data does not wrap within pages, but due to only using
    // 12 (32K) or 13 (64K) bits are used, they will wrap within the memory limits
    // of the installed EEPROM
    //
    // NOTE: hardware WIRE libraries may have a limit of a 32 byte recieve buffer.  The 
    // effect of this is that only 32 bytes can be read at one time.
    uint8_t GetMemory(uint16_t memoryAddress, uint8_t* pValue, uint8_t countBytes)
    {
        // set address to read from
        _wire.beginTransmission(_address);
        _wire.write(memoryAddress);
        _lastError = _wire.endTransmission();

        if (_lastError != 0)
        {
            return 0;
        }

        // read the data
        uint8_t countRead = 0;

        countRead = _wire.requestFrom(_address, countBytes);
        countBytes = countRead;

        while (countBytes-- > 0)
        {
            *pValue++ = _wire.read();
        }

        return countRead;
    }

private:
    const uint8_t _address;
    
    T_WIRE_METHOD& _wire;
    uint8_t _lastError;
    

};
class DS3232AlarmOne
{
public:
    DS3232AlarmOne( uint8_t dayOf,
            uint8_t hour,
            uint8_t minute,
            uint8_t second,
            DS3232AlarmOneControl controlFlags) :
        _flags(controlFlags),
        _dayOf(dayOf),
        _hour(hour),
        _minute(minute),
        _second(second)
    {
    }

    uint8_t DayOf() const
    {
        return _dayOf;
    }

    uint8_t Hour() const
    {
        return _hour;
    }

    uint8_t Minute() const
    {
        return _minute;
    }

    uint8_t Second() const
    {
        return _second;
    }

    DS3232AlarmOneControl ControlFlags() const
    {
        return _flags;
    }

    bool operator == (const DS3232AlarmOne& other) const
    {
        return (_dayOf == other._dayOf &&
                _hour == other._hour &&
                _minute == other._minute &&
                _second == other._second &&
                _flags == other._flags);
    }

    bool operator != (const DS3232AlarmOne& other) const
    {
        return !(*this == other);
    }

protected:
    DS3232AlarmOneControl _flags;

    uint8_t _dayOf;
    uint8_t _hour;
    uint8_t _minute;
    uint8_t _second;  
};

// minutes accuracy
enum DS3232AlarmTwoControl
{
    // bit order:  A2M4  DY/DT  A2M3  A2M2
    DS3232AlarmTwoControl_HoursMinutesDayOfMonthMatch = 0x00,
    DS3232AlarmTwoControl_OncePerMinute = 0x0b,
    DS3232AlarmTwoControl_MinutesMatch = 0x0a,
    DS3232AlarmTwoControl_HoursMinutesMatch = 0x08,
    DS3232AlarmTwoControl_HoursMinutesDayOfWeekMatch = 0x04,
};

class DS3232AlarmTwo
{
public:
    DS3232AlarmTwo( uint8_t dayOf,
            uint8_t hour,
            uint8_t minute,
            DS3232AlarmTwoControl controlFlags) :
        _flags(controlFlags),
        _dayOf(dayOf),
        _hour(hour),
        _minute(minute)
    {
    }

    uint8_t DayOf() const
    {
        return _dayOf;
    }

    uint8_t Hour() const
    {
        return _hour;
    }

    uint8_t Minute() const
    {
        return _minute;
    }

    DS3232AlarmTwoControl ControlFlags() const
    {
        return _flags;
    }

    bool operator == (const DS3232AlarmTwo& other) const
    {
        return (_dayOf == other._dayOf &&
                _hour == other._hour &&
                _minute == other._minute &&
                _flags == other._flags);
    }

    bool operator != (const DS3232AlarmTwo& other) const
    {
        return !(*this == other);
    }

protected:
    DS3232AlarmTwoControl _flags;

    uint8_t _dayOf;
    uint8_t _hour;
    uint8_t _minute;
};


enum DS3232SquareWaveClock
{
    DS3232SquareWaveClock_1Hz  = 0b00000000,
    DS3232SquareWaveClock_1kHz = 0b00001000,
    DS3232SquareWaveClock_4kHz = 0b00010000,
    DS3232SquareWaveClock_8kHz = 0b00011000,
};

enum DS3232SquareWavePinMode
{
    DS3232SquareWavePin_ModeNone,
    DS3232SquareWavePin_ModeAlarmOne,
    DS3232SquareWavePin_ModeAlarmTwo,
    // note:  the same as DS3232SquareWavePin_ModeAlarmOne | DS3232SquareWavePin_ModeAlarmTwo
    DS3232SquareWavePin_ModeAlarmBoth, 
    DS3232SquareWavePin_ModeClock
};

enum DS3232AlarmFlag
{
    DS3232AlarmFlag_Alarm1 = 0x01,
    DS3232AlarmFlag_Alarm2 = 0x02,
    DS3232AlarmFlag_AlarmBoth = 0x03,
};

template<class T_WIRE_METHOD> class RtcDS3232
{
public:
    RtcDS3232(T_WIRE_METHOD& wire) :
        _wire(wire),
        _lastError(0)
    {
    }

    void Begin()
    {
        _wire.begin();
    }

    void Begin(int sda, int scl)
    {
        _wire.begin(sda, scl);
    }

    uint8_t LastError()
    {
        return _lastError;
    }

    bool IsDateTimeValid()
    {
        uint8_t status = getReg(DS3232_REG_STATUS);
        return !(status & _BV(DS3232_OSF));
    }

    bool GetIsRunning()
    {
        uint8_t creg = getReg(DS3232_REG_CONTROL);
        return !(creg & _BV(DS3232_EOSC));
    }

    void SetIsRunning(bool isRunning)
    {
        uint8_t creg = getReg(DS3232_REG_CONTROL);
        if (isRunning)
        {
            creg &= ~_BV(DS3232_EOSC);
        }
        else
        {
            creg |= _BV(DS3232_EOSC);
        }
        setReg(DS3232_REG_CONTROL, creg);
    }

    void SetDateTime(const RtcDateTime& dt)
    {
        // clear the invalid flag
        uint8_t status = getReg(DS3232_REG_STATUS);
        status &= ~_BV(DS3232_OSF); // clear the flag
        setReg(DS3232_REG_STATUS, status);

        // set the date time
        _wire.beginTransmission(DS3232_ADDRESS);
        _wire.write(DS3232_REG_TIMEDATE);

        _wire.write(Uint8ToBcd(dt.Second()));
        _wire.write(Uint8ToBcd(dt.Minute()));
        _wire.write(Uint8ToBcd(dt.Hour())); // 24 hour mode only

        uint8_t year = dt.Year() - 2000;
        uint8_t centuryFlag = 0;

        if (year >= 100)
        {
            year -= 100;
            centuryFlag = _BV(7);
        }

        // RTC Hardware Day of Week is 1-7, 1 = Monday
        // convert our Day of Week to Rtc Day of Week
        uint8_t rtcDow = RtcDateTime::ConvertDowToRtc(dt.DayOfWeek());

        _wire.write(Uint8ToBcd(rtcDow));

        _wire.write(Uint8ToBcd(dt.Day()));
        _wire.write(Uint8ToBcd(dt.Month()) | centuryFlag);
        _wire.write(Uint8ToBcd(year));

        _lastError = _wire.endTransmission();
    }
    
    RtcDateTime GetDateTime()
    {
        _wire.beginTransmission(DS3232_ADDRESS);
        _wire.write(DS3232_REG_TIMEDATE);
        _lastError = _wire.endTransmission();
        if (_lastError != 0)
        {
            return RtcDateTime(0);
        }

        _wire.requestFrom(DS3232_ADDRESS, DS3232_REG_TIMEDATE_SIZE);
        uint8_t second = BcdToUint8(_wire.read() & 0x7F);
        uint8_t minute = BcdToUint8(_wire.read());
        uint8_t hour = BcdToBin24Hour(_wire.read());

        _wire.read();  // throwing away day of week as we calculate it

        uint8_t dayOfMonth = BcdToUint8(_wire.read());
        uint8_t monthRaw = _wire.read();
        uint16_t year = BcdToUint8(_wire.read()) + 2000;

        if (monthRaw & _BV(7)) // century wrap flag
        {
            year += 100;
        }
        uint8_t month = BcdToUint8(monthRaw & 0x7f);


        return RtcDateTime(year, month, dayOfMonth, hour, minute, second);
    }

    RtcTemperature GetTemperature()
    {
        _wire.beginTransmission(DS3232_ADDRESS);
        _wire.write(DS3232_REG_TEMP);
        _lastError = _wire.endTransmission();
        if (_lastError != 0)
        {
            return RtcTemperature(0);
        }

        // Temperature is represented as a 10-bit code with a resolution
        // of 1/4th °C and is accessable as a signed 16-bit integer at
        // locations 11h and 12h.
        //
        //       |         r11h          | DP |         r12h         |
        // Bit:   15 14 13 12 11 10  9  8   .  7  6  5  4  3  2  1  0  -1 -2
        //         s  i  i  i  i  i  i  i   .  f  f  0  0  0  0  0  0
        //
        // As it takes (8) right-shifts to register the decimal point (DP) to
        // the right of the 0th bit, the overall word scaling equals 256.
        //
        // For example, at +/- 25.25°C, concatenated registers <r11h:r12h> =
        // 256 * (+/- 25+(1/4)) = +/- 6464, or 1940h / E6C0h.

        _wire.requestFrom(DS3232_ADDRESS, DS3232_REG_TEMP_SIZE);
        int8_t  r11h = _wire.read();                  // MS byte, signed temperature
        return RtcTemperature( r11h, _wire.read() );  // LS byte is r12h
    }

    void Enable32kHzPin(bool enable)
    {
        uint8_t sreg = getReg(DS3232_REG_STATUS);

        if (enable == true)
        {
            sreg |= _BV(DS3232_EN32KHZ);
        }
        else
        {
            sreg &= ~_BV(DS3232_EN32KHZ);
        }

        setReg(DS3232_REG_STATUS, sreg);
    }

    void SetSquareWavePin(DS3232SquareWavePinMode pinMode, bool enableWhileInBatteryBackup = true)
    {
        uint8_t creg = getReg(DS3232_REG_CONTROL);

        // clear all relevant bits to a known "off" state
        creg &= ~(DS3232_AIEMASK | _BV(DS3232_BBSQW));
        creg |= _BV(DS3232_INTCN);  // set INTCN to disables clock SQW

        if (pinMode != DS3232SquareWavePin_ModeNone)
        {
            if (pinMode == DS3232SquareWavePin_ModeClock)
            {
                creg &= ~_BV(DS3232_INTCN); // clear INTCN to enable clock SQW 
            }
            else
            {
                if (pinMode & DS3232SquareWavePin_ModeAlarmOne)
                {
                    creg |= _BV(DS3232_A1IE);
                }
                if (pinMode & DS3232SquareWavePin_ModeAlarmTwo)
                {
                    creg |= _BV(DS3232_A2IE);
                }
            }

            if (enableWhileInBatteryBackup)
            {
                creg |= _BV(DS3232_BBSQW); // set enable int/sqw while in battery backup flag
            }
        }
        setReg(DS3232_REG_CONTROL, creg);
    }

    void SetSquareWavePinClockFrequency(DS3232SquareWaveClock freq)
    {
        uint8_t creg = getReg(DS3232_REG_CONTROL);

        creg &= ~DS3232_RSMASK; // Set to 0
        creg |= (freq & DS3232_RSMASK); // Set freq bits

        setReg(DS3232_REG_CONTROL, creg);
    }


    void SetAlarmOne(const DS3232AlarmOne& alarm)
    {
        _wire.beginTransmission(DS3232_ADDRESS);
        _wire.write(DS3232_REG_ALARMONE);

        _wire.write(Uint8ToBcd(alarm.Second()) | ((alarm.ControlFlags() & 0x01) << 7));
        _wire.write(Uint8ToBcd(alarm.Minute()) | ((alarm.ControlFlags() & 0x02) << 6));
        _wire.write(Uint8ToBcd(alarm.Hour()) | ((alarm.ControlFlags() & 0x04) << 5)); // 24 hour mode only

        uint8_t rtcDow = alarm.DayOf();
        if (alarm.ControlFlags() == DS3232AlarmOneControl_HoursMinutesSecondsDayOfWeekMatch)
        {
            rtcDow = RtcDateTime::ConvertDowToRtc(rtcDow);
        }

        _wire.write(Uint8ToBcd(rtcDow) | ((alarm.ControlFlags() & 0x18) << 3));

        _lastError = _wire.endTransmission();
    }

    void SetAlarmTwo(const DS3232AlarmTwo& alarm)
    {
        _wire.beginTransmission(DS3232_ADDRESS);
        _wire.write(DS3232_REG_ALARMTWO);

        _wire.write(Uint8ToBcd(alarm.Minute()) | ((alarm.ControlFlags() & 0x01) << 7));
        _wire.write(Uint8ToBcd(alarm.Hour()) | ((alarm.ControlFlags() & 0x02) << 6)); // 24 hour mode only

        // convert our Day of Week to Rtc Day of Week if needed
        uint8_t rtcDow = alarm.DayOf();
        if (alarm.ControlFlags() == DS3232AlarmTwoControl_HoursMinutesDayOfWeekMatch)
        {
            rtcDow = RtcDateTime::ConvertDowToRtc(rtcDow);
        }
        
        _wire.write(Uint8ToBcd(rtcDow) | ((alarm.ControlFlags() & 0x0c) << 4));

        _lastError = _wire.endTransmission();
    }

    DS3232AlarmOne GetAlarmOne()
    {
        _wire.beginTransmission(DS3232_ADDRESS);
        _wire.write(DS3232_REG_ALARMONE);
        _lastError = _wire.endTransmission();
        if (_lastError != 0)
        {
            return DS3232AlarmOne(0, 0, 0, 0, DS3232AlarmOneControl_HoursMinutesSecondsDayOfMonthMatch);
        }

        _wire.requestFrom(DS3232_ADDRESS, DS3232_REG_ALARMONE_SIZE);

        uint8_t raw = _wire.read();
        uint8_t flags = (raw & 0x80) >> 7;
        uint8_t second = BcdToUint8(raw & 0x7F);

        raw = _wire.read();
        flags |= (raw & 0x80) >> 6;
        uint8_t minute = BcdToUint8(raw & 0x7F);

        raw = _wire.read();
        flags |= (raw & 0x80) >> 5;
        uint8_t hour = BcdToBin24Hour(raw & 0x7f);

        raw = _wire.read();
        flags |= (raw & 0xc0) >> 3;
        uint8_t dayOf = BcdToUint8(raw & 0x3f);

        if (flags == DS3232AlarmOneControl_HoursMinutesSecondsDayOfWeekMatch)
        {
            dayOf = RtcDateTime::ConvertRtcToDow(dayOf);
        }

        return DS3232AlarmOne(dayOf, hour, minute, second, (DS3232AlarmOneControl)flags);
    }

    DS3232AlarmTwo GetAlarmTwo()
    {
        _wire.beginTransmission(DS3232_ADDRESS);
        _wire.write(DS3232_REG_ALARMTWO);
        _lastError = _wire.endTransmission();
        if (_lastError != 0)
        {
            return DS3232AlarmTwo(0, 0, 0, DS3232AlarmTwoControl_HoursMinutesDayOfMonthMatch);
        }

        _wire.requestFrom(DS3232_ADDRESS, DS3232_REG_ALARMTWO_SIZE);

        uint8_t raw = _wire.read();
        uint8_t flags = (raw & 0x80) >> 7;
        uint8_t minute = BcdToUint8(raw & 0x7F);

        raw = _wire.read();
        flags |= (raw & 0x80) >> 6;
        uint8_t hour = BcdToBin24Hour(raw & 0x7f);

        raw = _wire.read();
        flags |= (raw & 0xc0) >> 4;
        uint8_t dayOf = BcdToUint8(raw & 0x3f);

        if (flags == DS3232AlarmTwoControl_HoursMinutesDayOfWeekMatch)
        {
            dayOf = RtcDateTime::ConvertRtcToDow(dayOf);
        }

        return DS3232AlarmTwo(dayOf, hour, minute, (DS3232AlarmTwoControl)flags);
    }

    // Latch must be called after an alarm otherwise it will not
    // trigger again
    DS3232AlarmFlag LatchAlarmsTriggeredFlags()
    {
        uint8_t sreg = getReg(DS3232_REG_STATUS);
        uint8_t alarmFlags = (sreg & DS3232_AIFMASK);
        sreg &= ~DS3232_AIFMASK; // clear the flags
        setReg(DS3232_REG_STATUS, sreg);
        return (DS3232AlarmFlag)alarmFlags;
    }

    void ForceTemperatureCompensationUpdate(bool block)
    {
        uint8_t creg = getReg(DS3232_REG_CONTROL);
        creg |= _BV(DS3232_CONV); // Write CONV bit
        setReg(DS3232_REG_CONTROL, creg);

        while (block && (creg & _BV(DS3232_CONV)) != 0)
        {
            // Block until CONV is 0
            creg = getReg(DS3232_REG_CONTROL);
        }
    }

    int8_t GetAgingOffset()
    {
        return getReg(DS3232_REG_AGING);
    }

    void SetAgingOffset(int8_t value)
    {
        setReg(DS3232_REG_AGING, value);
    }

private:
    T_WIRE_METHOD& _wire;
    uint8_t _lastError;

    uint8_t getReg(uint8_t regAddress)
    {
        _wire.beginTransmission(DS3232_ADDRESS);
        _wire.write(regAddress);
        _lastError = _wire.endTransmission();
        if (_lastError != 0)
        {
            return 0;
        }

        // control register
        _wire.requestFrom(DS3232_ADDRESS, (uint8_t)1);

        uint8_t regValue = _wire.read();
        return regValue;
    }

    void setReg(uint8_t regAddress, uint8_t regValue)
    {
        _wire.beginTransmission(DS3232_ADDRESS);
        _wire.write(regAddress);
        _wire.write(regValue);
        _lastError = _wire.endTransmission();
    }

};

#endif // __RTCDS3232_H__
