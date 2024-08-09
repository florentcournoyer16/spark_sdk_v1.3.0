/** @file  sr1000_reg_v8_2.h
 *  @brief SR1010/SR1020 v8.2 registers map.
 *
 *  @copyright Copyright (C) 2019 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SR1000_REG_V8_2_H_
#define SR1000_REG_V8_2_H_

/* CONSTANTS ******************************************************************/
#define REG_STATUS1         0x00
#define   BIT_STAT2IRQ     BIT(7)
#define   BIT_PKBEGINI     BIT(6)
#define   BIT_RXTIMEOI     BIT(5)
#define   BIT_TXENDI       BIT(4)
#define   BIT_NEWPKTI      BIT(3)
#define   BIT_ADDRMATI     BIT(2)
#define   BIT_BRDCASTI     BIT(1)
#define   BIT_CRCPASSI     BIT(0)
#define REG_IRQMASK1        0x00
#define   BIT_IRQPOLAR     BIT(7)
#define   BIT_PKBEGINE     BIT(6)
#define   BIT_RXTIMEOE     BIT(5)
#define   BIT_TXENDE       BIT(4)
#define   BIT_NEWPKTE      BIT(3)
#define   BIT_ADDRMATE     BIT(2)
#define   BIT_BRDCASTE     BIT(1)
#define   BIT_CRCPASSE     BIT(0)
#define REG_STATUS2         0x01
#define   BIT_XOTIMERI     BIT(7)
#define   BIT_WAKEUPI      BIT(6)
#define   BIT_CSCFAILI     BIT(5)
#define   BIT_TXUDRFLI     BIT(4)
#define   BIT_RXOVRFLI     BIT(3)
#define   BIT_TXOVRFLI     BIT(2)
#define   BIT_BUFLOADI     BIT(1)
#define   BIT_BUFSTOPI     BIT(0)
#define REG_IRQMASK2        0x01
#define   BIT_XOTIMERE     BIT(7)
#define   BIT_WAKEUPE      BIT(6)
#define   BIT_CSCFAILE     BIT(5)
#define   BIT_TXUDRFLE     BIT(4)
#define   BIT_RXOVRFLE     BIT(3)
#define   BIT_TXOVRFLE     BIT(2)
#define   BIT_BUFLOADE     BIT(1)
#define   BIT_BUFSTOPE     BIT(0)
#define REG_TXFIFOSTAT      0x02
#define   BITS_TXBUFLOAD   BITS8(7, 0)
#define REG_TXFIFOTHRESH    0x02
#define   BIT_TXIRQEN      BIT(7)
#define   BITS_TXTHRESH    BITS8(6, 0)
#define REG_RXFIFOSTAT      0x03
#define   BITS_RXBUFLOAD   BITS8(7, 0)
#define REG_RXFIFOTHRESH    0x03
#define   BIT_RXIRQEN      BIT(7)
#define   BITS_RXTHRESH    BITS8(6, 0)
#define REG_SLEEPCONF       0x04
#define   BITS_SLPDEPTH    BITS8(7, 6)
#define   BIT_SLPRXTO      BIT(5)
#define   BIT_SLPTXEND     BIT(4)
#define   BIT_SLPRXEND     BIT(3)
#define   BIT_SLPMATCH     BIT(2)
#define   BIT_SLPBRDCA     BIT(1)
#define   BIT_SLPNOISY     BIT(0)
#define REG_TIMERCONF       0x05
#define   BIT_AUTOWAKE     BIT(7)
#define   BIT_WAKEONCE     BIT(6)
#define   BIT_SYNATEND     BIT(5)
#define   BIT_SYNTXSTA     BIT(4)
#define   BIT_SYNRXSTA     BIT(3)
#define   BIT_SYNMATCH     BIT(2)
#define   BIT_SYNBRDCA     BIT(1)
#define   BIT_SYNRXCRC     BIT(0)
#define REG_TIMERCOUNT1     0x06
#define   BITS_XTALCOUNT8  BITS8(7, 0)
#define REG_TIMERPERIOD1    0x06
#define   BITS_SLPPERIOD8  BITS8(7, 0)
#define REG_TIMERCOUNT0     0x07
#define  BITS_XTALCOUNT0   BITS8(7, 0)
#define REG_TIMERPERIOD0    0x07
#define   BITS_SLPPERIOD0  BITS8(7, 0)
#define REG_RXTIMEOUT1      0x08
#define   BITS_RXPERIOD4   BITS8(7, 0)
#define REG_RXTIMEOUT0      0x09
#define   BITS_RXPERIOD0   BITS8(7, 4)
#define   BITS_RXPUDELAY   BITS8(3, 0)
#define REG_PWRUPDELAY      0x0A
#define   BITS_PWRUPDEL    BITS8(7, 0)
#define REG_PLLSTARTUP      0x0B
#define   BITS_PLLPUWAIT   BITS8(7, 0)
#define REG_PLLRATIO        0x0C
#define   BITS_RPLYTXDEL   BITS8(7, 6)
#define   BITS_PLLRATIO    BITS8(5, 0)
#define REG_RESISTUNE       0x0D
#define   BIT_LNAIMPED     BIT(7)
#define   BITS_PLLRES      BITS8(6, 4)
#define   BITS_VREFTUNE    BITS8(3, 0)
#define REG_DISABLES        0x0E
#define   BIT_STDSPI       BIT(7)
#define   BIT_AUTOFLUSHDIS BIT(6)
#define   BIT_1VSWDIS      BIT(5)
#define   BIT_DCDCDIS      BIT(4)
#define   BIT_PLLDIS       BIT(3)
#define   BIT_SYMBCSRC     BIT(2)
#define   BIT_XTALCSRC     BIT(1)
#define   BIT_OUTPXTAL     BIT(0)
#define REG_RXFILTERS       0x0F
#define   BITS_LNAPEAK     BITS8(7, 5)
#define   BITS_RFFILFREQ   BITS8(4, 0)
#define REG_TXPULSE12       0x10
#define REG_TXPULSE11       0x11
#define REG_TXPULSE10       0x12
#define REG_TXPULSE9        0x13
#define REG_TXPULSE8        0x14
#define REG_TXPULSE7        0x15
#define REG_TXPULSE6        0x16
#define REG_TXPULSE5        0x17
#define REG_TXPULSE4        0x18
#define REG_TXPULSE3        0x19
#define REG_TXPULSE2        0x1A
#define REG_TXPULSE1        0x1B
#define   BITS_PULSEWID    BITS8(7, 5)
#define   BITS_PULSEFREQ   BITS8(4, 0)
#define   TXPULSE_DISABLE  0x00
#define REG_TXPARAMS        0x1C
#define   BIT_HOLDTXON     BIT(7)      /* RW */
#define   BIT_RNDPHASE     BIT(6)      /* RW */
#define   BITS_TXPOWER     BITS8(5, 4) /* RW */
#define   BIT_IFFILTEN     BIT(3)      /* RW */
#define   BITS_VDDLEVEL    BITS8(2, 0) /* RO */
#define REG_DLLTUNING       0x1D
#define   BITS_DLTUNE      BITS8(7, 4) /* RW */
#define   BIT_ECO          BIT(3)      /* WO */
#define   BIT_LEADLAG      BIT(3)      /* RO */
#define   BIT_INTEGLEN     BIT(2)      /* RW */
#define   BITS_INTEGGAIN   BITS8(1, 0) /* RW */
#define REG_CALIBREQUEST    0x1E
#define   BITS_DCROCODE    BITS8(4, 0)
#define REG_CALIBRESULT     0x1E
#define   BITS_DCROFREQ    BITS8(7, 0)
#define REG_PWRSTATUS       0x1F
#define   BIT_ROMEN        BIT(7)
#define   BIT_RXEN         BIT(6)
#define   BIT_TXEN         BIT(5)
#define   BIT_AWAKE        BIT(4)
#define   BIT_MODEMON      BIT(3)
#define   BIT_DCDCEN       BIT(2)
#define   BIT_PLLEN        BIT(1)
#define   BIT_REFEN        BIT(0)
#define REG_ACTIONS         0x1F
#define   BIT_CALIBRAT     BIT(7)
#define   BIT_SKIPWAKE     BIT(6)
#define   BIT_RXMODE       BIT(5)
#define   BIT_STARTTX      BIT(4)
#define   BIT_INITTIME     BIT(3)
#define   BIT_GOTOSLP      BIT(2)
#define   BIT_FLUSHRX      BIT(1)
#define   BIT_FLUSHTX      BIT(0)
#define REG_NVMVALUE        0x20
#define   BITS_ROMBYTE     BITS8(7, 0)
#define REG_NVMADDRESS      0x20
#define   BIT_ROMPWRSW     BIT(7)
#define   BITS_ROMADDR     BITS8(6, 0)
#define REG_BITTHRES        0x21
#define   BITS_BITTHRES    BITS8(3, 0)
#define REG_RSSI            0x22
#define   BITS_RSSI        BITS8(5, 0)
#define REG_RNSI            0x23
#define   BITS_RNSI        BITS8(5, 0)
#define REG_RXWAITTIME1     0x24
#define   BIT_RXWAISRC     BIT(7)      /* RW */
#define   BITS_RXWAITED8   BITS8(6, 0) /* RO */
#define REG_RXWAITTIME0     0x25
#define   BITS_RXWAITED0   BITS8(7, 0)
#define REG_FRAMEADDR1      0x26
#define   BITS_PKTADDR8    BITS8(7, 0)
#define REG_FRAMEADDR0      0x27
#define   BITS_PKTADDR0    BITS8(7, 0)
#define REG_NOISELVL        0x28
#define   BITS_MAXADCSIG   BITS8(7, 4)
#define   BITS_MINADCSIG   BITS8(3, 0)
#define REG_PHSDATA1        0x28
#define   BITS_PHSDATA1    BITS8(7, 0)
#define REG_PREFRAMECFG     0x29
#define   BIT_DLAYRPLY     BIT(7)
#define   BITS_NUMPREDET   BITS8(6, 4)
#define   BITS_PREATHRES   BITS8(3, 0)
#define REG_PHSDATA2        0x29
#define   BITS_PHSDATA2    BITS8(7, 0)
#define REG_MODEMGAIN       0x2A
#define   BITS_DATASRC     BITS8(7, 6)
#define   BITS_MANUGAIN    BITS8(5, 0)
#define REG_PHSDATA3        0x2A
#define   BITS_PHSDATA3    BITS8(7, 0)
#define REG_DEBUGMODEM      0x2B
#define   BIT_OVERRIDE     BIT(7)
#define   BITS_MANUPHASE   BITS8(6, 5)
#define   BIT_MANBITHR     BIT(4)
#define   BIT_BITTHRADJ    BITS8(3, 0)
#define REG_PHSDATA4        0x2B
#define   BITS_PHSDATA4    BITS8(7, 0)
#define REG_MAINMODEM       0x2C
#define   BIT_AUTOTX       BIT(7)
#define   BIT_AUTORPLY     BIT(6)
#define   BITS_ISIMITIG    BITS8(5, 4)
#define   BITS_MODULATION  BITS8(3, 2)
#define   BITS_FECLEVEL    BITS8(1, 0)
#define REG_CLEARTOSEND     0x2D
#define   BITS_IDLERXPWR   BITS8(7, 6)
#define   BITS_CSTHRES     BITS8(5, 0)
#define REG_RXPAUSETIME     0x2E
#define   BITS_RXOFFTIME   BITS8(7, 0)
#define REG_PREAMBLEN       0x2F
#define   BITS_PREAMBLEN   BITS8(7, 0)
#define REG_CONSTGAINS      0x30
#define   BITS_IFGAIN3     BITS8(7, 5)
#define   BITS_IFOAGAIN    BITS8(4, 2)
#define   BITS_RFGAIN      BITS8(1, 0)
#define REG_SYNCWORDCFG     0x31
#define   BIT_SWLENGTH     BIT(7)
#define   BITS_SWBITCOST   BITS8(6, 5)
#define   BITS_SWCORRTOL   BITS8(4, 0)
#define REG_SYNCWORD3       0x32
#define   BITS_SYNCWORD24  BITS8(7, 0)
#define REG_SYNCWORD2       0x33
#define   BITS_SYNCWORD16  BITS8(7, 0)
#define REG_SYNCWORD1       0x34
#define   BITS_SYNCWORD8   BITS8(7, 0)
#define REG_SYNCWORD0       0x35
#define   BITS_SYNCWORD0   BITS8(7, 0)
#define REG_CRCPOLYNOM1     0x36
#define   BITS_CRCPOLYNO8  BITS8(7, 0)
#define REG_CRCPOLYNOM0     0x37
#define   BITS_CRCPOLYNO0  BITS8(7, 0)
#define REG_REMOTADDR1      0x38
#define   BITS_REMOTADDR8  BITS8(7, 0)
#define REG_REMOTADDR0      0x39
#define   BITS_REMOTADDR0  BITS8(7, 0)
#define REG_LOCALADDR1      0x3A
#define   BITS_LOCALADDR8  BITS8(7, 0)
#define REG_LOCALADDR0      0x3B
#define   BITS_LOCALADDR0  BITS8(7, 0)
#define REG_TXPKTSIZE       0x3C
#define   BITS_TXPKTSIZE   BITS8(7, 0)
#define REG_RXPKTSIZE       0x3D
#define   BITS_RXPKTSIZE   BITS8(7, 0)
#define REG_PACKETCFG       0x3E
#define   BITS_ADDRFILT    BITS8(7, 6)
#define   BIT_ADDRLEN      BIT(5)
#define   BIT_ADDRHDRE     BIT(4)
#define   BIT_SIZEHDRE     BIT(3)
#define   BIT_SIZESRC      BIT(2)
#define   BIT_SAVEADDR     BIT(1)
#define   BIT_SAVESIZE     BIT(0)
#define REG_RXFIFO          0x3F
#define   BITS_RXBUFFER    BITS8(7, 0)
#define REG_TXFIFO          0x3F
#define   BITS_TXBUFFER    BITS8(7, 0)

/**********************************************************/
/* These are 16-bit accesses to the registers. */
/* It assumes a little-endian processor. */
/**********************************************************/
#define REG16_STATUS         0x00
#define   BIT16_STAT2IRQ     BIT16_1(7)
#define   BIT16_PKBEGINI     BIT16_1(6)
#define   BIT16_RXTIMEOI     BIT16_1(5)
#define   BIT16_TXENDI       BIT16_1(4)
#define   BIT16_NEWPKTI      BIT16_1(3)
#define   BIT16_ADDRMATI     BIT16_1(2)
#define   BIT16_BRDCASTI     BIT16_1(1)
#define   BIT16_CRCPASSI     BIT16_1(0)
#define   BIT16_XOTIMERI     BIT16_2(7)
#define   BIT16_WAKEUPI      BIT16_2(6)
#define   BIT16_CSCFAILI     BIT16_2(5)
#define   BIT16_TXUDRFLI     BIT16_2(4)
#define   BIT16_RXOVRFLI     BIT16_2(3)
#define   BIT16_TXOVRFLI     BIT16_2(2)
#define   BIT16_BUFLOADI     BIT16_2(1)
#define   BIT16_BUFSTOPI     BIT16_2(0)
#define REG16_IRQMASK        0x00
#define   BIT16_IRQPOLAR     BIT16_1(7)
#define   BIT16_PKBEGINE     BIT16_1(6)
#define   BIT16_RXTIMEOE     BIT16_1(5)
#define   BIT16_TXENDE       BIT16_1(4)
#define   BIT16_NEWPKTE      BIT16_1(3)
#define   BIT16_ADDRMATE     BIT16_1(2)
#define   BIT16_BRDCASTE     BIT16_1(1)
#define   BIT16_CRCPASSE     BIT16_1(0)
#define   BIT16_XOTIMERE     BIT16_2(7)
#define   BIT16_WAKEUPE      BIT16_2(6)
#define   BIT16_CSCFAILE     BIT16_2(5)
#define   BIT16_TXUDRFLE     BIT16_2(4)
#define   BIT16_RXOVRFLE     BIT16_2(3)
#define   BIT16_TXOVRFLE     BIT16_2(2)
#define   BIT16_BUFLOADE     BIT16_2(1)
#define   BIT16_BUFSTOPE     BIT16_2(0)
#define REG16_INTFLG_DFLT         (BIT16_IRQPOLAR | BIT16_TXENDE | BIT16_NEWPKTE | BIT16_WAKEUPE | BIT16_CSCFAILE | \
                                   BIT16_TXUDRFLE | BIT16_RXOVRFLE | BIT16_TXOVRFLE | BIT16_BUFLOADE)
#define REG16_INTFLG_INT_BITS (BITS16_1(6, 0) | BITS16_2(7, 0))

#endif /* SR1000_REG_V8_2_H_ */
