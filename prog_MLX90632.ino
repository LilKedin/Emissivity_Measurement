
/***************************************************************************************************

file:          i2c_sensor.c

description:   Head module for support of i2c


history:       date         author              changes
               20019/07/24   V.Dorscht        initial release

***************************************************************************************************/



// ***** dependencies
//#include "global.h"
//#include "main.h"
//#include "main_apl.h"
//#include "events_apl.h"
//#include "i2cHw.h"
//#include "i2c_sensor.h"
#include "math.h"



// ***** constant definitions
const u8 i2c_sensor_fileName [] = __FILE__;  // used for exception purposes
u8 IrSensStM = ST_NODEF;    /* ir-sensor state machine */
const f32 i2c_EmsFctIni = 1.; /* intalization value for ir-sensor emissivity */

// ***** variable definitions
u8   IrSensEEP[76];     // buffer for sonsor constants (eeprom)
u8   IrSensRAM[20];     // buffer sonsor ram values
u16  EE_Ha_eeval;     // (0x2481) Ha customer calibration constant
u16  EE_Hb_eeval;     // (0x2482) Hb customer calibration constant
u16  EE_Ha_cust = 0x4000; //  Ha customer calibration value
u16  EE_Hb_cust = 0x0000; //  Hb customer calibration value

u16  EE_Control;    // (0x24D4) EEPROM control register (sleep step mode = 0x02; step mode = 0x04; continuous mode = 0x06)
u16  EE_I2C_Address;  // (0x24D5) I²C slave address >> 1 (standard address  0x3A >> 1 = 0x1D)

u16  REG_I2C_Address;   // (0x3000) I²C address register value
u16  REG_Control;   // (0x3001) control register (sleep step mode = 0x02; step mode = 0x04; continuous mode = 0x06)
u16  REG_Status;    // (0x3FFF) status register (bit0: new_data; bit2-6: cycle_pos; bit8: brown_out; bit9: eeprom_busy; bit10: device_busy)

/* sensor constants */
f32  P_R = 0.;
f32  P_G = 0.;
f32  P_T = 0.;
f32  P_O = 0.;
f32  Fa = 0.;
f32  Fb = 0.;
f32  Ga = 0.;
f32  Ea = 0.;
f32  Eb = 0.;
f32  Gb = 0.;
f32  Ka = 0.;
f32  Ha = 0.;
f32  Hb = 0.;
f32  i2c_EmsFct = 1.; // emissivity parameter (not stored in sensor EEPROM)

f32  Ta_C = 0.;   // ambient temperature in °C
f32  To_C = 0.;     // object temperature in °C

// ***** function definitions
/***************************************************************************************************
i2c_SensorInit
****************************************************************************************************
description:   performs initialization of IR sensor

history:       date         author              changes
         2019-10-14   V. Dorscht          initial release

arguments:
  - no arguments
returns:
  - (u8) success
***************************************************************************************************/
u8 i2cSensorInit (void)
{
  u8 retVal = ERR_OK;
  #ifdef MOD_I2C  /* module supported */

  if( i2c_AddressedRead(EEPROM_STARTADR, EE_SIZE) == ERR_OK)    /* read all constants from sensor EEPROM */
  {
    EE_Ha_eeval = i2c_ReadRegister(EE_Ha);          /* read Ha value */
    EE_Hb_eeval = i2c_ReadRegister(EE_Hb);          /* read Hb value */
    EE_Control = i2c_ReadRegister(EE_CONTROL);        /* read EE_Control value */
    EE_I2C_Address = i2c_ReadRegister(EE_I2C_ADDRESS);    /* read EE_I2C address value */

    REG_I2C_Address = i2c_ReadRegister(REG_I2C_ADDRESS);  /* read register I2C address value */
    REG_Control = i2c_ReadRegister(REG_CONTROL);      /* read register control value */
    REG_Status = i2c_ReadRegister(REG_STATUS);        /* read register status value */
  }
  else
    return ERR_RESOURCE;

  // when device is busy stop initialization and try again later
  if((REG_Control & REG_STAT_DEVBUSY)== REG_STAT_DEVBUSY)
    return ERR_BUSY;

  // when eeprom is busy stop initialization and try again later
  if((REG_Control & REG_STAT_EEPBUSY)== REG_STAT_EEPBUSY)
    return ERR_BUSY;

  // check if customer constants are stored in eeprom
  if(EE_Ha_eeval != EE_Ha_cust)
  {
    (void) i2c_WriteRegister (EE_Ha, 0x0000);     // write 0 to eeprom
    (void) i2c_WriteRegister (EE_Ha, EE_Ha_cust);   // write value to eeprom
  }
  if(EE_Hb_eeval != EE_Hb_cust)
  {
    (void) i2c_WriteRegister (EE_Hb, 0x0000);     // write 0 to eeprom
    (void) i2c_WriteRegister (EE_Hb, EE_Hb_cust);   // write value to eeprom
  }


  // check if eeprom and register are similar to slave address
  if((REG_I2C_Address != EE_I2C_Address) || (REG_I2C_Address != (SLAVE_ADDRESS >> 2)))
  {
//    EE_I2C_Address = i2c_ReadRegister(EE_I2C_ADDRESS);    /* read EE_I2C address value */
//    (void) i2c_WriteRegister (EE_I2C_ADDRESS, 0x0000);      // write 0 to eeprom
//    (void) i2c_WriteRegister (EE_I2C_ADDRESS, SLAVE_ADDRESS); // write value to eeprom
  }

  // check measurement mode
  if((REG_Control & REG_CTRL_MODE_MASK) != REG_CTRL_CONT_MODE)
    (void) i2c_WriteRegister (REG_CONTROL, REG_CTRL_CONT_MODE); // set continuous mode

  if(i2c_ReadRegister(REG_CONTROL) != REG_CTRL_CONT_MODE)
    retVal = ERR_INITFAIL;

  #endif
  return (retVal);
} // *** end of i2c_prepare ()



/***************************************************************************************************
i2cRequest
****************************************************************************************************
description:   requests i2c

history:       date         author              changes
         2019-07-24   V. Dorscht          initial release

arguments:
  - no arguments
returns:
  - (u8) success

returns:
   - (u32) depending data (if any)
***************************************************************************************************/
u32 i2c_SensorRequest (REQUEST u32 request, DATA void *data)
{
  u32 retVal = 0;
  
  #ifdef MOD_I2C  /* module supported */
  switch (request)
  {
/*vd
    case REQ_CLEAREVT:
    {
      switch ( *( (u16 *) data) )
      {
        default: break; // unknown event to clear: ignore
      }
    } break;
*/
    // unknown request
    default:
    break; 
  }
  #endif

  return retVal;
} // *** end of i2cRequest ()

s32 Get_s32Value(u8* pdata)
{
  return (*(pdata+2) << 24| *(pdata+3)<<16 | *pdata<<8 | (*(pdata+1) & 0xFF) );
}
s32 Get_s16Value(u8* pdata)
{
  return (*pdata<<8 | (*(pdata+1) & 0xFF));
}
/***************************************************************************************************
i2c_PreCalculation
****************************************************************************************************
description:   starts pre-calculation of sensor parameters

history:       date         author              changes
         2019-07-24   V. Dorscht          initial release

arguments:
  - no arguments
returns:
  - (u8) success
***************************************************************************************************/
u8 i2c_PreCalculation (void)
{
  u8 retVal = ERR_OK;

  #ifdef MOD_I2C  /* module supported */
  retVal = i2c_AddressedRead(EEPROM_STARTADR, EE_SIZE);   /* read all constants from sensor EEPROM */
  retVal = i2c_DirectRead ();               /* read all ram variable from sensor */

  if(retVal == ERR_OK)
  {
    P_R = Get_s32Value(&IrSensEEP[2])  * pow(2,(-8));   // P_R = EE_P_R * 2^(-8)
    P_R = Get_s32Value(&IrSensEEP[2])  * exp2(-8);    // P_R = EE_P_R * 2^(-8)
    P_G = Get_s32Value(&IrSensEEP[6])  * pow(2,(-20));    // P_G = EE_P_G * 2^(-20)
    P_T = Get_s32Value(&IrSensEEP[10]) * pow(2,(-44));    // P_T = EE_P_T * 2^(-44)
    P_O = Get_s32Value(&IrSensEEP[14]) * pow(2,(-8));   // P_O = EE_P_O * 2^(-8)

    Ea = Get_s32Value(&IrSensEEP[50]) * pow(2,(-16));   // Ea = EE_Ea * 2^(-16)
    Eb = Get_s32Value(&IrSensEEP[54]) * pow(2,(-8));    // Eb = EE_Eb * 2^(-8)
    Fa = Get_s32Value(&IrSensEEP[58]) * pow(2,(-46));   // Fa = EE_Fa * 2^(-46)
    Fb = Get_s32Value(&IrSensEEP[62]) * pow(2,(-36));   // Fb = EE_Fb * 2^(-36)
    Ga = Get_s32Value(&IrSensEEP[66]) * pow(2,(-36));   // Ga = EE_Ga * 2^(-36)
    Gb = Get_s16Value(&IrSensEEP[70]) * pow(2,(-10));   // Gb = EE_Gb * 2^(-10)
    Ka = Get_s16Value(&IrSensEEP[72]) * pow(2,(-10));   // Ka = EE_Ka * 2^(-10)
    Ha = EE_Ha_eeval * pow(2,(-14));            // Ha = EE_Ha * 2^(-14)  Customer calibration constant
    Hb = EE_Hb_eeval * pow(2,(-14));            // Hb = EE_Hb * 2^(-14)  Customer calibration constant
  }
  #endif
  return (retVal);
} // *** end of i2c_PreCalculation ()

/***************************************************************************************************
i2c_TempCalc
****************************************************************************************************
description:   main temperature calculation function

history:       date         author              changes
         2019-07-24   V. Dorscht          initial release

arguments:
  - no arguments
returns:
  - (u8) success
***************************************************************************************************/
u8 i2c_TempCalc (void)
{
  u8 retVal = ERR_OK;
  static f32  TOdut = 25.;  // use for first temp. calculation
  f32  TAdut = 0.;
  f32  Ta_k = 0.;
  f32  VRta = 0.;
  f32  AMB = 0.;
  f32  VRto = 0.;
  f32  Sto = 0.;
  s16  S = 0;
  f32 tempTo_C = 0.;
  f32 TOo = 25.;
  f32 TAo = 25.;

  #ifdef MOD_I2C  /* module supported */

  if( i2c_DirectRead () != ERR_OK) return ERR_ILLDATA;

  /******** ambient temperature calculation *********************************************************************/
  VRta = Get_s16Value(&IrSensRAM[18]) + Gb * Get_s16Value(&IrSensRAM[12]) / 12; // VRta = RAM9 + Gb * RAM6/12

  AMB = (Get_s16Value(&IrSensRAM[12]) / 12) / VRta * pow(2,19);         // AMB = (RAM6/12) * VRta * 2^19

  Ta_C = P_O + (AMB-P_R)/P_G + P_T * pow((AMB-P_R),2);              // Ta_C = P_O + (AMB-P_R)/P_G + P_T * (AMB-P_R)²
  /*----------------------------------------------------------------------------------------------------------------*/

  /******** object temperature calculation *********************************************************************/
  REG_Status = Get_s16Value(&IrSensRAM[0]);

  if( (REG_Status & REG_STAT_CYCLPOS_MASK) == REG_STAT_CYCLPOS_1)
    S =(Get_s16Value(&IrSensRAM[8]) + Get_s16Value(&IrSensRAM[10])) / 2;  // S = (RAM4 + RAM5)/2
  else if((REG_Status & REG_STAT_CYCLPOS_MASK) == REG_STAT_CYCLPOS_2)
    S =(Get_s16Value(&IrSensRAM[14]) + Get_s16Value(&IrSensRAM[16])) / 2;   // S = (RAM7 + RAM8)/2
  else
    S =(Get_s16Value(&IrSensRAM[14]) + Get_s16Value(&IrSensRAM[16])) / 2;   // S = (RAM7 + RAM8)/2

  VRto = Get_s16Value(&IrSensRAM[18]) + Ka * Get_s16Value(&IrSensRAM[12]) / 12; // VRto = RAM9 + Ka * RAM6/12;
  Sto =  (S/12) / VRto * exp2(19);                        // Sto = (S/12) / VRto * 2^19                         // TOdut = (AMB-Eb)/Ea + 25°C
  TAdut = (AMB-Eb)/Ea + 25.;                            // TAdut = (AMB-Eb)/Ea + 25°C
  Ta_k = TAdut + 273.15;                              // Ta[K] = TAdut + 273.15 Kelvin
/*
 * Testweise eingefügt um git zu testen neur versuch
  Ta_k4 = pow(Ta_k,4);
  a = (1+Ga*(TOdut-TOo)+Fb*(TAdut-TAo));
  b = EmsFct*Fa*Ha;
  c = Sto/(a*b);
  d = c+Ta_k4;
  e = pow(d,0.25);
*/
  if((i2c_EmsFct >= 0.5) && (i2c_EmsFct <= 1.5))
  {
    tempTo_C = pow( ((Sto / (i2c_EmsFct*Fa*Ha*(1+Ga*(TOdut-TOo)+Fb*(TAdut-TAo)))) + pow(Ta_k,4)), 0.25 ) - 273.15 - Hb;

    if((tempTo_C > -10.) && (tempTo_C < 150.))
      TOdut = To_C = tempTo_C;
    else TOdut = 25;
  }

  #endif
  return (retVal);
} // *** end of i2c_PreCalculation ()

u16 IrSensVar [10];
/***************************************************************************************************
i2cTic
****************************************************************************************************
description:   timer base for this module

history:       date         author              changes
               2019/07/24   V.Dorscht        initial release

arguments:
  - (u16) ms tic
returns:
  - nothing
***************************************************************************************************/
void i2c_SensorHndl (TIME u16 time)
{
  #ifdef MOD_I2C  /* module supported */
  if(time == 1000)
  {
    switch(IrSensStM)
    {
      case ST_NODEF:
      {
        if( i2cSensorInit() == ERR_OK)
          IrSensStM = ST_INITIALZING;
        else
          IrSensStM = ST_NODEF;
      }break;

      case ST_INITIALZING:
      {
        if( i2c_PreCalculation() == ERR_OK)
          IrSensStM = ST_RUNNING;
        else
          IrSensStM = ST_INITIALZING;
      }break;

      case ST_RUNNING:
      {
        if(i2c_TempCalc() != ERR_OK)
          IrSensStM = ST_NODEF;
      }break;

      default: IrSensStM = ST_NODEF;
           break;

    }

  }
  #endif
} // *** end of i2c_Tic ()



// *** end of file
