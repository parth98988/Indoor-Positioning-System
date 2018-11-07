/*==== DECLARATION CONTROL ===================================================*/
/*==== INCLUDES ==============================================================*/

#include "hal_main.h"
#include "per_test_main.h"
#include "stdio.h"
#include "ioCC2510.h"

#include <hal_types.h>
#include <hal_defs.h>
#include <hal_cc8051.h>
#include <ioCCxx10_bitdef.h>

/*==== MACROS ======================================================*/
#define CONST_volt 0.00732421 // (3.75/511)
#define CONST 2.446118 // (1250/511)
#define OFFSET_DATASHEET 750
#define OFFSET_MEASURED_AT_25_DEGREES_CELCIUS 29.75
#define OFFSET (OFFSET_DATASHEET + OFFSET_MEASURED_AT_25_DEGREES_CELCIUS)
#define TEMP_COEFF 2.43
#define ACT_MODE_TIME  10000

/*==== PUBLIC FUNCTIONS ======================================================*/
UINT8 i;
UINT8 volt_adch;
UINT8 volt_adcl;
UINT8 temp_adch;
UINT8 temp_adcl;

static UINT16  adc_result_temp; 
float temp_voltage;
float temp;

static UINT16  adc_result_volt; 
float voltage;

/***********************************************************************************
* LOCAL VARIABLES
*/


// Variable for active mode duration
static uint32 __xdata activeModeCnt = 0;

// Initialization of source buffers and DMA descriptor for the DMA transfer
// (ref. CC111xFx/CC251xFx Errata Note)
static uint8 __xdata PM2_BUF[7] = {0x06,0x06,0x06,0x06,0x06,0x06,0x04};
static uint8 __xdata dmaDesc[8] = {0x00,0x00,0xDF,0xBE,0x00,0x07,0x20,0x42};

static int8 EVENT0_HIGH = 0x50;
static int8 EVENT0_LOW = 0x00;


/***********************************************************************************
* LOCAL FUNCTIONS
*/

/***********************************************************************************
* @fn          setup_sleep_interrupt
*
* @brief       Function which sets up the Sleep Timer Interrupt
*              for Power Mode 2 usage.
*
* @param       void
*
* @return      void
*/
void setup_sleep_interrupt(void)
{
    // Clear Sleep Timer CPU Interrupt flag (IRCON.STIF = 0)
    STIF = 0;
    // Clear Sleep Timer Module Interrupt Flag (WORIRQ.EVENT0_FLAG = 0)
    WORIRQ &= ~WORIRQ_EVENT0_FLAG;
    // Enable Sleep Timer Module Interrupt (WORIRQ.EVENT0_MASK = 1)
    WORIRQ |= WORIRQ_EVENT0_MASK;
    // Enable Sleep Timer CPU Interrupt (IEN0.STIE = 1)
    STIE = 1;
    // Enable Global Interrupt (IEN0.EA = 1)
    EA =1;
}


/***********************************************************************************
* @fn          sleep_timer_isr
*
* @brief       Sleep Timer Interrupt Service Routine, which executes when
*              the Sleep Timer expires. Note that the [SLEEP.MODE] bits must
*              be cleared inside this ISR in order to prevent unintentional
*              Power Mode 2 entry.
*
* @param       void
*
* @return      void
*/
#pragma vector = ST_VECTOR
__interrupt void sleep_timer_isr(void)
{
    // Clear Sleep Timer CPU interrupt flag (IRCON.STIF = 0)
    STIF = 0;
    // Clear Sleep Timer Module Interrupt Flag (WORIRQ.EVENT0_FLAG = 0)
    WORIRQ &= ~WORIRQ_EVENT0_FLAG;
    // Clear the [SLEEP.MODE] bits, because an interrupt can also occur
    // before the SoC has actually entered Power Mode 2.
    SLEEP &= ~SLEEP_MODE;
    // Set SRF04EB LED1 to indicate Power Mode 2 exit
   // P1_0 = 0;

}  
       
/******************************************************************************
* @fn  main
*
* @brief
*      Main function. Triggers setup menus and main loops for both receiver
*      and transmitter. This function supports both CC1110 and CC2510.
*
* Parameters:
*
* @param  void
*
* @return void
*
******************************************************************************/
void main(void)
{
  
    volatile uint8 storedDescHigh = DMA0CFGH;
    volatile uint8 storedDescLow = DMA0CFGL;
    volatile int8 temp;
   
    // Initialize P1_1/3 for SRF04EB LED1/3
    P1SEL &= ~(BIT3 | BIT0);
    P1_0 = 1; P1_3 = 1;
    P1DIR |= (BIT3 | BIT0);

  /* I/O-Port configuration :
   * PIN0_7 is configured to an ADC input pin.
   */
  // Set [ADCCFG.ADCCFG7 = 1].
  ADCCFG |= ADCCFG_7;
  
    // Setup + enable the Sleep Timer Interrupt, which is
    // intended to wake-up the SoC from Power Mode 2.
        setup_sleep_interrupt(); 

    // Infinite loop:
    // Enter/exit Power Mode 2.
        
    while(1)
    {
      P1_0 = 0;
      // Choose the crystal oscillator as the system clock
      SLEEP &= ~(0x04);  // bit maks used to power down system clock oscillators ... powering down all oscillators
      while(!(0x40))  // bit mask used to check the stability of XOSC// waiting until the oscillator is stable
      asm("NOP");
      CLKCON &= ~(0x7F);  // bit mask used to control the system clock oscillator // starting the Crystal Oscillator
      SLEEP |= (0x04);  // bit maks used to power down system clock oscillators      // powering down the unused oscillator

      
/*******************************************************************Voltage******************************************************************************************************/

  // Set [ADCCON1.STSEL] according to ADC configuration */
//  ADCCON1 = ADCCON1_ST | BIT1 | BIT0;
    ADCCON1 = 0x73; 
  // Set [ADCCON2.SREF/SDIV/SCH] according to ADC configuration */
//  ADCCON2 = ADCCON2 &~(ADCCON2_SDIV0) | ADCCON2_SDIV1 | ADCCON2_SCH0 | ADCCON2_SCH1 | ADCCON2_SCH2 | ADCCON2_SCH3 ; 
    ADCCON2 = 0x2F;
    /* ADC conversion :
   * The ADC conversion is triggered by setting [ADCCON1.ST = 1].
   * The CPU will then poll [ADCCON1.EOC] until the conversion is completed.
   */

 /* Set [ADCCON1.ST] and await completion (ADCCON1.EOC = 1) */
//  ADCCON1 |= ADCCON1_ST | BIT1 | BIT0;
    ADCCON1 = 0X43;
    while( !(ADCCON1 && 0x80));

  /* Store the ADC result from the ADCH/L register to the adc_result variable.
   * The 4 LSBs in ADCL will not contain valid data, and are masked out.
   */
    halWait(1);
//    adc_result_volt = ADCL;
    volt_adcl = ADCL ;
    volt_adch = ADCH ;    
    
/****************************************************************************************************************************************/    
// printf("Adc Result adcl : %d \n", adc_result_volt);
//   printf("ADCH : %d \n", volt_adch);
//  adc_result_volt |= (((unsigned int)ADCH) << 8);                     //Make adch & adcl as 1 variable
//  adc_result_volt >>= 6;                             //Discard 6 bits for 10 bit resilution     
//  voltage = CONST_volt*adc_result_volt;
//  printf("Battery Voltage : %f", voltage);
 
 /*******************************************************************Temperature******************************************************************************************************/

  // Set [ADCCON1.STSEL] according to ADC configuration */
//  ADCCON1 = ADCCON1_ST | BIT1 | BIT0;
    ADCCON1 = 0X73;
  // Set [ADCCON2.SREF/SDIV/SCH] according to ADC configuration */
 // ADCCON2 = ADCCON2 &~(ADCCON2_SDIV0 | ADCCON2_SCH0 ) | ADCCON2_SDIV1 | ADCCON2_SCH1 | ADCCON2_SCH2 | ADCCON2_SCH3 ; 
    ADCCON2 = 0X3E;


  /* ADC conversion :
   * The ADC conversion is triggered by setting [ADCCON1.ST = 1].
   * The CPU will then poll [ADCCON1.EOC] until the conversion is completed.
   */

  /* Set [ADCCON1.ST] and await completion (ADCCON1.EOC = 1) */
  //ADCCON1 |= ADCCON1_ST | BIT1 | BIT0;
  ADCCON1 = 0x43;
  //while( !(ADCCON1 & ADCCON1_EOC));
  while(!(ADCCON1 & 0x80));
  /* Store the ADC result from the ADCH/L register to the adc_result variable.
   * The 4 LSBs in ADCL will not contain valid data, and are masked out.
   */
  temp_adcl = ADCL;
  temp_adch = ADCH; 
  halWait(1);
 
/****************************************************************************************************************************************/  
//  adc_result_temp = ADCL;
// printf("%x \n", ADCL); //If i comment this im  not getting proper output  
//  adc_result_temp |= (((unsigned int)ADCH) << 8);                     //Make adch & adcl as 1 variable
//  adc_result_temp >>= 6;                             //Discard 6 bits for 10 bit resilution 
//  printf("%x \n", adc_result_temp);  
//  temp_voltage = adc_result_temp*CONST;
//  printf("%f \n", temp_voltage);
//  temp = ((temp_voltage - OFFSET) / TEMP_COEFF); 
//  printf("Battery Temperature : %f \n", temp);
/****************************************************************************************************************************************/
  
  ADCL = 0x00;
  ADCH = 0x00;

  /*******************************************************************Start Transmission******************************************************************************************************/
//    printf("\nStart  \n");
    // Check that chip version is not too old or too new to be supported by
    // the register settings used in this software.
    checkChipVersion();
/***********************************************************************************************************************************************/
    // Common radio settings for CCxx10, any frequency and data rate
    // Settings from SmartRFStudio for CC2510, VERSION == 0x04
    // 250 kBaud, MSK modulation, 540 kHz RX filter bandwidth.
    //For 2420 MHz
    
    //Address byte missing 
    IOCFG2 = 0x0B;              // GDO2 output pin configuration.
    //IOCFG1 missing
    IOCFG0 = 0x06;              // GDO0 output pin configuration. Sync word.
    // FIFO Threshold missing
    // Sync1
    // Sync0
    PKTLEN = PACKET_LENGTH;     // Packet length.
    PKTCTRL1 = 0x04;            // Packet automation control.
    PKTCTRL0 = 0x45;            // Packet automation control. Data whitening on.
    ADDR     = 0x00;                // Device address. Not used.
    CHANNR   = 0x29;            // Channel number.
    FSCTRL1  = 0x0A;            // Frequency synthesizer control.
    FSCTRL0  = 0x00;            // Frequency synthesizer control.
    FREQ2    = 0x5D;
    FREQ1    = 0x13;
    FREQ0    = 0xB1;
    MDMCFG4  = 0x2D;   // Modem configuration.
    MDMCFG3  = 0x3B;   // Modem configuration.
    MDMCFG2  = 0x73;   // Modem configuration.
    MDMCFG1  = 0x22;   // Modem configuration.
    MDMCFG0  = 0xF8;   // Modem configuration.
    DEVIATN  = 0x00;   // Modem deviation setting (when FSK modulation is enabled).
    FREND1   = 0xB6;   // Front end RX configuration.
    FREND0   = 0x10;   // Front end RX configuration.
    MCSM1    = 0x30;   // Main Radio Control State Machine configuration.
    MCSM0    = 0x14;   // Main Radio Control State Machine configuration.
    FOCCFG   = 0x1D;   // Frequency Offset Compensation Configuration.
    BSCFG    = 0x1C;   // Bit synchronization Configuration.
    AGCCTRL2 = 0xC7;   // AGC control.
    AGCCTRL1 = 0x00;   // AGC control.
    AGCCTRL0 = 0xB2;   // AGC control.
    //WOREVT1
    //WOREVT0
    //WORCTRL
    //FREND1
    //FREND0
    FSCAL3   = 0xEA;   // Frequency synthesizer calibration.
    FSCAL2   = 0x0A;   // Frequency synthesizer calibration.
    FSCAL0   = 0x11;   // Frequency synthesizer calibration.
    //RCCTRL1
    //RCCTRL0
    //FSTEST
    //PTEST
    //AGCTEST
    TEST2    = 0x88;   // Various test settings.
    TEST1    = 0x31;   // Various test settings.
    TEST0    = 0x0B;   // Various test settings.
    PA_TABLE0 = 0xFE;  // 0dbm Power 
    
    //perRssiOffset = 71;// Set proper RSSI offset for receiver
 
/**************************************************************************************************************************************************/
   //   printf("Start Transmission\n");     
/*****************************************************DMA Configurations********************************************************************************************/      
    // Set up the DMA to move packet data from buffer to radio
    // Some configuration that are common for both TX and RX:
    // CPU has priority over DMA
    // Use 8 bits for transfer count
    // No DMA interrupt when done
    // DMA triggers on radio
    // Single transfer per trigger.
    // One byte is transferred each time.
    dmaConfig.PRIORITY       = DMA_PRI_LOW;
    dmaConfig.M8             = DMA_M8_USE_8_BITS;
    dmaConfig.IRQMASK        = DMA_IRQMASK_DISABLE;
    dmaConfig.TRIG           = DMA_TRIG_RADIO;
    dmaConfig.TMODE          = DMA_TMODE_SINGLE;
    dmaConfig.WORDSIZE       = DMA_WORDSIZE_BYTE;
    
    // Transmitter specific DMA settings
    // Source: radioPktBuffer
    // Destination: RFD register
    // Use the first byte read + 1
    // Sets the maximum transfer count allowed (length byte + data)
    // Data source address is incremented by 1 byte
    // Destination address is constant
    SET_WORD(dmaConfig.SRCADDRH, dmaConfig.SRCADDRL, radioPktBuffer);
    SET_WORD(dmaConfig.DESTADDRH, dmaConfig.DESTADDRL, &X_RFD);
    dmaConfig.VLEN           = DMA_VLEN_FIRST_BYTE_P_1;
    SET_WORD(dmaConfig.LENH, dmaConfig.LENL, (PACKET_LENGTH + 1));
    dmaConfig.SRCINC         = DMA_SRCINC_1;
    dmaConfig.DESTINC        = DMA_DESTINC_0;
    // Save pointer to the DMA configuration struct into DMA-channel 0
    // configuration registers
    SET_WORD(DMA0CFGH, DMA0CFGL, &dmaConfig);
  
/*************************************************Interrupt Configurations***********************************************************************************************/

        // Configure interrupt for every time a packet is sent
        HAL_INT_ENABLE(INUM_RF, INT_ON);    // Enable RF general interrupt
        RFIM = IRQ_DONE;                    // Mask IRQ_DONE flag only
        INT_GLOBAL_ENABLE(INT_ON);          // Enable interrupts globally

/************************************************************************************************************************************************************************/
        // Construct the packet to be transmitted in buffer
        radioPktBuffer[0] = PACKET_LENGTH;                  // Length byte
//        radioPktBuffer[1] = (BYTE) (NETWORK_ID_KEY>>8);     // Network identifier
//        radioPktBuffer[2] = (BYTE) NETWORK_ID_KEY;
        // radioPktBuffer[3:6] = 4 byte packet sequence number, written later
        // Fill rest of payload with dummy data. Radio is using data whitening.
        halWait(10);
        radioPktBuffer[1] = 90;
        radioPktBuffer[2] = 194;
        radioPktBuffer[3] = 15;
        radioPktBuffer[4] = 161;
        radioPktBuffer[5] = 00;
        radioPktBuffer[6] = 03 ;
        radioPktBuffer[7] = 245 ; 
        radioPktBuffer[8] = volt_adcl;
        radioPktBuffer[9] = volt_adch;
        radioPktBuffer[10] = 0;
        radioPktBuffer[11] = temp_adcl;
        radioPktBuffer[12] = temp_adch;
        radioPktBuffer[13] = 170;
        

        // Send the packet
        DMAARM |= DMAARM_CHANNEL0;  // Arm DMA channel 0
        RFST = STROBE_TX;           // Switch radio to TX

        // Wait until the radio transfer is completed,
        // and then reset pktSentFlag

        while(!pktSentFlag);
        pktSentFlag = FALSE;
        
  //      printf("Packets Transmistted \n");
  //      printf("Transmistted \n");
        
/**************************************************End of Transmission**********************************************************************************************************************/

        halWait(5);
        // Switch system clock source to HS RCOSC and max CPU speed:
        // Note that this is critical for Power Mode 2. After reset or
        // exiting Power Mode 2 the system clock source is HS RCOSC,
        // but to emphasize the requirement we choose to be explicit here.
        SLEEP &= ~SLEEP_OSC_PD;
        while( !(SLEEP & SLEEP_HFRC_S) );
        CLKCON = (CLKCON & ~CLKCON_CLKSPD) | CLKCON_OSC | CLKSPD_DIV_2;
        while ( !(CLKCON & CLKCON_OSC) ) ;
        SLEEP |= SLEEP_OSC_PD;

        // Set LS XOSC as the Sleep Timer clock source (CLKCON.OSC32 = 0)
        CLKCON &= ~CLKCON_OSC32;

        // Wait some time in Active Mode, and set SRF04EB LED1 before
        // entering Power Mode 2
        for(activeModeCnt = 0; activeModeCnt < ACT_MODE_TIME; activeModeCnt++);
        P1_0 = 1;

        ///////////////////////////////////////////////////////////////////////
        ////////// CC111xFx/CC251xFx Errata Note Code section Begin ///////////
        ///////////////////////////////////////////////////////////////////////

        // Store current DMA channel 0 descriptor and abort any ongoing transfers,
        // if the channel is in use.
        storedDescHigh = DMA0CFGH;
        storedDescLow = DMA0CFGL;
        DMAARM |= (DMAARM_ABORT | DMAARM0);

        // Update descriptor with correct source.
        dmaDesc[0] = (uint16)&PM2_BUF >> 8;
        dmaDesc[1] = (uint16)&PM2_BUF;
        // Associate the descriptor with DMA channel 0 and arm the DMA channel
        DMA0CFGH = (uint16)&dmaDesc >> 8;
        DMA0CFGL = (uint16)&dmaDesc;
        DMAARM = DMAARM0;

        // NOTE! At this point, make sure all interrupts that will not be used to
        // wake from PM are disabled as described in the "Power Management Control"
        // chapter of the data sheet.

        // The following code is timing critical and should be done in the
        // order as shown here with no intervening code.

        WORCTRL |= 0x05; //0x05 for 20 seconds delay
        
        // Align with positive 32 kHz clock edge as described in the
        // "Sleep Timer and Power Modes" chapter of the data sheet.
        temp = WORTIME0;
        while(temp == WORTIME0);

        temp = WORTIME0;
        while(temp == WORTIME0);
        
        // Set Sleep Timer Interval
        WOREVT1 = EVENT0_HIGH;
        WOREVT0 = EVENT0_LOW;

        temp = WORTIME0;
        while(temp == WORTIME0);
        
        // Make sure HS XOSC is powered down when entering PM{2 - 3} and that
        // the flash cache is disabled.
        MEMCTR |= MEMCTR_CACHD;
        SLEEP = 0x06;

        // Enter power mode as described in chapter "Power Management Control"
        // in the data sheet. Make sure DMA channel 0 is triggered just before
        // setting [PCON.IDLE].
        asm("NOP");
        asm("NOP");
        asm("NOP");
        
        if(SLEEP & 0x03)
        {
            asm("MOV 0xD7,#0x01");      // DMAREQ = 0x01;
            asm("NOP");                 // Needed to perfectly align the DMA transfer.
            asm("ORL 0x87,#0x01");      // PCON |= 0x01 -- Now in PM2;
            asm("NOP");                 // First call when awake
        }
        // End of timing critical code
        
        // Enable Flash Cache.
        MEMCTR &= ~MEMCTR_CACHD;

        // Update DMA channel 0 with original descriptor and arm channel if it was
        // in use before PM was entered.
        DMA0CFGH = storedDescHigh;
        DMA0CFGL = storedDescLow;
        DMAARM = DMAARM0;

        ///////////////////////////////////////////////////////////////////////
        /////////// CC111xFx/CC251xFx Errata Note Code section End ////////////
        ///////////////////////////////////////////////////////////////////////

        // Wait until HS RCOSC is stable
        while( !(SLEEP & SLEEP_HFRC_S) );

        // Set LS XOSC as the clock oscillator for the Sleep Timer (CLKCON.OSC32 = 0)
        CLKCON &= ~CLKCON_OSC32;

}

}

       


/*==== INTERRUPT SERVICE ROUTINES ============================================*/

/******************************************************************************
* @fn  rf_IRQ
*
* @brief
*      The only interrupt flag which throws this interrupt is the IRQ_DONE interrupt.
*      So this is the code which runs after a packet has been received or
*      transmitted.
*
* Parameters:
*
* @param  void
*
* @return void
*
******************************************************************************/
#pragma vector=RF_VECTOR
__interrupt void rf_IRQ(void) {
    RFIF &= ~IRQ_DONE;        // Tx/Rx completed, clear interrupt flag
    S1CON &= ~0x03;           // Clear the general RFIF interrupt registers
    pktSentFlag = TRUE;           
    
}

/*==== END OF FILE ==========================================================*/
