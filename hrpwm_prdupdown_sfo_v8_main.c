//###########################################################################
//
// FILE:    HRPWM_PrdUpDown_SFO_V8.c
//
// TITLE:   F2837xD Device HRPWM SFO V8 High-Resolution Period (Up-Down Count)
//          example
//
//###########################################################################

//
// Included Files
//
#include "F28x_Project.h"
#include "math.h"
#include "SFO_V8.h"
#include "ringbuffer.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "F2837xD_gpio.h"

//
// Defines

//Serial commands
//#define



#define PWM_CH            9        // # of PWM channels

//249 => 200kHz; 124 => 400kHz;333=>150kHz
#define PWM_PERIOD_TICKS  334       //required -1!

#define RX_BUFFER_SIZE 128
#define TX_BUFFER_SIZE 128

#define DEBUG_PIN 8

ringbuffer_t TXBufferStruct;
ringbuffer_t RXBufferStruct;

Uint16 TXBuffer[TX_BUFFER_SIZE];
Uint16 RXBuffer[RX_BUFFER_SIZE];
Uint16 command[16];


//
// Globals
//
int MEP_ScaleFactor; // Global variable used by the SFO library
                     // Result can be used for all HRPWM channels
                     // This variable is also copied to HRMSTEP
                     // register by SFO(0) function.


float32 phaseShiftPWM1to8  = 0; //0% - 25%; 0.2%steps; round downwards
float32 phaseShiftPWM2to7  = 0; //0% - 25% minus phaseShiftPWM1to3; 0.2%steps; round downwards

int16 updateDB = 0;
int16 deadBand = 4;
Uint16 status;
Uint16 adcEndOfConv = 0;
Uint16 pwmStartConversion = 0;
Uint16 newCommandLength = 0;
int i = 0;

float32 Kp = 0.001;
float32 Ki = 0.0001;
float32 K_scaling=36.6139;
float32 eSumNorm = 0;
float32 uOutTargetNorm = 2;//in V
float32 uOutNorm=0;
float32 uFbk=0;
float32 u_i=0;
float32 U_max=1;
float32 U_min=-5;

Uint16 ISRTicker=0;
float32 K1=0.998;
float32 K2=0.002;
float32 vout=0;
float32 vout1=0;
float32 Vout_DC=0;
float32 Vout1_DC=0;
float32 temp=0;
float32 temp1=0;
Uint16 power=0;


// Array of pointers to EPwm register structures:
// *ePWM[0] is defined as dummy value not used in the example
volatile struct EPWM_REGS *ePWM[PWM_CH] =
{  &EPwm1Regs, &EPwm1Regs, &EPwm2Regs, &EPwm3Regs, &EPwm4Regs, &EPwm5Regs,
   &EPwm6Regs, &EPwm7Regs, &EPwm8Regs};


// Function Prototypes
void HRPWM1_Config(int);
void HRPWM2_Config(int);
void HRPWM7_Config(int);
void HRPWM8_Config(int);
void HRPWMupdatePhases(float32 phaseShift1_3, float32 phaseShift3_4);
void HRPWMupdateDB(int deadBand);
void ConfigureADC(void);
interrupt void adca0_isr(void);
interrupt void sciaTxFifoIsr(void);
interrupt void sciaRxFifoIsr(void);
interrupt void DC_cal_isr(void);
void scia_fifo_init(void);

void error(void);


//
// Main
//
void main(void)
{
    // Initialize System Control for Control and Analog Subsystems
    // Enable Peripheral Clocks
    InitSysCtrl();

    //init debug pin
    InitGpio();
    GPIO_SetupPinMux(DEBUG_PIN, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(DEBUG_PIN, GPIO_OUTPUT, GPIO_PUSHPULL);


    // EPWM1A and EPWM1B through PWM1-3
    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm7Gpio();
    InitEPwm8Gpio();




    DINT; // Disable CPU interrupts
    // Initialize PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Map ISR functions
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.ADCB1_INT = &DC_cal_isr;        //function for ADC-B interrupt 1
    PieVectTable.SCIA_RX_INT = &sciaRxFifoIsr;  //ISR for RX interrupt
    PieVectTable.SCIA_TX_INT = &sciaTxFifoIsr;  //ISR for TX interrupt
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // configure Serial
    scia_fifo_init();

    // Configure the ADC and power it up
    ConfigureADC();



    // enable PIE interrupts
//  PieCtrlRegs.PIEIER1.bit.INTx1  = 1;     //enable interrupt for ADC A
    PieCtrlRegs.PIEIER1.bit.INTx2  = 1;     //enable interrupt for ADC B

    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;      // PIE Group 9, INT1, TX interrupt
    PieCtrlRegs.PIEIER9.bit.INTx2 = 1;      // PIE Group 9, INT2, RX interrupt

    IER |= M_INT9;                          // Enable CPU INT group 9 (RX/TX)
    IER |= M_INT1;                          // Enable CPU INT group 1 (ADC)



    // Enable global Interrupts and higher priority real-time debug events
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM




   status = SFO_INCOMPLETE;


    // Calling SFO() updates the HRMSTEP register with calibrated MEP_ScaleFactor.
    // HRMSTEP must be populated with a scale factor value prior to enabling
    // high resolution period control.
    //
    while(status == SFO_INCOMPLETE) // Call until complete
    {
        status = SFO();
        if (status == SFO_ERROR)
        {
            error();    // SFO function returns 2 if an error occurs & # of MEP
        }               // steps/coarse step exceeds maximum of 255.
    }


    EALLOW;
    InputXbarRegs.INPUT5SELECT = 0xA;   //Input5 is normally connected to GPIO0 which is PWM1 output-> every PWM cycle reset
                                        //use not as input
                                        //GPIO10 -> INPUT5 -> EXTSYNCIN1 -> Sync singal for PWM modules (S. 1737)

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;   // Disable TBCLK within the EPWM
    EDIS;
    //PWM frequence EPWMCLK = 100MHz (maximum)
    HRPWM1_Config(PWM_PERIOD_TICKS);   // ePWMx target
    HRPWM2_Config(PWM_PERIOD_TICKS);   // ePWMx target
    HRPWM8_Config(PWM_PERIOD_TICKS);   // ePWMx target
    HRPWM7_Config(PWM_PERIOD_TICKS);   // ePWMx target
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;      // Enable TBCLK within the EPWM
    EPwm1Regs.TBCTL.bit.SWFSYNC = 1;          // Synchronize high resolution phase to start HR period
    EDIS;

    //init RX/TX buffer
    TXBufferStruct.buffer = TXBuffer;
    RXBufferStruct.buffer = RXBuffer;
    TXBufferStruct.size = TX_BUFFER_SIZE;
    RXBufferStruct.size = RX_BUFFER_SIZE;
    ringbuffer_reset(&TXBufferStruct);
    ringbuffer_reset(&RXBufferStruct);

    HRPWMupdatePhases(phaseShiftPWM1to8, phaseShiftPWM2to7);
    for(;;)
    {

//sending Data example
        //ringbuffer_put(&TXBufferStruct, 'A');
        //ringbuffer_put(&TXBufferStruct, 'B');
        //SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending






        if(updateDB)
        {
            HRPWMupdateDB(deadBand);
            updateDB = 0;
        }






//        if (adcEndOfConv == 1)
//        {
//
//            //control loop for output voltage
//
//            float32 uOutNorm = AdcaResultRegs.ADCRESULT0 / 4096;
//            float32 e = uOutTargetNorm - uOutNorm;
//            eSumNorm += e;
//
//
//
//            int32 tempb = AdcbResultRegs.ADCRESULT0;
//
//            //calculate output
//            //set output
//
//
//
//            GPIO_WritePin(DEBUG_PIN, 0);
//            adcEndOfConv = 0;
//        }


        if(!ringbuffer_empty(&RXBufferStruct))
        {
            ringbuffer_get(&RXBufferStruct, &command[i]); //copx character
            if(command[i] == 0x000D)
            {
                newCommandLength = i;
                i = 0;
            }
            i++;
        }



        if(newCommandLength > 0)  //process new command
        {
            if(memcmp(command, "getData\n", newCommandLength) == 0)
            {
                //send data
                int aaa= 5;
            }

            if(memcmp(command, "getadcavalue\n", newCommandLength) == 0)
            {
                int32 tempa = AdcaResultRegs.ADCRESULT0;

                Uint16 str[5];


             //   sprintf(str, "%d", 1234);
                ringbuffer_put_array(&TXBufferStruct, "HALLO", 5);
                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //start sending

            }

            newCommandLength = 0;
        }



//        if(ringbuffer_length(&RXBufferStruct) >= 8)
 //       {
   //         int i;
 //           for(i = 0; i<8; i++)
 //           {
 //               ringbuffer_get(&RXBufferStruct, &TempBuffer[i]);
             //   strcmp
 //           }

 //       }


        DELAY_US(1000); //delay for 1ms to allow ADC time to power up



    } // end infinite for loop
}

void ConfigureADC(void)
{
    //TODO: 2 ADC input auf zwei ADC converter, nur ein interrupt für den zweiten gestarteten Interrupt?


    //Configure ADC A for input A0
    //Configure start of conversion block 0 (SOC0) to pin A0 (Datasheet S.92, Figure 2-2)
    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14; //sample window is 100 SYSCLK cycles

    //trigger ADC by PWM1
//    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // ADCTRIG5 - ePWM1, ADCSOCA


    // Setup the post-processing block 1 to be associated with SOC0 EOC0
    AdcaRegs.ADCPPB1CONFIG.bit.CONFIG = 0;

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
//    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag

    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4, Max ADCCLK is 50MHz -> /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 0; //Set pulse positions to end of conversion
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;    //power up the ADC

    //Configure ADC B for input B0
    //Configure start of conversion block 0 (SOC0) to pin A1 (Datasheet S.92, Figure 2-2)
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 1;  //SOC0 will convert pin B1
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 14; //sample window is 100 SYSCLK cycles
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // ADCTRIG5 - ePWM1, ADCSOCA

    // Setup the post-processing block 1 to be associated with SOC0 EOC0
    AdcbRegs.ADCPPB1CONFIG.bit.CONFIG = 0;

    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag

    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4, Max ADCCLK is 50MHz -> /4
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 0; //Set pulse positions to end of conversion
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;    //power up the ADC

    //Configure ADC C for input C2
    //Configure start of conversion block 0 (SOC0) to pin A1 (Datasheet S.92, Figure 2-2)
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 2;  //SOC0 will convert pin B1
    AdccRegs.ADCSOC0CTL.bit.ACQPS = 14; //sample window is 100 SYSCLK cycles
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // ADCTRIG5 - ePWM1, ADCSOCA

    // Setup the post-processing block 1 to be associated with SOC0 EOC0
    AdccRegs.ADCPPB1CONFIG.bit.CONFIG = 0;

    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag

    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4, Max ADCCLK is 50MHz -> /4
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 0; //Set pulse positions to end of conversion
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;    //power up the ADC

    DELAY_US(1000); //delay for 1ms to allow ADC time to power up
    EDIS;
}

//
// adca1_isr - ADCA1 Interrupt Service Routine
//
interrupt void adca0_isr(void)
{
    GPIO_WritePin(DEBUG_PIN, 1);
//    adcEndOfConv = 1;
	uFbk = (float32) AdcbResultRegs.ADCRESULT0 *K_scaling / 4096;
  if(Vout_DC>0.1 && power==1){
//	uFbk=uOutNorm*K_scaling;
	uOutTargetNorm=Vout_DC;
	float32 e = uOutTargetNorm - uFbk;
	u_i += e*Ki;
	 if (u_i > U_max) {
	  u_i = U_max;
	} else if (u_i < U_min) {
	  u_i = U_min;
	}
	float32 u_out=u_i+e*Kp;
	 if (u_out > U_max) {
	  u_out = U_max;
	} else if (u_out < U_min) {
	  u_out = U_min;
	}
	phaseShiftPWM1to8=u_out;
	}
  else{
	ISRTicker=0;
	EALLOW;  // This is needed to write to EALLOW protected registers
	PieVectTable.ADCB1_INT = &DC_cal_isr;        //function for ADC-B interrupt 1
	EDIS;    // This is needed to disable write to EALLOW protected registers
  }

  HRPWMupdatePhases(phaseShiftPWM1to8, phaseShiftPWM2to7);

	//calculate output
	//set output



	GPIO_WritePin(DEBUG_PIN, 0);

//    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag ADC A
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag ADC B
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}

interrupt void DC_cal_isr(void)
{   ISRTicker++;
	if(ISRTicker<5000){
	phaseShiftPWM1to8=0;
	vout = (float32) AdcbResultRegs.ADCRESULT0 *K_scaling / 4096;
	Vout_DC=K1*temp+K2*vout;
    temp=Vout_DC;
	phaseShiftPWM2to7=0;
	vout1 = (float32) AdccResultRegs.ADCRESULT0 *K_scaling / 4096;
	Vout1_DC=K1*temp1+K2*vout1;
    temp1=Vout1_DC;}
   if (ISRTicker>20000){
	EALLOW;  // This is needed to write to EALLOW protected registers
	PieVectTable.ADCB1_INT = &adca0_isr;        //function for ADC-B interrupt 1
	EDIS;    // This is needed to disable write to EALLOW protected registers
    }
   AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag ADC B
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


void HRPWMupdatePhases(float32 Phi, float32 dPhi)
{
    float32 phaseShift1_3 = PWM_PERIOD_TICKS/2 + 2*PWM_PERIOD_TICKS/100.0 * Phi -1;
    float32 phaseShift1_3_main = floor(phaseShift1_3);
    float32 phaseShift1_3_rest = phaseShift1_3 - phaseShift1_3_main;
    int16 phaseShiftMEP1_3 = 255 * phaseShift1_3_rest;

    EPwm8Regs.CMPA.bit.CMPA = phaseShift1_3_main;
    EPwm8Regs.CMPA.bit.CMPAHR = phaseShiftMEP1_3 <<8;
    EPwm8Regs.CMPB.bit.CMPBHR = phaseShiftMEP1_3 <<8;

    float32 phi1_4 = dPhi;
//    if(phi1_4 < 0)
//    {
//        phi1_4 = 0;
//    }

    float32 phaseShift1_4 = PWM_PERIOD_TICKS/2 + 2*PWM_PERIOD_TICKS/100.0 * phi1_4 ;
    float32 phaseShift1_4_main = floor(phaseShift1_4);
    float32 phaseShift1_4_rest = phaseShift1_4 - phaseShift1_4_main;
    int16 phaseShiftMEP1_4 = 255 * phaseShift1_4_rest;

    EPwm7Regs.CMPA.bit.CMPA = phaseShift1_4_main;
    EPwm7Regs.CMPA.bit.CMPAHR = phaseShiftMEP1_4 <<8;
    EPwm7Regs.CMPB.bit.CMPBHR = phaseShiftMEP1_4 <<8;
}

void HRPWMupdateDB(int DB)
{
    EPwm1Regs.DBRED.bit.DBRED = DB;
    EPwm1Regs.DBFED.bit.DBFED = DB;

    EPwm2Regs.DBRED.bit.DBRED = DB;
    EPwm2Regs.DBFED.bit.DBFED = DB;

    EPwm3Regs.DBRED.bit.DBRED = DB;
    EPwm3Regs.DBFED.bit.DBFED = DB;

    EPwm4Regs.DBRED.bit.DBRED = DB;
    EPwm4Regs.DBFED.bit.DBFED = DB;
}



//
// HRPWM_Config - Configures all ePWM channels and sets up HRPWM
//                on ePWMxA channels &  ePWMxB channels
//
void HRPWM1_Config(period)
{
    EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;  // set Shadow load
    EPwm1Regs.TBPRD = period;               // PWM frequency = 1/(2*TBPRD)
    EPwm1Regs.CMPA.bit.CMPA = period / 2;   // set duty 50% initially
    EPwm1Regs.CMPA.bit.CMPAHR = (1 << 8);   // initialize HRPWM extension
    EPwm1Regs.CMPB.bit.CMPB = period / 2;   // set duty 50% initially
    EPwm1Regs.CMPB.all |= 1;                // ?????????????????
    EPwm1Regs.TBPHS.all = 0;                // value which is loaded into TBCTR when sinc puls occur
    EPwm1Regs.TBCTR = 0;                    // set counter to 0

    //EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Select up-down count mode
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Select up-down count mode
    EPwm1Regs.TBCTL.bit.PHSEN = TB_ENABLE;         // to synchronize from EXTSYNCIN1
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;     //sync feed trough to PWM2
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // TBCLK = SYSCLKOUT
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 11;

    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // LOAD CMPA on CTR = 0
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;


   // EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;             // PWM toggle high/low
    EPwm1Regs.AQCTLA.bit.CAU = AQ_TOGGLE;             // PWM toggle high/low
 //   EPwm1Regs.AQCTLA.bit.CBD = AQ_CLEAR;
    //EPwm1Regs.AQCTLB.bit.CAU = AQ_CLEAR;         //PWM_B generated throu DB module
    //EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;

    //configure SOC for ADC A
    EPwm1Regs.ETSEL.bit.SOCASEL =  0x6;             //100: Enable event time-base counter equal to CMPA when the timer
                                                    //is incrementing or CMPC when the timer is incrementing
//    EPwm1Regs.ETSEL.bit.INTSEL = 0x4;               //Enable event time-base counter equal to CMPA when the timer
//                                                    //is incrementing or CMPC when the timer is incrementing
//    EPwm1Regs.ETSEL.bit.INTEN = 0x1;                //1: Enable EPWMx_INT generation

    EPwm1Regs.ETSEL.bit.SOCAEN = 0x1;               //Enable EPWMxSOCA pulse.
//    EPwm1Regs.ETPS.bit.INTPSSEL = 0;                //Selects ETPS [INTCNT, and INTPRD] registers to determine
//                                                    //frequency of events (interrupt once every 0-3 events).

//    EPwm1Regs.ETPS.bit.INTPRD = 0x1;                //01: Generate an interrupt on the first event INTCNT = 01 (first event)


    EPwm1Regs.ETPS.bit.SOCAPRD = 0x2;               //01: Generate the EPWMxSOCA pulse on the first event




    //
    // Active Low PWMs - Setup Deadband
    //
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm1Regs.DBCTL.bit.HALFCYCLE = 1;


    EPwm1Regs.DBRED.bit.DBRED = deadBand;
    EPwm1Regs.DBFED.bit.DBFED = deadBand;
}


void HRPWM2_Config(period)
{
    EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;  // set Shadow load
    EPwm2Regs.TBPRD = period;               // PWM frequency = 1/(2*TBPRD)
    EPwm2Regs.CMPA.bit.CMPA = period / 2 ;   // set duty 50% initially
    EPwm2Regs.CMPA.bit.CMPAHR = (1 << 8);   // initialize HRPWM extension
    EPwm2Regs.CMPB.bit.CMPB = period / 2 ;   // set duty 50% initially
    EPwm2Regs.CMPB.all |= 1;                // ?????????????????
    EPwm2Regs.TBPHS.all = 0;                // value which is loaded into TBCTR when sinc puls occur
    EPwm2Regs.TBCTR = 0;                    // set counter to 0

    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Select up-down count mode
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;         // sync signal from PWM1 enable
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;     //sync feed trough to PWM3
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // TBCLK = SYSCLKOUT
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 11;

    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // LOAD CMPA on CTR = 0
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;

    EPwm2Regs.AQCTLA.bit.CAU = AQ_TOGGLE;             // PWM toggle high/low
  //  EPwm2Regs.AQCTLA.bit.CBD = AQ_CLEAR;
//    EPwm2Regs.AQCTLB.bit.CAU = AQ_CLEAR;             // PWM toggle high/low
//    EPwm2Regs.AQCTLB.bit.CBD = AQ_SET;


    //
    // Active Low PWMs - Setup Deadband
    //
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.HALFCYCLE = 1;

    EPwm2Regs.DBRED.bit.DBRED = deadBand;
    EPwm2Regs.DBFED.bit.DBFED = deadBand;
}




void HRPWM8_Config(period)
{
    EPwm8Regs.TBCTL.bit.PRDLD = TB_SHADOW;  // set Shadow load
    EPwm8Regs.TBPRD = period;               // PWM frequency = 1/(2*TBPRD)
    EPwm8Regs.CMPA.bit.CMPA = period / 2;   // set duty 50% initially
    EPwm8Regs.CMPA.bit.CMPAHR = (1 << 8);   // initialize HRPWM extension
    EPwm8Regs.CMPB.bit.CMPB = period / 2;   // set duty 50% initially
    EPwm8Regs.CMPB.all |= 1;                // ?????????????????
    EPwm8Regs.TBPHS.all = 0;                // value which is loaded into TBCTR when sinc puls occur
    EPwm8Regs.TBCTR = 0;                    // set counter to 0

    EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Select up-down count mode
    EPwm8Regs.TBCTL.bit.PHSEN = TB_ENABLE;         // sync signal from PWM2 enable
    EPwm8Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;     //sync feed trough to PWM4 (not used)
    EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // TBCLK = SYSCLKOUT
    EPwm8Regs.TBCTL.bit.FREE_SOFT = 11;

    EPwm8Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // LOAD CMPA on CTR = 0
    EPwm8Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm8Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm8Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;

    EPwm8Regs.AQCTLA.bit.CAU = AQ_TOGGLE;             // PWM toggle high/low
 //   EPwm3Regs.AQCTLA.bit.CBD = AQ_CLEAR;
//    EPwm3Regs.AQCTLB.bit.CAU = AQ_CLEAR;             // PWM toggle high/low
//    EPwm3Regs.AQCTLB.bit.CBD = AQ_SET;

    //
    // Active Low PWMs - Setup Deadband
    //
    EPwm8Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm8Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm8Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm8Regs.DBCTL.bit.HALFCYCLE = 1;

    EPwm8Regs.DBRED.bit.DBRED = deadBand;
    EPwm8Regs.DBFED.bit.DBFED = deadBand;


    EALLOW;
    EPwm8Regs.HRCNFG.all = 0x0;
    EPwm8Regs.HRCNFG.bit.EDGMODE = HR_BEP;          // Rising on A
    EPwm8Regs.HRCNFG.bit.EDGMODEB = HR_BEP;         // Falling on B
    EPwm8Regs.HRCNFG.bit.CTLMODE = HR_CMP;          // CMPAHR and TBPRDHR HR control
    EPwm8Regs.HRCNFG.bit.CTLMODEB = HR_CMP;          // CMPBHR and TBPRDHR HR control
    EPwm8Regs.HRCNFG.bit.HRLOAD  = HR_CTR_ZERO_PRD; // load on CTR = 0 and CTR = TBPRD
    EPwm8Regs.HRCNFG.bit.HRLOADB  = HR_CTR_ZERO_PRD; // load on CTR = 0 and CTR = TBPRD
 //   EPwm3Regs.HRCNFG.bit.SWAPAB = 1;
    EPwm8Regs.HRCNFG.bit.AUTOCONV = 1;              // Enable autoconversion for HR period
    EPwm8Regs.HRPCTL.bit.TBPHSHRLOADE = 1;          // Enable TBPHSHR sync (required for updwn count HR control)
    EPwm8Regs.HRPCTL.bit.HRPE = 1;                  // Turn on high-resolution period control.
    EDIS;
}

void HRPWM7_Config(period)
{
    EPwm7Regs.TBCTL.bit.PRDLD = TB_SHADOW;  // set Shadow load
    EPwm7Regs.TBPRD = period;               // PWM frequency = 1/(2*TBPRD)
    EPwm7Regs.CMPA.bit.CMPA = period / 2;   // set duty 50% initially
    EPwm7Regs.CMPA.bit.CMPAHR = (1 << 8);   // initialize HRPWM extension
    EPwm7Regs.CMPB.bit.CMPB = period / 2;   // set duty 50% initially
    EPwm7Regs.CMPB.all |= 1;                // ?????????????????
    EPwm7Regs.TBPHS.all = 0;                // value which is loaded into TBCTR when sinc puls occur
    EPwm7Regs.TBCTR = 0;                    // set counter to 0

    EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Select up-down count mode
    EPwm7Regs.TBCTL.bit.PHSEN = TB_ENABLE;         // to synchronize from EXTSYNCIN1
    EPwm7Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;     //sync feed trough to PWM2
    EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // TBCLK = SYSCLKOUT
    EPwm7Regs.TBCTL.bit.FREE_SOFT = 11;

    EPwm7Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // LOAD CMPA on CTR = 0
    EPwm7Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm7Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm7Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;


    EPwm7Regs.AQCTLA.bit.CAU = AQ_TOGGLE;             // PWM toggle high/low
  //  EPwm4Regs.AQCTLA.bit.CBD = AQ_CLEAR;
    //EPwm4Regs.AQCTLB.bit.CAU = AQ_CLEAR;         //PWM_B generated throu DB module
    //EPwm4Regs.AQCTLB.bit.CBD = AQ_SET;

    //
    // Active Low PWMs - Setup Deadband
    //
    EPwm7Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm7Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm7Regs.DBCTL.bit.HALFCYCLE = 1;

    EPwm7Regs.DBRED.bit.DBRED = deadBand;
    EPwm7Regs.DBFED.bit.DBFED = deadBand;

    EALLOW;
    EPwm7Regs.HRCNFG.all = 0x0;
    EPwm7Regs.HRCNFG.bit.EDGMODE = HR_BEP;          // Rising on A
    EPwm7Regs.HRCNFG.bit.EDGMODEB = HR_BEP;         // Falling on B
    EPwm7Regs.HRCNFG.bit.CTLMODE = HR_CMP;          // CMPAHR and TBPRDHR HR control
    EPwm7Regs.HRCNFG.bit.CTLMODEB = HR_CMP;          // CMPBHR and TBPRDHR HR control
    EPwm7Regs.HRCNFG.bit.HRLOAD  = HR_CTR_ZERO_PRD; // load on CTR = 0 and CTR = TBPRD
    EPwm7Regs.HRCNFG.bit.HRLOADB  = HR_CTR_ZERO_PRD; // load on CTR = 0 and CTR = TBPRD
 //   EPwm4Regs.HRCNFG.bit.SWAPAB = 1;
    EPwm7Regs.HRCNFG.bit.AUTOCONV = 1;              // Enable autoconversion for HR period
    EPwm7Regs.HRPCTL.bit.TBPHSHRLOADE = 1;          // Enable TBPHSHR sync (required for updwn count HR control)
    EPwm7Regs.HRPCTL.bit.HRPE = 1;                  // Turn on high-resolution period control.
    EDIS;
}


//
// scia_fifo_init - Configure SCIA FIFO
//
void scia_fifo_init()
{


    GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC);


    EALLOW;
   CpuSysRegs.PCLKCR7.bit.SCI_A = 1; // enable clock


   SciaRegs.SCICCR.all = 0x0007;      // 1 stop bit,  No loopback
                                      // No parity,8 char bits,
                                      // async mode, idle-line protocol
   SciaRegs.SCICTL1.all = 0x0003;     // enable TX, RX, internal SCICLK,
                                      // Disable RX ERR, SLEEP, TXWAKE
   SciaRegs.SCICTL2.bit.TXINTENA = 1;
   SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
   SciaRegs.SCIHBAUD.all = 0x0000;    // 115200 baud @LSPCLK = 50MHz
   SciaRegs.SCILBAUD.all = 0x0035;    //(200 MHz SYSCLK).
   SciaRegs.SCICCR.bit.LOOPBKENA = 0; // Enable loop back


   SciaRegs.SCIFFRX.bit.RXFFIL = 1;       //Receive FIFO interrupt level bits, after how many bits interrupt
   SciaRegs.SCIFFRX.bit.RXFFIENA = 1;     //Receive FIFO interrupt enable
   SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;   //Receive FIFO interrupt clear
   //SciaRegs.SCIFFRX.bit.RXFFINT      //interrupt register, read only
   //SciaRegs.SCIFFRX.bit.RXFFST       //how many byte received, read only
   SciaRegs.SCIFFRX.bit.RXFIFORESET = 1; //Receive FIFO reset pointer to zero
   SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;   //clear overflow flag if set
   //SciaRegs.SCIFFRX.bit.RXFFOVF         //overflow bit, read only


   SciaRegs.SCIFFTX.bit.TXFFIL = 2;
   SciaRegs.SCIFFTX.bit.TXFFIENA = 0;
   SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;
 //  SciaRegs.SCIFFTX.bit.TXFFINT    interrupt flag
 //  SciaRegs.SCIFFTX.bit.TXFFST    how many byte the FIFO contains
   SciaRegs.SCIFFTX.bit.TXFIFORESET = 1;
   SciaRegs.SCIFFTX.bit.SCIFFENA = 1;
   SciaRegs.SCIFFTX.bit.SCIRST = 1;

   SciaRegs.SCIFFCT.all = 0x00;


   SciaRegs.SCIFFTX.bit.TXFIFORESET = 1;
   SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
   SciaRegs.SCICTL1.bit.SWRESET = 1;
   EDIS;
}


//
// sciaTxFifoIsr - SCIA Transmit FIFO ISR
//
interrupt void sciaTxFifoIsr(void)
{
    Uint16 data;

    for(;(SciaRegs.SCIFFTX.bit.TXFFST < 0xF) & (!ringbuffer_empty(&TXBufferStruct));)
    {
        ringbuffer_get(&TXBufferStruct, &data);
        SciaRegs.SCITXBUF.bit.TXDT = (Uint16)data;
    }

    if(ringbuffer_empty(&TXBufferStruct))           //if sending buffer is empty disable interrupt for empty TX buffer
    {
        SciaRegs.SCIFFTX.bit.TXFFIENA = 0;
    }

    SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;   // Clear SCI Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ACK
}

//
// sciaRxFifoIsr - SCIA Receive FIFO ISR
//
interrupt void sciaRxFifoIsr(void)
{
    uint16_t data;

    while((SciaRegs.SCIFFRX.bit.RXFFST > 0) & (!ringbuffer_full(&RXBufferStruct)))
    {
        /* Read data from RX FIFO */
        data = (uint16_t)SciaRegs.SCIRXBUF.bit.SAR;
        /* Write data to ring buffer */
        ringbuffer_put(&RXBufferStruct, data);
    }

    SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack
}


//
// error - Halt debugger when error occurs
//
void error (void)
{
    ESTOP0;         // Stop here and handle error
}

//
// End of file
//
