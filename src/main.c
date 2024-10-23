#include "sdk_project_config.h"
#include <interrupt_manager.h>
#include <stdint.h>
#include <stdbool.h>

// Karl Leiss - karl.leiss@fp-fahrzeugtechnik.de
// 2024

#define LED_RD          15U
#define LED_GN          16U

#define BTN1_PIN        13U
#define BTN2_PIN        12U


// H-bridge port for the 2 Infineon BTN9970LV
// H1_IN  	PTB5
// H1_INH 	PTB9
// H1_IS	PTB12(ADC1_SE7)
// H2_IN  	PTD14
// H2_INH 	PTB2
// H2_IS	PTD4(ADC1_SE6)

#define H1_IN           5U
#define H1_INH          9U
#define H1_IS           7U
#define H2_IN           14U
#define H2_INH          2U
#define H2_IS           6U
#define H_ADC_SAMPLES   6
#define H_UNLOCK_SHUTOFF_CNT   1200
#define H_LOCK_SHUTOFF_CNT   704

#define CAN_RX_ID		0x400
#define CAN_TX_ID		0x410

typedef enum
{
	STOP_REQUESTED = 0,
    LOCK_REQUESTED = 0x5,
    UNLOCK_REQUESTED = 0xA,
	RESET_REQUESTED = 0x66,
	UNLOCK_PGM_REQUESTED = 0x77,
	LOCK_PGM_REQUESTED = 0x88
} h_bridge_request;

typedef enum
{
	STOP_STATE = 0,
	RESET_STATE = 0x66,
    LOCKED_STATE = 0x5,
    UNLOCKED_STATE = 0xA,
} h_bridge_status;

/* Variable to store value from ADC conversion */
volatile uint16_t u16_h1_is[H_ADC_SAMPLES], u16_h2_is[H_ADC_SAMPLES], u16_adc_pot[H_ADC_SAMPLES];
volatile uint8_t u8_sample_idx, u8_pin_state, u8_h_bridge_request, u8_h_bridge_status;
volatile uint16_t u16_unlock_shutoff_is, u16_lock_shutoff_is;
uint16_t u16_h1_is_avg, u16_h2_is_avg;
uint8_t u8_alive_cnt;
volatile int exit_code = 0;

volatile uint32_t  RxCODE;              /* Received message buffer code */
volatile uint32_t  RxID;                /* Received message ID */
volatile uint32_t  RxLENGTH;            /* Received message number of data bytes */
volatile uint32_t  RxDATA[2];           /* Received message data (2 words) */
volatile uint32_t  RxTIMESTAMP;         /* Received message time */


/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void irq_port_c_pin(void);
void irq_can_0_rx(void);
void irq_lpit_0_tmr_0(void);
void irq_lpit_0_tmr_1(void);


void FLEXCAN0_init(void)
{
#define MSG_BUF_SIZE  4    /* Msg Buffer Size. (CAN 2.0AB: 2 hdr +  2 data= 4 words) */
  uint32_t   i=0;
  PCC->PCCn[PCC_FlexCAN0_INDEX] |= PCC_PCCn_CGC_MASK; /* CGC=1: enable clock to FlexCAN0 */
  CAN0->MCR |= CAN_MCR_MDIS_MASK;         /* MDIS=1: Disable module before selecting clock */
  CAN0->CTRL1 &= ~CAN_CTRL1_CLKSRC_MASK;  /* CLKSRC=0: Clock Source = oscillator (8 MHz) */
  CAN0->MCR &= ~CAN_MCR_MDIS_MASK;        /* MDIS=0; Enable module config. (Sets FRZ, HALT)*/
  while (!((CAN0->MCR & CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT))  {}
                 /* Good practice: wait for FRZACK=1 on freeze mode entry/exit */
  CAN0->CTRL1 = 0x00DB0006; /* Configure for 500 KHz bit time */
                            /* Time quanta freq = 16 time quanta x 500 KHz bit time= 8MHz */
                            /* PRESDIV+1 = Fclksrc/Ftq = 8 MHz/8 MHz = 1 */
                            /*    so PRESDIV = 0 */
                            /* PSEG2 = Phase_Seg2 - 1 = 4 - 1 = 3 */
                            /* PSEG1 = PSEG2 = 3 */
                            /* PROPSEG= Prop_Seg - 1 = 7 - 1 = 6 */
                            /* RJW: since Phase_Seg2 >=4, RJW+1=4 so RJW=3. */
                            /* SMP = 1: use 3 bits per CAN sample */
                            /* CLKSRC=0 (unchanged): Fcanclk= Fosc= 8 MHz */
  for(i=0; i<128; i++ ) {   /* CAN0: clear 32 msg bufs x 4 words/msg buf = 128 words*/
    CAN0->RAMn[i] = 0;      /* Clear msg buf word */
  }
  for(i=0; i<16; i++ ) {          /* In FRZ mode, init CAN0 16 msg buf filters */
    CAN0->RXIMR[i] = 0xFFFFFFFF;  /* Check all ID bits for incoming messages */
  }
  CAN0->RXMGMASK = 0x1FFFFFFF;  /* Global acceptance mask: check all ID bits */
  CAN0->RAMn[ 4*MSG_BUF_SIZE + 0] = 0x04000000; /* Msg Buf 4, word 0: Enable for reception */
                                                /* EDL,BRS,ESI=0: CANFD not used */
                                                /* CODE=4: MB set to RX inactive */
                                                /* IDE=0: Standard ID */
                                                /* SRR, RTR, TIME STAMP = 0: not applicable */
                                        /* Node B to receive msg with std ID 0x555 */
  CAN0->RAMn[ 4*MSG_BUF_SIZE + 1] = CAN_RX_ID << 18; /* Msg Buf 4, word 1: Standard ID  */

                                                /* PRIO = 0: CANFD not used */
  CAN0->MCR = 0x0000001F;       /* Negate FlexCAN 1 halt state for 32 MBs */
  while ((CAN0->MCR && CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT)  {}
                 /* Good practice: wait for FRZACK to clear (not in freeze mode) */
  while ((CAN0->MCR && CAN_MCR_NOTRDY_MASK) >> CAN_MCR_NOTRDY_SHIFT)  {}
                 /* Good practice: wait for NOTRDY to clear (module ready)  */

  // Activate CAN 0 MB 4 IRQ
  CAN0->IMASK1 = 0x00000010;
}

void FLEXCAN0_transmit_msg(void)
{ /* Assumption:  Message buffer CODE is INACTIVE */
  CAN0->IFLAG1 = 0x00000001;       /* Clear CAN 0 MB 0 flag without clearing others*/

  /* MB0 word 3: data word 1 */
  if(u8_h_bridge_status == UNLOCKED_STATE)
  {
	  CAN0->RAMn[ 0*MSG_BUF_SIZE + 3] = u16_unlock_shutoff_is << 16 | u8_h_bridge_status << 8 | (( u8_pin_state << 4) & 0xF0) | (u8_alive_cnt & 0xF);
  }
  else if(u8_h_bridge_status == LOCKED_STATE)
  {
	  CAN0->RAMn[ 0*MSG_BUF_SIZE + 3] = u16_lock_shutoff_is << 16 | u8_h_bridge_status << 8 | (( u8_pin_state << 4) & 0xF0) | (u8_alive_cnt & 0xF);
  }
  else
  {
	  CAN0->RAMn[ 0*MSG_BUF_SIZE + 3] =  u8_h_bridge_status << 8 | (( u8_pin_state << 4) & 0xF0) | (u8_alive_cnt & 0xF);
  }

  /* MB0 word 2: data word 0 */
  CAN0->RAMn[ 0*MSG_BUF_SIZE + 2] = u16_h2_is[u8_sample_idx - 1] << 16 | u16_h1_is[u8_sample_idx - 1];

  // MB0 word 1: Tx msg with STD ID
  CAN0->RAMn[ 0*MSG_BUF_SIZE + 1] = CAN_TX_ID << 18;
  CAN0->RAMn[ 0*MSG_BUF_SIZE + 0] = 0x0C400000 | 8 <<CAN_WMBn_CS_DLC_SHIFT; /* MB0 word 0: */
                                                /* EDL,BRS,ESI=0: CANFD not used */
                                                /* CODE=0xC: Activate msg buf to transmit */
                                                /* IDE=0: Standard ID */
                                                /* SRR=1 Tx frame (not req'd for std ID) */
                                                /* RTR = 0: data, not remote tx request frame*/
                                                /* DLC = 8 bytes */
}


void FLEXCAN0_receive_msg(void)
{
  uint32_t i;

  RxCODE   = (CAN0->RAMn[ 4*MSG_BUF_SIZE + 0] & 0x07000000) >> 24;  /* Read CODE field */
  RxID     = (CAN0->RAMn[ 4*MSG_BUF_SIZE + 1] & CAN_WMBn_ID_ID_MASK) >> 18;
  RxLENGTH =(CAN0->RAMn[ 4*MSG_BUF_SIZE + 0] & CAN_WMBn_CS_DLC_MASK) >> CAN_WMBn_CS_DLC_SHIFT;
  for (i=0; i<2; i++) {  /* Read two words of data (8 bytes) */
    RxDATA[i] = CAN0->RAMn[ 4*MSG_BUF_SIZE + 2 + i];
  }
  RxTIMESTAMP = (CAN0->RAMn[ 0*MSG_BUF_SIZE + 0] & 0x000FFFF);
  i = CAN0->TIMER;             /* Read TIMER to unlock message buffers */
  CAN0->IFLAG1 = 0x00000010;       /* Clear CAN 0 MB 4 flag without clearing others*/
}

/******************************************************************************
 * Functions
 ******************************************************************************/

/**
 * Button interrupt handler
 */
void irq_port_c_pin(void)
{
    /* Check if one of the buttons was pressed */
    uint32_t buttonsPressed = PINS_DRV_GetPortIntFlag(PORTC) &
                                           ((1 << BTN1_PIN) | (1 << BTN2_PIN));

	// stop LPIT to be able to re trigger
	LPIT0->CLRTEN = 0x3;

    if(buttonsPressed != 0)
    {

    	PINS_DRV_SetPins(PTD, (1 << LED_GN));

        // shutoff LED after period
        LPIT0->SETTEN = 0x2;

        /* Set FlexCAN TX value according to the button pressed */
        switch (buttonsPressed)
        {
            case (1 << BTN1_PIN):
				u8_h_bridge_request = LOCK_REQUESTED;
                /* Clear interrupt flag */
                PINS_DRV_ClearPinIntFlagCmd(PORTC, BTN1_PIN);

                // H bridges off
                PINS_DRV_ClearPins(PTD, (1 << H2_IN));

                // spin right
                PINS_DRV_SetPins(PTB, (1 << H1_IN));
                break;

            case (1 << BTN2_PIN):
				u8_h_bridge_request = UNLOCK_REQUESTED;
                /* Clear interrupt flag */
                PINS_DRV_ClearPinIntFlagCmd(PORTC, BTN2_PIN);

                // H bridges off
                PINS_DRV_ClearPins(PTB, (1 << H1_IN));

                // spin right
                PINS_DRV_SetPins(PTD, (1 << H2_IN));

                // start timer to shut down after specified period for motor protection purpose
                LPIT0->SETTEN = 0x1;
                break;

            default:
            	u8_h_bridge_request = STOP_REQUESTED;
            	u8_h_bridge_status = STOP_STATE;

                PINS_DRV_ClearPortIntFlagCmd(PORTC);
                // stop h bridges
                PINS_DRV_ClearPins(PTB, (1 << H1_INH));
                PINS_DRV_ClearPins(PTB, (1 << H2_INH));
                break;
        }
    }
}


void irq_lpit_0_tmr_0(void)
{
	LPIT0->MSR = LPIT_MSR_TIF0_MASK; /* Clear LPIT0 timer flag 0 */

	u8_h_bridge_status = UNLOCKED_STATE;

    // H bridges off
    PINS_DRV_ClearPins(PTB, (1 << H1_IN));
    PINS_DRV_ClearPins(PTD, (1 << H2_IN));
}

void irq_lpit_0_tmr_1(void)
{
	LPIT0->MSR = LPIT_MSR_TIF1_MASK; /* Clear LPIT0 timer flag 0 */

	PINS_DRV_ClearPins(PTD, (1 << LED_GN));
}

void irq_can_0_rx(void)
{
	FLEXCAN0_receive_msg();

	PINS_DRV_SetPins(PTD, (1 << LED_GN));

	// stop LPIT to be able to re trigger
	LPIT0->CLRTEN = 0x3;

    /* Check the received message ID and payload */
    if((RxDATA[0] == UNLOCK_REQUESTED) && RxID == CAN_RX_ID)
    {
    	u8_h_bridge_request = UNLOCK_REQUESTED;

        // H bridges off
        PINS_DRV_ClearPins(PTB, (1 << H1_IN));

        // spin right
        PINS_DRV_SetPins(PTD, (1 << H2_IN));

        // start timer to shut down after specified period for motor protection purpose
        LPIT0->SETTEN = 0x1;
    }
    else if((RxDATA[0] == LOCK_REQUESTED) && RxID == CAN_RX_ID)
    {
    	u8_h_bridge_request = LOCK_REQUESTED;

        // H bridges off
        PINS_DRV_ClearPins(PTD, (1 << H2_IN));

        // spin right
        PINS_DRV_SetPins(PTB, (1 << H1_IN));
    }
    else if((RxDATA[0] == RESET_REQUESTED) && RxID == CAN_RX_ID)
    {
    	u8_h_bridge_request = RESET_REQUESTED;
    	u8_h_bridge_status = RESET_STATE;

        // H bridges off
        PINS_DRV_ClearPins(PTB, (1 << H1_IN));
        PINS_DRV_ClearPins(PTD, (1 << H2_IN));

        // allow H bridges to work
        PINS_DRV_SetPins(PTB, (1 << H1_INH));
        PINS_DRV_SetPins(PTB, (1 << H2_INH));
    }
    else if((RxDATA[0] == UNLOCK_PGM_REQUESTED) && RxID == CAN_RX_ID)
    {
    	u8_h_bridge_request = RESET_REQUESTED;
    	u8_h_bridge_status = RESET_STATE;

    	u16_unlock_shutoff_is = RxDATA[1] & 0xFFF;

        // H bridges off
        PINS_DRV_ClearPins(PTB, (1 << H1_IN));
        PINS_DRV_ClearPins(PTD, (1 << H2_IN));

        // allow H bridges to work
        PINS_DRV_SetPins(PTB, (1 << H1_INH));
        PINS_DRV_SetPins(PTB, (1 << H2_INH));
    }
    else if((RxDATA[0] == LOCK_PGM_REQUESTED) && RxID == CAN_RX_ID)
    {
    	u8_h_bridge_request = RESET_REQUESTED;
    	u8_h_bridge_status = RESET_STATE;

    	u16_lock_shutoff_is = RxDATA[1] & 0xFFF;

        // H bridges off
        PINS_DRV_ClearPins(PTB, (1 << H1_IN));
        PINS_DRV_ClearPins(PTD, (1 << H2_IN));

        // allow H bridges to work
        PINS_DRV_SetPins(PTB, (1 << H1_INH));
        PINS_DRV_SetPins(PTB, (1 << H2_INH));
    }

    else
    {
    	u8_h_bridge_request = STOP_REQUESTED;
    	u8_h_bridge_status = STOP_STATE;

        // H bridges off
        PINS_DRV_ClearPins(PTB, (1 << H1_IN));
        PINS_DRV_ClearPins(PTD, (1 << H2_IN));
    }

    // shutoff LED after period
    LPIT0->SETTEN = 0x2;
}


int main(void)
 {
	uint32_t i;
	uint32_t u32_tmp,u32_tmp2;

	u8_alive_cnt = 0;
	u8_h_bridge_request = STOP_REQUESTED;
	u8_h_bridge_status = STOP_STATE;
	u16_unlock_shutoff_is = H_UNLOCK_SHUTOFF_CNT;
	u16_lock_shutoff_is = H_LOCK_SHUTOFF_CNT;

    CLOCK_DRV_Init(&clockMan1_InitConfig0);

    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS0, g_pin_mux_InitConfigArr0);

    WDOG->CNT = 0xD928C520; //unlock watchdog
    while((WDOG->CS & WDOG_CS_ULK_MASK)==0);  //wait until registers are unlocked
    WDOG->TOVAL = 13840; //set timeout value, based on LPO 128kHz clock, 108ms timeout
    WDOG->CS = WDOG_CS_EN(1) | WDOG_CS_CLK(1) | WDOG_CS_INT(0) |
              WDOG_CS_WIN(0) | WDOG_CS_UPDATE(0);
    while((WDOG->CS & WDOG_CS_RCS_MASK) ==0); //wait until new configuration takes effect

	/*
    PINS_DRV_SetPinsDirection(PTD, (1 << LED1) | (1 << LED0) | (1 << H2_IN));
    PINS_DRV_SetPinsDirection(PTB, (1 << H1_INH) | (1 << H1_IN) | (1 << H2_INH));
    PINS_DRV_SetPins(PTD, (1 << LED1) | (1 << LED0));
    PINS_DRV_SetPinsDirection(PTC, ~((1 << BTN1_PIN)|(1 << BTN2_PIN)));
    */
    /* Setup button pins interrupt */
    PINS_DRV_SetPinIntSel(PORTC, BTN1_PIN, PORT_INT_RISING_EDGE);
    PINS_DRV_SetPinIntSel(PORTC, BTN2_PIN, PORT_INT_RISING_EDGE);

    INT_SYS_InstallHandler(PORTC_IRQn, &irq_port_c_pin, NULL);


    FLEXCAN0_init();
    INT_SYS_InstallHandler(CAN0_ORed_0_15_MB_IRQn, &irq_can_0_rx, NULL);

    PCC->PCCn[PCC_LPIT_INDEX] &=~ PCC_PCCn_CGC_MASK;     /* Disable clock to change PCS */
    PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(2);    /* Clock Src = 6 (SIRC_DIV2_CLK) 8MHz */
    PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable clk to LPIT0 regs */
    LPIT0->MCR = 0x00000001;    /* DBG_EN-0: Timer chans stop in Debug mode */
                                /* DOZE_EN=0: Timer chans are stopped in DOZE mode */
                                /* SW_RST=0: SW reset does not reset timer chans, regs */
                                /* M_CEN=1: enable module clk (allow writing other LPIT0 regs) */
    LPIT0->MIER = 0x3;   /* TIE0=1: Timer Interrupt Enabled fot Chan 0 */
    LPIT0->TMR[0].TVAL = 18400000;    /* Chan 0 Timeout period: 2,5s timeout */
    LPIT0->TMR[1].TVAL = 3200000;    /* Chan 1 Timeout period: 400ms timeout */
    /* T_EN=1: Timer channel is enabled */
    /* CHAIN=0: channel chaining is disabled */
    /* MODE=0: 32 periodic counter mode */
    /* TSOT=0: Timer decrements immediately based on restart */
    /* TSOI=1: Timer does stop after timeout */
    /* TROT=0 Timer will not reload on trigger */
    /* TRG_SRC=0: External trigger source */
    /* TRG_SEL=0: Timer chan 0 trigger source is selected*/
    LPIT0->TMR[0].TCTRL = LPIT_TMR_TCTRL_TROT(0) | LPIT_TMR_TCTRL_TSOI(1) | LPIT_TMR_TCTRL_TSOT(0) | LPIT_TMR_TCTRL_T_EN(0);
    LPIT0->TMR[1].TCTRL = LPIT_TMR_TCTRL_TROT(0) | LPIT_TMR_TCTRL_TSOI(1) | LPIT_TMR_TCTRL_TSOT(0) | LPIT_TMR_TCTRL_T_EN(0);
	INT_SYS_InstallHandler(LPIT0_Ch0_IRQn, &irq_lpit_0_tmr_0, NULL);
	INT_SYS_InstallHandler(LPIT0_Ch1_IRQn, &irq_lpit_0_tmr_1, NULL);

    // activate green and red led
    PINS_DRV_SetPins(PTD, (1 << LED_RD) | (1 << LED_GN));

    /************************************************
     * Calibrate ADC0
     ***********************************************/
    PCC->PCCn[PCC_ADC1_INDEX] &=~ PCC_PCCn_CGC_MASK;     /* Disable clock to change PCS */
    PCC->PCCn[PCC_ADC1_INDEX] |= PCC_PCCn_PCS(3); /* PCS = 3: Select FIRCDIV2 */
    PCC->PCCn[PCC_ADC1_INDEX] |= PCC_PCCn_CGC_MASK;     /* Enable bus clock in ADC */

    ADC1->SC3 = ADC_SC3_CAL_MASK /* CAL = 1: Start calibration sequence */
    		| ADC_SC3_AVGE_MASK /* AVGE = 1: Enable hardware average */
			| ADC_SC3_AVGS(3); /* AVGS = 11b: 32 samples averaged */

    /* Wait for completion */
    while(((ADC1->SC1[0] & ADC_SC1_COCO_MASK)>>ADC_SC1_COCO_SHIFT) == 0);

    /************************************************
     * Initialize ADC0:
     * External channel 12, hardware trigger,
     * single conversion, 12-bit resolution
     *
     * NOTE: ADC0->SC1[4] corresponds to ADC0_SC1E register
     ***********************************************/
    ADC1->SC1[0] = ADC_SC1_ADCH_MASK; /* ADCH = 1F: Module is disabled for conversions*/
    ADC1->SC1[1] = ADC_SC1_ADCH_MASK;

    ADC1->CFG1 = ADC_CFG1_ADIV(0) | ADC_CFG1_MODE(1);    /* ADIV = 0: Divide ratio = 1 */
    /* MODE = 1: 12-bit conversion */

    ADC1->CFG2 = ADC_CFG2_SMPLTS(12); /* SMPLTS = 12: sample time is 13 ADC clks */

    ADC1->SC2 = ADC_SC2_ADTRG(1); /* ADTRG = 1: HW trigger */

    ADC1->SC1[0] = ADC_SC1_ADCH(H1_IS);
    ADC1->SC1[1] = ADC_SC1_ADCH(H2_IS);


    ADC1->SC3 = 0x00000000; /* ADCO = 0: One conversion performed */
    /* AVGE,AVGS = 0: HW average function disabled */


    PCC->PCCn[PCC_PDB1_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable bus clock in PDB */

    PDB1->SC = PDB_SC_PRESCALER(6)  /* PRESCALER = 6: clk divided by (64 x Mult factor) */
    | PDB_SC_TRGSEL(15) /* TRGSEL = 15: Software trigger selected */
    | PDB_SC_MULT(3) /* MULT = 3: Multiplication factor is 40 */
    | PDB_SC_CONT_MASK; /* CONT = 1: Enable operation in continuous mode */

       /* PDB Period = (System Clock / (Prescaler x Mult factor)) / Modulus */
       /* PDB Period = (48 MHz / (64 x 40)) / 18750 */
       /* PDB Period = (18750 Hz) / (1875) = 10 Hz */
    PDB1->MOD = 1875;

    PDB1->CH[0].C1 = (PDB_C1_BB(0x2) /* BB = E0h: Back-to-back for pre-triggers  */
    | PDB_C1_TOS(0x1) /* TOS = 10h: Pre-trigger 0 asserts with DLY match */
    | PDB_C1_EN(0x3)); /* EN = F0h: Pre-triggers 0/1 enabled */

    PDB1->CH[0].DLY[0] = 100; /* Delay set to half the PDB period = 9375 */

    PDB1->SC |= PDB_SC_PDBEN_MASK | PDB_SC_LDOK_MASK;    /* Enable PDB. Load MOD and DLY */

    PDB1->SC |= PDB_SC_SWTRIG_MASK; /* Single initial PDB trigger */

    u8_sample_idx = 0;

    // clear red led when we reached this point
    PINS_DRV_ClearPins(PTD, (1 << LED_RD));

    // H bridges off
    PINS_DRV_ClearPins(PTB, (1 << H1_IN));
    PINS_DRV_ClearPins(PTD, (1 << H2_IN));

    // allow H bridges to work
    PINS_DRV_SetPins(PTB, (1 << H1_INH));
    PINS_DRV_SetPins(PTB, (1 << H2_INH));

    u8_h_bridge_request = RESET_REQUESTED;


    INT_SYS_EnableIRQ(PORTC_IRQn);
    INT_SYS_EnableIRQ(CAN0_ORed_0_15_MB_IRQn);
    INT_SYS_EnableIRQ(LPIT0_Ch0_IRQn);
    INT_SYS_EnableIRQ(LPIT0_Ch1_IRQn);

    PINS_DRV_ClearPins(PTD, (1 << LED_GN));

    while(1)
    {
        /* Wait for last conversion in the sequence to complete (ADC0_SC1H) */
         while(((ADC1->SC1[1] & ADC_SC1_COCO_MASK)>>ADC_SC1_COCO_SHIFT) == 0);

         if(u8_sample_idx > (H_ADC_SAMPLES - 1))
        	 u8_sample_idx = 0;

         u16_h1_is[u8_sample_idx] = ADC1->R[0];
         u16_h2_is[u8_sample_idx] = ADC1->R[1];
         u8_sample_idx++;

         u32_tmp = 0;
         u32_tmp2 = 0;
         for(i=0; i<H_ADC_SAMPLES; i++)
         {
        	 u32_tmp += u16_h1_is[i];
        	 u32_tmp2 += u16_h2_is[i];
         }
         u16_h1_is_avg = u32_tmp/H_ADC_SAMPLES;
         u16_h2_is_avg = u32_tmp2/H_ADC_SAMPLES;


         if(u16_h1_is_avg > u16_lock_shutoff_is)
         {
        	 u8_h_bridge_status = LOCKED_STATE;
        	 PINS_DRV_ClearPins(PTB, (1 << H1_IN));
         }

         if(u16_h2_is_avg > u16_unlock_shutoff_is)
         {
        	 u8_h_bridge_status = UNLOCKED_STATE;
        	 PINS_DRV_ClearPins(PTD, (1 << H2_IN));
         }

         if(PINS_DRV_ReadPins(PTD) & (1 << H2_IN))
         {
        	 u8_pin_state |= 0x2;
         }
         else
         {
        	 u8_pin_state &= ~0x2;
         }

         if(PINS_DRV_ReadPins(PTB) & (1 << H1_IN))
         {
        	 u8_pin_state |= 0x1;
         }
         else
         {
        	 u8_pin_state &= ~0x1;
         }

         u8_alive_cnt++;

		 if(u8_alive_cnt > 15)
			 u8_alive_cnt = 0;

         FLEXCAN0_transmit_msg();

         WDOG->CNT = 0xA602;
         WDOG->CNT = 0xB480;
    }

    for(;;)
    {
      if(exit_code != 0)
      {
        break;
      }
    }
    return exit_code;
}

