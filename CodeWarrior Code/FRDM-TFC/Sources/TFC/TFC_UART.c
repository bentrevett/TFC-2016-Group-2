#include "TFC\TFC.h"

void uart0_init (int sysclk, int baud);
void uart2_init (int sysclk, int baud);

ByteQueue SDA_SERIAL_OUTGOING_QUEUE;
ByteQueue SDA_SERIAL_INCOMING_QUEUE;


uint8_t SDA_SERIAL_OUTGOING_QUEUE_Storage[SDA_SERIAL_OUTGOING_QUEUE_SIZE];
uint8_t SDA_SERIAL_INCOMING_QUEUE_Storage[SDA_SERIAL_INCOMING_QUEUE_SIZE];


void TFC_InitUARTs(uint32_t uart0_baud, uint32_t uart2_baud)
{
	SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK|SIM_SCGC5_PORTD_MASK);

	InitByteQueue(&SDA_SERIAL_OUTGOING_QUEUE,SDA_SERIAL_OUTGOING_QUEUE_SIZE,SDA_SERIAL_OUTGOING_QUEUE_Storage);
	InitByteQueue(&SDA_SERIAL_INCOMING_QUEUE,SDA_SERIAL_INCOMING_QUEUE_SIZE,SDA_SERIAL_INCOMING_QUEUE_Storage);
	
	PORTA_PCR1 = PORT_PCR_MUX(2) | PORT_PCR_DSE_MASK;   
	PORTA_PCR2 = PORT_PCR_MUX(2) | PORT_PCR_DSE_MASK;
	
	PORTD_PCR2 = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK; 
	PORTD_PCR3 = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK; 
	
	//Select PLL/2 Clock
	SIM_SOPT2 &= ~(3<<26);
	SIM_SOPT2 |= SIM_SOPT2_UART0SRC(1); 
	SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;
	
	//We have to feed this function the clock in KHz!
     uart0_init (CORE_CLOCK/2/1000, uart0_baud);
     uart2_init (CORE_CLOCK/2/1000, uart2_baud);
     
     //Enable recieve interrupts
     UART0_C2 |= UART_C2_RIE_MASK;
     enable_irq(INT_UART0-16);
     
     // TODO: Setup UART_2 INTERRUPT if receiver needed
     //
	
}


void TFC_UART_Process()
{
	if(BytesInQueue(&SDA_SERIAL_OUTGOING_QUEUE)>0 && (UART0_S1 & UART_S1_TDRE_MASK))
			UART0_C2 |= UART_C2_TIE_MASK; //Enable Transmitter Interrupts
}


void UART0_IRQHandler()
{
	uint8_t Temp;
		
	if(UART0_S1 & UART_S1_RDRF_MASK)
	{
		ByteEnqueue(&SDA_SERIAL_INCOMING_QUEUE,UART0_D);
	}
	if(UART0_S1 & UART_S1_TDRE_MASK)
	{
		if(BytesInQueue(&SDA_SERIAL_OUTGOING_QUEUE)>0)
		{
			ByteDequeue(&SDA_SERIAL_OUTGOING_QUEUE,&Temp);
			UART0_D = Temp;
		}
		else
		{
			//if there is nothing left in the queue then disable interrupts
			UART0_C2 &= ~UART_C2_TIE_MASK; //Disable the  Interrupts
		}
	}
}



void uart0_init (int sysclk, int baud)
{
    uint8 i;
    uint32 calculated_baud = 0;
    uint32 baud_diff = 0;
    uint32 osr_val = 0;
    uint32 sbr_val, uart0clk;
    uint32 baud_rate;
    uint32 reg_temp = 0;
    uint32 temp = 0;
    
    SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
    
    // Disable UART0 before changing registers
    UART0_C2 &= ~(UART0_C2_TE_MASK | UART0_C2_RE_MASK);
  
    // Verify that a valid clock value has been passed to the function 
    if ((sysclk > 50000) || (sysclk < 32))
    {
        sysclk = 0;
        reg_temp = SIM_SOPT2;
        reg_temp &= ~SIM_SOPT2_UART0SRC_MASK;
        reg_temp |= SIM_SOPT2_UART0SRC(0);
        SIM_SOPT2 = reg_temp;
			
			  // Enter inifinite loop because the 
			  // the desired system clock value is 
			  // invalid!!
			  while(1)
				{}
    }
   
    
    // Initialize baud rate
    baud_rate = baud;
    
    // Change units to Hz
    uart0clk = sysclk * 1000;
    // Calculate the first baud rate using the lowest OSR value possible.  
    i = 4;
    sbr_val = (uint32)(uart0clk/(baud_rate * i));
    calculated_baud = (uart0clk / (i * sbr_val));
        
    if (calculated_baud > baud_rate)
        baud_diff = calculated_baud - baud_rate;
    else
        baud_diff = baud_rate - calculated_baud;
    
    osr_val = i;
        
    // Select the best OSR value
    for (i = 5; i <= 32; i++)
    {
        sbr_val = (uint32)(uart0clk/(baud_rate * i));
        calculated_baud = (uart0clk / (i * sbr_val));
        
        if (calculated_baud > baud_rate)
            temp = calculated_baud - baud_rate;
        else
            temp = baud_rate - calculated_baud;
        
        if (temp <= baud_diff)
        {
            baud_diff = temp;
            osr_val = i; 
        }
    }
    
    if (baud_diff < ((baud_rate / 100) * 3))
    {
        // If the OSR is between 4x and 8x then both
        // edge sampling MUST be turned on.  
        if ((osr_val >3) && (osr_val < 9))
            UART0_C5|= UART0_C5_BOTHEDGE_MASK;
        
        // Setup OSR value 
        reg_temp = UART0_C4;
        reg_temp &= ~UART0_C4_OSR_MASK;
        reg_temp |= UART0_C4_OSR(osr_val-1);
    
        // Write reg_temp to C4 register
        UART0_C4 = reg_temp;
        
        reg_temp = (reg_temp & UART0_C4_OSR_MASK) + 1;
        sbr_val = (uint32)((uart0clk)/(baud_rate * (reg_temp)));
        
         /* Save off the current value of the uartx_BDH except for the SBR field */
        reg_temp = UART0_BDH & ~(UART0_BDH_SBR(0x1F));
   
        UART0_BDH = reg_temp |  UART0_BDH_SBR(((sbr_val & 0x1F00) >> 8));
        UART0_BDL = (uint8)(sbr_val & UART0_BDL_SBR_MASK);
        
        /* Enable receiver and transmitter */
        UART0_C2 |= (UART0_C2_TE_MASK
                    | UART0_C2_RE_MASK );
    }
    else
		{
        // Unacceptable baud rate difference
        // More than 3% difference!!
        // Enter infinite loop!
        //while(1)
			//	{}
		}					
    
}

void UART2_IRQHandler()
{
	// Needs Setup
}

void uart2_init (int sysclk, int baud)
{
    uint8 i;
    uint32 calculated_baud = 0;
    uint32 baud_diff = 0;
    uint32 sbr_val, uart2clk;
    uint32 baud_rate;
    uint32 reg_temp = 0;
    
    SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;
    
    // Disable UART2 before changing registers
    UART2_C2 &= ~(UART0_C2_TE_MASK | UART0_C2_RE_MASK);
  
    // Verify that a valid clock value has been passed to the function 
    if ((sysclk > 50000) || (sysclk < 32))
    {		
			  // Enter inifinite loop because the 
			  // the desired system clock value is 
			  // invalid!!
			  while(1)
				{}
    }
   
    
    // Initialize baud rate
    baud_rate = baud;
    
    // Change units to Hz
    uart2clk = sysclk * 1000;
    // Calculate the first baud rate using the lowest OSR value possible.  
    i = 16;		// ME: static value for UART2 (manual p.751)
    sbr_val = (uint32)(uart2clk/(baud_rate * i));
    calculated_baud = (uart2clk / (i * sbr_val));
        
    if (calculated_baud > baud_rate)
        baud_diff = calculated_baud - baud_rate;
    else
        baud_diff = baud_rate - calculated_baud;
    
/*   osr_val = i;
        
    // Select the best OSR value
    for (i = 5; i <= 32; i++)
    {
        sbr_val = (uint32)(uart2clk/(baud_rate * i));
        calculated_baud = (uart2clk / (i * sbr_val));
        
        if (calculated_baud > baud_rate)
            temp = calculated_baud - baud_rate;
        else
            temp = baud_rate - calculated_baud;
        
        if (temp <= baud_diff)
        {
            baud_diff = temp;
            osr_val = i; 
        }
    } */
    
    if (baud_diff < ((baud_rate / 100) * 3))
    {
        // If the OSR is between 4x and 8x then both
        // edge sampling MUST be turned on.  
        // if ((osr_val >3) && (osr_val < 9))
        //    UART0_C5|= UART0_C5_BOTHEDGE_MASK;
        
        // Setup OSR value 
        //reg_temp = UART0_C4;
        //reg_temp &= ~UART0_C4_OSR_MASK;
        //reg_temp |= UART0_C4_OSR(osr_val-1);
    
        // Write reg_temp to C4 register
        //UART2_C4 = reg_temp;
        
        //reg_temp = (reg_temp & UART0_C4_OSR_MASK) + 1;
        sbr_val = (uint32)((uart2clk)/(baud_rate * i));
        
         /* Save off the current value of the uartx_BDH except for the SBR field */
        reg_temp = UART2_BDH & ~(UART0_BDH_SBR(0x1F));
             
        UART2_BDH = reg_temp |  UART0_BDH_SBR(((sbr_val & 0x1F00) >> 8));
        UART2_BDL = (uint8)(sbr_val & UART0_BDL_SBR_MASK);
        
        /* Enable receiver and transmitter */
        UART2_C2 |= (UART0_C2_TE_MASK
                    | UART0_C2_RE_MASK );
    }
    else
		{
        // Unacceptable baud rate difference
        // More than 3% difference!!
        // Enter infinite loop!
        while(1)
			{}
		}					
    
}

/********************************************************************/
/*
 * Wait for a character to be received on the specified uart
 *
 * Parameters:
 *  channel      uart channel to read from
 *
 * Return Values:
 *  the received character
 */
char uart_getchar (UART_MemMapPtr channel)
{
      /* Wait until character has been received */
      while (!(UART_S1_REG(channel) & UART_S1_RDRF_MASK));
    
      /* Return the 8-bit data from the receiver */
      return UART_D_REG(channel);
}
/********************************************************************/
/*
 * Wait for space in the uart Tx FIFO and then send a character
 *
 * Parameters:
 *  channel      uart channel to send to
 *  ch			 character to send
 */ 
void uart_putchar (UART_MemMapPtr channel, char ch)
{
      /* Wait until space is available in the FIFO */
      while(!(UART_S1_REG(channel) & UART_S1_TDRE_MASK));
    
      /* Send the character */
      UART_D_REG(channel) = (uint8)ch;
    
 }
/********************************************************************/
/*
 * Check to see if a character has been received
 *
 * Parameters:
 *  channel      uart channel to check for a character
 *
 * Return values:
 *  0       No character received
 *  1       Character has been received
 */
int uart_getchar_present (UART_MemMapPtr channel)
{
    return (UART_S1_REG(channel) & UART_S1_RDRF_MASK);
}
/********************************************************************/

void TFC_BluetoothModuleSetBaud(unsigned int baudRate)
{
	uart_putchar(UART2_BASE_PTR, 'A');
	uart_putchar(UART2_BASE_PTR, 'T');
	uart_putchar(UART2_BASE_PTR, '+');
	uart_putchar(UART2_BASE_PTR, 'U');
	uart_putchar(UART2_BASE_PTR, 'A');
	uart_putchar(UART2_BASE_PTR, 'R');
	uart_putchar(UART2_BASE_PTR, 'T');
	uart_putchar(UART2_BASE_PTR, '=');
	
	switch (baudRate)
	{
		case 4800:
			uart_putchar(UART2_BASE_PTR, '4');
			uart_putchar(UART2_BASE_PTR, '8');
			uart_putchar(UART2_BASE_PTR, '0');
			uart_putchar(UART2_BASE_PTR, '0');
			break;
	
		case 9600:
			uart_putchar(UART2_BASE_PTR, '9');
			uart_putchar(UART2_BASE_PTR, '6');
			uart_putchar(UART2_BASE_PTR, '0');
			uart_putchar(UART2_BASE_PTR, '0');
			break;
			
		case 19200:
			uart_putchar(UART2_BASE_PTR, '1');
			uart_putchar(UART2_BASE_PTR, '9');
			uart_putchar(UART2_BASE_PTR, '2');
			uart_putchar(UART2_BASE_PTR, '0');
			uart_putchar(UART2_BASE_PTR, '0');
			break;
			
		case 38400:
			uart_putchar(UART2_BASE_PTR, '3');
			uart_putchar(UART2_BASE_PTR, '8');
			uart_putchar(UART2_BASE_PTR, '4');
			uart_putchar(UART2_BASE_PTR, '0');
			uart_putchar(UART2_BASE_PTR, '0');	
			break;
			
		case 57600:
			uart_putchar(UART2_BASE_PTR, '5');
			uart_putchar(UART2_BASE_PTR, '7');
			uart_putchar(UART2_BASE_PTR, '6');
			uart_putchar(UART2_BASE_PTR, '0');
			uart_putchar(UART2_BASE_PTR, '0');
			break;
		
		case 115200:
			uart_putchar(UART2_BASE_PTR, '1');
			uart_putchar(UART2_BASE_PTR, '1');
			uart_putchar(UART2_BASE_PTR, '5');
			uart_putchar(UART2_BASE_PTR, '2');
			uart_putchar(UART2_BASE_PTR, '0');
			uart_putchar(UART2_BASE_PTR, '0');
			break;
			
		case 230400:
			uart_putchar(UART2_BASE_PTR, '2');
			uart_putchar(UART2_BASE_PTR, '3');
			uart_putchar(UART2_BASE_PTR, '0');
			uart_putchar(UART2_BASE_PTR, '4');
			uart_putchar(UART2_BASE_PTR, '0');
			uart_putchar(UART2_BASE_PTR, '0');
			break;
			
		case 460800:
			uart_putchar(UART2_BASE_PTR, '4');
			uart_putchar(UART2_BASE_PTR, '6');
			uart_putchar(UART2_BASE_PTR, '0');
			uart_putchar(UART2_BASE_PTR, '8');
			uart_putchar(UART2_BASE_PTR, '0');
			uart_putchar(UART2_BASE_PTR, '0');
			break;
			
		case 921600:
			uart_putchar(UART2_BASE_PTR, '9');
			uart_putchar(UART2_BASE_PTR, '2');
			uart_putchar(UART2_BASE_PTR, '1');
			uart_putchar(UART2_BASE_PTR, '6');
			uart_putchar(UART2_BASE_PTR, '0');
			uart_putchar(UART2_BASE_PTR, '0');
			break;
			
		case 1382400:
			uart_putchar(UART2_BASE_PTR, '1');
			uart_putchar(UART2_BASE_PTR, '3');
			uart_putchar(UART2_BASE_PTR, '8');
			uart_putchar(UART2_BASE_PTR, '2');
			uart_putchar(UART2_BASE_PTR, '4');
			uart_putchar(UART2_BASE_PTR, '0');
			uart_putchar(UART2_BASE_PTR, '0');
			break;	
			
		default:
			while(1);	// Wrong baud rate
	}
	
	uart_putchar(UART2_BASE_PTR, ',');
	uart_putchar(UART2_BASE_PTR, '0');
	uart_putchar(UART2_BASE_PTR, ',');
	uart_putchar(UART2_BASE_PTR, '0');
	uart_putchar(UART2_BASE_PTR, '\r');
	uart_putchar(UART2_BASE_PTR, '\n');
}

