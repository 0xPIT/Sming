/****
 * Sming Framework Project - Open Source framework for high efficiency native ESP8266 development.
 * Created 2015 by Skurydin Alexey
 * http://github.com/anakod/Sming
 * All files of the Sming Core are provided under the LGPL v3 license.
 ****/

// HardwareSerial based on Espressif Systems code

#include "../SmingCore/HardwareSerial.h"
#include "../Wiring/WiringFrameworkIncludes.h"
#include <cstdarg>

#include "../SmingCore/Clock.h"
#include "../SmingCore/Interrupts.h"

#define ETS_UART_INTR_ENABLE()  _xt_isr_unmask(1 << ETS_UART_INUM)
#define ETS_UART_INTR_DISABLE() _xt_isr_mask(1 << ETS_UART_INUM)
#define UART_INTR_MASK          0x1ff
#define UART_LINE_INV_MASK      (0x3f<<19)

#define ETS_UART_INUM       5
#define ETS_UART1_INUM      5

LOCAL STATUS uart_tx_one_char(uint8 uart, uint8 TxChar) {
    while (true) {
        uint32 fifo_cnt = READ_PERI_REG(UART_STATUS(uart)) & (UART_TXFIFO_CNT << UART_TXFIFO_CNT_S);

        if ((fifo_cnt >> UART_TXFIFO_CNT_S & UART_TXFIFO_CNT) < 126) {
            break;
        }
    }

    WRITE_PERI_REG(UART_FIFO(uart) , TxChar);
    return OK;
}

LOCAL void null_write_char(char c) {
}

LOCAL void uart1_write_char(char c) {
    if (c == '\n') {
        uart_tx_one_char(UART1, '\r');
        uart_tx_one_char(UART1, '\n');
    } 
    else if (c == '\r') {
    } 
    else {
        uart_tx_one_char(UART1, c);
    }
}

LOCAL void uart0_write_char(char c) {
    if (c == '\n') {
        uart_tx_one_char(UART0, '\r');
        uart_tx_one_char(UART0, '\n');
    } 
    else if (c == '\r') {
    } 
    else {
        uart_tx_one_char(UART0, c);
    }
}




void ICACHE_FLASH_ATTR
UART_SetWordLength(UartPort uart_no, UartWordLength len)
{
    SET_PERI_REG_BITS(UART_CONF0(uart_no), UART_BIT_NUM, len, UART_BIT_NUM_S);
}

void ICACHE_FLASH_ATTR
UART_SetStopBits(UartPort uart_no, UartStopBits bit_num)
{
    SET_PERI_REG_BITS(UART_CONF0(uart_no), UART_STOP_BIT_NUM, bit_num, UART_STOP_BIT_NUM_S);
}

void ICACHE_FLASH_ATTR
UART_SetLineInverse(UartPort uart_no, UartLineLevelInverse inverse_mask)
{
    CLEAR_PERI_REG_MASK(UART_CONF0(uart_no), UART_LINE_INV_MASK);
    SET_PERI_REG_MASK(UART_CONF0(uart_no), inverse_mask);
}

void ICACHE_FLASH_ATTR
UART_SetParity(UartPort uart_no, UartParityMode Parity_mode)
{
    CLEAR_PERI_REG_MASK(UART_CONF0(uart_no), UART_PARITY | UART_PARITY_EN);

    if (Parity_mode == Parity_None) {
    } else {
        SET_PERI_REG_MASK(UART_CONF0(uart_no), Parity_mode | UART_PARITY_EN);
    }
}


void ICACHE_FLASH_ATTR
UART_SetBaudrate(UartPort uart_no, uint32 baud_rate)
{
    //uart_div_modify(uart_no, UART_CLK_FREQ / baud_rate);
}

//only when FlowControl_RTS is set , will the rx_thresh value be set.
void ICACHE_FLASH_ATTR
UART_SetFlowCtrl(UartPort uart_no, UartFlowCtrl flow_ctrl, uint8 rx_thresh)
{
    if (flow_ctrl & FlowControl_RTS) {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_U0RTS);
        SET_PERI_REG_BITS(UART_CONF1(uart_no), UART_RX_FLOW_THRHD, rx_thresh, UART_RX_FLOW_THRHD_S);
        SET_PERI_REG_MASK(UART_CONF1(uart_no), UART_RX_FLOW_EN);
    } else {
        CLEAR_PERI_REG_MASK(UART_CONF1(uart_no), UART_RX_FLOW_EN);
    }

    if (flow_ctrl & FlowControl_CTS) {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_UART0_CTS);
        SET_PERI_REG_MASK(UART_CONF0(uart_no), UART_TX_FLOW_EN);
    } else {
        CLEAR_PERI_REG_MASK(UART_CONF0(uart_no), UART_TX_FLOW_EN);
    }
}

void ICACHE_FLASH_ATTR
UART_WaitTxFifoEmpty(UartPort uart_no) //do not use if tx flow control enabled
{
    while (READ_PERI_REG(UART_STATUS(uart_no)) & (UART_TXFIFO_CNT << UART_TXFIFO_CNT_S));
}

void ICACHE_FLASH_ATTR
UART_ResetFifo(UartPort uart_no)
{
    SET_PERI_REG_MASK(UART_CONF0(uart_no), UART_RXFIFO_RST | UART_TXFIFO_RST);
    CLEAR_PERI_REG_MASK(UART_CONF0(uart_no), UART_RXFIFO_RST | UART_TXFIFO_RST);
}

void ICACHE_FLASH_ATTR
UART_ClearIntrStatus(UartPort uart_no, uint32 clr_mask)
{
    WRITE_PERI_REG(UART_INT_CLR(uart_no), clr_mask);
}

void ICACHE_FLASH_ATTR
UART_SetIntrEna(UartPort uart_no, uint32 ena_mask)
{
    SET_PERI_REG_MASK(UART_INT_ENA(uart_no), ena_mask);
}

void ICACHE_FLASH_ATTR
UART_intr_handler_register(_xt_isr fn)
{
    _xt_isr_attach(ETS_UART_INUM, fn);
}

void ICACHE_FLASH_ATTR
UART_SetPrintPort(UartPort uart_no)
{
    if (uart_no == 1) {
        os_install_putc1(uart1_write_char);
    } else {
        os_install_putc1(uart0_write_char);
    }
}

void ICACHE_FLASH_ATTR
UART_ParamConfig(UartPort uart_no,  UartConfigTypeDef *pUARTConfig)
{
    if (uart_no == UART1) {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_U1TXD_BK);
    } else {
        PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD);
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD);
    }

    UART_SetFlowCtrl(uart_no, pUARTConfig->flow_ctrl, pUARTConfig->UART_RxFlowThresh);
    UART_SetBaudrate(uart_no, pUARTConfig->baud_rate);

    WRITE_PERI_REG(UART_CONF0(uart_no),
                   ((pUARTConfig->parity == Parity_None) ? 0x0 : (UART_PARITY_EN | pUARTConfig->parity))
                   | (pUARTConfig->stop_bits << UART_STOP_BIT_NUM_S)
                   | (pUARTConfig->data_bits << UART_BIT_NUM_S)
                   | ((pUARTConfig->flow_ctrl & FlowControl_CTS) ? UART_TX_FLOW_EN : 0x0)
                   | pUARTConfig->UART_InverseMask);

    UART_ResetFifo(uart_no);
}

void ICACHE_FLASH_ATTR
UART_IntrConfig(UartPort uart_no,  UartIntrConfTypeDef *pUARTIntrConf)
{

    uint32 reg_val = 0;
    UART_ClearIntrStatus(uart_no, UART_INTR_MASK);
    reg_val = READ_PERI_REG(UART_CONF1(uart_no)) & ~((UART_RX_FLOW_THRHD << UART_RX_FLOW_THRHD_S) | UART_RX_FLOW_EN) ;

    reg_val |= ((pUARTIntrConf->UART_IntrEnMask & UART_RXFIFO_TOUT_INT_ENA) ?
                ((((pUARTIntrConf->UART_RX_TimeOutIntrThresh)&UART_RX_TOUT_THRHD) << UART_RX_TOUT_THRHD_S) | UART_RX_TOUT_EN) : 0);

    reg_val |= ((pUARTIntrConf->UART_IntrEnMask & UART_RXFIFO_FULL_INT_ENA) ?
                (((pUARTIntrConf->UART_RX_FifoFullIntrThresh)&UART_RXFIFO_FULL_THRHD) << UART_RXFIFO_FULL_THRHD_S) : 0);

    reg_val |= ((pUARTIntrConf->UART_IntrEnMask & UART_TXFIFO_EMPTY_INT_ENA) ?
                (((pUARTIntrConf->UART_TX_FifoEmptyIntrThresh)&UART_TXFIFO_EMPTY_THRHD) << UART_TXFIFO_EMPTY_THRHD_S) : 0);

    WRITE_PERI_REG(UART_CONF1(uart_no), reg_val);
    CLEAR_PERI_REG_MASK(UART_INT_ENA(uart_no), UART_INTR_MASK);
    SET_PERI_REG_MASK(UART_INT_ENA(uart_no), pUARTIntrConf->UART_IntrEnMask);
}


// UartDev is defined and initialized in ROM code.
UartDevice UartDev;

// StreamDataAvailableDelegate HardwareSerial::HWSDelegates[2];

HWSerialMemberData HardwareSerial::memberData[NUMBER_UARTS];

HardwareSerial::HardwareSerial(const int uartPort)
	: uart(uartPort)
{
	resetCallback();
}

void HardwareSerial::begin(const uint32_t baud/* = 9600*/)
{
	//TODO: Move to params!
	UartDev.baud_rate = (UartBaudRate)baud;
	UartDev.parity = Parity_None;
	UartDev.exist_parity = STICK_PARITY_DIS;
	UartDev.stop_bits = ONE_STOP_BIT;
	UartDev.data_bits = EIGHT_BITS;

	//ETS_UART_INTR_ATTACH((void*)uart0_rx_intr_handler,  &(UartDev.rcv_buff));
  UART_intr_handler_register((_xt_isr)uart0_rx_intr_handler);

	PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD);

	//uart_div_modify(uart, UART_CLK_FREQ / (UartDev.baud_rate));

	WRITE_PERI_REG(UART_CONF0(uart),    UartDev.exist_parity
				   | UartDev.parity
				   | (UartDev.stop_bits << UART_STOP_BIT_NUM_S)
				   | (UartDev.data_bits << UART_BIT_NUM_S));


	//clear rx and tx fifo,not ready
	SET_PERI_REG_MASK(UART_CONF0(uart), UART_RXFIFO_RST | UART_TXFIFO_RST);
	CLEAR_PERI_REG_MASK(UART_CONF0(uart), UART_RXFIFO_RST | UART_TXFIFO_RST);

	//set rx fifo trigger
	WRITE_PERI_REG(UART_CONF1(uart), (UartDev.rcv_buff.TrigLvl & UART_RXFIFO_FULL_THRHD) << UART_RXFIFO_FULL_THRHD_S);

	//clear all interrupt
	WRITE_PERI_REG(UART_INT_CLR(uart), 0xffff);
	//enable rx_interrupt
	SET_PERI_REG_MASK(UART_INT_ENA(uart), UART_RXFIFO_FULL_INT_ENA);

	ETS_UART_INTR_ENABLE();
	delay(10);
	Serial.println("\r\n"); // after SPAM :)
}

size_t HardwareSerial::write(uint8_t oneChar)
{
	//if (oneChar == '\0') return 0;

  uart0_write_char(oneChar);
//	uart_tx_one_char(oneChar);

	return 1;
}

int HardwareSerial::available()
{
	RcvMsgBuff &rxBuf = UartDev.rcv_buff;
	if (rxBuf.pWritePos != rxBuf.pReadPos)
	{
		if (rxBuf.pWritePos > rxBuf.pReadPos)
			return rxBuf.pWritePos - rxBuf.pReadPos;
		else
			return (((uint32_t)rxBuf.pRcvMsgBuff + RX_BUFF_SIZE) - (uint32_t)UartDev.rcv_buff.pReadPos) + (uint32_t)rxBuf.pWritePos;
	}
	else
		return 0;
}

int HardwareSerial::read()
{
	if (available() == 0)
		return -1;

	noInterrupts();
	RcvMsgBuff &rxBuf = UartDev.rcv_buff;
	char res = *(rxBuf.pReadPos);
	rxBuf.pReadPos++;

	if (rxBuf.pReadPos == (rxBuf.pRcvMsgBuff + RX_BUFF_SIZE))
		rxBuf.pReadPos = rxBuf.pRcvMsgBuff ;

	interrupts();
	return res;
}

int HardwareSerial::readMemoryBlock(char* buf, int max_len)
{
	RcvMsgBuff &rxBuf = UartDev.rcv_buff;
	int num = 0;

	noInterrupts();
	if (rxBuf.pWritePos != rxBuf.pReadPos) {
		while ((rxBuf.pWritePos != rxBuf.pReadPos) && (max_len-- > 0)) {
			*buf = *rxBuf.pReadPos;		// Read data from Buffer
			num++;						// Increase counter of read bytes
			buf++;						// Increase Buffer pointer

			// Set pointer to next data word in ring buffer
			rxBuf.pReadPos++;
			if (rxBuf.pReadPos == (rxBuf.pRcvMsgBuff + RX_BUFF_SIZE))
				rxBuf.pReadPos = rxBuf.pRcvMsgBuff ;
		}
	}
	interrupts();

	return num;
}

int HardwareSerial::peek()
{
	if (available() == 0)
		return -1;

	noInterrupts();
	RcvMsgBuff &rxBuf = UartDev.rcv_buff;
	char res = *(rxBuf.pReadPos);
	interrupts();
	return res;
}

void HardwareSerial::flush()
{
}


/*void HardwareSerial::printf(const char *fmt, ...)
{
	// Doesn't work :(
	va_list va;
	va_start(va, fmt);
	ets_uart_printf(fmt, va);
	va_end(va);
}*/

void HardwareSerial::systemDebugOutput(bool enabled)
{
	if (uart == UART_ID_0)
		os_install_putc1(enabled ? uart0_write_char : null_write_char);
	//else
	//	os_install_putc1(enabled ? (void *)uart1_tx_one_char : NULL); //TODO: Debug serial
}

void HardwareSerial::setCallback(StreamDataReceivedDelegate reqDelegate, bool useSerialRxBuffer /* = true */)
{
	memberData[uart].HWSDelegate = reqDelegate;
	memberData[uart].useRxBuff = useSerialRxBuffer;
}

void HardwareSerial::resetCallback()
{
	memberData[uart].HWSDelegate = nullptr;
	memberData[uart].useRxBuff = true;
}


void HardwareSerial::uart0_rx_intr_handler(void *para)
{
    /* uart0 and uart1 intr combine togther, when interrupt occur, see reg 0x3ff20020, bit2, bit0 represents
     * uart1 and uart0 respectively
     */
    RcvMsgBuff *pRxBuff = (RcvMsgBuff *)para;
    uint8 RcvChar;

    if (UART_RXFIFO_FULL_INT_ST != (READ_PERI_REG(UART_INT_ST(UART_ID_0)) & UART_RXFIFO_FULL_INT_ST))
        return;

    WRITE_PERI_REG(UART_INT_CLR(UART_ID_0), UART_RXFIFO_FULL_INT_CLR);

    while (READ_PERI_REG(UART_STATUS(UART_ID_0)) & (UART_RXFIFO_CNT << UART_RXFIFO_CNT_S))
    {
        RcvChar = READ_PERI_REG(UART_FIFO(UART_ID_0)) & 0xFF;

        /* you can add your handle code below.*/
      if (memberData[UART_ID_0].useRxBuff)
      {
        *(pRxBuff->pWritePos) = RcvChar;

        // insert here for get one command line from uart
        if (RcvChar == '\n' )
            pRxBuff->BuffState = WRITE_OVER;

        pRxBuff->pWritePos++;

        if (pRxBuff->pWritePos == (pRxBuff->pRcvMsgBuff + RX_BUFF_SIZE))
        {
            // overflow ...we may need more error handle here.
        	pRxBuff->pWritePos = pRxBuff->pRcvMsgBuff;
        }

        if (pRxBuff->pWritePos == pRxBuff->pReadPos)
        {   // Prevent readbuffer overflow
			if (pRxBuff->pReadPos == (pRxBuff->pRcvMsgBuff + RX_BUFF_SIZE))
			{
				pRxBuff->pReadPos = pRxBuff->pRcvMsgBuff ;
			} else {
				pRxBuff->pReadPos++;
			}
		}
      }
      if (memberData[UART_ID_0].HWSDelegate)
        {
        	unsigned short cc;
        	cc = (pRxBuff->pWritePos < pRxBuff->pReadPos) ? ((pRxBuff->pWritePos + RX_BUFF_SIZE) - pRxBuff->pReadPos)
        													: (pRxBuff->pWritePos - pRxBuff->pReadPos);
        	memberData[UART_ID_0].HWSDelegate(Serial, RcvChar, cc);
        }
    }

}


HardwareSerial Serial(UART_ID_0);
