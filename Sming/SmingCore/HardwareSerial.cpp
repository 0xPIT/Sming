/****
 * Sming Framework Project - Open Source framework for high efficiency native ESP8266 development.
 * Created 2015 by Skurydin Alexey
 * http://github.com/anakod/Sming
 * All files of the Sming Core are provided under the LGPL v3 license.
 ****/

// HardwareSerial based on Espressif Systems code

#include "../SmingCore/HardwareSerial.h"
#include "../Wiring/WiringFrameworkIncludes.h"
//#include <cstdarg>
//#include "../SmingCore/Clock.h"
//#include "../SmingCore/Interrupts.h"

#define UART_INUM       5
#define UART1_INUM      5

#define UART_INTR_ENABLE()  _xt_isr_unmask(1 << UART_INUM)
#define UART_INTR_DISABLE() _xt_isr_mask(1 << UART_INUM)
#define UART_INTR_MASK      0x1ff
#define UART_LINE_INV_MASK  (0x3f<<19)

#define ESP8266_CLOCK 80000000UL
#define ESP8266_REG(addr) *((volatile uint32_t *)(0x60000000+(addr)))
#define USD(u) ESP8266_REG(0x014+(0xF00*(u&1))) // CLKDIV

#define SERIAL_BUFFER_SIZE 128

static UartDevice UartDev;
HWSerialMemberData HardwareSerial::memberData[NUMBER_UARTS];

static void null_write_char(char c)
{
}

static STATUS uart_tx_one_char(uint8 uart, uint8 TxChar) 
{
    while (true) {
        uint32 fifo_cnt = READ_PERI_REG(UART_STATUS(uart)) & (UART_TXFIFO_CNT << UART_TXFIFO_CNT_S);

        if ((fifo_cnt >> UART_TXFIFO_CNT_S & UART_TXFIFO_CNT) < 126) {
            break;
        }
    }

    WRITE_PERI_REG(UART_FIFO(uart), TxChar);
    return OK;
}

static void uart1_write_char(char c)
{
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

static void uart0_write_char(char c)
{
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
setWordLength(UartPort uart_no, UartWordLength len)
{
    SET_PERI_REG_BITS(UART_CONF0(uart_no), UART_BIT_NUM, len, UART_BIT_NUM_S);
}

void ICACHE_FLASH_ATTR
setStopBits(UartPort uart_no, UartStopBits bit_num)
{
    SET_PERI_REG_BITS(UART_CONF0(uart_no), UART_STOP_BIT_NUM, bit_num, UART_STOP_BIT_NUM_S);
}

void ICACHE_FLASH_ATTR
setLineInverse(UartPort uart_no, UartLineLevelInverse inverse_mask)
{
    CLEAR_PERI_REG_MASK(UART_CONF0(uart_no), UART_LINE_INV_MASK);
    SET_PERI_REG_MASK(UART_CONF0(uart_no), inverse_mask);
}

void ICACHE_FLASH_ATTR
setParity(UartPort uart_no, UartParityMode Parity_mode)
{
    CLEAR_PERI_REG_MASK(UART_CONF0(uart_no), UART_PARITY | UART_PARITY_EN);

    if (Parity_mode == Parity_None) {
    } 
    else {
        SET_PERI_REG_MASK(UART_CONF0(uart_no), Parity_mode | UART_PARITY_EN);
    }
}

void ICACHE_FLASH_ATTR
setBaudrate(UartPort uart_no, uint32 baud_rate)
{
    USD(uart_no) = (ESP8266_CLOCK / baud_rate);
}

//only when FlowControl_RTS is set , will the rx_thresh value be set.
void ICACHE_FLASH_ATTR
setFlowCtrl(UartPort uart_no, UartFlowCtrl flow_ctrl, uint8 rx_thresh)
{
    if (flow_ctrl & FlowControl_RTS) {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_U0RTS);
        SET_PERI_REG_BITS(UART_CONF1(uart_no), UART_RX_FLOW_THRHD, rx_thresh, UART_RX_FLOW_THRHD_S);
        SET_PERI_REG_MASK(UART_CONF1(uart_no), UART_RX_FLOW_EN);
    } 
    else {
        CLEAR_PERI_REG_MASK(UART_CONF1(uart_no), UART_RX_FLOW_EN);
    }

    if (flow_ctrl & FlowControl_CTS) {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_UART0_CTS);
        SET_PERI_REG_MASK(UART_CONF0(uart_no), UART_TX_FLOW_EN);
    } 
    else {
        CLEAR_PERI_REG_MASK(UART_CONF0(uart_no), UART_TX_FLOW_EN);
    }
}

void ICACHE_FLASH_ATTR
waitTxFifoEmpty(UartPort uart_no) //do not use if tx flow control enabled
{
    while (READ_PERI_REG(UART_STATUS(uart_no)) & (UART_TXFIFO_CNT << UART_TXFIFO_CNT_S));
}

void ICACHE_FLASH_ATTR
resetFifo(UartPort uart_no)
{
    SET_PERI_REG_MASK(UART_CONF0(uart_no), UART_RXFIFO_RST | UART_TXFIFO_RST);
    CLEAR_PERI_REG_MASK(UART_CONF0(uart_no), UART_RXFIFO_RST | UART_TXFIFO_RST);
}

void ICACHE_FLASH_ATTR
clearIntrStatus(UartPort uart_no, uint32 clr_mask)
{
    WRITE_PERI_REG(UART_INT_CLR(uart_no), clr_mask);
}

void ICACHE_FLASH_ATTR
setIntrEna(UartPort uart_no, uint32 ena_mask)
{
    SET_PERI_REG_MASK(UART_INT_ENA(uart_no), ena_mask);
}

void ICACHE_FLASH_ATTR
interruptHandlerRegister(_xt_isr fn)
{
    _xt_isr_attach(UART_INUM, fn);
}

void ICACHE_FLASH_ATTR
setPrintPort(UartPort uart_no)
{
    if (uart_no == 1) {
        os_install_putc1(uart1_write_char);
    } 
    else {
        os_install_putc1(uart0_write_char);
    }
}

void ICACHE_FLASH_ATTR
paramConfig(UartPort uart_no,  UartConfigTypeDef *pUARTConfig)
{
    if (uart_no == UART1) {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_U1TXD_BK);
    } 
    else {
        PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD);
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD);
    }

    setFlowCtrl(uart_no, pUARTConfig->flow_ctrl, pUARTConfig->UART_RxFlowThresh);
    setBaudrate(uart_no, pUARTConfig->baud_rate);

    WRITE_PERI_REG(UART_CONF0(uart_no),
                   ((pUARTConfig->parity == Parity_None) ? 0x0 : (UART_PARITY_EN | pUARTConfig->parity))
                   | (pUARTConfig->stop_bits << UART_STOP_BIT_NUM_S)
                   | (pUARTConfig->data_bits << UART_BIT_NUM_S)
                   | ((pUARTConfig->flow_ctrl & FlowControl_CTS) ? UART_TX_FLOW_EN : 0x0)
                   | pUARTConfig->UART_InverseMask);

    resetFifo(uart_no);
}

void ICACHE_FLASH_ATTR
interruptConfig(UartPort uart_no,  UartIntrConfTypeDef *pUARTIntrConf)
{
    uint32 reg_val = 0;
    clearIntrStatus(uart_no, UART_INTR_MASK);
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

// ----------------------------------------------------------------------------

HardwareSerial::HardwareSerial(const int uartPort)
	: uart(uartPort)
{
	resetCallback();
}

void HardwareSerial::getRxQueue(xQueueHandle *qh) {
  *qh = rxQueue;
}

void HardwareSerial::begin(const uint32_t baud)
{
    waitTxFifoEmpty(UART0);
    waitTxFifoEmpty(UART1);

    rxQueue = xQueueCreate(SERIAL_BUFFER_SIZE, sizeof(uint8_t));
    //txQueue = xQueueCreate(SERIAL_BUFFER_SIZE, sizeof(uint8_t));

    uartConfig.baud_rate         = (UartBaudRate)baud;
    uartConfig.data_bits         = EIGHT_BITS;
    uartConfig.parity            = Parity_None;
    uartConfig.stop_bits         = ONE_STOP_BIT;
    uartConfig.flow_ctrl         = FlowControl_None;
    uartConfig.UART_RxFlowThresh = 120;
    uartConfig.UART_InverseMask  = UART_None_Inverse;
    paramConfig(UART0, &uartConfig);

    UartIntrConfTypeDef uartIntr;
    uartIntr.UART_IntrEnMask = UART_RXFIFO_TOUT_INT_ENA | UART_FRM_ERR_INT_ENA | UART_RXFIFO_FULL_INT_ENA | UART_TXFIFO_EMPTY_INT_ENA;
    uartIntr.UART_RX_FifoFullIntrThresh = 10;
    uartIntr.UART_RX_TimeOutIntrThresh = 2;
    uartIntr.UART_TX_FifoEmptyIntrThresh = 20;
    interruptConfig(UART0, &uartIntr);

    setPrintPort(UART0);
    interruptHandlerRegister((_xt_isr)rxInterruptHandler);
    UART_INTR_ENABLE();
}

size_t HardwareSerial::write(uint8_t oneChar)
{
    if (oneChar == '\0') {
        return 0;
    }

    uart0_write_char(oneChar);

	return 1;
}

int HardwareSerial::available()
{
    return uxQueueMessagesWaiting(rxQueue);
}

int HardwareSerial::peek()
{
    uint8_t c;
    xQueuePeek(rxQueue, &c, (portTickType)portMAX_DELAY);
    return c;
}

int HardwareSerial::read()
{
    uint8_t c;

	if (available() == 0) {
		return -1;
    }

    if (xQueueReceive(rxQueue, &c, (portTickType)portMAX_DELAY)) {
        return c;
    }

    return 0;
}

void HardwareSerial::flush()
{
    xQueueReset(rxQueue);
}

void HardwareSerial::systemDebugOutput(bool enable)
{
	if (uart == UART0)
		os_install_putc1(enable ? uart0_write_char : null_write_char);
	//else
	//	os_install_putc1(enabled ? (void *)uart1_tx_one_char : NULL); //TODO: Debug serial
}

void delegateTask(void* parameter) {
    HardwareSerial *instance = (HardwareSerial *)parameter;

    if (instance->memberData[instance->uart].HWSDelegate) {
        uint8_t c; 
        if (xQueueReceive(instance->rxQueue, &c, (portTickType)portMAX_DELAY)) {
            uint8_t availableChars = uxQueueMessagesWaiting(instance->rxQueue);
            instance->memberData[instance->uart].HWSDelegate(Serial, c, availableChars);
        }
    }
}

void HardwareSerial::setCallback(StreamDataReceivedDelegate reqDelegate, bool useSerialRxBuffer)
{
    const s8_t taskId[5] = { 'S', 'e', 'r', (s8_t)('0' + uart), '\0' };
 
 	memberData[uart].HWSDelegate = reqDelegate;
	memberData[uart].useRxBuff = useSerialRxBuffer;

    xTaskCreate(&delegateTask,
                taskId,
                128,          // stack
                (void *)this, // parameter
                (tskIDLE_PRIORITY + 3),
                &delegateTaskHandle
    );
}

void HardwareSerial::resetCallback()
{
    vTaskDelete(delegateTaskHandle);
	memberData[uart].HWSDelegate = nullptr;
	memberData[uart].useRxBuff = true;
}

void HardwareSerial::rxInterruptHandler(void *parameter) {
    portBASE_TYPE xHigherPriorityTaskWoken;
    HardwareSerial* instance = (HardwareSerial *)parameter;
    uint8 rxChar;
    uint8 bufIdx = 0;
    uint8 fifoLen = 0;
    
    uint32 uartIntStatus = READ_PERI_REG(UART_INT_ST(instance->uart));

    while (uartIntStatus != 0x0) {
        if (UART_FRM_ERR_INT_ST == (uartIntStatus & UART_FRM_ERR_INT_ST)) {
            WRITE_PERI_REG(UART_INT_CLR(instance->uart), UART_FRM_ERR_INT_CLR);
        } 
        else if (UART_RXFIFO_FULL_INT_ST == (uartIntStatus & UART_RXFIFO_FULL_INT_ST)) {
            fifoLen = (READ_PERI_REG(UART_STATUS(instance->uart)) >> UART_RXFIFO_CNT_S)&UART_RXFIFO_CNT;
            bufIdx = 0;

            while (bufIdx < fifoLen) {
                rxChar = READ_PERI_REG(UART_FIFO(instance->uart)) & 0xFF;
                xQueueSendFromISR(instance->rxQueue, (void *)&rxChar, &xHigherPriorityTaskWoken);
                bufIdx++;
            }

            WRITE_PERI_REG(UART_INT_CLR(instance->uart), UART_RXFIFO_FULL_INT_CLR);
        } 
        else if (UART_RXFIFO_TOUT_INT_ST == (uartIntStatus & UART_RXFIFO_TOUT_INT_ST)) {
            fifoLen = (READ_PERI_REG(UART_STATUS(instance->uart)) >> UART_RXFIFO_CNT_S)&UART_RXFIFO_CNT;
            bufIdx = 0;

            while (bufIdx < fifoLen) {
                rxChar = READ_PERI_REG(UART_FIFO(instance->uart)) & 0xFF;
                xQueueSendFromISR(instance->rxQueue, (void *)&rxChar, &xHigherPriorityTaskWoken);
                bufIdx++;
            }

            WRITE_PERI_REG(UART_INT_CLR(instance->uart), UART_RXFIFO_TOUT_INT_CLR);
        } 
        else if (UART_TXFIFO_EMPTY_INT_ST == (uartIntStatus & UART_TXFIFO_EMPTY_INT_ST)) {
            WRITE_PERI_REG(UART_INT_CLR(instance->uart), UART_TXFIFO_EMPTY_INT_CLR);
            CLEAR_PERI_REG_MASK(UART_INT_ENA(instance->uart), UART_TXFIFO_EMPTY_INT_ENA);
        } 
        else {
            // skip
        }

        uartIntStatus = READ_PERI_REG(UART_INT_ST(instance->uart));
    }

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}


HardwareSerial Serial(UART0);



#if 0
void HardwareSerial::__uart0_rx_intr_handler(void *para)
{
    RcvMsgBuff *pRxBuff = (RcvMsgBuff *)para;
    uint8 RcvChar;

    if (UART_RXFIFO_FULL_INT_ST != (READ_PERI_REG(UART_INT_ST(UART_ID_0)) & UART_RXFIFO_FULL_INT_ST)) {
        return;
    }

    WRITE_PERI_REG(UART_INT_CLR(UART_ID_0), UART_RXFIFO_FULL_INT_CLR);

    while (READ_PERI_REG(UART_STATUS(UART_ID_0)) & (UART_RXFIFO_CNT << UART_RXFIFO_CNT_S)) {
        RcvChar = READ_PERI_REG(UART_FIFO(UART_ID_0)) & 0xFF;

        if (memberData[UART_ID_0].useRxBuff) {
            *(pRxBuff->pWritePos) = RcvChar;

            // insert here for get one command line from uart
            if (RcvChar == '\n') {
                pRxBuff->BuffState = WRITE_OVER;
            }

            pRxBuff->pWritePos++;

            if (pRxBuff->pWritePos == (pRxBuff->pRcvMsgBuff + RX_BUFF_SIZE)) { // overflow ...we may need more error handle here.
                pRxBuff->pWritePos = pRxBuff->pRcvMsgBuff;
            }

            if (pRxBuff->pWritePos == pRxBuff->pReadPos) {   // Prevent readbuffer overflow
                if (pRxBuff->pReadPos == (pRxBuff->pRcvMsgBuff + RX_BUFF_SIZE)) {
                    pRxBuff->pReadPos = pRxBuff->pRcvMsgBuff;
                }
                else {
                    pRxBuff->pReadPos++;
                }
            }
        }

        if (memberData[UART_ID_0].HWSDelegate) {
            unsigned short cc;
            cc = (pRxBuff->pWritePos < pRxBuff->pReadPos) ? ((pRxBuff->pWritePos + RX_BUFF_SIZE) - pRxBuff->pReadPos)
                                                            : (pRxBuff->pWritePos - pRxBuff->pReadPos);
            memberData[UART_ID_0].HWSDelegate(Serial, RcvChar, cc);
        }
    }
}
#endif