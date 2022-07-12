
#include "uart_register.h"


  // install_uart_tout();

  
void mycallback(){
  // Serial.println("Working");
  flag = 1;

}

void serialEvent(){
  
}

void uart0_rx_intr_handler(void *para){

  uint8_t Rcv_Char;
  uint8_t uart_no= UART0;
  uint8_t fifo_len = 0;
  uint8_t buf_idx = 0;
  uint32_t uart_intr_status = READ_PERI_REG(UART_INT_ST(uart_no)); //get uart intr status
  while (uart_intr_status != 0x0) {
    if (UART_FRM_ERR_INT_ST == (uart_intr_status & UART_FRM_ERR_INT_ST)){ //if it is caused by a frm_err interrupt
      WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_FRM_ERR_INT_CLR);
      Serial.println("caused by a frm_err interrupt");
    } else if (UART_RXFIFO_FULL_INT_ST == (uart_intr_status & UART_RXFIFO_FULL_INT_ST)) { //if it is caused by a fifo_full interrupt
      Serial.println("caused by a fifo_full interrupt");
      fifo_len = (READ_PERI_REG(UART_STATUS(uart_no)) >> UART_RXFIFO_CNT_S) & UART_RXFIFO_CNT; //read rx fifo length
      char r[fifo_len];
      buf_idx = 0;
      while (buf_idx < fifo_len){
        r[buf_idx] = READ_PERI_REG(UART_FIFO(uart_no)) & 0xFF;
        buf_idx++;
      }
      r[fifo_len] = '\0';
      WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_RXFIFO_FULL_INT_CLR); //clear full interrupt state
    } else if (UART_RXFIFO_TOUT_INT_ST == (uart_intr_status & UART_RXFIFO_TOUT_INT_ST)) { //if it is caused by a time_out interrupt
      // Serial.println("caused by a time_out interrupt");
      mycallback();
      // fifo_len = (READ_PERI_REG(UART_STATUS(uart_no)) >> UART_RXFIFO_CNT_S) & UART_RXFIFO_CNT; //read rx fifo length
      // char r[fifo_len];
      
      // buf_idx = 0;
      // while (buf_idx < fifo_len) {
      //   r[buf_idx] = READ_PERI_REG(UART_FIFO(uart_no)) & 0xFF;
      //   buf_idx++;
      // }
      // r[fifo_len] = '\0';

      // esp_now_send(0, (uint8_t*)r, buf_idx);
      // Serial.write((uint8_t*)r, buf_idx);
      WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_RXFIFO_TOUT_INT_CLR); //clear full interrupt state
    } else if (UART_TXFIFO_EMPTY_INT_ST == (uart_intr_status & UART_TXFIFO_EMPTY_INT_ST)){ //if it is caused by a tx_empty interrupt
      Serial.println("caused by a tx_empty interrupt");
      WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_TXFIFO_EMPTY_INT_CLR);
      CLEAR_PERI_REG_MASK(UART_INT_ENA(uart_no), UART_TXFIFO_EMPTY_INT_ENA);
    } else {

    }

    uart_intr_status = READ_PERI_REG(UART_INT_ST(uart_no)); //update interrupt status

  }


}

static void install_uart_tout(){
  ETS_UART_INTR_DISABLE(); //Disable UART Interrupt
  ETS_UART_INTR_ATTACH(uart0_rx_intr_handler, NULL); //Attach handler function to uart0_rx_intr_handler

  SET_PERI_REG_MASK(UART_INT_ENA(0), UART_RXFIFO_TOUT_INT_ENA);

  WRITE_PERI_REG(UART_CONF1(0), UART_RX_TOUT_EN | 
    ((0x2 & UART_RX_TOUT_THRHD) << UART_RX_TOUT_THRHD_S)); //Enable UART RX Timeout function and set the timeout period as the time transmitting 2 bits

  WRITE_PERI_REG(UART_INT_CLR(0), 0xffff); //Clear UART Interrupts flags
  SET_PERI_REG_MASK(UART_INT_ENA(0), UART_RXFIFO_TOUT_INT_ENA); //Enable UART RX Timeout interrupt
  CLEAR_PERI_REG_MASK(UART_INT_ENA(0), UART_RXFIFO_FULL_INT_ENA); //Disable UART RX Full interrupt

  ETS_UART_INTR_ENABLE();
}
