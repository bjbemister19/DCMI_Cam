#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/cm3/nvic.h>
#include <stdlib.h>
#include <libopencm3/stm32/dma.h>

#include "dcmi_cam.h"
#include "nucleo_clock.h"
#include "rw_i2c.h"
#include "io_util.h"
#include "intr_util.h"
//#include "stm32f4xx_hal_dcmi.h"

#include "ov5642_regs.h"

uint32_t sensor_addr = 0x78;
// Image buffer

//#define PERIPH_BASE           ((uint32_t)0x40000000) /*!< Peripheral base address in the alias region */
//#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000)
//#define DCMI_BASE             (AHB2PERIPH_BASE + 0x50000)

#define mmio32(x)   (*(volatile uint32_t *)(x))


void delay_cycles( uint32_t cycles )
{
    while ( cycles ) {
        __asm__("nop");
        cycles--;
    }
}

void rcc_setup(void){
    rcc_periph_clock_enable(RCC_DCMI);
    //rcc_periph_clock_enable(RCC_AHB2ENR_DCMIEN);
    //rcc_periph_clock_enable(RCC_AHB2LPENR_DCMILPEN);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    // enable clocks for i2c
    rcc_periph_clock_enable(RCC_I2C2);
    // clock for USART2
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_DMA2);
}

void gpio_setup(void){
    //DCMI Setup
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO4 | GPIO6 | GPIO9 | GPIO10);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO4 | GPIO6 | GPIO9 | GPIO10);
    gpio_set_af(GPIOA, GPIO_AF13, GPIO4 | GPIO6 | GPIO9 | GPIO10 );

    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO6 | GPIO7 | GPIO8 | GPIO9 | GPIO10);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO6 | GPIO7 | GPIO8 | GPIO9 | GPIO10);
    gpio_set_af(GPIOB, GPIO_AF13, GPIO6 | GPIO7 | GPIO8 | GPIO9 | GPIO10 );  

    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO0 | GPIO8 | GPIO9 | GPIO10 | GPIO11);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO0 | GPIO8 | GPIO9 | GPIO10 | GPIO11);
    gpio_set_af(GPIOC, GPIO_AF13, GPIO0 | GPIO8 | GPIO9 | GPIO10 | GPIO11 );

    // usart pins PA02, PA03 set up
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);

    // select usart alternate function
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3 );

    // i2c pin setup SCLK is PB10 SDATA is PB3
    gpio_set(GPIOB, GPIO3|GPIO10);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3|GPIO10);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, 
        GPIO3 | GPIO10);
    gpio_set_af(GPIOB, GPIO_AF4, GPIO3 | GPIO10);

    // Set Power down pin to PC0
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);
    //gpio_set(GPIOC,GPIO0);
}

static void write_registers(uint16_t reg, uint8_t data){
   uint8_t tx_buf[3] = {reg >> 8, reg & 0x00ff,data};
   rw_i2c_write( I2C2, sensor_addr >> 1, tx_buf, 1 );
}

static void write_struct_regs(const struct sensor_reg reglist[]){
    uint16_t reg_addr = 0;
    uint8_t reg_val = 0; 
    const struct sensor_reg *next = reglist;
    while ((reg_addr != 0xffff) | (reg_val != 0xff)){
       reg_addr = next->reg;
       //char  buf[6];
       //__itoa(reg_addr, buf, 16);
       //display_line(USART2,buf);    
       reg_val = next->val;
       //char  buf2[6];
       //__itoa(reg_val, buf2, 16);
       //display_line(USART2,buf2); 
       write_registers(reg_addr, reg_val);
       next = next + 1;
    }
}

static void init_jpg(){
   //Set up for JPG capture
   //uint8_t tx_buf[1];
   //rw_i2c_read( I2C2, 0x3008, tx_buf, 1 );
   //char  buf[6];
   //__itoa(tx_buf[0], buf, 16);
   //display_line(USART2,buf);   
   write_registers(0x3008, 0x80);
   //uint8_t tx_buf2[1];
   //tx_buf2[0] = 0;
   //rw_i2c_read( I2C2, sensor_addr, tx_buf2, 1 );
   //char  buf2[6];
   //__itoa(tx_buf2[0], buf2, 16);
   //display_line(USART2,buf2); 
   write_struct_regs(OV5642_QVGA_Preview);
   delay_cycles(10000);
   write_struct_regs(OV5642_JPEG_Capture_QSXGA);
   write_struct_regs(ov5642_320x240);
   write_registers(0x3818, 0xa8);
   write_registers(0x3621, 0x10);
   write_registers(0x3801, 0xb0);
   write_registers(0x4407, 0x0C); 
}

static void power_up(){
   //lower the powerdown pin to ensure cam is on
   gpio_clear(GPIOC,GPIO0);
   init_jpg();
}
/**
static void power_down(){
   gpio_set(GPIOC,GPIO0);
   delay_cycles(1000000);
   gpio_clear(GPIOC,GPIO0);
}
*/
static void set_capture_mode(uint32_t mode){
   uint32_t control_reg = mmio32(CR);
   control_reg &= ~(DCMI_CR_CM);
   control_reg |= mode;
   mmio32(CR) = control_reg;
}

static void set_extended_data_mode(uint32_t mode){
   uint32_t control_reg = mmio32(CR);
   control_reg &= ~(DCMI_CR_EDM_0);
   control_reg &= ~(DCMI_CR_EDM_1);
   control_reg |= mode;
   mmio32(CR) = control_reg;
}

static void set_capture_rate(uint32_t mode){
   uint32_t control_reg = mmio32(CR);
   control_reg &= ~(DCMI_CR_FCRC_0);
   control_reg &= ~(DCMI_CR_FCRC_1);
   control_reg |= mode;
   mmio32(CR) = control_reg;
}

static void set_PCKPolarity(uint32_t mode){
   uint32_t control_reg = mmio32(CR);
   control_reg &= ~(DCMI_CR_PCKPOL);
   control_reg |= mode;
   mmio32(CR) = control_reg;
}

static void set_HSPolarity(uint32_t mode){
   uint32_t control_reg = mmio32(CR);
   control_reg &= ~(DCMI_CR_HSPOL);
   control_reg |= mode;
   mmio32(CR) = control_reg;
}

static void set_VSPolarity(uint32_t mode){
   uint32_t control_reg = mmio32(CR);
   control_reg &= ~(DCMI_CR_VSPOL);
   control_reg |= mode;
   mmio32(CR) = control_reg;
}

static void set_synchro_mode(uint32_t mode){
   uint32_t control_reg = mmio32(CR);
   control_reg &= ~(DCMI_CR_ESS);
   control_reg |= mode;
   mmio32(CR) = control_reg;
}

static void DCMI_IT_FRAME_ENABLE(void){
   uint32_t interupt_enable_reg = mmio32(IER);
   interupt_enable_reg &= ~(DCMI_IER_FRAME_IE);
   interupt_enable_reg |= DCMI_IT_FRAME;
   mmio32(IER) = interupt_enable_reg;
}

static void DCMI_IT_OVF_ENABLE(void){
   uint32_t interupt_enable_reg = mmio32(IER);
   interupt_enable_reg &= ~(DCMI_IER_OVF_IE);
   interupt_enable_reg |= DCMI_IT_OVR;
   mmio32(IER) = interupt_enable_reg;
}

static void DCMI_IT_ERR_ENABLE(void){ 
   uint32_t interupt_enable_reg = mmio32(IER);
   interupt_enable_reg &= ~(DCMI_IER_ERR_IE);
   interupt_enable_reg |= DCMI_IT_ERR;
   mmio32(IER) = interupt_enable_reg;
}

static void set_jpg_mode(void){
   uint32_t control_reg = mmio32(CR);
   control_reg &= ~(DCMI_CR_JPEG);
   control_reg |= DCMI_JPEG_ENABLE;
   mmio32(CR) = control_reg;
}

static void DCMI_Cmd(uint32_t mode){
   uint32_t control_reg = mmio32(CR);
   control_reg &= ~(DCMI_CR_ENABLE);
   control_reg |= mode;
   mmio32(CR) = control_reg;
}

static void DCMI_CaptureCmd(uint32_t mode){
   uint32_t control_reg = mmio32(CR);
   control_reg &= ~(DCMI_CR_CAPTURE);
   control_reg |= mode;
   mmio32(CR) = control_reg;
}

volatile uint16_t frame_buffer[1024];

static void DCMI_Setup(){
    uint32_t control_reg = mmio32(CR);
    uint32_t usart = NVIC_USART2_IRQ;
    set_capture_mode(DCMI_MODE_SNAPSHOT);
    set_jpg_mode();
    set_extended_data_mode(DCMI_EXTEND_DATA_8B);
    set_capture_rate(DCMI_CR_ALL_FRAME);
    set_PCKPolarity(DCMI_PCKPOLARITY_RISING);
    set_HSPolarity(DCMI_HSPOLARITY_LOW);
    set_VSPolarity(DCMI_VSPOLARITY_HIGH);
    set_synchro_mode(DCMI_SYNCHRO_HARDWARE);
    DCMI_IT_FRAME_ENABLE();
    DCMI_IT_OVF_ENABLE();
    DCMI_IT_ERR_ENABLE();

    uint32_t control_reg2 = mmio32(CR);

    nvic_enable_irq(NVIC_DMA2_STREAM1_IRQ);

    dma_stream_reset(DMA2, DMA_STREAM1);
    dma_set_priority(DMA2, DMA_STREAM1, DMA_SxCR_PL_HIGH);
    dma_set_memory_size(DMA2, DMA_STREAM1, DMA_SxCR_MSIZE_8BIT);
    dma_set_peripheral_size(DMA2, DMA_STREAM1, DMA_SxCR_PSIZE_8BIT);
    dma_enable_memory_increment_mode(DMA2, DMA_STREAM1);
    dma_enable_circular_mode(DMA2, DMA_STREAM1);
    dma_set_transfer_mode(DMA2, DMA_STREAM1, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_set_peripheral_address(DMA2, DMA_STREAM1, DCMI_PERIPH_ADDR);
    dma_enable_fifo_mode(DMA2, DMA_STREAM1);
    dma_set_fifo_threshold(DMA2, DMA_STREAM1,DMA_SxFCR_FTH_4_4_FULL );		

    //  array to write image data
    dma_set_memory_address(DMA2, DMA_STREAM1, (uint32_t) frame_buffer);
    dma_set_number_of_data(DMA2, DMA_STREAM1, 1024 / 2);
    dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM1);

    dma_channel_select(DMA2, DMA_STREAM1, DMA_SxCR_CHSEL_7);
    dma_enable_stream(DMA2, DMA_STREAM1);
    
    nvic_enable_irq(NVIC_DCMI_IRQ);
    
    DCMI_Cmd(ENABLE);
    DCMI_CaptureCmd(CAPTURE);
    
}

volatile uint8_t temp_buffer[1024];

void dumpFrame(void) {

	uint8_t *buffer = (uint8_t *) frame_buffer;
	int length = 1024;
	// Copy every other byte from the main frame buffer to our temporary buffer (this converts the image to grey scale)
	int i;
	for (i = 1; i < length; i += 2) {
		temp_buffer[i / 2] = buffer[i];
	}
	// We only send the sync frame if it has been requested
	
    for (i = 0x7f; i > 0; i--) {
	   uint8_t val = i;
       usart_send_blocking(USART2,(uint16_t)&val);
	}

	for (i = 0; i < (length / 2); i++) {
		if (i > 100) {
			usart_send_blocking(USART2, (uint16_t)&temp_buffer[i]);
		} else {
			uint8_t val = 0xff;
			usart_send_blocking(USART2,(uint16_t)&val); // Change first 100 pixels to white to provide a reference for where the frame starts
		}
	}
	// Enable capture and DMA after we have sent the photo. This is a workaround for the timing issues I've been having where
	// the DMA transfer is not in sync with the frames being sent
    dma_enable_stream(DMA2, DMA_STREAM1);
	DCMI_Cmd(ENABLE);
	DCMI_CaptureCmd(CAPTURE);
}


uint32_t frame_flag = 0;

void dma1_stream5_isr(void)
{
    display_line(USART2,"DMA Interupt");
    if (dma_get_interrupt_flag(DMA1, DMA_STREAM5, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_STREAM5, DMA_TCIF);
        frame_flag = true;
        // finished signal
    }
}



void dcmi_isr(void){
   display_line(USART2,"DCMI Interupt");
   uint32_t status_reg = mmio32(SR);
   if((status_reg & 0x0001) == 1){
      dma_disable_stream(DMA2, DMA_STREAM1);
      DCMI_Cmd(DISABLE);
      DCMI_CaptureCmd(STOP_CAPTURE);
   }
}

int main( void )
{
    ram_intr_setup();
    nucleo_clock_sysclk( 64 );
    nucleo_clock_test_setup();
    rcc_setup();
    gpio_setup();    
    delay_cycles(1000000);    
    //usart_setup(USART2);
    pccom_setup( USART2 );
    display_line(USART2,"Init");    
    display_line(USART2,"Clock Set");
    rw_i2c_setup( I2C2, 100000 ); // 100000kHz
    display_line(USART2,"I2C Set");
    power_up();
    display_line(USART2,"Powered Up");
    DCMI_Setup();
    display_line(USART2,"Fin");
    while(1){
        if(frame_flag == 1){
           frame_flag = 0;
           dumpFrame();
        }
        //display_line(USART2,"looping");
    }
}
