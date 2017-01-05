#ifndef DCMI_CAM_H
#define DCMI_CAM_H
#endif
/********************  Bits definition for DCMI_CR register  ******************/
#define DCMI_CR_CAPTURE                      ((uint32_t)0x00000001)
#define DCMI_CR_CM                           ((uint32_t)0x00000002)
#define DCMI_CR_CROP                         ((uint32_t)0x00000004)
#define DCMI_CR_JPEG                         ((uint32_t)0x00000008)
#define DCMI_CR_ESS                          ((uint32_t)0x00000010)
#define DCMI_CR_PCKPOL                       ((uint32_t)0x00000020)
#define DCMI_CR_HSPOL                        ((uint32_t)0x00000040)
#define DCMI_CR_VSPOL                        ((uint32_t)0x00000080)
#define DCMI_CR_FCRC_0                       ((uint32_t)0x00000100)
#define DCMI_CR_FCRC_1                       ((uint32_t)0x00000200)
#define DCMI_CR_EDM_0                        ((uint32_t)0x00000400)
#define DCMI_CR_EDM_1                        ((uint32_t)0x00000800)
#define DCMI_CR_CRE                          ((uint32_t)0x00001000)
#define DCMI_CR_ENABLE                       ((uint32_t)0x00004000)

/********************  Bits definition for DCMI_SR register  ******************/
#define DCMI_SR_HSYNC                        ((uint32_t)0x00000001)
#define DCMI_SR_VSYNC                        ((uint32_t)0x00000002)
#define DCMI_SR_FNE                          ((uint32_t)0x00000004)

/********************  Bits definition for DCMI_RISR register  ****************/
#define DCMI_RISR_FRAME_RIS                  ((uint32_t)0x00000001)
#define DCMI_RISR_OVF_RIS                    ((uint32_t)0x00000002)
#define DCMI_RISR_ERR_RIS                    ((uint32_t)0x00000004)
#define DCMI_RISR_VSYNC_RIS                  ((uint32_t)0x00000008)
#define DCMI_RISR_LINE_RIS                   ((uint32_t)0x00000010)

/********************  Bits definition for DCMI_IER register  *****************/
#define DCMI_IER_FRAME_IE                    ((uint32_t)0x00000001)
#define DCMI_IER_OVF_IE                      ((uint32_t)0x00000002)
#define DCMI_IER_ERR_IE                      ((uint32_t)0x00000004)
#define DCMI_IER_VSYNC_IE                    ((uint32_t)0x00000008)
#define DCMI_IER_LINE_IE                     ((uint32_t)0x00000010)

/********************  Bits definition for DCMI_MISR register  ****************/
#define DCMI_MISR_FRAME_MIS                  ((uint32_t)0x00000001)
#define DCMI_MISR_OVF_MIS                    ((uint32_t)0x00000002)
#define DCMI_MISR_ERR_MIS                    ((uint32_t)0x00000004)
#define DCMI_MISR_VSYNC_MIS                  ((uint32_t)0x00000008)
#define DCMI_MISR_LINE_MIS                   ((uint32_t)0x00000010)

/********************  Bits definition for DCMI_ICR register  *****************/
#define DCMI_ICR_FRAME_ISC                   ((uint32_t)0x00000001)
#define DCMI_ICR_OVF_ISC                     ((uint32_t)0x00000002)
#define DCMI_ICR_ERR_ISC                     ((uint32_t)0x00000004)
#define DCMI_ICR_VSYNC_ISC                   ((uint32_t)0x00000008)
#define DCMI_ICR_LINE_ISC                    ((uint32_t)0x00000010)

#define DCMI_MODE_CONTINUOUS           ((uint32_t)0x00000000U) /*!< The received data are transferred continuously
                                                                    into the destination memory through the DMA             */
#define DCMI_MODE_SNAPSHOT             ((uint32_t)DCMI_CR_CM)  /*!< Once activated, the interface waits for the start of
                                                                    frame and then transfers a single frame through the DMA */
/**
  * @}
  */

/** @defgroup DCMI_Synchronization_Mode DCMI Synchronization Mode
  * @{
  */
#define DCMI_SYNCHRO_HARDWARE        ((uint32_t)0x00000000U)  /*!< Hardware synchronization data capture (frame/line start/stop)
                                                                   is synchronized with the HSYNC/VSYNC signals                  */
#define DCMI_SYNCHRO_EMBEDDED        ((uint32_t)DCMI_CR_ESS)  /*!< Embedded synchronization data capture is synchronized with
                                                                   synchronization codes embedded in the data flow               */
#define ENABLE                       ((uint32_t)DCMI_CR_ENABLE)
#define DISABLE                      ((uint32_t)0x00000000U)

#define CAPTURE                      ((uint32_t)DCMI_CR_CAPTURE)
#define STOP_CAPTURE                 ((uint32_t)0x00000000U)
/**
  * @}
  */

/** @defgroup DCMI_PIXCK_Polarity DCMI PIXCK Polarity
  * @{
  */
#define DCMI_PCKPOLARITY_FALLING    ((uint32_t)0x00000000U)     /*!< Pixel clock active on Falling edge */
#define DCMI_PCKPOLARITY_RISING     ((uint32_t)DCMI_CR_PCKPOL)  /*!< Pixel clock active on Rising edge  */

/**
  * @}
  */

/** @defgroup DCMI_VSYNC_Polarity DCMI VSYNC Polarity
  * @{
  */
#define DCMI_VSPOLARITY_LOW     ((uint32_t)0x00000000U)    /*!< Vertical synchronization active Low  */
#define DCMI_VSPOLARITY_HIGH    ((uint32_t)DCMI_CR_VSPOL)  /*!< Vertical synchronization active High */

/**
  * @}
  */

/** @defgroup DCMI_HSYNC_Polarity DCMI HSYNC Polarity
  * @{
  */ 
#define DCMI_HSPOLARITY_LOW     ((uint32_t)0x00000000U)    /*!< Horizontal synchronization active Low  */
#define DCMI_HSPOLARITY_HIGH    ((uint32_t)DCMI_CR_HSPOL)  /*!< Horizontal synchronization active High */

/**
  * @}
  */

/** @defgroup DCMI_MODE_JPEG DCMI MODE JPEG
  * @{
  */
#define DCMI_JPEG_DISABLE   ((uint32_t)0x00000000U)   /*!< Mode JPEG Disabled  */
#define DCMI_JPEG_ENABLE    ((uint32_t)DCMI_CR_JPEG)  /*!< Mode JPEG Enabled   */

/**
  * @}
  */

#define DCMI_SR_INDEX         ((uint32_t)0x2000) /*!< DCMI SR register index  */

#define DCMI_FLAG_HSYNC     ((uint32_t)DCMI_SR_INDEX|DCMI_SR_HSYNC) /*!< HSYNC pin state (active line / synchronization between lines)   */
#define DCMI_FLAG_VSYNC     ((uint32_t)DCMI_SR_INDEX|DCMI_SR_VSYNC) /*!< VSYNC pin state (active frame / synchronization between frames) */
#define DCMI_FLAG_FNE       ((uint32_t)DCMI_SR_INDEX|DCMI_SR_FNE)   /*!< FIFO not empty flag   */

/** @defgroup DCMI_Capture_Rate DCMI Capture Rate
  * @{
  */
#define DCMI_CR_ALL_FRAME            ((uint32_t)0x00000000U)     /*!< All frames are captured        */
#define DCMI_CR_ALTERNATE_2_FRAME    ((uint32_t)DCMI_CR_FCRC_0)  /*!< Every alternate frame captured */
#define DCMI_CR_ALTERNATE_4_FRAME    ((uint32_t)DCMI_CR_FCRC_1)  /*!< One frame in 4 frames captured */
#define DCMI_CR_ALL_FRAME            ((uint32_t)0x00000000U)     /*!< All frames are captured        */

#define DCMI_EXTEND_DATA_8B     ((uint32_t)0x00000000U)    /*!< Interface captures 8-bit data on every pixel clock  */
#define DCMI_IT_FRAME    ((uint32_t)DCMI_IER_FRAME_IE)    /*!< Capture complete interrupt      */
#define DCMI_IT_OVR      ((uint32_t)DCMI_IER_OVF_IE)      /*!< Overrun interrupt               */
#define DCMI_IT_ERR      ((uint32_t)DCMI_IER_ERR_IE)      /*!< Synchronization error interrupt */
#define DCMI_IT_VSYNC    ((uint32_t)DCMI_IER_VSYNC_IE)    /*!< VSYNC interrupt                 */
#define DCMI_IT_LINE     ((uint32_t)DCMI_IER_LINE_IE)     /*!< Line interrupt  */



#define CR        (DCMI_BASE)         /*!< DCMI control register 1,                       Address offset: 0x00 */
#define SR        (DCMI_BASE + 0x04)  /*!< DCMI status register,                          Address offset: 0x04 */
#define RISR      (DCMI_BASE + 0x08)  /*!< DCMI raw interrupt status register,            Address offset: 0x08 */
#define IER       (DCMI_BASE + 0x0C)  /*!< DCMI interrupt enable register,                Address offset: 0x0C */
#define MISR      (DCMI_BASE + 0x10)  /*!< DCMI masked interrupt status register,         Address offset: 0x10 */
#define ICR       (DCMI_BASE + 0x14)  /*!< DCMI interrupt clear register,                 Address offset: 0x14 */
#define ESCR      (DCMI_BASE + 0x18)  /*!< DCMI embedded synchronization code register,   Address offset: 0x18 */
#define ESUR      (DCMI_BASE + 0x1C)  /*!< DCMI embedded synchronization unmask register, Address offset: 0x1C */
#define CWSTRTR   (DCMI_BASE + 0x20)  /*!< DCMI crop window start,                        Address offset: 0x20 */
#define CWSIZER   (DCMI_BASE + 0x24)  /*!< DCMI crop window size,                         Address offset: 0x24 */
#define DR        (DCMI_BASE + 0x28)  /*!< DCMI data register,                            Address offset: 0x28 */

#define DCMI_PERIPH_ADDR ((uint32_t)DCMI_BASE +0x28)

#define DCMI_IRQn   ((uint8_t)78)
