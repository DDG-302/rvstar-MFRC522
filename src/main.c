#include "nuclei_sdk_hal.h"
#include "RC522.H"
#include "stdio.h"
#include "stdlib.h"
#include "gd32vf103_exmc.h"
#include "gd32vf103_exti.h"
// #include "RC522.C"

// NSS为低时从机接收数据流，若要接收多个数据流，则NSS需要拉高一次

uint8_t DefaultKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t DDG_Key_block[16] = {1, 2, 3, 4, 5, 6 ,255, 7, 128, 105, 255, 255, 255, 255, 255, 255};

uint8_t write_data[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
uint8_t valid_key[16] = {114, 51, 4, 19, 19, 8, 10};
uint8_t id[4] = {0, 0, 0, 0}; // 卡id
uint8_t status = MI_ERR; // 读卡状态
uint8_t temp[4]; // 临时变量
uint8_t card_block_data[16]; // 读出的卡数据

void SPI_port_init()
{
  rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_AF); // 复用功能时钟
  rcu_periph_clock_enable(RCU_SPI1);
  rcu_periph_clock_enable(RCU_GPIOA);
  gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NSS);

  // 高电平不接受信号

  gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, MISO);
  gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, SCK);
  gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, MOSI);

  spi_i2s_deinit(SPI1);
  gpio_bit_set(GPIOA, NSS);

  spi_parameter_struct spi_para;
  spi_struct_para_init(&spi_para);

  spi_para.device_mode = SPI_MASTER;
  spi_para.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
  spi_para.frame_size = SPI_FRAMESIZE_8BIT;
  spi_para.nss = SPI_NSS_SOFT;
  spi_para.endian = SPI_ENDIAN_MSB;
  spi_para.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
  spi_para.prescale = SPI_PSC_128;

  spi_init(SPI1, &spi_para);
  // spi_enable(SPI1);
  // spi_nss_output_enable(SPI1);
  spi_i2s_data_frame_format_config(SPI1, SPI_FRAMESIZE_8BIT);
  // spi_nss_internal_high(SPI1);
}

void GPIO_init()
{
  // GPIOA和B已经在SPI_init中打开了
  gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_GREEN);
  gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_RED);
  gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_YELLOW);
  
  gpio_bit_set(GPIOB, LED_GREEN);
  gpio_bit_set(GPIOB, LED_RED);
  gpio_bit_set(GPIOB, LED_YELLOW);
}

void irq_init(){
  // int32_t returnCode = ECLIC_Register_IRQ(
  //   SysTimer_IRQn, 
  //   ECLIC_NON_VECTOR_INTERRUPT, 
  //   ECLIC_LEVEL_TRIGGER, 
  //   1, 
  //   0,
  //   mtimer_irq_handler
  //   ); /* register system timer interrupt */
  //  __enable_irq();
  //  setup_timer();
    

}

void find_card()
{
  unsigned char temp[4];
  uint8_t status = request(0x52, temp);

  if (status != MI_OK)
  {
    printf("error_happend...\n");
  }
  if (temp[0] == 0x04 && temp[1] == 0x00)
    printf("MFOne-S50\n");
  else if (temp[0] == 0x02 && temp[1] == 0x00)
    printf("MFOne-S70\n");
  else if (temp[0] == 0x44 && temp[1] == 0x00)
    printf("MF-UltraLight\n");
  else if (temp[0] == 0x08 && temp[1] == 0x00)
    printf("MF-Pro\n");
  else if (temp[0] == 0x44 && temp[1] == 0x03)
    printf("MF Desire\n");
  else
    printf("Unknown\ttemp[0]=%d temp[1]=%d\n", temp[0], temp[1]);
}

void EXTI0_IRQHandler()
{
}

void show_card_data(){
  status = MI_ERR;
  status = request(0x26, temp);
  if (status != MI_OK)
  {
    printf("waiting for card...\n");
    return;
  }
  status = anti_coll(id);
  if (status != MI_OK)
  {
    return;
  }
  
  status = select_card(id);
  if (status != MI_OK)
  {
    return;
  }


  
  if (status == MI_OK)
  {
    for(int i = 0; i < 64; i++){
      status = auth_state(0x60, i, DefaultKey, id);
      read_card(i, card_block_data);
      printf("%d: ", i);
      for(int j = 0; j < 16; j++){
        printf("%x ", card_block_data[j]);
      }
      printf("\n");
    }
  }
  else{
    printf("key error\n");
  }

  
  card_halt();
  printf("ok!\n");
  delay_1ms(5000);
}

void read_card_loop_debug()
{
  // find_card();
  status = MI_ERR;
  status = request(0x26, temp);
  status = anti_coll(id);
  if (status == MI_OK)
  {
    printf("anticoll correct\n");
  }
  else
  {
    printf("anticoll error\n");
  }
  
  for (int i = 0; i < 4; i++)
  {
    printf("%x ", id[i]);
  }

  printf("\n");
  status = select_card(id);
  if (status != MI_OK)
  {
    printf("select error\n");
    printf("error_is:%x\n", status);
  }
  else
  {
    printf("select correct\n");
  }

  status = auth_state(0x60, 4, DDG_Key_block, id);
  if (status == MI_OK)
  {
    printf("key correct\n");
  }
  else
  {
    printf("key error\n");
  }
  read_card(7, card_block_data);
  card_halt();
}

int read_card_loop(){
  status = MI_ERR;
  status = request(0x26, temp);
  status = anti_coll(id);
  if (status != MI_OK)
  {
    return;
  }
  
  status = select_card(id);
  if (status != MI_OK)
  {
    return;
  }


  status = auth_state(0x60, 4, DDG_Key_block, id);
  if (status == MI_OK)
  {
    printf("key correct\n");
  }
  else
  {
    printf("key error\n");
  }
  read_card(7, card_block_data);
  card_halt();
}

int main()
{
  SPI_port_init();
  spi_enable(SPI1);
  // GPIO_init();

  int count = 0;



  RC522_init();
  antenna_on();

  uint8_t card_addr = 4;                               // 指定的地址，第一扇区第一块
  uint8_t valid_data[7] = {114, 51, 4, 19, 19, 8, 10}; // 指定校验码

  // 与M1通信流程
  // 寻卡 -> 防冲撞(获得id) -> 选卡 -> 校验密码 -> 读/写
  while (1)
  {

    // printf("no:%d\t\n", count);
    // count = (count + 1) % 65530;
    // find_card_loop_debug();
    show_card_data();
    delay_1ms(1000);
  }
}
