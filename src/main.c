#include "nuclei_sdk_hal.h"
#include "RC522.H"
#include "stdio.h"
#include "stdlib.h"

// #include "RC522.C"

// NSS为低时从机接收数据流，若要接收多个数据流，则NSS需要拉高一次

// uint16_t NSS = GPIO_PIN_5;
// uint16_t SCK = GPIO_PIN_13;
// uint16_t MOSI = GPIO_PIN_15; // 主机输出
// uint16_t MISO = GPIO_PIN_14; // 主机输入

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
  rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_GPIOA);
  gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NSS);

  // 高电平不接受信号

  gpio_init(GPIOB, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, MISO);
  gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SCK);
  gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, MOSI);
  gpio_bit_reset(GPIOA, NSS);
  gpio_bit_reset(GPIOB, SCK);
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

int main()
{
  SPI_port_init();
  spi_enable(SPI1);
  // GPIO_init();

  int count = 0;
  gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1); // 板载LED灯（绿色）


  RC522_init();
  antenna_on();

  while (1)
  {
    printf("no:%d\t", count);
    count = (count + 1) % 65530;
    // uint16_t receive_data;
    // receive_data = read_byte(CommandReg);
    find_card();
    uint8_t id[4];
    uint8_t status;
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
      printf("%x", id[i]);
    }

    printf("\n");
    status = select_card(id);

    uint8_t DefaultKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    status = auth_state(0x60, 10, DefaultKey, id);
    if (status == MI_OK)
    {
      printf("key correct\n");
    }
    else
    {
      printf("key error\n");
    }
    uint8_t dddd[16];
    read_card(10, dddd);

    gpio_bit_set(GPIOA, GPIO_PIN_1);
    delay_1ms(500);
    gpio_bit_reset(GPIOA, GPIO_PIN_1);
    delay_1ms(500);
  }
}
