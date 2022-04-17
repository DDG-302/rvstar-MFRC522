/* 实验1：点亮LED */
#include "nuclei_sdk_hal.h"
#include "RC522.H"
#include "stdio.h"
#include "stdlib.h"

// NSS为低时从机接收数据流，若要接收多个数据流，则NSS需要拉高一次
uint32_t NSS = GPIO_PIN_5;
uint32_t SCK = GPIO_PIN_13;
uint32_t MOSI = GPIO_PIN_15; // 主机输出
uint32_t MISO = GPIO_PIN_14; // 主机输入

void SPI_port_init()
{

    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NSS);

    // 高电平不接受信号
    // gpio_bit_set(GPIOB, NSS);
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, MISO);
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, SCK);
    // MOSI需初始化为浮空输入模式
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, MOSI);

    rcu_periph_clock_enable(RCU_SPI1);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF); // 复用外部时钟
    spi_i2s_deinit(SPI1);

    spi_parameter_struct spi_para;
    spi_struct_para_init(&spi_para);

    spi_para.device_mode = SPI_MASTER;
    spi_para.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
    spi_para.frame_size = SPI_FRAMESIZE_8BIT;
    spi_para.nss = SPI_NSS_SOFT;
    spi_para.endian = SPI_ENDIAN_LSB;
    spi_para.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
    spi_para.prescale = SPI_PSC_128;

    spi_init(SPI1, &spi_para);
    // spi_enable(SPI1);
    // spi_nss_output_enable(SPI1);
    // spi_i2s_data_frame_format_config(SPI1, SPI_FRAMESIZE_8BIT);
    // spi_nss_internal_high(SPI1);
}

uint8_t convert_reg_addr(int8_t addr, BOOL is_read)
{
    // addr格式为
    // 7       -       6 - 5 - 4 - 3 - 2 - 1 - 0
    // 1/0(read/write) |          address     | RFU
    // RFU要求一直为0，地址是6位，要放到第1-6位
    uint8_t address = 0;
    if (is_read)
    {
        address = 0b10000000;
    }
    int mask = 0b111111;
    address += (addr & mask) << 1;
    return address;
}

// addr: 设备内寄存器地址
// data: 要写入的数据
uint16_t write_byte(uint8_t addr, uint8_t data)
{
    uint16_t write_data_cmd;
    write_data_cmd = ((uint16_t)convert_reg_addr(addr, FALSE) << 8) + (uint16_t)data;
    return write_data_cmd;
}

// addr1: 读取的寄存器地址
// addr2: 读取的寄存器地址2，可以为空
uint16_t read_byte(uint8_t addr1, uint8_t addr2)
{
    uint16_t read_data_cmd;
    read_data_cmd = ((uint16_t)convert_reg_addr(addr1, TRUE) << 8) + ((uint16_t)convert_reg_addr(addr2, TRUE));
    return read_data_cmd;
}

int main()
{
    SPI_port_init();
    spi_enable(SPI1);
    int count = 0;

    rcu_periph_clock_enable(RCU_GPIOA);
    // 将PA1初始化为推挽输出模式
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);

/***** 初始化reset *****/
    // delay_1ms(2000);
    // printf("waiting flag 1 INIT\n\r");
    // while (RESET == spi_i2s_flag_get(SPI1, SPI_FLAG_TBE))
    //     ;
    // printf("sending\n\r");

    // gpio_bit_reset(GPIOB, NSS);
    // spi_i2s_data_transmit(SPI1, convert_reg_addr(CommandReg, FALSE));
    // spi_i2s_data_transmit(SPI1, PCD_RESETPHASE);
    // printf("waiting init response\n\r");
    // while (RESET == spi_i2s_flag_get(SPI1, SPI_FLAG_RBNE))
    //     ;
    // spi_i2s_data_receive(SPI1);
    // gpio_bit_set(GPIOB, NSS);
    // delay_1ms(100);
    gpio_bit_reset(GPIOB, NSS);

    while (1)
    {
        printf("no:%d\n", count);
        count = (count + 1) % 65530;
        while (RESET == spi_i2s_flag_get(SPI1, SPI_FLAG_TBE))// 输入缓冲
            ;
        printf("sending\n");

        // gpio_bit_reset(GPIOB, NSS);
        spi_i2s_data_transmit(SPI1, convert_reg_addr(CommandReg, TRUE));

        while (RESET == spi_i2s_flag_get(SPI1, SPI_FLAG_RBNE))// 读取缓冲
            ;
        // gpio_bit_set(GPIOB, NSS);
        uint16_t *receive_data;
        printf("reading\n");
        receive_data = spi_i2s_data_receive(SPI1);
        printf("%u\n", receive_data);

        gpio_bit_set(GPIOA, GPIO_PIN_1);
        delay_1ms(500);
        gpio_bit_reset(GPIOA, GPIO_PIN_1);
        delay_1ms(500);
    }
}
