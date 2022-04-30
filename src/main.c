#include "nuclei_sdk_hal.h"
#include "RC522.H"
#include "stdio.h"
#include "nuclei_sdk_soc.h"
#include "riscv_encoding.h"
// #include "RC522.C"

// NSS为低时从机接收数据流，若要接收多个数据流，则NSS需要拉高一次

uint8_t DefaultKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 
uint8_t DDG_Key_block[16] = {1, 2, 3, 4, 5, 6, 255, 7, 128, 105, 255, 255, 255, 255, 255, 255};// 我的控制块数据

// uint8_t write_data_test[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
uint8_t valid_key[16] = {114, 51, 4, 19, 19, 8, 10}; // 有效密码
uint8_t id[4] = {0, 0, 0, 0};                        // 卡id
uint8_t status = MI_ERR;                             // 读卡状态
uint8_t temp[4];                                     // 临时变量
uint8_t card_block_data[16];                         // 读出的卡数据
uint8_t card_addr = 4;                               // 指定的地址，第一扇区第一块
uint8_t valid_data[7] = {114, 51, 4, 19, 19, 8, 10}; // 指定校验码

BOOL timer_ok = FALSE;
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

__INTERRUPT void TIMER2_IRQHandler()
{
    if (SET == timer_interrupt_flag_get(TIMER2, TIMER_INT_FLAG_UP))
    {
        timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
        printf("irq!\n");
        // to do
    }
    timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
    timer_disable(TIMER2);
    timer_ok = FALSE;


}

void irq_init()
{
    // uint32_t return_code = ECLIC_Register_IRQ(
    //     TIMER1_IRQn,
    // 	ECLIC_NON_VECTOR_INTERRUPT,
    // 	ECLIC_LEVEL_TRIGGER,
    // 	1,
    // 	0,
    // 	NULL); /* 配置timer中断*/
    // ECLIC_SetTrigIRQ(TIMER1, );
    // ECLIC_SetPriorityIRQ(TIMER1, 0);
    // ECLIC_Init();
    // ECLIC_EnableIRQ(TIMER1);
    // timer_parameter_struct
    // timer_init(TIMER1, &timer_struct);
    // timer_enable(TIMER1);
}

// __INTERRUPT void eclic_mtip_handler(){
//     printf("tick\n");
// }

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

void my_timer_init()
{
    rcu_periph_clock_enable(RCU_TIMER2);
    timer_deinit(TIMER2);
    timer_update_source_config(TIMER2, TIMER_UPDATE_SRC_REGULAR);
    timer_parameter_struct timer_initpara;
    timer_struct_para_init(&timer_initpara) ;
    timer_initpara.prescaler = 53999;
    timer_initpara.period = 2000; // 大约1秒
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    
    timer_init(TIMER2, &timer_initpara);
    timer_interrupt_enable(TIMER2, TIMER_INT_UP);
    timer_enable(TIMER2);
    timer_ok = TRUE;
}

// void mtimer_irq_handler()
// {
// }

// void EXTI0_IRQHandler()
// {
// }



// 用默认密码遍历卡扇区
void show_card_data()
{
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
        for (int i = 0; i < 64; i++)
        {
            status = auth_state(0x60, i, DefaultKey, id);
            read_card(i, card_block_data);
            printf("%d: ", i);
            for (int j = 0; j < 16; j++)
            {
                printf("%x ", card_block_data[j]);
            }
            printf("\n");
        }
    }
    else
    {
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

int read_card_loop()
{
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

void key_exti_init(void)
{
    
    rcu_periph_clock_enable(RCU_AF);
    gpio_exti_source_select(WAKEUP_KEY_EXTI_PORT_SOURCE, WAKEUP_KEY_EXTI_PIN_SOURCE);

    exti_init(EXTI_0, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_0);
}

__INTERRUPT void EXTI0_IRQHandler(void)
{
    if(!timer_ok){
        timer_enable(TIMER2);
    }
    else{
        timer_disable(TIMER2);
    }
    timer_ok = !timer_ok;
    if (RESET != exti_interrupt_flag_get(WAKEUP_KEY_PIN))
    {

        if (RESET == gd_rvstar_key_state_get(KEY_WAKEUP))
        {
            /* toggle RED led */
            printf("key\n");
        }
    }
    /* clear EXTI lines pending flag */
    exti_interrupt_flag_clear(WAKEUP_KEY_PIN);
}

int main()
{
    gd_rvstar_key_init(WAKEUP_KEY_GPIO_PORT, KEY_MODE_EXTI);
    key_exti_init();
    my_timer_init();
    uint32_t returnCode;
    returnCode = ECLIC_Register_IRQ(EXTI0_IRQn,             // 中断线
                                             ECLIC_VECTOR_INTERRUPT, // 向量中断/非向量中断
                                             ECLIC_LEVEL_TRIGGER,    // 中断触发条件
                                             1,                      // 中断等级
                                             0,                      // 中断优先级
                                             NULL);                  // 中断处理函数
     ECLIC_Register_IRQ(TIMER2_IRQn,             // 中断线
                                             ECLIC_VECTOR_INTERRUPT, // 向量中断/非向量中断
                                             ECLIC_LEVEL_TRIGGER,    // 中断触发条件
                                             1,                      // 中断等级
                                             0,                      // 中断优先级
                                             NULL);                  // 中
    __enable_irq();

    SPI_port_init();
    spi_enable(SPI1);
    // GPIO_init();

    int count = 0;

    RC522_init();
    antenna_on();

    // 与M1通信流程
    // 寻卡 -> 防冲撞(获得id) -> 选卡 -> 校验密码 -> 读/写
    while (1)
    {

        // printf("no:%d\t\n", count);
        // count = (count + 1) % 65530;
        // find_card_loop_debug();
        // show_card_data();
        printf("TIMER OK = %d\n", timer_ok);
        printf("main_loop\n");
        delay_1ms(1000);
        // timer_disable(TIMER2);
    }
}
