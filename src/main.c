#include "nuclei_sdk_hal.h"
#include "RC522.H"
#include "stdio.h"
#include "nuclei_sdk_soc.h"
#include "riscv_encoding.h"
// #include "RC522.C"

// NSS为低时从机接收数据流，若要接收多个数据流，则NSS需要拉高一次

uint8_t DefaultKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t DDG_Key_block[16] = {1, 2, 3, 4, 5, 6, 255, 7, 128, 105, 255, 255, 255, 255, 255, 255}; // 我的控制块数据

// uint8_t write_data_test[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

uint8_t id[4] = {0, 0, 0, 0};                         // 卡id
uint8_t status = MI_ERR;                              // 读卡状态
uint8_t card_block_data[16];                          // 读出的卡数据
uint8_t card_addr = 4;                                // 指定的地址，第一扇区第一块
uint8_t valid_data[16] = {114, 51, 4, 19, 19, 8, 10}; // 指定校验码
uint8_t zeros_data[16] = {0};
BOOL timer_ok = FALSE;
uint8_t condition = 0;     // 业务流程，0：读卡，1：写卡，2：删除
uint8_t condition_num = 3; // n种情况

#define CONDITION_READ_CARD 0
#define CONDITION_WRITE_CARD 1
#define CONDITION_DEL_CARD 2
uint8_t read_status = CARD_NOT_FOUND;

#define DOOR_CLOSE 0
#define DOOR_OPEN 1
#define DOOR_CLOSING 2
#define DOOR_OPENING 3
uint8_t doors_condition = DOOR_CLOSE;

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
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_BOARD_GREEN);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_BOARD_RED);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_BOARD_BLUE);

    gpio_bit_set(GPIOA, LED_BOARD_GREEN);
    gpio_bit_set(GPIOA, LED_BOARD_RED);
    gpio_bit_set(GPIOA, LED_BOARD_BLUE);

    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_PERIPH_RED);
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_PERIPH_GREEN);
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_PERIPH_YELLOW);

    gpio_bit_reset(GPIOB, LED_PERIPH_RED);
    gpio_bit_reset(GPIOB, LED_PERIPH_GREEN);
    gpio_bit_reset(GPIOB, LED_PERIPH_YELLOW);

    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_PERIPH_DOOR_RED);
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_PERIPH_DOOR_GREEN);
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_PERIPH_DOOR_YELLOW);

    gpio_bit_set(GPIOB, LED_PERIPH_DOOR_RED);
    gpio_bit_reset(GPIOB, LED_PERIPH_DOOR_GREEN);
    gpio_bit_reset(GPIOB, LED_PERIPH_DOOR_YELLOW);
    doors_condition = DOOR_CLOSE;
}

__INTERRUPT void TIMER2_IRQHandler()
{
    printf("irq!\n");
    if (SET == timer_interrupt_flag_get(TIMER2, TIMER_INT_FLAG_UP))
    {
        timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
        if (RESET == gpio_output_bit_get(GPIOA, LED_BOARD_GREEN))
        {
            gpio_bit_set(GPIOA, LED_BOARD_GREEN);
        }
        if (RESET == gpio_output_bit_get(GPIOA, LED_BOARD_RED))
        {
            gpio_bit_set(GPIOA, LED_BOARD_RED);
        }
        if (RESET == gpio_output_bit_get(GPIOA, LED_BOARD_BLUE))
        {
            gpio_bit_set(GPIOA, LED_BOARD_BLUE);
        }

        timer_disable(TIMER2);
    }
}

__INTERRUPT void TIMER1_IRQHandler()
{
    if (SET == timer_interrupt_flag_get(TIMER1, TIMER_INT_FLAG_UP))
    {
        timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);
        switch (doors_condition)
        {
        case DOOR_CLOSE:
            doors_condition = DOOR_OPENING;
            gpio_bit_reset(GPIOB, LED_PERIPH_DOOR_GREEN);
            gpio_bit_reset(GPIOB, LED_PERIPH_DOOR_RED);
            gpio_bit_set(GPIOB, LED_PERIPH_DOOR_YELLOW);
            timer_disable(TIMER1);
            door_open_close_simulation_timer_set();
            timer_enable(TIMER1);
            break;
        case DOOR_OPEN:
            doors_condition = DOOR_CLOSING;
            gpio_bit_reset(GPIOB, LED_PERIPH_DOOR_GREEN);
            gpio_bit_reset(GPIOB, LED_PERIPH_DOOR_RED);
            gpio_bit_set(GPIOB, LED_PERIPH_DOOR_YELLOW);
            timer_disable(TIMER1);
            door_open_close_simulation_timer_set();
            timer_enable(TIMER1);
            break;
        case DOOR_CLOSING:
            doors_condition = DOOR_CLOSE;
            gpio_bit_reset(GPIOB, LED_PERIPH_DOOR_GREEN);
            gpio_bit_set(GPIOB, LED_PERIPH_DOOR_RED);
            gpio_bit_reset(GPIOB, LED_PERIPH_DOOR_YELLOW);
            timer_disable(TIMER1);
            door_open_close_simulation_timer_set();

            break;
        case DOOR_OPENING:
            doors_condition = DOOR_OPEN;
            gpio_bit_set(GPIOB, LED_PERIPH_DOOR_GREEN);
            gpio_bit_reset(GPIOB, LED_PERIPH_DOOR_RED);
            gpio_bit_reset(GPIOB, LED_PERIPH_DOOR_YELLOW);
            timer_disable(TIMER1);
            door_open_timer_set();
            timer_enable(TIMER1);
            break;
        default:
            break;
        }
    }
}

// 按钮及其中断初始化
void key_exti_init(void)
{

    rcu_periph_clock_enable(RCU_AF);
    gpio_exti_source_select(WAKEUP_KEY_EXTI_PORT_SOURCE, WAKEUP_KEY_EXTI_PIN_SOURCE);

    exti_init(EXTI_0, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_0);
}

// 按钮中断
void EXTI0_IRQHandler(void)
{

    if (RESET != exti_interrupt_flag_get(WAKEUP_KEY_PIN))
    {
        if (RESET == gd_rvstar_key_state_get(KEY_WAKEUP))
        {
            condition = (condition + 1) % condition_num;
            read_status = CARD_NOT_FOUND;
        }
    }
    exti_interrupt_flag_clear(WAKEUP_KEY_PIN);
}

void irq_init()
{
    ECLIC_Register_IRQ(EXTI0_IRQn,                 // 中断线
                       ECLIC_NON_VECTOR_INTERRUPT, // 向量中断/非向量中断
                       ECLIC_LEVEL_TRIGGER,        // 中断触发条件
                       0,                          // 中断等级
                       0,                          // 中断优先级
                       NULL);                      // 中断处理函数
    ECLIC_Register_IRQ(TIMER2_IRQn,                // 中断线
                       ECLIC_VECTOR_INTERRUPT,     // 向量中断/非向量中断
                       ECLIC_LEVEL_TRIGGER,        // 中断触发条件
                       0,                          // 中断等级
                       0,                          // 中断优先级
                       NULL);
    ECLIC_Register_IRQ(TIMER1_IRQn,            // 中断线
                       ECLIC_VECTOR_INTERRUPT, // 向量中断/非向量中断
                       ECLIC_LEVEL_TRIGGER,    // 中断触发条件
                       0,                      // 中断等级
                       0,                      // 中断优先级
                       NULL);
    __enable_irq();
}

// signal_delat_seconds 指示灯亮起时间
void signal_timer_init(uint8_t signal_delay_seconds)
{
    // TIMER2 指示灯时间
    rcu_periph_clock_enable(RCU_TIMER2);
    timer_deinit(TIMER2);
    timer_update_source_config(TIMER2, TIMER_UPDATE_SRC_REGULAR);
    timer_parameter_struct timer_initpara;
    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler = 53999;
    timer_initpara.period = 2000 * signal_delay_seconds; // 2000大约1秒
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_init(TIMER2, &timer_initpara);
    timer_interrupt_enable(TIMER2, TIMER_INT_UP);
}

// 设置timer1模拟开关
void door_open_close_simulation_timer_set()
{
    rcu_periph_clock_enable(RCU_TIMER1);
    timer_deinit(TIMER1);
    timer_update_source_config(TIMER1, TIMER_UPDATE_SRC_REGULAR);
    timer_parameter_struct timer_initpara_2;
    timer_struct_para_init(&timer_initpara_2);
    timer_initpara_2.prescaler = 53999;
    timer_initpara_2.period = 2000 * 1; // 2000大约1秒
    timer_initpara_2.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara_2.counterdirection = TIMER_COUNTER_UP;

    timer_init(TIMER1, &timer_initpara_2);
    timer_interrupt_enable(TIMER1, TIMER_INT_UP);
}

// 设置timer1等待开启时间
void door_open_timer_set()
{
    // 门开启关闭时间
    rcu_periph_clock_enable(RCU_TIMER1);
    timer_deinit(TIMER1);
    timer_update_source_config(TIMER1, TIMER_UPDATE_SRC_REGULAR);
    timer_parameter_struct timer_initpara_2;
    timer_struct_para_init(&timer_initpara_2);
    timer_initpara_2.prescaler = 53999;
    timer_initpara_2.period = 2000 * 5; // 2000大约1秒
    timer_initpara_2.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara_2.counterdirection = TIMER_COUNTER_UP;

    timer_init(TIMER1, &timer_initpara_2);
    timer_interrupt_enable(TIMER1, TIMER_INT_UP);
}

// 用默认密码遍历卡扇区
void show_card_data()
{
    unsigned char temp[4];
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

// 带有串口输出信息的读卡循环
void read_card_loop_debug()
{
    unsigned char temp[4];
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

    status = auth_state(0x60, card_addr, DDG_Key_block, id);
    if (status == MI_OK)
    {
        printf("key correct\n");
    }
    else
    {
        printf("key error\n");
    }
    read_card(card_addr, card_block_data);
    card_halt();
}

// 写控制块
uint8_t set_control_block()
{
    unsigned char temp[4];
    status = MI_ERR;
    status = request(0x26, temp);
    status = anti_coll(id);
    if (status != MI_OK)
    {
        return status;
    }

    status = select_card(id);
    if (status != MI_OK)
    {
        return status;
    }

    status = auth_state(0x60, card_addr, DDG_Key_block, id);
    if (status == MI_OK)
    {
        status = write_card(7, DDG_Key_block);
    }
    card_halt();
    return status;
}

// 重置控制块
uint8_t reset_control_block()
{
    unsigned char temp[4];
    status = MI_ERR;
    status = request(0x26, temp);
    status = anti_coll(id);
    if (status != MI_OK)
    {
        return status;
    }

    status = select_card(id);
    if (status != MI_OK)
    {
        return status;
    }

    status = auth_state(0x60, card_addr, DDG_Key_block, id);
    if (status == MI_OK)
    {
        status = write_card(7, DDG_Key_block);
    }
    card_halt();
    return status;
}

// 用于串口输出发现的卡片类型
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

// 输入16位数据，判断是否接受
BOOL is_card_data_verifed()
{
    for (uint8_t i = 0; i < 7; i++)
    {
        if (card_block_data[i] != valid_data[i])
        {
            return FALSE;
        }
    }
    return TRUE;
}

// 主业务循环
uint8_t card_loop(uint8_t condition)
{
    unsigned char temp[4];
    status = MI_ERR;
    if (condition == CONDITION_READ_CARD)
        status = request(0x52, temp);
    else
        status = request(0x26, temp);
    status = anti_coll(id);
    if (status != MI_OK)
    {
        return CARD_NOT_FOUND;
    }

    status = select_card(id);
    if (status != MI_OK)
    {
        return CARD_ERR;
    }

    status = auth_state(0x60, card_addr, DDG_Key_block, id);
    if (status != MI_OK)
    {
        return CARD_DENY;
    }

    switch (condition)
    {
    case CONDITION_READ_CARD:
        read_card(card_addr, card_block_data);
        if (!is_card_data_verifed())
        {
            return CARD_DENY;
        }
        break;
    case CONDITION_WRITE_CARD:
        status = write_card(card_addr, valid_data);
        if (status != MI_OK)
        {
            return CARD_ERR;
        }
        break;
    case CONDITION_DEL_CARD:
        status = write_card(card_addr, zeros_data);
        if (status != MI_OK)
        {
            return CARD_ERR;
        }

    default:
        break;
    }

    return CARD_VERIFIED;
}

void open_the_door()
{

    if (doors_condition == DOOR_OPEN)
    {
        timer_disable(TIMER1);
        door_open_timer_set();
        // door_open_close_simulation_timer_set();
        timer_enable(TIMER1);
    }
    else if (doors_condition == DOOR_CLOSE)
    {
        timer_disable(TIMER1);
        door_open_close_simulation_timer_set();
        doors_condition = DOOR_OPENING;
        gpio_bit_set(GPIOB, LED_PERIPH_DOOR_YELLOW);
        gpio_bit_reset(GPIOB, LED_PERIPH_DOOR_GREEN);
        gpio_bit_reset(GPIOB, LED_PERIPH_DOOR_RED);
        timer_enable(TIMER1);
    }
}

int main()
{
    gd_rvstar_key_init(WAKEUP_KEY_GPIO_PORT, KEY_MODE_EXTI);
    key_exti_init();
    signal_timer_init(2);

    irq_init();

    SPI_port_init();
    spi_enable(SPI1);
    GPIO_init();

    int count = 0;

    RC522_init();
    antenna_on();

    // 与M1通信流程
    // 寻卡 -> 防冲撞(获得id) -> 选卡 -> 校验密码 -> 读/写
    while (1)
    {
        delay_1ms(500);
        read_status = CARD_NOT_FOUND;
        // card_halt();
        read_status = card_loop(condition);
        printf("read_status=%d\n", read_status);
        printf("condition:%d\n\n", condition);
        switch (condition)
        {
        case CONDITION_READ_CARD:
            gpio_bit_reset(GPIOB, LED_PERIPH_YELLOW);
            gpio_bit_reset(GPIOB, LED_PERIPH_RED);
            gpio_bit_set(GPIOB, LED_PERIPH_GREEN);
            switch (read_status)
            {
            case CARD_NOT_FOUND:
                gpio_bit_set(GPIOA, LED_BOARD_RED);
                gpio_bit_set(GPIOA, LED_BOARD_GREEN);
                gpio_bit_set(GPIOA, LED_BOARD_BLUE);
                break;
            case CARD_DENY:
                gpio_bit_set(GPIOA, LED_BOARD_GREEN);
                gpio_bit_set(GPIOA, LED_BOARD_BLUE);
                gpio_bit_reset(GPIOA, LED_BOARD_RED);
                break;
            case CARD_ERR:
                gpio_bit_set(GPIOA, LED_BOARD_GREEN);
                gpio_bit_set(GPIOA, LED_BOARD_BLUE);
                gpio_bit_reset(GPIOA, LED_BOARD_RED);
                break;
            case CARD_VERIFIED:
                open_the_door();
                gpio_bit_set(GPIOA, LED_BOARD_RED);
                gpio_bit_set(GPIOA, LED_BOARD_BLUE);
                gpio_bit_reset(GPIOA, LED_BOARD_GREEN);
                break;
            }
            break;
        case CONDITION_WRITE_CARD:
            gpio_bit_set(GPIOB, LED_PERIPH_YELLOW);
            gpio_bit_reset(GPIOB, LED_PERIPH_RED);
            gpio_bit_reset(GPIOB, LED_PERIPH_GREEN);
            switch (read_status)
            {
            case CARD_NOT_FOUND:

                break;
            case CARD_VERIFIED:
                // condition = CONDITION_READ_CARD;
                timer_disable(TIMER2);
                gpio_bit_reset(GPIOA, LED_BOARD_GREEN);
                timer_enable(TIMER2);
                break;

            default:
                timer_disable(TIMER2);
                gpio_bit_reset(GPIOA, LED_BOARD_RED);
                timer_enable(TIMER2);
                break;
            }
            break;
        case CONDITION_DEL_CARD:
            gpio_bit_reset(GPIOB, LED_PERIPH_YELLOW);
            gpio_bit_set(GPIOB, LED_PERIPH_RED);
            gpio_bit_reset(GPIOB, LED_PERIPH_GREEN);
            switch (read_status)
            {
            case CARD_NOT_FOUND:
                break;
            case CARD_VERIFIED:
                // condition = CONDITION_READ_CARD;
                timer_disable(TIMER2);
                gpio_bit_reset(GPIOA, LED_BOARD_GREEN);
                timer_enable(TIMER2);
                break;

            default:
                timer_disable(TIMER2);
                gpio_bit_reset(GPIOA, LED_BOARD_RED);
                timer_enable(TIMER2);
                break;
            }
            break;
        default:
            break;
        }

        card_halt();
        // timer_disable(TIMER2);
    }
}
