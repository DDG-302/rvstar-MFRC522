#include "nuclei_sdk_hal.h"
#include "RC522.H"
#include "stdio.h"

// TX是串口通信引脚

uint16_t convert_reg_addr(uint8_t addr, BOOL is_read)
{
    // addr格式为
    // 7       -       6 - 5 - 4 - 3 - 2 - 1 - 0
    // 1/0(read/write) |          address     | RFU
    // RFU要求一直为0，地址是6位，要放到第1-6位
    uint16_t address = 0;
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
void write_byte(uint16_t addr, uint16_t data)
{
    gpio_bit_reset(GPIOA, NSS); // 片选开

    /*  准备发送地址  */
    while (RESET == spi_i2s_flag_get(SPI1, SPI_FLAG_TBE))// 输入缓冲
        ;
    spi_i2s_data_transmit(SPI1, convert_reg_addr(addr, FALSE));
    while (RESET == spi_i2s_flag_get(SPI1, SPI_FLAG_RBNE))// 读取缓冲
        ;
    spi_i2s_data_receive(SPI1);// 读缓冲，重置缓冲区满信号

    /*  准备发送数据  */
    while (RESET == spi_i2s_flag_get(SPI1, SPI_FLAG_TBE))// 输入缓冲
        ;
    spi_i2s_data_transmit(SPI1, data); // 传数据
    while (RESET == spi_i2s_flag_get(SPI1, SPI_FLAG_RBNE))// 读取缓冲
        ;
    spi_i2s_data_receive(SPI1); // 读缓冲，重置缓冲区满信号

    // 发送完成
    gpio_bit_set(GPIOA, NSS); // 片选关  
}

// addr: 读取的寄存器地址
uint16_t read_byte(uint16_t addr)
{
    gpio_bit_reset(GPIOA, NSS); // 片选开

    /*  准备发送地址  */
    while (RESET == spi_i2s_flag_get(SPI1, SPI_FLAG_TBE))// 输入缓冲
        ;   
    spi_i2s_data_transmit(SPI1, convert_reg_addr(addr, TRUE));
    while (RESET == spi_i2s_flag_get(SPI1, SPI_FLAG_RBNE))// 读取缓冲
        ;
    spi_i2s_data_receive(SPI1); // 读缓冲，重置缓冲区满信号
    /*  再发送一个空地址
        RVSTAR要求发送缓冲区不为空才能读取
        如果不发送，则无法正常读取  */
    while (RESET == spi_i2s_flag_get(SPI1, SPI_FLAG_TBE))// 输入缓冲
        ;
    spi_i2s_data_transmit(SPI1, convert_reg_addr(0, TRUE)); // 发送空
    while (RESET == spi_i2s_flag_get(SPI1, SPI_FLAG_RBNE))// 读取缓冲
        ;
    uint16_t receive_data;
    receive_data = spi_i2s_data_receive(SPI1); // 读缓冲，重置缓冲区满信号
    gpio_bit_set(GPIOA, NSS); // 片选关
    return receive_data;
}

// 初始化RC522
void RC522_init(){
    write_byte(CommandReg, PCD_RESETPHASE);
    delay_1ms(500); // 等待初始化
    write_byte(ModeReg, 0x3D);
    write_byte(TReloadRegL, 30);
    write_byte(TReloadRegH, 0);
    write_byte(TModeReg, 0x8D);
    write_byte(TPrescalerReg, 0x3E);
    write_byte(TxASKReg, 0x40);
}

// 将寄存器中mask为1的位置为0
void clear_bit_mask(uint8_t addr, uint8_t mask){
    uint8_t temp = 0;
    temp = read_byte(addr) & (~mask);
    write_byte(addr, temp);
}

// 将mask位置为1
void set_bit_mask(uint8_t addr, uint8_t mask){
    uint8_t temp = 0;
    temp = read_byte(addr) | mask;
    write_byte(addr, temp);
}

// 计算crc
void calulate_CRC(unsigned char *pIndata,unsigned char len,unsigned char *pOutData)
{
    unsigned char i,n;
    clear_bit_mask(DivIrqReg,0x04);
    write_byte(CommandReg,PCD_IDLE);
    set_bit_mask(FIFOLevelReg,0x80);
    for (i=0; i<len; i++)
    {   write_byte(FIFODataReg, *(pIndata+i));   }
    write_byte(CommandReg, PCD_CALCCRC);
    i = 0xFF;
    do 
    {
        n = read_byte(DivIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x04));
    pOutData[0] = read_byte(CRCResultRegL);
    pOutData[1] = read_byte(CRCResultRegM);
}


// RC522与IS014443卡通信
uint8_t  com_with_MF522(uint8_t cmd, unsigned char *sending_data, 
                    uint8_t sending_length, unsigned char *receiving_data,
                    uint8_t* receiving_length)
{
    uint8_t status = MI_ERR;
    uint8_t irqEn = 0;
    uint8_t waitFor = 0;
    uint8_t lastBits = 0;
    switch (cmd)
    {
    case PCD_TRANSCEIVE:
        irqEn = 0x77;

        // 等待读到数据结束中断信号
        // 等待指令自行终止中断信号
        waitFor = 0x30;
        break;
    case PCD_AUTHENT:
        irqEn   = 0x12;
        waitFor = 0x10;
        break;
    
    default:
        break;
    }
    write_byte(ComIEnReg, irqEn|0x80);
    // 首位设0表示清除中断信号
    clear_bit_mask(ComIrqReg, 0x80);
    write_byte(CommandReg, PCD_IDLE);
    // 设最高位1，清空FIFOLevel
    // FIFOLevelReg低7位是数据
    // 表示FIFODataReg中数据量
    set_bit_mask(FIFOLevelReg, 0x80);

    for(int i = 0; i < sending_length; i++){
        write_byte(FIFODataReg, sending_data[i]); // 写FIFO
    }
    write_byte(CommandReg, cmd); // 命令写入命令寄存器
    if(cmd == PCD_TRANSCEIVE){
        // BitFramingReg高位置1表示开始transceive
        set_bit_mask(BitFramingReg, 0x80); // 开始发送
    }
    uint8_t n;
    uint32_t i = 10000;// 根据时钟频率调整，操作M1卡最大等待时间
    do{
        n = read_byte(ComIrqReg);
        i--;
    }
    while((i!=0) && !(n&0x01) && !(n&waitFor));
    // 停止发送
    clear_bit_mask(BitFramingReg, 0x80);
    if(i != 0){
        if(!(read_byte(ErrorReg)&0x1B)){
            status = MI_OK;
            if(n&irqEn&0x01){
                // printf("no card \n"); 
                // printf("n = %d\n", n);
                status = MI_NOTAGERR;
            }     
            else{
                // printf("find card! \n");
            }
            if(cmd == PCD_TRANSCEIVE){
                // 获取读到的数据大小
                n = read_byte(FIFOLevelReg);
                lastBits = read_byte(ControlReg)&0x07;
                if(lastBits){
                    *receiving_length = (n-1)*8 + lastBits;
                    printf("not full byte\n");
                }
                else{
                    *receiving_length = n*8;
                }
                if(n == 0)
                    n = 1;
                if(n > MAXRLEN)
                    n = MAXRLEN ;
                for(i = 0; i < n; i++){
                    // FIFODataReg是内部64字节的输入输出缓冲区接口
                    receiving_data[i] = read_byte(FIFODataReg);
                }
            }
        }
        else{
            printf("errorreg:%d\n", read_byte(ErrorReg));
            status = MI_ERR;
        }
        
    }
    // else{
        // printf("timeout\n");
    // }
    set_bit_mask(ControlReg,0x80);// 停止时钟
    write_byte(CommandReg,PCD_IDLE); 
    return status;
}

// 寻卡
uint8_t request(uint8_t req_code, u_char *pTagType){
    uint8_t status;
    uint8_t unLen;
    u_char ucComMF522Buf[MAXRLEN];

    // 清除MFCrtpto1On位，软件清除
    // 若找到M1FARE卡则会自动将这一位置为1
    clear_bit_mask(Status2Reg, 0x08); 

    // 低3位表示传输最后一个字节的位数(bits)
    // 000b表示最后一个字节全部发送
    // 这里0x07表示要发送最后一个字节的7位
    write_byte(BitFramingReg, 0x07);

    // 0x03表示将TX1和TX2的输出都设为13.56MHz传输
    set_bit_mask(TxControlReg, 0x03);

    // 寻卡命令：
    // 0x52：所有卡（会一直读）
    // 0x26：休眠中的卡（进入读卡区后只读一次）
    ucComMF522Buf[0] = req_code;
    status = com_with_MF522(PCD_TRANSCEIVE, ucComMF522Buf, 1, ucComMF522Buf, &unLen);   
    *pTagType = ucComMF522Buf[0];
    *(pTagType+1) = ucComMF522Buf[1];

    return status;
}

// 选卡
uint8_t select_card(uint8_t *pSnr)
{

    // for(int j = 0; j < 4; j++){
    //     printf("%d", j);
    // }
    // printf("\n");



    uint8_t status;
    uint8_t i;
    uint8_t  unLen;
    uint8_t ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i=0; i<4; i++)
    {
    	ucComMF522Buf[i+2] = *(pSnr+i);
    	ucComMF522Buf[6]  ^= *(pSnr+i);
    }
    calulate_CRC(ucComMF522Buf,7,&ucComMF522Buf[7]);
  
    clear_bit_mask(Status2Reg,0x08);

    status = com_with_MF522(PCD_TRANSCEIVE,ucComMF522Buf,9,ucComMF522Buf,&unLen);
    
    if ((status == MI_OK) && (unLen == 0x18))
    {   status = MI_OK;  }
    else
    {   status = MI_ERR;    }

    return status;
}

// 防冲撞
uint8_t anti_coll(uint8_t *pSnr)
{
    uint8_t status;
    uint8_t i,snr_check=0;
    unsigned int  unLen;
    uint8_t ucComMF522Buf[MAXRLEN]; 
    

    clear_bit_mask(Status2Reg,0x08);
    write_byte(BitFramingReg,0x00);
    clear_bit_mask(CollReg,0x80);
 
    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x20;

    status = com_with_MF522(PCD_TRANSCEIVE,ucComMF522Buf,2,ucComMF522Buf,&unLen);

    if (status == MI_OK)
    {
    	 for (i=0; i<4; i++)
         {   
             *(pSnr+i)  = ucComMF522Buf[i];
             snr_check ^= ucComMF522Buf[i];
         }
         if (snr_check != ucComMF522Buf[i])
         {   status = MI_ERR;    }
    }
    
    set_bit_mask(CollReg,0x80);
    return status;
}

// 验证密码
uint8_t auth_state(unsigned char auth_mode,unsigned char addr,unsigned char *pKey,unsigned char *pSnr){
    // printf("auth cardid: ");
    // for(int j = 0; j < 4; j++){
    //     printf("%d", pSnr[j]);
    // }
    // printf("\n");
    // printf("auth key: ");
    // for(int j = 0; j < 6; j++){
    //     printf("%x", pKey[j]);
    // }
    // printf("\n");
    
    char status;
    uint8_t  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = auth_mode;
    ucComMF522Buf[1] = addr;
    for (i=0; i<6; i++)
    {    ucComMF522Buf[i+2] = *(pKey+i);   }
    for (i=0; i<4; i++)
    {    ucComMF522Buf[i+8] = *(pSnr+i);   }
 //   memcpy(&ucComMF522Buf[2], pKey, 6); 
 //   memcpy(&ucComMF522Buf[8], pSnr, 4); 
    
    status = com_with_MF522(PCD_AUTHENT,ucComMF522Buf,12,ucComMF522Buf,&unLen);

    
    if ((status != MI_OK) || (!(read_byte(Status2Reg) & 0x08)))
    {   status = MI_ERR;   }
    
    return status;
}

// 读卡
uint8_t read_card(uint8_t addr, uint8_t* data){
    char status;
    uint8_t unLen;
    uint8_t ucComMF522Buf[MAXRLEN];
    ucComMF522Buf[0] = PICC_READ;
    ucComMF522Buf[1] = addr;
    calulate_CRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);
    status = com_with_MF522(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);
    if(status == MI_OK && unLen == 0x90){
        // printf("unlen: %d\n", unLen);
        for(int i = 0; i < 16; i++){
            data[i] = ucComMF522Buf[i];
            printf("%d, ", data[i]);
        }
        printf("\n");
    }
    else{
        printf("read error\n");
    }
    return status;
}

uint8_t card_halt()
{
    uint8_t status;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_HALT;
    ucComMF522Buf[1] = 0;
    calulate_CRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = com_with_MF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    return MI_OK;
}

char write_card(unsigned char addr,unsigned char *pData)
{
    char status;
    uint8_t  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = PICC_WRITE;
    ucComMF522Buf[1] = addr;
    calulate_CRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = com_with_MF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
        printf("1auth_com_status:%X\n", status);
        printf("1auth_com_unlen:%x\n", unLen);
        printf("1auth_com_ucComMF522Buf[0] & 0x0F:%x\n", ucComMF522Buf[0] & 0x0F);
        printf("=====\n");
    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
        
    if (status == MI_OK)
    {
        //memcpy(ucComMF522Buf, pData, 16);
        for (i=0; i<16; i++)
        {    ucComMF522Buf[i] = *(pData+i);   }
        calulate_CRC(ucComMF522Buf,16,&ucComMF522Buf[16]);

        status = com_with_MF522(PCD_TRANSCEIVE,ucComMF522Buf,18,ucComMF522Buf,&unLen);
        printf("auth_com_status:%X\n", status);
        printf("auth_com_unlen:%x\n", unLen);
        printf("auth_com_ucComMF522Buf[0] & 0x0F:%x\n", ucComMF522Buf[0] & 0x0F);
        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {   status = MI_ERR;   }
    }
    
    return status;
}

void antenna_on(){
    uint8_t i = 0;
    write_byte(TxASKReg, 0x40);
    // 天线开启与关闭之间需要有至少1毫秒时间差
    delay_1ms(100);

    if(!(i&0x03)){
        set_bit_mask(TxControlReg, 0x03);
        i = read_byte(TxControlReg);
        printf("set! txControlReg = %d\n",i);
    }
    i = read_byte(TxControlReg);
    printf("txControlReg = %d\n",i);
    i = read_byte(TxControlReg);
    printf("txControlReg = %d\n",i);
    
}