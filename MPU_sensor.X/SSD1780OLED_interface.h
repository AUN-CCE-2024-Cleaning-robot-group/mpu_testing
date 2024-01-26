/* 
 * File:   SSD1780OLED_interface.h
 * Author: mohamed
 *
 * Created on January 3, 2024, 4:23 AM
 */

#ifndef SSD1780OLED_INTERFACE_H
#define	SSD1780OLED_INTERFACE_H

#ifdef	__cplusplus
extern "C" {
#endif

typedef enum {SendData,SendCommand}SendByte_T;    

typedef enum {PageAddressing,HorizontalAddressing,VerticalAddressing}OledScreenAdressing_T;    


void VOledInit(); 
#inline
uint8_t VOledSend(uint8_t ucSendByte,SendByte_T xByteType);    
uint8_t VOledSendCommand(uint8_t ucCommand);    
void VOledSendData(uint8_t ucData);    
void VOledSetAddressingMode(OledScreenAdressing_T xScreenAdressing);
void VOledSetAddressSpace(
                    uint16_t usColumnAddressStart,uint16_t usColumnAddressEnd,
                    uint16_t usPageAddressStart,uint16_t usPageAddressEnd);
void VOledDrawPixel(uint8_t x, uint8_t y,uint8_t ucColor);
void VOledDisplay();
void VOledFillScreen(uint8_t ucColor);
void VOled_DrawChar(uint16_t x, uint16_t y, uint8_t c, uint8_t size = 1);
#inline 
void VOled_DrawStringFast(uint16_t x, uint16_t y, uint8_t* ptrString);
void VOled_Print(uint16_t x, uint16_t y, uint8_t* ptrString, uint8_t size = 1);
void VOled_writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                             uint16_t color);
void VOledFillRect(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
        uint8_t ucColor);

#ifdef	__cplusplus
}
#endif

#endif	/* SSD1780OLED_INTERFACE_H */

