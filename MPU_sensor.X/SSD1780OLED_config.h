/* 
 * File:   SSD1780OLED_config.h
 * Author: mohamed
 *
 * Created on January 3, 2024, 4:23 AM
 */

#ifndef SSD1780OLED_CONFIG_H
#define	SSD1780OLED_CONFIG_H

#ifdef	__cplusplus
extern "C" {
#endif

#define SSD1780OLED_Adress 0x78
    
#define SSD1780OLED_ControlByteCommand 0b00
#define SSD1780OLED_ControlByteData 0b01
    
    
#define SSD1780OLED_DisplayON 0xAF
#define SSD1780OLED_DisplayOFF 0xAE

#define SSD1780OLED_SetMultiplexRatio 0xA8

#define SSD1780OLED_SetDisplayOffset 0xD3


#define SSD1780OLED_SetDisplayStartLine 0x40      

    
#define SSD1780OLED_SetSegmentRemap 0xA0      
    
#define SSD1780OLED_SetCOMOutputScanDirection 0xC0
    
#define SSD1780OLED_SetCOMPinsHardwareConfiguration 0xDA
    
#define SSD1780OLED_SetContrastControl 0x81
#define SSD1780OLED_EntireDisplayON 0xA4

#define SSD1780OLED_SetNormalDisplayMode 0xA6   
    
#define SSD1780OLED_SetDisplayClockDivideRatio 0xD5  
    
#define SSD1780OLED_EnableChargePumpRegulator 0x8D    

    
    
#define SSD1780OLED_LCDHEIGHT 64
#define SSD1780OLED_LCDWIDTH 128
    

#define SSD1780OLED_SetColumnAddress 0x21   
#define SSD1780OLED_SetPageAddress 0x22  
   
 
#ifdef	__cplusplus
}
#endif

#endif	/* SSD1780OLED_CONFIG_H */

