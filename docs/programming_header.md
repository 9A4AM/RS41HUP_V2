# Vaisala RS41 Expansion Connector Pinout
Based on work by DF8OE


Viewed into port from outside
```
----------|     |----------
|  9    7    5    3    1  |
|                         |
|  10   8    6    4    2  |
---------------------------
```

Suitable Plug: Molex 87568-1073  (element14 order code 1365756)

* 1 - GND
* 2 - I2C_SDA (PB11)
* 3 - I2C_SCL (PB10)
* 4 - +VDD_MCU
* 5 - MCU Switched 3.3V out to external device
* 6 - +V_Battery
* 7 - RST
* 8 - SWCLK (PA14)
* 9 - SWDIO (PA13)
* 10 - GND



## STLink-v2 Header to Vaisala Cable

RS41			STLink 
----------------------
1				4
2		N/C
3		N/C
4				1
5		N/C
6		N/C
7				15
8				9
9				7
10		N/C

All other pins Not connected