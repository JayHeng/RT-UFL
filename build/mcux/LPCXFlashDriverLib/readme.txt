LPCXpresso Flash driver library
===============================

Library project containing headers and main polling routine used
by LPCXpresso flash driver projects

[ Updated - 22 October 2019 
  - new build configuration to allow mini config but with Mass Erase
    required for Kinetis small parts where sector erase breaks the part
    
Updated - October 2019
  - new build configuration to add Sector Hashing support    

Updated - 18 October 2016
  - new flash device type INTKFMM 9 for internal Kinetis Flash Memory Module

Updated - 10 June 2015
  - New versions of lpcx_flash_info.h, lpcx_flash_msgif.h and
    lpcx_flash_driver.h to add functionality required for 
    LPC18/43 generic SPIFI flash driver (where driver info 
    is updated by call to Init() ).
  
  Initial version - February 2015
]
        