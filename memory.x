MEMORY
{
  /* STM32H755ZI Memory Layout */
  FLASH : ORIGIN = 0x08000000, LENGTH = 2M
  RAM : ORIGIN = 0x20000000, LENGTH = 128K
  AXISRAM : ORIGIN = 0x24000000, LENGTH = 512K
  SRAM1 : ORIGIN = 0x30000000, LENGTH = 128K
  SRAM2 : ORIGIN = 0x30020000, LENGTH = 128K
  SRAM3 : ORIGIN = 0x30040000, LENGTH = 32K
  SRAM4 : ORIGIN = 0x38000000, LENGTH = 64K
  BSRAM : ORIGIN = 0x38800000, LENGTH = 4K
  ITCM  : ORIGIN = 0x00000000, LENGTH = 64K
}
