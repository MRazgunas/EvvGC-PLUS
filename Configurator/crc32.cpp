/**
 * The following code implements CRC32 algorythm used
 * in STM32 microcontrollers and is adapted from:
 * - https://my.st.com/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/Flat.aspx? \
 *   RootFolder=%2Fpublic%2FSTe2ecommunities%2Fmcu%2FLists%2Fcortex_mx_stm32%2FCRC%20computation \
 *   &FolderCTID=0x01200200770978C69A1141439FE559EB459D7580009C4E14902C3CDE46A77F0FFD06506F5B
 */

#include "crc32.h"

/* Nibble lookup table for 0x04C11DB7 polynomial. */
static const quint32 crc_tab[16] = {
    0x00000000,0x04C11DB7,0x09823B6E,0x0D4326D9,
    0x130476DC,0x17C56B6B,0x1A864DB2,0x1E475005,
    0x2608EDB8,0x22C9F00F,0x2F8AD6D6,0x2B4BCB61,
    0x350C9B64,0x31CD86D3,0x3C8EA00A,0x384FBDBD
};

/**
 * @brief crc32 - calculates CRC32 checksum of the data buffer.
 * @param pBuf - address of the data buffer.
 * @param length - length of the buffer.
 * @return CRC32 checksum.
 */
quint32 crc32(const quint32 pBuf[], size_t length)
{
    quint32 i;
    /* Initial XOR value. */
    quint32 crc = 0xFFFFFFFF;

    for (i = 0; i < length; i++) {
        /* Apply all 32-bits: */
        crc ^= pBuf[i];

        /* Process 32-bits, 4 at a time, or 8 rounds.
         * - Assumes 32-bit reg, masking index to 4-bits;
         * - 0x04C11DB7 Polynomial used in STM32.
         */
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
    }
    return crc;
}
