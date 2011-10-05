// $Id: ExtFlashC.nc,v 1.1 2009/11/10 07:03:34 rflury Exp $

/*
 *
 *
 * "Copyright (c) 2000-2005 The Regents of the University  of California.  
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 * 
 * IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF
 * CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 *
 */

/**
 * @author Jonathan Hui <jwhui@cs.berkeley.edu>
 * @author Roland Flury <roland.flury@shockfish.com>
 */

module ExtFlashC {
  provides {
    interface Init;
    interface StdControl;
    interface ExtFlash;
  }
}

/**
 * Simple reader module to access the Atmel at45db041 flash chip
 */
implementation {

  uint32_t addr;

  command error_t Init.init() {

    TOSH_SET_FLASH_CS_PIN(); // inverted, deselect by default
    TOSH_MAKE_FLASH_CS_OUTPUT();

    TOSH_CLR_FLASH_CLK_PIN();
    TOSH_MAKE_FLASH_CLK_OUTPUT();

    TOSH_SET_FLASH_OUT_PIN();
    TOSH_MAKE_FLASH_OUT_OUTPUT();      

    TOSH_MAKE_FLASH_IN_INPUT();

		TOSH_SET_FLASH_RESET_PIN(); // inverted
		TOSH_MAKE_FLASH_RESET_OUTPUT();

    return SUCCESS; 
  }

  command error_t StdControl.start() { return SUCCESS; }
  command error_t StdControl.stop() { return SUCCESS; }
	
	/**
	 * Write a Byte over the SPI bus and receive a Byte
	 *
	 * upon calling this function, /CS must be CLR
	 */
  uint8_t SPIByte(uint8_t out) {
    uint8_t in = 0;
    uint8_t i;

    for ( i = 0; i < 8; i++, out <<= 1 ) {
      // write bit
      if (out & 0x80) {
				TOSH_SET_FLASH_OUT_PIN();
			} else {
				TOSH_CLR_FLASH_OUT_PIN();
			}
			
      // clock
      TOSH_SET_FLASH_CLK_PIN();
			
      // read bit
      in <<= 1;
      if (TOSH_READ_FLASH_IN_PIN()) {
				in |= 1;
			}
			
      // clock
      TOSH_CLR_FLASH_CLK_PIN();
    }

    return in;
  }



	/**
	 * Initializes the flash to read Byte after Byte starting
	 * from the given address. 
	 *
	 * Subsequent calls to readByte() will return the Bytes 
	 * starting from the specified address. 
	 *
	 * stopRead() terminates this process and disables the Flash. 
	 */
  command void ExtFlash.startRead(uint32_t newAddr) {
    uint8_t  cmdBuf[4];
    uint8_t  i;

		// we're using "Waveform 1 - Inactive Clock Polarity Low"
		// see p.7 of data sheet
    TOSH_CLR_FLASH_CLK_PIN(); 
    TOSH_CLR_FLASH_CS_PIN(); // select the flash

    addr = newAddr;

		// we only use 256 Bytes per block (of 264 Bytes)
    cmdBuf[0] = 0x52; // command for reading data starting at the following address
    cmdBuf[1] = (addr >> 15) & 0xff; // 4 LSbits 
    cmdBuf[2] = (addr >> 7) & 0xfe;  // 7 MSbits with the above 4 bits describe page to read
    cmdBuf[3] = addr & 0xff;         // Offset to Byte in page to read

		// transmit read command
    for(i = 0; i < 4; i++) {
      SPIByte(cmdBuf[i]);
		}
		// transmit 4 Bytes "don't care" as to spec
    for(i = 0; i < 4; i++) {
      SPIByte(0x0);
		}
    
		// need to do one additional clock transition before reading
		TOSH_SET_FLASH_CLK_PIN();
		TOSH_CLR_FLASH_CLK_PIN();
  }

  command uint8_t ExtFlash.readByte() {
		uint8_t b = SPIByte(0); // write anything, read Byte
		addr++;
		if(0 == (addr & 0xFF)) {
			// we've just read the last Byte from a page
			// initialize the Flash to continue reading on the new page
      call ExtFlash.stopRead();
      call ExtFlash.startRead(addr);
		} 
		return b;
  }

  command void ExtFlash.stopRead() {
    TOSH_SET_FLASH_CS_PIN(); // disble Flash & tri-state the OUT-pin
  }

}
