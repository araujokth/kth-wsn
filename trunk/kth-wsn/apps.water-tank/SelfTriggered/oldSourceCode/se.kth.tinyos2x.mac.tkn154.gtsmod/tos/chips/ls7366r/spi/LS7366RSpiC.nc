/*
 * LS7366RSpiC.nc
 * 
 * KTH | Royal Institute of Technology
 * Automatic Control
 *
 * 		  	 Project: se.kth.tinyos2x.mac.tkn154
 *  	  Created on: 2010/06/02  
 * Last modification:  
 *     		  Author: Aitor Hernandez <aitorhh@kth.se>
 *     
 */

generic configuration LS7366RSpiC() {

  provides interface Resource;
  provides interface ChipSpiResource;

  // registers
  provides interface LS7366RRegister as MDR0;
  provides interface LS7366RRegister as MDR1;
  provides interface LS7366RRegister as DTR;
  provides interface LS7366RRegister as CNTR;
  provides interface LS7366RRegister as OTR;
  provides interface LS7366RRegister as STR;
  
  provides interface LS7366RStrobe as LDOTR;
  provides interface LS7366RStrobe as CLRCNTR;


}

implementation {

  enum {
    CLIENT_ID = unique( "LS7366RSpi.Resource" ),
  };
  
  components HplLS7366RPinsC as Pins;
  components LS7366RSpiWireC as Spi;
  
  ChipSpiResource = Spi.ChipSpiResource;
  Resource = Spi.Resource[ CLIENT_ID ];

  // registers
  MDR0 = Spi.Reg[ LS7366R_REG_MDR0 ];
  MDR1 = Spi.Reg[ LS7366R_REG_MDR1 ];
  DTR = Spi.Reg[ LS7366R_REG_DTR ];
  CNTR = Spi.Reg[ LS7366R_REG_CNTR ];
  OTR = Spi.Reg[ LS7366R_REG_OTR ];
  STR = Spi.Reg[ LS7366R_REG_STR ];
  
  // strobe
  LDOTR = Spi.Strobe[ LS7366R_OP_LOAD | LS7366R_REG_OTR ];
  CLRCNTR = Spi.Strobe[ LS7366R_OP_CLR | LS7366R_REG_CNTR ];

}

