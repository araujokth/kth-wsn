/* $Id: DemoSensorNowC.nc,v 1.5 2007/05/22 20:59:01 idgay Exp $
 * Copyright (c) 2006 Intel Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached INTEL-LICENSE     
 * file. If you do not find these files, copies can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA, 
 * 94704.  Attention:  Intel License Inquiry.
 */
/**
 * The micaZ doesn't have any built-in sensors - this DemoSensor just reads
 * the ground value.
 *
 * @author David Gay
 */

generic configuration DemoSensorNowC()
{
  provides interface Resource;
  provides interface ReadNow<uint16_t>;
}
implementation {
  components new VoltageNowC() as Sensor;

  Resource = Sensor;
  ReadNow = Sensor;
}
