<?php

// 
// Folder where the app are
//
$projectFolder = "/home/kthwsn/workspace/apps.water-tank/SelfTriggered/apps/";
$sensorsFolder = $projectFolder."WTSensorETApp/";
$actuatorsFolder = $projectFolder."WTActuatorApp/";
$baseStationFolder = $projectFolder."BaseStation/";
$dummyFolder = $projectFolder."TempApp/";
$snifferFolder = $projectFolder."PacketSnifferPerformance/";

//
// Optional configuration in case we want to configure different networks
//
// check if we have the correct number of arguments


$numberWaterTanksST = $argv[1];
$numberWaterTanksET = $argv[2];
$numberSensors = $argv[3];
$channel = $argv[4];
$panId = $argv[5];
$offsetWaterTanks = $argv[6];

//
// Motes constants to compile the correct ones
//
#$sensorList = array("M4A0M2EO", "MFT4LNUE", "MFT4LMJN", "MFT4LPNZ", "M4AP7GUD", "MFT4LNZQ", "MFT4LNK3", "MFTFL779");
$sensorList = array("M4A0M2EY");
$sensorList = array_slice($sensorList, $offsetWaterTanks);
$sensorListPorts = "";

$actuatorList = array("M49W90I1", "MFTLVUZ", "MFT4LX4L", "MFSHW9KC", "MFT4LII6", "MFSLYF54", "MFT4LLHH", "MFT4LNZR");
$actuatorList = array("MFTFL779");
$actuatorList = array_slice($actuatorList, $offsetWaterTanks);
$actuatorListPorts = "";

//$snifferList = array("M4A0M2DR");
$snifferList = array("M49W90FU");
$snifferListPorts = "";

$dummyList = array("");
$dummyListPorts = "";

//$baseStation = array("MFT4LNK7");
$baseStation = array("MFTFIYP0");
$baseStationPort = "";

$nWaterTanks = count($sensorList);
$nDummy = count($dummyList);

$portCounter = array(0, 0, 0, 0, 0);

$motes = array();
$motesPortArgument = "";

//
// Get the motelist
//
$bashtest = exec("motelist",$output,$exitCode);
$motelist = join("\n",$output);
$motelist = str_replace("\n\n","\n",$motelist);
$motelist = split("\n",$motelist);

//
//adjust according to the output of motelist command (starting line and end line)
//
		for ($i = 2; $i < count($motelist) ; $i++) {			
			$cumo = $motelist[$i];
			$cumoli = explode("   ",$cumo);
			$mote = array();
			$mote["reference"] = trim($cumoli[0]);
			$mote["port"] = trim($cumoli[1]);
			array_shift  ( $cumoli );
			array_shift  ( $cumoli );
			$mote["description"] = trim(join(",",$cumoli));
			array_push($motes,$mote);
		}

$nMotes = count($motes);
$nWaterTanks = count($sensorList);
$nDummy = count($dummyList);

for ($i = 0; $i < $nMotes  ; $i++) {
	$mote = $motes[$i];

	if (in_array($mote["reference"], $sensorList)) {
		$idx = array_keys($sensorList, $mote["reference"]);
		$idx = 2*( $idx[0] +1 + $offsetWaterTanks ) - 1;
		$portCounter[0] ++;
		$sensorListPorts = $sensorListPorts.$idx." ".$mote["port"]." ";

	    	//echo "fprintf('Got Sensor \\n');\n";
	}else if (in_array($mote["reference"], $actuatorList)){
		$idx = array_keys($actuatorList, $mote["reference"]);
		$idx = 2*( $idx[0] +1 + $offsetWaterTanks );
		$portCounter[1] ++;
		$actuatorListPorts = $actuatorListPorts.$idx." ".$mote["port"]." ";
	   	//echo "fprintf('Got Actuator \\n');\n";

	}else if (in_array($mote["reference"], $dummyList)){
		$idx = array_keys($dummyList, $mote["reference"]);
		$idx = $idx[0] + $offsetWaterTanks+ $nWaterTanks*2;
		$portCounter[2] ++;
	    	//echo "Got Dummy \\n";
		$dummyListPorts = $dummyListPorts.$idx." ".$mote["port"]." ";

	}else if (in_array($mote["reference"], $baseStation)){
		$idx = 0;
		$portCounter[3] ++;
	    	//echo "fprintf('Got Base Station \\n');\n";
		$baseStationPort = $baseStationPort.$idx." ".$mote["port"]." ";

	}else if (in_array($mote["reference"], $snifferList)){
		$idx = $nDummy  + $nWaterTanks*2 +1 - $offsetWaterTanks;
		$portCounter[4] ++;
	    	//echo "fprintf('Got Sniffer \\n');\n";
		$snifferListPorts = $snifferListPorts.$idx." ".$mote["port"]." ";
	}

}

	$exitCode = array();

//
// Before compiling lets modify the header file to fit our requirements
//
	$cmd = "setSelfTriggeredVariables '".$projectFolder."' ".$numberWaterTanksET." ".$numberWaterTanksST." ".$numberSensors." ".$channel." ".$panId;
	$bashtest = exec($cmd,$output,$exitCode[0]);	

/*if ($portCounter[0] == $portCounter[1] && $portCounter[0] == $nWaterTanks && $portCounter[3] == 1){*/
if (true){

	// Programm the motes
	$exitCode = array();

	//echo $sensorListPorts." ".$actuatorListPorts." ".$dummyListPorts." ".$baseStationPort."\n";

	
	// Program the sensors
	$cmd = "moteallClass ".$sensorsFolder." SENSORS ".$sensorListPorts."\n";
	//echo "fprintf('".$cmd."\\n')\n";
	$bashtest = exec($cmd,$output,$exitCode[0]);	

	// Program the actuators
	$cmd = "moteallClass ".$actuatorsFolder." ACTUATORS ".$actuatorListPorts."\n";
	//echo "fprintf('".$cmd."\\n')\n";
	$bashtest = exec($cmd,$output,$exitCode[0]);	

	// Program the dummy sensors
	$cmd = "moteallClass ".$dummyFolder." DUMMY ".$dummyListPorts."\n";
	//$cmd = "moteallDummy ".$dummyListPorts;
	$bashtest = exec($cmd,$output,$exitCode[$i]);	

	$cmd = "moteallClass ".$baseStationFolder." BASESTATION ".$baseStationPort."\n";
	//echo "fprintf('".$cmd."\\n')\n";
	$bashtest = exec($cmd,$output,$exitCode[$i]);

	$cmd = "moteallClass ".$snifferFolder." SNIFFER ".$snifferListPorts."\n";
	//echo "fprintf('".$cmd."\\n')\n";
	$bashtest = exec($cmd,$output,$exitCode[$i]);
}

//
// Write the motes information in a mFile format.
//
echo "motes.nMotes=".$nMotes.";\n";
echo "motes.moteList=cell(".$nMotes.",1);\n";

// Fill the moteList cell
for ($i = 0; $i < $nMotes ; $i++) {
	$cmo = $motes[$i];
	$portNumber = explode("/dev/ttyUSB",$cmo["port"]);
	echo "motes.moteList{".($i+1)."}.reference='".$cmo["reference"]."';\n";
	echo "motes.moteList{".($i+1)."}.port=".$portNumber[1].";\n";
	echo "motes.moteList{".($i+1)."}.id=".$i.";\n";
	echo "motes.moteList{".($i+1)."}.description='".$cmo["description"]."';\n";
}
echo "motes.baseStation='".$baseStation[0]."';\n";
echo "motes.sniffer='".$snifferList[0]."';\n";
return

	
?>
