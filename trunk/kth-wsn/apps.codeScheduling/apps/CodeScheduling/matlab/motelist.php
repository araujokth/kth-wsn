<?php
//
// Get the motelist
//
$bashtest = exec("motelist",$output,$exitCode);
$motelist = join("\n",$output);
$motelist = str_replace("\n\n","\n",$motelist);
$motelist = split("\n",$motelist);

$motes = array();
$motesReferences = array();
$motesPortArgument = "";
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
			//array_push($motesReferences,$mote["reference"]);
			#echo $mote["reference"]."\t".$mote["port"]."\t".$mote["description"]."\n";		
		}
// sort the references to compile them in order
sort($motes);
		for ($i = 2; $i < count($motelist) ; $i++) {		
			$motesPortArgument = $motesPortArgument.$mote["port"]." ";	
}

$nMotes = count($motes);
// Programm the motes
$exitCode = array();

$cmd = "moteall ".$motesPortArgument;

$bashtest = exec($cmd,$output,$exitCode[$i]);		

		/*for ($i = 0; $i < $nMotes  ; $i++) {
			$cmo = $motes[$i];
			$cmd = "make tmote install,".$i." bsl,".$cmo["port"]." &";
			$bashtest = exec($cmd,$output,$exitCode[$i]);		
		}*/

		//while(in_array(TRUE, $exitCode, TRUE));
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

?>
