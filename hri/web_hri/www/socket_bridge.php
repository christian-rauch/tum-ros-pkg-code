<?php

$server = gethostbyname(($_POST['server']) ? $_POST['server'] : $_GET['server']);
$port = ($_POST['port']) ? $_POST['port'] : $_GET['port'];
$req = ($_POST['req']) ? $_POST['req'] : $_GET['req'];

//$port = 6969;
//$server = gethostbyname('goedel.laas.fr');

// The web service returns JSON. Set the Content-Type appropriately
header("Content-Type: text");

if ($req === NULL) {
    echo "Oups! Got an empty request!";
}

// Create a TCP/IP socket. 
$socket = socket_create(AF_INET, SOCK_STREAM, SOL_TCP);
if ($socket === false) {
    echo "Oups! I'm unable to create a socket! Reason: " . socket_strerror(socket_last_error());
}

//echo "Attempting to connect to '$server' on port '$port'...";

$result = socket_connect($socket, $server, $port);
if ($result === false) {
    echo "Oups! I'm unable to connect to my socket: reason: ($result) " . socket_strerror(socket_last_error($socket));
}


//////////////////////////////////////////////////////////////////////////////////////////////////
// Create the request to the ontology server

$in = "len " . strlen($rep) . "\n";
$in .= $req;

//////////////////////////////////////////////////////////////////////////////////////////////////
// Send it and read the response

$out = '';
$tmp = '';

//echo "Sending request to oro-server...";
socket_write($socket, $in, strlen($in));
//echo "OK.\n";

//echo "Reading response:\n\n";

$tmp = socket_read($socket, 4096, PHP_NORMAL_READ);

list($s, $len) = explode(" ", rtrim($tmp));

if ($s === "len") {
	$out = socket_read($socket, (int)$len);
	echo $out;
}
else {
	echo "Oups! The response on the socket should start with a line containing the length in byte of the msg 'len ...'";
}

socket_close($socket);


?>


