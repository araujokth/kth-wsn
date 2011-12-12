import socket

request    = "poipoipoipoi"
myAddress  = '' #means 'all'
myPort     = 21568
hisAddress = '2001:470:1f05:dff::12'
hisPort    = 9

print "Testing AppTcpPrint...\n"

socket_handler = socket.socket(socket.AF_INET6,socket.SOCK_STREAM)
socket_handler.connect((hisAddress,hisPort))
socket_handler.send(request)
print "\nrequest "+myAddress+"%"+str(myPort)+" -> "+hisAddress+"%"+str(hisPort)
print request+" ("+str(len(request))+" bytes)"
socket_handler.close()

raw_input("\nPress return to close this window...")
