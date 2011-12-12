import socket

request    = "poipoipoipoi"
myAddress  = '' #means 'all'
myPort     = 21568
hisAddress = '2001:470:1f05:dff:1415:920b:0301:0028'
hisPort    = 9

print "Testing AppUdpPrint...\n"

socket_handler = socket.socket(socket.AF_INET6,socket.SOCK_DGRAM)
socket_handler.sendto(request,(hisAddress,hisPort))
print "\nrequest "+myAddress+"%"+str(myPort)+" -> "+hisAddress+"%"+str(hisPort)
print request+" ("+str(len(request))+" bytes)"
socket_handler.close()

raw_input("\nPress return to close this window...")
