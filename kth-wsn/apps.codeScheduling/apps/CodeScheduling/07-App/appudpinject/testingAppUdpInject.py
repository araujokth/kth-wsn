import socket

request    = "poipoipoipoi"
myAddress  = '' #means 'all'
myPort     = 2188

print "Testing AppUdpInject...\n"

socket_handler = socket.socket(socket.AF_INET6,socket.SOCK_DGRAM)
socket_handler.settimeout(5)
socket_handler.bind((myAddress,myPort))
receivedAnswer = False
while receivedAnswer==False:
   try:
      reply,dist_addr = socket_handler.recvfrom(1024)
   except socket.timeout:
      print "\nno reply"
   else:
      print "\nreply "+str(dist_addr[0])+"%"+str(dist_addr[1])+" -> "+myAddress+"%"+str(myPort)
      print reply+" ("+str(len(reply))+" bytes)"
      receivedAnswer = True
socket_handler.close()

raw_input("\nPress return to close this window...")
