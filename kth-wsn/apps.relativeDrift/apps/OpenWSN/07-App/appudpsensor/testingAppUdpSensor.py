import socket

myAddress  = '' #means 'all'
myPort     = 2189

print "Testing AppUdpSensor...\n"

socket_handler = socket.socket(socket.AF_INET6,socket.SOCK_DGRAM)
socket_handler.settimeout(5)
socket_handler.bind((myAddress,myPort))
receivedAnswer = False
while receivedAnswer==False:
   try:
      reply,dist_addr = socket_handler.recvfrom(1024)
   except socket.timeout:
      print "\nno packet"
   else:
      print "\npacket "+str(dist_addr[0])+"%"+str(dist_addr[1])+" -> "+myAddress+"%"+str(myPort)
      print reply+" ("+str(len(reply))+" bytes)"
      receivedAnswer = True
socket_handler.close()

raw_input("\nPress return to close this window...")
