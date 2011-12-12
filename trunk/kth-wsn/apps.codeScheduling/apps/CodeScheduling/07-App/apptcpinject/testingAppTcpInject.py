import socket

myAddress  = '' #means 'all'
myPort     = 2188

print "Testing AppTcpInject...\n"

socket_handler = socket.socket(socket.AF_INET6,socket.SOCK_STREAM)
socket_handler.bind((myAddress,myPort))
socket_handler.listen(1)
conn,hisAddress = socket_handler.accept()
input = conn.recv(1024)
print "\nreceived "+str(hisAddress[0])+"%"+str(hisAddress[1])+" -> "+myAddress+"%"+str(myPort)
print input+" ("+str(len(input))+" bytes)"
conn.close()

raw_input("\nPress return to close this window...")
