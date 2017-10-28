import socket
import struct

HOST = '' #all interfaces
UDP_PORT = 8802
#UDP_IP = "2804:7f4:3b80:9440:212:4b00:d70:7"
#UDP_IP = "2804:7f4:3b80:9440:212:4b00:797:6981"
UDP_IP = "2804:7f4:3b80:c6e8:212:4b00:f28:c486"
#UDP_IP = "raspberry-6lbr.local"
#UDP_IP = "localhost"

LED_TOGGLE_REQUEST = (0x79)
LED_SET_STATE = (0x7A)
LED_GET_STATE = (0x7B)
LED_STATE = (0x7C)

LED_CURRENT_STATE = 1

MESSAGE = "Sending an UDP packet using python!"

# socket.SOCK_DGRAM = UDP connection
sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM) # UDP IPv6
#sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP IPv4

sock.bind((HOST, UDP_PORT))

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT
#print "message:", MESSAGE

# send LED_STATE
#pkt_to_send = struct.pack('bb', LED_STATE, LED_CURRENT_STATE)

# send LED_TOGGLE_REQUEST
pkt_to_send = struct.pack('b', LED_TOGGLE_REQUEST)
sock.sendto(pkt_to_send, (UDP_IP, UDP_PORT))

print "waiting for response..."
data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
fx, led_state_response = struct.unpack('bb', data)
print "resposta recebida fx: ", fx, " estado: ", led_state_response

if fx == LED_SET_STATE:
    print "setar LEDs: ", led_state_response
    LED_CURRENT_STATE = led_state_response
    pkt_to_send = struct.pack('bb', LED_STATE, LED_CURRENT_STATE)
    sock.sendto(pkt_to_send, addr)

