import socket

localPort = 8888
bufferSize = 1024

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def init():
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(('',localPort))
    print(f"UDP server: {get_ip()}:{localPort}")

def get_ip():
    ip_address = ''
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8",80))
    ip_address = s.getsockname()[0]
    s.close()
    return ip_address

def main():
    while True:
        # data, addr = sock.recvfrom(1024) # get data
        # print("received message: {} from {}\n".format(data,addr))
        address = ('192.168.100.123', 9696)
        message = input("enter right and left velocity: ")
        # sock.sendto(str("RPi received OK"),addr)
        sock.sendto(message.encode(),address)


if __name__ =="__main__":
    init()
    main()