"""Send a command to the running deploy notebook:   python send_command.py hammer"""
import socket, sys

def send(text, host='127.0.0.1', port=5005):
    socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(text.encode(), (host, port))

if __name__ == '__main__':
    send(' '.join(sys.argv[1:]) or 'reset')
