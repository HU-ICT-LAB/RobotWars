"""Server for handling requests."""
import socketserver


class Server(socketserver.BaseRequestHandler):
    """
    The request handler class for the server.

    It's instantiated once per connection to the server.
    """

    def handle(self):
        """
        Handle an incoming request.

        Sends a file to the connection.
        """
        print("connection opened with: " + str(self.client_address))
        file = open("test", "rb")
        opened_file = file.read(1024)
        self.request.sendall(opened_file)


if __name__ == "__main__":
    HOST, PORT = "localhost", 9999
    with socketserver.TCPServer((HOST, PORT), Server) as server:
        server.serve_forever()  # Keeps the server active
