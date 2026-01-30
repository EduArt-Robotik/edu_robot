/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 * 
 * UART to Network Bridge
 * 
 * This program bridges UART communication to a TCP/IP network connection.
 * It opens a UART device and listens on a TCP port. When a client connects,
 * it forwards data bidirectionally between UART and the network connection.
 * 
 * Usage: uart_network_bridge <uart_device> <port>
 * Example: uart_network_bridge /dev/ttyUSB0 5000
 */

#include <iostream>
#include <cstring>
#include <thread>
#include <atomic>
#include <vector>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>

#if _WITH_MRAA
#include <memory>
#include <mraa/common.hpp>
#include <mraa/uart.hpp>
#endif

class UartNetworkBridge {
public:
  UartNetworkBridge(const std::string& uart_device, int port)
    : _uart_device(uart_device)
    , _port(port)
    , _server_fd(-1)
    , _client_fd(-1)
    , _running(false)
  {
  }

  ~UartNetworkBridge() {
    stop();
  }

  bool initialize() {
#if _WITH_MRAA
    // Open and configure UART via MRAA
    _uart = std::make_unique<mraa::Uart>(_uart_device.c_str());

    if (_uart->setBaudRate(115200) != mraa::SUCCESS) {
      std::cerr << "Failed to set UART baud rate to 115200." << std::endl;
      _uart.reset();
      return false;
    }
    if (_uart->setMode(8, mraa::UART_PARITY_NONE, 1) != mraa::SUCCESS) {
      std::cerr << "Failed to set UART mode (8N1)." << std::endl;
      _uart.reset();
      return false;
    }
    if (_uart->setFlowcontrol(false, false) != mraa::SUCCESS) {
      std::cerr << "Failed to disable UART flow control." << std::endl;
      _uart.reset();
      return false;
    }

    _uart->flush();
    while (_uart->dataAvailable(1)) { char c; _uart->read(&c, 1); }

    std::cout << "UART (MRAA) device " << _uart_device << " opened successfully" << std::endl;
#else
    std::cerr << "MRAA not available. Please build on target with MRAA." << std::endl;
    return false;
#endif

    // Create TCP server socket
    _server_fd = socket(AF_INET, SOCK_STREAM, 0);

    if (_server_fd < 0) {
      std::cerr << "Failed to create server socket: " << strerror(errno) << std::endl;
      return false;
    }

    // Allow socket reuse
    int opt = 1;

    if (setsockopt(_server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
      std::cerr << "Failed to set socket options: " << strerror(errno) << std::endl;
      close(_server_fd);
      _server_fd = -1;

      return false;
    }

    // Bind to port
    struct sockaddr_in server_addr;
    std::memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(_port);

    if (bind(_server_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
      std::cerr << "Failed to bind to port " << _port << ": " << strerror(errno) << std::endl;
      close(_server_fd);
      _server_fd = -1;

      return false;
    }

    // Listen for connections
    if (listen(_server_fd, 1) < 0) {
      std::cerr << "Failed to listen on socket: " << strerror(errno) << std::endl;
      close(_server_fd);
      _server_fd = -1;

      return false;
    }

    std::cout << "TCP server listening on port " << _port << std::endl;
    return true;
  }

  void run() {
    _running = true;

    while (_running) {
      std::cout << "Waiting for client connection..." << std::endl;
      
      struct sockaddr_in client_addr;
      socklen_t client_len = sizeof(client_addr);
      _client_fd = accept(_server_fd, (struct sockaddr*)&client_addr, &client_len);
      
      if (_client_fd < 0) {
        if (_running) {
          std::cerr << "Failed to accept connection: " << strerror(errno) << std::endl;
        }
        break;
      }

      std::cout << "Client connected from " << inet_ntoa(client_addr.sin_addr) << std::endl;

      // Start forwarding threads
      std::thread uart_to_network([this]() { forwardUartToNetwork(); });
      std::thread network_to_uart([this]() { forwardNetworkToUart(); });

      // Wait for threads to finish (happens when connection is closed)
      uart_to_network.join();
      network_to_uart.join();

      close(_client_fd);
      _client_fd = -1;
      std::cout << "Client disconnected" << std::endl;
    }
  }

  void stop() {
    _running = false;
    
    if (_client_fd >= 0) {
      close(_client_fd);
      _client_fd = -1;
    }
    
    if (_server_fd >= 0) {
      close(_server_fd);
      _server_fd = -1;
    }
    
    // Close UART
#if _WITH_MRAA
    if (_uart) { _uart->flush(); _uart->close(); _uart.reset(); }
#endif
  }

private:
  void forwardUartToNetwork() {
#if _WITH_MRAA
    constexpr std::size_t buffer_size = 32;
    std::vector<char> buffer(buffer_size);
    
    while (_running && _client_fd >= 0) {
      // Wait up to 1 ms for data to avoid busy waiting
      if (!_uart->dataAvailable(1)) {
        usleep(100);
        continue;
      }

      // reading until 32 bytes are read
      int bytes_read = 0;

      while (bytes_read < static_cast<int>(buffer.size()) && _uart->dataAvailable(100) && _running && _client_fd >= 0) {
        bytes_read += _uart->read(
          static_cast<char*>(static_cast<void*>(buffer.data())) + bytes_read, 1
        );
      }

      if (bytes_read == buffer_size) {
        ssize_t bytes_sent = send(_client_fd, buffer.data(), static_cast<size_t>(bytes_read), 0);

        if (bytes_sent < 0) {
          std::cerr << "Failed to send to network: " << strerror(errno) << std::endl;
          break;
        }
      }
      // else: incomplete read, try again in next iteration
    }
#endif
  }

  void forwardNetworkToUart() {
#if _WITH_MRAA
    std::vector<char> buffer(256);
    
    while (_running && _client_fd >= 0) {
      ssize_t bytes_read = recv(_client_fd, buffer.data(), buffer.size(), 0);
      
      if (bytes_read > 0) {
        int written = _uart->write(buffer.data(), static_cast<unsigned int>(bytes_read));
        if (written < 0 || written != bytes_read) {
          std::cerr << "Failed to write to UART (MRAA)." << std::endl;
          break;
        }
      }
      else if (bytes_read == 0) {
        // Connection closed
        std::cout << "Network connection closed" << std::endl;
        break;
      }
      
      usleep(100); // Small delay to prevent busy waiting
    }
#endif
  }

  std::string _uart_device;
  int _port;
#if _WITH_MRAA
  std::unique_ptr<mraa::Uart> _uart;
#endif
  int _server_fd;
  int _client_fd;
  std::atomic<bool> _running;
};

int main(int argc, char* argv[]) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <uart_device> <port>" << std::endl;
    std::cerr << "Example: " << argv[0] << " /dev/ttyUSB0 5000" << std::endl;
    return 1;
  }

  std::string uart_device = argv[1];
  int port = std::atoi(argv[2]);

  if (port <= 0 || port > 65535) {
    std::cerr << "Invalid port number: " << port << std::endl;
    return 1;
  }

  std::cout << "Starting UART to Network Bridge" << std::endl;
  std::cout << "UART Device: " << uart_device << std::endl;
  std::cout << "TCP Port: " << port << std::endl;

  UartNetworkBridge bridge(uart_device, port);
  
  if (!bridge.initialize()) {
    std::cerr << "Failed to initialize bridge" << std::endl;
    return 1;
  }

  std::cout << "Bridge initialized successfully" << std::endl;
  std::cout << "Press Ctrl+C to stop" << std::endl;

  bridge.run();

  return 0;
}
