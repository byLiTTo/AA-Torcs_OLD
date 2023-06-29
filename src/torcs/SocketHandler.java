package torcs;

import java.io.IOException;
import java.net.*;

/**
 * The SocketHandler class provides methods for sending and receiving datagrams over a network socket.
 * It utilizes UDP (User Datagram Protocol) for communication.
 *
 * @author Daniele Loiacono
 */
public class SocketHandler {

    private InetAddress address; // The remote address to send and receive datagrams
    private int port; // The port number to send and receive datagrams
    private DatagramSocket socket; // The socket for sending and receiving datagrams
    private boolean verbose; // Indicates whether to print verbose output

    /**
     * Constructs a new SocketHandler with the specified host, port, and verbosity.
     *
     * @param host    The remote host address.
     * @param port    The remote port number.
     * @param verbose True to enable verbose output, false otherwise.
     */
    public SocketHandler(String host, int port, boolean verbose) {
        try {
            this.address = InetAddress.getByName(host);
        } catch (UnknownHostException e) {
            e.printStackTrace();
        }
        this.port = port;
        try {
            socket = new DatagramSocket();
        } catch (SocketException e) {
            e.printStackTrace();
        }
        this.verbose = verbose;
    }

    /**
     * Sends a message over the socket.
     *
     * @param msg The message to send.
     */
    public void send(String msg) {
        if (verbose) {
            System.out.println("Sending: " + msg);
        }
        try {
            byte[] buffer = msg.getBytes();
            socket.send(new DatagramPacket(buffer, buffer.length, address, port));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Receives a message from the socket.
     *
     * @return The received message as a string.
     */
    public String receive() {
        try {
            byte[] buffer = new byte[1024];
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
            socket.receive(packet);
            String received = new String(packet.getData(), 0, packet.getLength());
            if (verbose) {
                System.out.println("Received: " + received);
            }
            return received;
        } catch (SocketTimeoutException se) {
            if (verbose) {
                System.out.println("Socket Timeout!");
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        return null;
    }

    /**
     * Receives a message from the socket with a specified timeout.
     *
     * @param timeout The timeout value in milliseconds.
     *
     * @return The received message as a string.
     */
    public String receive(int timeout) {
        try {
            socket.setSoTimeout(timeout);
            String received = receive();
            socket.setSoTimeout(0);
            return received;
        } catch (SocketException e) {
            e.printStackTrace();
        }
        return null;
    }

    /**
     * Closes the socket.
     */
    public void close() {
        socket.close();
    }
}