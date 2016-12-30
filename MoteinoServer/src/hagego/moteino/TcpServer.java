package hagego.moteino;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.concurrent.ExecutorService;
import java.util.logging.Logger;

/**
 * TCP server class which accepts connections from clients and creates a TcpRequestHandler
 * for each client.
 */
public class TcpServer implements Runnable {

	public TcpServer(ServerSocket serverSocket,ExecutorService threadPool,MoteinoServer moteinoServer) {
		this.serverSocket  = serverSocket;
		this.threadPool    = threadPool;
		this.moteinoServer = moteinoServer;
	}

	@Override
	public void run() {
		log.info("Starting communication server on port "+serverSocket.getLocalPort());
		
		// wait for clients to connect
		while(!serverSocket.isClosed()) {
			try {
				Socket clientSocket = serverSocket.accept();
				log.fine("new connection");
				threadPool.execute(new TcpRequestHandler(clientSocket,moteinoServer));
			} catch (IOException e) {
				log.severe("IO error during client connect");
				log.severe(e.getMessage());
			}
		}
		log.warning("Shutting down TCP Server");
	}
	
	//
	// private data members
	//
	private static final Logger log = Logger.getLogger( TcpServer.class.getName() );
	private final MoteinoServer   moteinoServer;
	private final ServerSocket    serverSocket;
	private final ExecutorService threadPool;
}
