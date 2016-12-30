/**
 * 
 */
package hagego.moteino;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.logging.Level;
import java.util.logging.LogManager;
import java.util.logging.Logger;

/**
 * handler class for client connections
 */
public class TcpRequestHandler implements Runnable {

	public TcpRequestHandler(Socket socket,MoteinoServer moteinoServer) {
		this.clientSocket  = socket;
		this.moteinoServer = moteinoServer;
		
		// create list with all command handlers
		commandHandlerList = new LinkedList<CommandHandler>();
		commandHandlerList.add(new CommandLoglevel());
		commandHandlerList.add(new CommandWrite());
		commandHandlerList.add(new CommandRead());
	}

	@Override
	public void run() {
		final int BUFFER_SIZE = 100; // size of command buffer
		log.info("client connected from "+clientSocket.getRemoteSocketAddress());

		try {
			boolean exit = false;
			BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
			char[] buffer = new char[BUFFER_SIZE];
			clientStream  = new PrintWriter(clientSocket.getOutputStream(), true);
			
			clientStream.println("connected to MoteinoServer");

			// read and process client commands until 'exit' command is received
			while (!exit) {
				clientStream.print("command: ");clientStream.flush();
				
				int length = bufferedReader.read(buffer, 0, BUFFER_SIZE);
				if (length <= 0) {
					log.info("client connection terminated");
					exit = true;
				} else if (length == BUFFER_SIZE) {
					log.warning("message from client exceeds max. length of "
							+ BUFFER_SIZE + ". Ignoring...");
				} else {
					String message = new String(buffer, 0, length).trim();
					log.fine("receiving message from client: " + message);

					// separate command from parameters
					int index = message.indexOf(' ');
					if (index < 0 || index==message.length()-1) {
						// no space - treat complete message as command
						command    = message.toLowerCase();
						parameters = null;
					} else {
						command    = message.substring(0, index).toLowerCase();
						parameters = message.substring(index+1).split(" ");
					}
					log.fine("received command: >>"+command+"<<");
					boolean isQuery = command.endsWith("?");
					if(isQuery) {
						command = command.substring(0, command.length()-1);
					}

					// process command
					if (command.equals("exit")) {
						clientSocket.close();
					}
					else if (command.equals("help")) {
						String answer = new String("commands:\n");
						for(CommandHandler handler:commandHandlerList) {
							answer += "  "+handler.getCommandName()+"\n";
						}
						clientStream.println(new ReturnCodeSuccess(answer));
						clientStream.flush();
					}
					else {
						Boolean processed = false;
						Iterator<CommandHandler> it = commandHandlerList.iterator();
						while(it.hasNext()) {
							CommandHandler handler = it.next();
							if(handler.getCommandName().equalsIgnoreCase(command)) {
								try {
									ReturnCode rc = handler.process(isQuery);
									log.fine("sending answer: >>"+rc+"<<");
									clientStream.println(rc);
									clientStream.flush();
									processed = true;
									break;
								} catch (CommandHandlerException e) {
									clientStream.println(new ReturnCodeError(e.getMessage()));
									clientStream.flush();
									log.info("command execution failed. message was: "+message);
								}
							}
						}
						
						if(!processed) {
							clientStream.println(new ReturnCodeError("Unknown command "+command));
							clientStream.flush();
							log.warning("received unknown command: "+command);
						}
					}

				}
			}
		} catch (IOException e) {
			log.info(e.getMessage());
		}
	}
	
	//
	// local private classes for handling of the individual commands
	//
	
	// Exception class - used for all errors
	private class CommandHandlerException extends Exception {
		private static final long serialVersionUID = 5687868525223365791L;
	}
	
	// return code for process method
	private abstract class ReturnCode {
		public ReturnCode(String message) {
			this.message = message;
		}
		
		protected String  message;
	}
	
	private class ReturnCodeSuccess extends ReturnCode {
		public ReturnCodeSuccess() {
			super("");
		}
		public ReturnCodeSuccess(String message) {
			super(message);
		}
		@Override
		public String toString() {
			return "OK\n"+message;		}
	}
	
	private class ReturnCodeError extends ReturnCode {
		public ReturnCodeError(String message) {
			super(message);
		}
		@Override
		public String toString() {
			return "ERROR\n"+message;
		}
	}

	// abstract base class
	private abstract class CommandHandler {
		
		public ReturnCode process(final boolean isQuery) throws CommandHandlerException {
			
			if(isQuery) {
				return get();
			}
			else {
				return set();
			}
		}
		
		protected abstract ReturnCode set() throws CommandHandlerException;
		
		protected abstract ReturnCode get() throws CommandHandlerException;
		
		protected abstract String getCommandName();
	};
	


	/**
	 * loglevel - log level
	 * sets or queries the current log level
	 * syntax   : loglevel <level>
	 * parameter: <level>  new java log level (warning,info,...)
	 */
	private class CommandLoglevel extends CommandHandler {
		
		@Override
		protected String getCommandName() {return "loglevel";}
		
		@Override
		public ReturnCode set() throws CommandHandlerException {
			try {
				LogManager.getLogManager().getLogger("alarmpi").setLevel(Level.parse(parameters[0].toUpperCase()));
				return new ReturnCodeSuccess();
			}
			catch(IllegalArgumentException e) {
				return new ReturnCodeError("invalid log level "+parameters[0]);
			}
		}
		
		@Override
		public ReturnCode get() throws CommandHandlerException{
			return new ReturnCodeSuccess(LogManager.getLogManager().getLogger("alarmpi").getLevel().toString());
		}
	};
	
	/**
	 */
	private class CommandWrite extends CommandHandler {
		
		@Override
		protected String getCommandName() {return "write";}
		
		@Override
		public ReturnCode set() throws CommandHandlerException {
			if(parameters==null || parameters.length!=3) {
				return new ReturnCodeError("invalid parameter count: "+parameters.length);
			}
			
			try {
				byte slaveID = (byte)Integer.parseInt(parameters[0]);
				byte address = (byte)Integer.parseInt(parameters[1]);
				byte value   = (byte)Integer.parseInt(parameters[2]);
				log.fine("IIC write slaveID="+slaveID+" address="+address+" value="+value);
				
				try {
					moteinoServer.write(slaveID, address, value);
				} catch (IICException e) {
					log.severe("IIC write exception: "+e.getMessage());
					return new ReturnCodeError(e.getMessage());
				}
				
				return new ReturnCodeSuccess();
				
				/*
				if( moteinoServer.lowLevelWrite(slaveID, address, value) ) {
					// check status register
					try {
						Thread.sleep(100);
					} catch (InterruptedException e) {
						log.severe("exception during sleep: "+e.getMessage());
					}
					Byte status;
					if( (status=moteinoServer.lowLevelRead(slaveID, (byte)0))!=null ) {
						if(status==0) {
							log.fine("IIC write success");
							return new ReturnCodeSuccess();
						}
						else {
							log.warning("IIC write: status="+status);
							return new ReturnCodeError("IIC write: status="+status);
						}
					}
					else {
						log.warning("IIC write: Unable to read status register");
						return new ReturnCodeError("IIC write: Unable to read status register");
					}
				}
				else {
					log.warning("IIC write error");
					return new ReturnCodeError("IIC write error");
				}
				*/
			}
			catch(NumberFormatException e) {
				return new ReturnCodeError("number format exception");
			}
		}
		
		@Override
		public ReturnCode get() throws CommandHandlerException{
			return new ReturnCodeError("write? not possible");
		}
	};
	
	
	private class CommandRead extends CommandHandler {
		
		@Override
		protected String getCommandName() {return "read";}
		
		@Override
		public ReturnCode set() throws CommandHandlerException {
			if(parameters==null || parameters.length!=2) {
				return new ReturnCodeError("invalid parameter count: "+parameters.length);
			}
			
			try {
				byte slaveID = (byte)Integer.parseInt(parameters[0]);
				byte address = (byte)Integer.parseInt(parameters[1]);

				Byte value;
				try {
					value = moteinoServer.read(slaveID, address);
				} catch (IICException e) {
					log.severe("IIC read exception: "+e.getMessage());
					return new ReturnCodeError(e.getMessage());
				}
				
				return new ReturnCodeSuccess(value.toString());
				
				/*
				Byte value;
				if( (value=moteinoServer.lowLevelRead(slaveID, address))!=null ) {
					return new ReturnCodeSuccess(value.toString());
				}
				else {
					return new ReturnCodeError("error during read");
				}
				*/
			}
			catch(NumberFormatException e) {
				return new ReturnCodeError("number format exception");
			}
		}
		
		@Override
		public ReturnCode get() throws CommandHandlerException{
			return new ReturnCodeError("read? not possible");
		}
	};


	
	//
	// private data members
	//
	private static final Logger log = Logger.getLogger( TcpRequestHandler.class.getName() );
	
	private final MoteinoServer        moteinoServer;
	private final Socket               clientSocket;
	private       PrintWriter          clientStream;
	private       String               command;
	private       String[]             parameters;
	private final List<CommandHandler> commandHandlerList;
	final ExecutorService threadPool = Executors.newCachedThreadPool();
}
