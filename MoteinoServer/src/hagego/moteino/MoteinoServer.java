package hagego.moteino;

import java.io.IOException;
import java.util.Locale;
import java.util.logging.LogManager;
import java.util.logging.Logger;

import org.eclipse.paho.client.mqttv3.IMqttMessageListener;
import org.eclipse.paho.client.mqttv3.MqttMessage;

import com.pi4j.io.i2c.I2CBus;
import com.pi4j.io.i2c.I2CDevice;
import com.pi4j.io.i2c.I2CFactory;
import com.pi4j.io.i2c.I2CFactory.UnsupportedBusNumberException;


public class MoteinoServer implements IMqttMessageListener {
	
	public static void main(String[] args) {
		
		// read configuration file.
		String configDir = "";
		if(System.getProperty("os.name").toLowerCase(Locale.ENGLISH).startsWith("windows")) {
			// on windows, expect configuration data in user.home\AppData\Local\PicturePi\config.xml
			configDir = System.getProperty("user.home")+"\\Documents\\Basteln\\git\\SmartHome\\Moteino\\MoteinoServer\\conf\\";
			log.info("MoteinoServer started, running on Windows");
		}
		else {
			// on Linux/Raspberry, expect configuration data in $HOME/AlarmPi/config
			// user.home property returned root home dir when started with sudo
			configDir = "/etc/moteinoserver/";
			log.info("MoteinoServer started, running on Linux/Raspberry");
		}
		log.info("config directory="+configDir);
		
		// can't get setting of the format to work ;-( 
		System.setProperty( "java.util.logging.SimpleFormatter.format","%4$s: %5$s");
		
		// force reading of logger / handler configuration file
		System.setProperty( "java.util.logging.config.file", configDir+"logging.properties" );
		try {
			LogManager.getLogManager().readConfiguration();
			log.info("config read successfully, loglevel="+log.getLevel()+ " name="+log.getName());
		}
		catch ( Exception e ) {
			e.printStackTrace();
			return;
		}
		
		// create MoteinoServer instance
		log.info("starting MoteinoServer");
		final MoteinoServer moteinoServer;
		try {
			moteinoServer = MoteinoServer.getInstance();
		} catch (IOException e) {
			log.severe("IO Exception during creation of server instance");
			log.severe(e.getMessage());
			
			return;
		}
		log.info("MoteinoServer started successfully");
		
		// register as MQTT client
		moteinoServer.registerMqttClient();
	}
	
	/**
	 * returns the singleton instance of MoteinoServer
	 * @return singleton instance of the MoteinoServer
	 * @throws IOException 
	 */
	static MoteinoServer getInstance() throws IOException {
		if(theInstance==null) {
			theInstance = new MoteinoServer();
		}
		
		return theInstance;
	}
	
	/**
	 * private constructor. Use getInstance.
	 * @throws IOException 
	 */
	private MoteinoServer() throws IOException {
		if(System.getProperty("os.name").toLowerCase(Locale.ENGLISH).startsWith("windows")) {
			moteino = null;
		}
		else {
	        I2CBus bus = null;
			try {
				bus = I2CFactory.getInstance(I2CBus.BUS_1);
			} catch (UnsupportedBusNumberException e) {
				log.severe("Exception during I2CFactory.getInstance: "+e.getMessage());
			}
	        moteino = bus.getDevice(0x04);
		}

	}

	/**
	 * low-level IIC write to Moteino Master
	 * writes the specified slaveID,address and value
	 * @param slaveID  Moteino Slave ID
	 * @param address  Address in Moteino Slave
	 * @param value    data to write
	 * @return         true in case of success, false in case of error
	 */
	boolean lowLevelWrite(byte slaveID,byte address,byte value) {
		log.fine("IIC low level write: slaveID="+slaveID+" address="+address+" value="+value);
		if(moteino!=null) {
			try {
				moteino.write(slaveID);
				moteino.write(address);
				moteino.write(value);
			} catch (IOException e) {
				log.severe("Exception during IIC write: "+e.getMessage());
				return false;
			}
		}
		
		return true;
	}
	
	
	/**
	 * low-level IIC read from Moteino Master
	 * @param  slaveID
	 * @param  address
	 * @return value of specified register address
	 */
	Byte lowLevelRead(byte slaveID,byte address) {
		log.fine("IIC low level read: slaveID="+slaveID+" address="+address);
		
		Byte value = null;
		if(moteino!=null) {
			try {
				moteino.write(slaveID);
				moteino.write(address);
			    Thread.sleep(100);
			    value = (byte)moteino.read();
			} catch (IOException | InterruptedException e) {
				log.severe("Exception during IIC read: "+e.getMessage());
			}
		}
		
		return value;
	}
	
	
	/**
	 * IIC write to Moteino including a check of the status register
	 * @param slaveID
	 * @param address
	 * @param value
	 * @throws IICException
	 */
	void write(byte slaveID,byte address,byte value) throws IICException {
		log.info("IIC write: slaveId="+slaveID+" address="+address+" value="+value);
		
		// do low-level write on the specified address
		if(!lowLevelWrite(slaveID, address, value)) {
			throw new IICException("low-level IIC IO error");
		}
		
		// check status register
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			log.severe("exception during sleep: "+e.getMessage());
			throw new IICException("low-level IIC IO error");
		}
		
		Byte status;
		if( (status=lowLevelRead(slaveID, COMMON_REG_STATUS))!=null ) {
			if(status==0) {
				log.fine("IIC write success");
				return;
			}
			else {
				log.warning("IIC write error: status="+status);
				throw new IICException("IIC write returned status error: "+status);
			}
		}
		else {
			log.warning("IIC write: Unable to read status register");
			throw new IICException("low-level IIC IO error");
		}
	}
	
	
	/**
	 * IIC read from Moteino. Sets the register to read and then reads back the data
	 * Includes status check.
	 * @param slaveID
	 * @param address
	 * @param value
	 * @throws IICException
	 */
	byte read(byte slaveID,byte address) throws IICException {
		log.info("IIC read: slaveId="+slaveID+" address="+address);
		
		try {
			// trigger read request
			write(slaveID,COMMON_REG_READ_REQUEST,address);
			
			// wait for processing
			Thread.sleep(200);
			
			// read result
			Byte result = lowLevelRead(slaveID, address);
			if(result==null) {
				throw new IICException("low-level IIC IO error");
			}
			else {
				Thread.sleep(100);
				Byte status = lowLevelRead(slaveID, COMMON_REG_STATUS);
				if(status==null) {
					throw new IICException("low-level IIC IO error");
				}
				else {
					if(status!=0) {
						log.warning("IIC read error: status="+status);
						throw new IICException("IIC read returned status error: "+status);
					}
					else {
						// success
						return result;
					}
				}
			}
		}
		catch (InterruptedException e) {
			log.severe("exception during sleep: "+e.getMessage());
			throw new IICException("low-level IIC IO error");
		}
	}

	/**
	 * register as MQTT client and subscribe for topic
	 */
	void registerMqttClient() {
		// create MQTT client (if specified)
		log.info("Creating MQTT client");
		MqttClient.getMqttClient();
		
		MqttClient.getMqttClient().subscribe("garage/door", this);
		
		log.info("succesfully registered as MQTT client");
	}
	
	@Override
	public void messageArrived(String topic, MqttMessage message) throws Exception {
		log.fine("MQTT message arrived: topic="+topic+" content="+message);
		
		byte slaveID = (byte)2;
		byte address = (byte)10;
		byte value   = (byte)1;
		log.fine("IIC write slaveID="+slaveID+" address="+address+" value="+value);
		
		try {
			write(slaveID, address, value);
		} catch (IICException e) {
			log.severe("IIC write exception: "+e.getMessage());
		}
		
	}
	
	
	// private members
	private static final Logger log  = Logger.getLogger( MoteinoServer.class.getName() );
	
	private static MoteinoServer theInstance = null;
	private final  I2CDevice     moteino;
	
	// Moteino register addresses
	private static final byte COMMON_REG_STATUS       = 0; // status register
	private static final byte COMMON_REG_READ_REQUEST = 1; // request a read on a register
}
