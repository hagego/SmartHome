package hagego.phonefinder;

import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.Service;
import android.content.Intent;
import android.content.SharedPreferences;
import android.media.AudioManager;
import android.media.Ringtone;
import android.media.RingtoneManager;
import android.net.Uri;
import android.os.IBinder;
import android.preference.PreferenceManager;
import android.util.Log;

import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.IMqttMessageListener;
import org.eclipse.paho.client.mqttv3.MqttAsyncClient;
import org.eclipse.paho.client.mqttv3.MqttCallbackExtended;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;

import java.util.Timer;
import java.util.TimerTask;

import androidx.core.app.NotificationCompat;
import androidx.core.app.NotificationManagerCompat;

import static android.app.Notification.EXTRA_NOTIFICATION_ID;

public class PhoneFinderService extends Service implements MqttCallbackExtended, IMqttMessageListener {
    public PhoneFinderService() {
    }

    @Override
    public IBinder onBind(Intent intent) {
        // TODO: Return the communication channel to the service.
        throw new UnsupportedOperationException("Not yet implemented");
    }

    /**
     * connects to MQTT server
     * @param server MQTT server name
     * @param id     unique phone ID
     */
    private void connect(String server,String id) {
        String uri = "tcp://"+server;

        try {
            Log.d(TAG,"trying initial connection to MQTT server, URI="+uri);

            MqttConnectOptions connectOptions = new MqttConnectOptions();
            connectOptions.setAutomaticReconnect(true);
            connectOptions.setCleanSession(true);

            mqttClient = new org.eclipse.paho.client.mqttv3.MqttAsyncClient(uri, id, new MemoryPersistence());
            mqttClient.setCallback(this);
            mqttClient.connect(connectOptions);

            Log.i(TAG,"initial connection request to MQTT server successful");
        }
        catch(MqttException | NullPointerException e) {
            // invalid URI string leads to null pointer exception
            mqttClient = null;
            Log.e(TAG,"Exception during initial connection in onCreate():",e);
        }
    }

    @Override
    public void onCreate() {
        Log.d(TAG, "onCreate called");
        super.onCreate();

        // create an Notification channel for the mandatory foreground service notification
        NotificationChannel channelRunning = new NotificationChannel(NOTIFICATION_CHANNEL_RUNNING,
                getString(R.string.notification_channel_running_name),
                NotificationManager.IMPORTANCE_NONE);
        channelRunning.setDescription(getString(R.string.notification_channel_running_description));

        NotificationManager notificationManager = getSystemService(NotificationManager.class);
        notificationManager.createNotificationChannel(channelRunning);

        // create an Notification channel for the real notification when someone searches the phone
        NotificationChannel channelActive = new NotificationChannel(NOTIFICATION_CHANNEL_ACTIVE,
                getString(R.string.notification_channel_active_name),
                NotificationManager.IMPORTANCE_HIGH);
        channelActive.setDescription(getString(R.string.notification_channel_active_description));

        notificationManager.createNotificationChannel(channelActive);

        // create intent to start main activity by the notification
        Intent intent = new Intent(this, MainActivity.class);
        intent.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK | Intent.FLAG_ACTIVITY_CLEAR_TASK);
        PendingIntent pendingIntent = PendingIntent.getActivity(this, 0, intent, 0);

        // now create the mandatory notification for this foreground service
        NotificationCompat.Builder builder = new NotificationCompat.Builder(this, NOTIFICATION_CHANNEL_RUNNING)
                .setSmallIcon(R.drawable.ic_stat_new_message)
                .setContentTitle(getString(R.string.notification_running_title))
                .setContentText(getString(R.string.notification_running_text))
                .setContentIntent(pendingIntent)
                .setPriority(NotificationCompat.PRIORITY_DEFAULT);

        startForeground(1,builder.build());

        /*
        // get MQTT related data from preferences
        SharedPreferences preferences = PreferenceManager.getDefaultSharedPreferences(this);
        String keyServer  = getString(R.string.preference_key_mqtt_uri);
        String keyPhoneID = getString(R.string.preference_key_phone_id);
        if(preferences.contains(keyServer) && preferences.contains(keyPhoneID)) {
            Log.d(TAG,"preferences found during onCreate");
            String server = preferences.getString(keyServer,null);
            String id     = preferences.getString(keyPhoneID,null);
            Log.d(TAG,"MQTT server: "+server+" Phone ID:"+id);

            if(server!=null && id!=null) {
                // create client and try to connect
                connect(server,id);
            }
            else {
                Log.w(TAG,"invalid preferences in onCreate(). MQTT server="+server+" Phone ID="+id);
            }
        }
        else {
            Log.d(TAG,"no preferences found in onCreate() - unable to connect");
        }
        */
    }


    @Override
    public int onStartCommand(Intent intent,int flags,int startId) {
        Log.d(TAG, "onStartCommand() called. Action="+intent.getAction());

        if(intent.getAction()!=null && intent.getAction().equals(ACTION_STOP_RINGING)) {
            Log.i(TAG, "onStartCommand(): received stop ringing command");

            stopRinging();
        }

        if(intent.getAction()!=null && intent.getAction().equals(Intent.ACTION_RUN)) {
            Log.i(TAG, "onStartCommand(): received START command");
            if(mqttClient == null) {
                Log.i(TAG, "MQTT client object still null in onStartCommand()");

                SharedPreferences preferences = PreferenceManager.getDefaultSharedPreferences(this);
                String keyServer = getString(R.string.preference_key_mqtt_uri);
                String keyPhoneID = getString(R.string.preference_key_phone_id);
                if (preferences.contains(keyServer) && preferences.contains(keyPhoneID)) {
                    Log.d(TAG, "preferences found during onStartCommand");
                    String server = preferences.getString(keyServer, null);
                    String id = preferences.getString(keyPhoneID, null);
                    Log.d(TAG, "MQTT server: " + server + " Phone ID:" + id);

                    if (server != null && id != null) {
                        // create client and try to connect
                        connect(server, id);
                    } else {
                        Log.w(TAG, "invalid preferences in onCreate(). MQTT server=" + server + " Phone ID=" + id);
                    }
                } else {
                    Log.w(TAG, "no preferences found in onStartCommand()");
                }
            }
        }

        return START_STICKY;
    }

    @Override
    public void connectionLost(Throwable cause) {
        isConnected = false;
        Log.d(TAG,"MQTT connection lost: "+cause.getMessage(),cause);

        broadcastStatus();
    }

    @Override
    public void messageArrived(String topic, MqttMessage message) {
        String data = message.toString();
        Log.d(TAG,"MQTT message arrived, topic="+topic+" content="+data);

        if(data.equals(MQTT_TOPIC_VALUE_TRIGGER)) {
            Log.d(TAG,"received MQTT command: trigger");

            Intent stopIntent = new Intent(this, PhoneFinderService.class);
            stopIntent.setAction(ACTION_STOP_RINGING);
            stopIntent.putExtra(EXTRA_NOTIFICATION_ID, 0);
            PendingIntent foundPendingIntent =
                    PendingIntent.getForegroundService(this, 0, stopIntent, 0);

            // create notification
            NotificationCompat.Builder builder = new NotificationCompat.Builder(this, NOTIFICATION_CHANNEL_ACTIVE)
                    .setSmallIcon(R.drawable.ic_stat_new_message)
                    .setContentTitle(getString(R.string.notification_active_title))
                    .setContentText(getString(R.string.notification_active_text))
                    .setPriority(NotificationCompat.PRIORITY_HIGH)
                    .addAction(R.drawable.ic_stat_new_message, getString(R.string.notification_active_action_stop),foundPendingIntent)
                    .setShowWhen(true)
                    .setUsesChronometer(true)
                    .setAutoCancel(true);

            NotificationManagerCompat notificationManager = NotificationManagerCompat.from(this);
            notificationManager.notify(2, builder.build());

            startRinging();

            // schedule a timer to stop ringing in case no-one stops manually
            Timer timerObj = new Timer();
            TimerTask timerTaskObj = new TimerTask() {
                public void run() {
                    Log.d(TAG,"timer to stop ringing expired");
                    stopRinging();
                }
            };
            timerObj.schedule(timerTaskObj, STOP_RINGING_TIMER_DEFFAULT*1000);
        }

        broadcastStatus();
    }


    @Override
    public void connectComplete(boolean reconnect, String serverURI) {
        isConnected = true;
        Log.d(TAG,"MQTT connection succeeded, reconnect="+reconnect);

        SharedPreferences preferences = PreferenceManager.getDefaultSharedPreferences(this);
        String keyPhoneID = getString(R.string.preference_key_phone_id);
        if (preferences.contains(keyPhoneID)) {
            String id = preferences.getString(keyPhoneID, null);
            if(id!=null) {
                String topic = MQTT_TOPIC_BASE+id;
                Log.d(TAG,"subscribing for topic "+topic);
                try {
                    mqttClient.subscribe(topic, 0, this);
                    Log.d(TAG,"subscription of topic "+topic+ " successfull.");
                } catch (MqttException e) {
                    Log.e(TAG, "MQTT subsribe failed: ", e);
                }
            }
        }

        broadcastStatus();
    }

    @Override
    public void deliveryComplete(IMqttDeliveryToken token) {

    }

    private synchronized void startRinging() {
        Uri alarmUri = RingtoneManager.getDefaultUri(RingtoneManager.TYPE_ALARM);
        if (alarmUri == null) {
            alarmUri = RingtoneManager.getDefaultUri(RingtoneManager.TYPE_NOTIFICATION);
        }


        // Get the audio manager instance
        AudioManager audioManager = (AudioManager) getSystemService(AUDIO_SERVICE);
        // Get the ringer maximum volume
        int max_volume_level = audioManager.getStreamMaxVolume(AudioManager.STREAM_RING);

        // Set the ringer volume
        audioManager.setStreamVolume(
                AudioManager.STREAM_RING,
                max_volume_level,
                AudioManager.FLAG_SHOW_UI
        );

        ringtone = RingtoneManager.getRingtone(this, alarmUri);
        ringtone.setVolume(1.0f);
        ringtone.setLooping(true);
        ringtone.play();
    }

    private synchronized void stopRinging() {
        if(ringtone!=null) {
            ringtone.stop();
            ringtone = null;
        }
    }

    private synchronized boolean getRinging() {
        return ringtone!=null;
    }

    private synchronized void broadcastStatus() {
        Intent intent = new Intent();
        intent.setAction(ACTION_UPDATE_STATUS);
        intent.putExtra(STATUS_MQTT_CONNECTED,isConnected);

        sendBroadcast(intent);
    }

    //
    // member data
    //
    private static final String TAG = PhoneFinderService.class.getSimpleName();   // logging tag

    private static final String MQTT_TOPIC_BASE          = "phonefinder/";
    private static final String MQTT_TOPIC_VALUE_TRIGGER = "trigger";

    private static final String NOTIFICATION_CHANNEL_RUNNING = "RUNNING";         // notification channel ID for required notification as foreground service
    private static final String NOTIFICATION_CHANNEL_ACTIVE  = "ACTIVE";          // notification channel ID for real notification

    static final String ACTION_STOP_RINGING         = "hagego.phonefinder.stop_ringing";
    static final String ACTION_UPDATE_STATUS        = "hagego.phonefinder.update_status";
    static final int    STOP_RINGING_TIMER_DEFFAULT = 30;                                 // default timeout in seconds to stop ringing

    static final String STATUS_MQTT_CONNECTED       = "hagego.phonefinder.connected_mqtt";

    private MqttAsyncClient mqttClient  = null;
    private boolean         isConnected = false;
    private Ringtone        ringtone    = null;
}
