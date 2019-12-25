package hagego.phonefinder;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.util.Log;

/**
 * helper class to register the service for automatic start
 */
public class PhoneFinderReceiver extends BroadcastReceiver {

    @Override
    public void onReceive(Context context, Intent intent) {
        if (intent.getAction()!=null && intent.getAction().equalsIgnoreCase(Intent.ACTION_BOOT_COMPLETED)) {
            Log.d(TAG,"received boot completed message");

            Intent intentStartService = new Intent(context, PhoneFinderService.class);
            intentStartService.setAction(Intent.ACTION_RUN);

            context.startForegroundService(intentStartService);
        }
    }

    private static final String TAG = PhoneFinderReceiver.class.getSimpleName();   // logging tag
}
