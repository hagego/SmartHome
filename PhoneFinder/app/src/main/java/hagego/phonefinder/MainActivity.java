package hagego.phonefinder;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.widget.Toolbar;

import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;



public class MainActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        Toolbar toolbar = findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            startActivity(new Intent(this,SettingsActivity.class));
            return true;
        }

        return super.onOptionsItemSelected(item);
    }

    @Override
    protected void onStart() {
        super.onStart();

        Log.d(TAG,"onStart() called");

        receiver = new BroadcastReceiver() {
            @Override
            public void onReceive(Context context, Intent intent) {
                Log.d(TAG,"receiver onReceive() called with action "+intent.getAction());
                Log.d(TAG,"value of "+PhoneFinderService.STATUS_MQTT_CONNECTED+":"+intent.getBooleanExtra(PhoneFinderService.STATUS_MQTT_CONNECTED,false));
            }
        };
        IntentFilter intentFilter = new IntentFilter(PhoneFinderService.ACTION_UPDATE_STATUS);
        registerReceiver(receiver,intentFilter);
    }

    @Override
    protected void onStop() {
        super.onStop();

        unregisterReceiver(receiver);
    }

    @Override
    protected void onResume() {
        super.onResume();

        Intent intent = new Intent(this, PhoneFinderService.class);
        intent.setAction(Intent.ACTION_RUN);

        startService(intent);


    }

    //
    // member data
    //
    private static final String TAG = MainActivity.class.getSimpleName();   // logging tag

    private BroadcastReceiver receiver;
}
