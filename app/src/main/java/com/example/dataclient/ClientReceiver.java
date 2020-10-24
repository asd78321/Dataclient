package com.example.dataclient;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.net.ConnectivityManager;
import android.net.NetworkInfo;
import android.util.Log;

public class ClientReceiver extends BroadcastReceiver {
    private static final String ClassName = BroadcastReceiver.class.getSimpleName();
    public void onReceive(Context context, Intent intent) {
        String action = intent.getAction();
//        ConnectivityManager connectivityManager = (ConnectivityManager) context
//                .getSystemService(Context.CONNECTIVITY_SERVICE);
//        NetworkInfo activeNetworkInfo = connectivityManager
//                .getActiveNetworkInfo();

        if (action.equals(Intent.ACTION_BOOT_COMPLETED)){
            Log.d(ClassName, "Connecting Server! Start mmWaveService");
            Intent ServiceIntent = new Intent(context, mmWaveService.class);
            context.startService(ServiceIntent);
        }
    }

}
