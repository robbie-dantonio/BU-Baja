package com.example.bajaapp;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.widget.TextView;
import android.widget.Toast;
import android.view.View;
import android.app.Activity;
import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.telephony.SmsManager;
import androidx.core.content.ContextCompat;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.model.CircleOptions;
import android.graphics.Color;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;

public class MainActivity extends AppCompatActivity implements LocationListener{

    private TextView speedTextView;
    private TextView accelerometerTextView;

    private static final int PERMISSION_REQUEST_SEND_SMS = 123;

    private GoogleMap map; // Declare the map variable
    private LocationManager locationManager;
    private SensorManager sensorManager;
    private Sensor accelerometer;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);


        // Obtain the SupportMapFragment and get notified when the map is ready to be used
        //SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager().findFragmentById(R.id.mapFragment);
        //mapFragment.getMapAsync(this);

        speedTextView = findViewById(R.id.speedTextView);
        accelerometerTextView = findViewById(R.id.accelerometerTextView);

        /*
        // Check and request location permissions
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED &&
                ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION}, 1);
        } else {
            initializeLocationManager();
        }

         */

        // Initialize accelerometer sensor
        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);

        if (accelerometer != null) {
            sensorManager.registerListener(accelerometerListener, accelerometer, SensorManager.SENSOR_DELAY_NORMAL);
        } else {
            Toast.makeText(this, "Accelerometer sensor not available", Toast.LENGTH_SHORT).show();
        }
    }

    // Implement the onMapReady method to initialize the map
    /*@Override
    public void onMapReady(GoogleMap googleMap) {
        map = googleMap;

        // Enable the zoom controls
        map.getUiSettings().setZoomControlsEnabled(true);

        // Add a marker for the car's initial position
        LatLng carPosition = new LatLng(0, 0); // Replace with your initial position
        map.addMarker(new MarkerOptions().position(carPosition).title("Car"));
        map.moveCamera(CameraUpdateFactory.newLatLngZoom(carPosition, 12));

        // Add a circle to represent the 4-mile radius
        CircleOptions circleOptions = new CircleOptions()
                .center(carPosition)
                .radius(6437.38) // 4 miles in meters
                .strokeWidth(2)
                .strokeColor(Color.RED)
                .fillColor(Color.argb(70, 255, 0, 0)); // Semi-transparent red
        map.addCircle(circleOptions);
    }
    private void initializeLocationManager() {
        locationManager = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
        if (locationManager != null) {
            locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, this);
        }
    }

     */

    private final SensorEventListener accelerometerListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent event) {
            float x = event.values[0];
            float y = event.values[1];
            float z = event.values[2];

            // Process accelerometer data
            accelerometerTextView.setText("X: " + x + "\nY: " + y + "\nZ: " + z);
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {
            // Handle accuracy changes
        }
    };

    @Override
    public void onLocationChanged(Location location) {
        // Process GPS data
        float speed = location.getSpeed();
        speedTextView.setText("Speed: " + speed);
    }

    @Override
    public void onStatusChanged(String provider, int status, Bundle extras) {
        // Handle status changes
    }

    @Override
    public void onProviderEnabled(String provider) {
        // Handle provider enabled
    }

    @Override
    public void onProviderDisabled(String provider) {
        // Handle provider disabled
    }

    private void sendMessage() {
        String phoneNumber = "+16095568516"; // Replace with the desired recipient phone number
        String message = "Hello, this is a text message!"; // Replace with your message content

        try {
            SmsManager smsManager = SmsManager.getDefault();
            smsManager.sendTextMessage(phoneNumber, null, message, null, null);
            Toast.makeText(getApplicationContext(), "SMS sent.", Toast.LENGTH_LONG).show();
        } catch (Exception e) {
            Toast.makeText(getApplicationContext(), "SMS failed, please try again.", Toast.LENGTH_LONG).show();
            e.printStackTrace();
        }
    }

    private void requestSmsPermission() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.SEND_SMS) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.SEND_SMS}, PERMISSION_REQUEST_SEND_SMS);
        } else {
            // Permission already granted, proceed with sending SMS
            sendMessage();
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        if (requestCode == PERMISSION_REQUEST_SEND_SMS) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                // Permission granted, proceed with sending SMS
                sendMessage();
            } else {
                // Permission denied, handle accordingly (e.g., show a message or disable the SMS functionality)
                Toast.makeText(this, "SMS permission denied.", Toast.LENGTH_SHORT).show();
            }
        }
    }

}
