package com.example.imucollector;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.widget.SwitchCompat;

import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;
import java.util.Optional;

public class MainActivity extends AppCompatActivity implements SensorEventListener {
    private SensorManager sensorManager;

    FileOutputStream fileOutputStreamAcc;
    String filenamePrefix;

    ImuDataCombiner imuDataCombiner;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        sensorManager.registerListener(this, sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED), SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(this, sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED), SensorManager.SENSOR_DELAY_FASTEST);

        imuDataCombiner = new ImuDataCombiner();

        SwitchCompat aSwitch = findViewById(R.id.switch1);
        aSwitch.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if (isChecked) {
                try {
                    filenamePrefix = String.valueOf(System.currentTimeMillis());
                    fileOutputStreamAcc = openFileOutput(filenamePrefix + "-imu.txt", MODE_PRIVATE);
                    fileOutputStreamAcc.write("timestamp,gx,gy,gz,gxb,gyb,gzb,ax,ay,az,axb,ayb,azb\n".getBytes());
                } catch (Exception e) {
                    e.printStackTrace();
                }
            } else {
                try {
                    Toast.makeText(getApplicationContext(), "Data saved to " + filenamePrefix + "-imu.txt", Toast.LENGTH_SHORT).show();
                    fileOutputStreamAcc.close();
                    fileOutputStreamAcc = null;
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (fileOutputStreamAcc == null) {
            return;
        }
        switch (event.sensor.getType()) {
            case Sensor.TYPE_ACCELEROMETER_UNCALIBRATED:
                imuDataCombiner.AddAcc(event.timestamp, event.values);
                break;
            case Sensor.TYPE_GYROSCOPE_UNCALIBRATED:
                imuDataCombiner.AddGyro(event.timestamp, event.values);
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + event.sensor.getType());
        }

        try {
            Optional<ImuData> imuDataOp = imuDataCombiner.TryGetImuData(event.timestamp);
            if (imuDataOp.isPresent()) {
                ImuData d = imuDataOp.get();
                fileOutputStreamAcc.write(String.format(Locale.ENGLISH,
                        "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                        d.timestamp,
                        d.ang[0], d.ang[1], d.ang[2], d.ang[3], d.ang[4], d.ang[5],
                        d.acc[0], d.acc[1], d.acc[2], d.acc[3], d.acc[4], d.acc[5]
                ).getBytes());
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }
}