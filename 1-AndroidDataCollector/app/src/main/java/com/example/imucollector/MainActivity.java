package com.example.imucollector;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.widget.SwitchCompat;

import com.example.imucollector.proto.GreeterGrpc;
import com.example.imucollector.proto.Types;
import com.google.common.primitives.Floats;
import com.google.protobuf.Timestamp;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import io.grpc.ManagedChannel;
import io.grpc.ManagedChannelBuilder;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    ImuDataCombiner imuDataCollector;

    private int seq = 0;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        ManagedChannel channel = ManagedChannelBuilder.forAddress("192.168.0.108", 32345).usePlaintext().build();
        var serviceStub = GreeterGrpc.newBlockingStub(channel);

        SensorManager sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        sensorManager.registerListener(this, sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED), SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(this, sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED), SensorManager.SENSOR_DELAY_FASTEST);

        imuDataCollector = new ImuDataCombiner();

        SwitchCompat aSwitch = findViewById(R.id.switch1);
        aSwitch.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if (isChecked) {
            } else {
                Toast.makeText(getApplicationContext(), "Data saved to imu.txt", Toast.LENGTH_SHORT).show();
            }
        });

        new Thread(() -> {
            while (true) {
                try {
                    Optional<ImuData> imuDataOp = imuDataCollector.TryPopOldestImuData();
                    if (imuDataOp.isPresent()) {
                        ImuData d = imuDataOp.get();

                        var request = Types.ImuData.newBuilder()
                                .setTimestamp(Timestamp
                                        .newBuilder()
                                        .setSeconds(d.timestamp / 1_000_000_000L)
                                        .setNanos((int) (d.timestamp % 1_000_000_000L))
                                        .build()
                                )
                                .setSeq(seq)
                                .addAllAngularVelocity(Floats.asList(d.ang))
                                .addAllLinearAcceleration(Floats.asList(d.acc))
                                .build();
                        var reply = serviceStub.sendImuData(request);

                        imuDataCollector.Clear();

                        ++seq;
                    }
                } catch (io.grpc.StatusRuntimeException e) {
                    e.printStackTrace();
                }
            }
        }, "imu-handler").start();
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        switch (event.sensor.getType()) {
            case Sensor.TYPE_ACCELEROMETER_UNCALIBRATED:
                imuDataCollector.AddAcc(event.timestamp, event.values);
                break;
            case Sensor.TYPE_GYROSCOPE_UNCALIBRATED:
                imuDataCollector.AddGyro(event.timestamp, event.values);
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + event.sensor.getType());
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }
}