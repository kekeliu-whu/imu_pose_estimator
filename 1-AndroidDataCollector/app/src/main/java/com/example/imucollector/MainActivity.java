package com.example.imucollector;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.widget.EditText;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.widget.SwitchCompat;

import com.example.imucollector.proto.GreeterGrpc;
import com.example.imucollector.proto.Types;
import com.google.common.base.Throwables;
import com.google.common.primitives.Floats;
import com.google.protobuf.Timestamp;

import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import io.grpc.ManagedChannel;
import io.grpc.ManagedChannelBuilder;
import io.grpc.StatusRuntimeException;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    private ImuDataCombiner imuDataCollector;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        var channel = new AtomicReference<ManagedChannel>();
        var serviceStub = new AtomicReference<GreeterGrpc.GreeterBlockingStub>();

        var sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        sensorManager.registerListener(this, sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED), SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(this, sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED), SensorManager.SENSOR_DELAY_FASTEST);

        imuDataCollector = new ImuDataCombiner();

        EditText editTextLog = (EditText) findViewById(R.id.editTextLog);

        SwitchCompat aSwitch = findViewById(R.id.switchSendImuData);
        aSwitch.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if (isChecked) {
                channel.set(
                        ManagedChannelBuilder
                                .forAddress(
                                        ((EditText) findViewById(R.id.editTextGrpcEndpointIP)).getText().toString(),
                                        Integer.parseInt(((EditText) findViewById(R.id.editTextGrpcEndpointPort)).getText().toString())
                                )
                                .usePlaintext()
                                .build()
                );
                serviceStub.set(GreeterGrpc.newBlockingStub(channel.get()));
                Toast.makeText(getApplicationContext(), "Sending data ...", Toast.LENGTH_SHORT).show();
            } else {
                Toast.makeText(getApplicationContext(), "Data send off", Toast.LENGTH_SHORT).show();
            }
        });

        new Thread(() -> {
            int seq = 0;
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
                        var reply = serviceStub.get().sendImuData(request);

                        imuDataCollector.Clear();

                        editTextLog.post(
                                () -> editTextLog.setText(
                                        String.format("Post message succeed:\n\n%s\n", request.toString())
                                )
                        );

                        ++seq;
                    }
                } catch (StatusRuntimeException e) {
                    e.printStackTrace();
                    editTextLog.post(
                            () -> editTextLog.setText(
//                                    String.format("Post message failed:\n\n%s\n", Throwables.getStackTraceAsString(e))
                                    String.format("Post message failed:\n\n%s\n", e)
                            )
                    );
                }
            }
        }, "imu-handler").start();
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (!((SwitchCompat) findViewById(R.id.switchSendImuData)).isChecked()) {
            return;
        }
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