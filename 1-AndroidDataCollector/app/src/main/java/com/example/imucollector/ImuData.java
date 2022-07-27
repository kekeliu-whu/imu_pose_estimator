package com.example.imucollector;

public class ImuData {
    long timestamp;
    float[] acc;
    float[] ang;

    public ImuData(long timestamp) {
        this.timestamp = timestamp;
    }
}
