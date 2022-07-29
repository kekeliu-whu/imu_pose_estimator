package com.example.imucollector;

import com.googlecode.concurrentlinkedhashmap.ConcurrentLinkedHashMap;

import java.util.Iterator;
import java.util.Optional;

public class ImuDataCombiner {

    private final ConcurrentLinkedHashMap<Long, ImuData> map;

    public ImuDataCombiner() {
        map = new ConcurrentLinkedHashMap.Builder<Long, ImuData>()
                .maximumWeightedCapacity(1000)
                .build();
    }

    public void AddAcc(long timestamp, float[] acc) {
        ImuData imuData = map.getOrDefault(timestamp, new ImuData(timestamp));
        assert imuData != null;
        imuData.acc = acc;
        map.put(timestamp, imuData);
    }

    public void AddGyro(long timestamp, float[] ang) {
        ImuData imuData = map.getOrDefault(timestamp, new ImuData(timestamp));
        assert imuData != null;
        imuData.ang = ang;
        map.put(timestamp, imuData);
    }

    public Optional<ImuData> TryPopOldestImuData() {
        Iterator<Long> iter = map.ascendingKeySet().iterator();
        if (iter.hasNext()) {
            Long key = iter.next();
            ImuData imuData = map.get(key);
            if (imuData != null && imuData.acc != null && imuData.ang != null) {
                map.remove(key);
                return Optional.of(imuData);
            }
        }
        return Optional.empty();
    }

    public int Size() {
        return map.size();
    }

    public void Clear() {
        for (Long key : map.ascendingKeySet()) {
            ImuData imuData = map.get(key);
            if (imuData != null && imuData.acc != null && imuData.ang != null) {
                map.remove(key);
            }
        }
    }
}
