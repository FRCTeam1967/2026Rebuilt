package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.hardware.ParentDevice;

import frc.robot.Constants;

public class BulkCANSignalUpdater {
    public static enum CANBusType {
        RIO,
        CANIVORE
    };

    private StatusSignalCollection rioCollection = new StatusSignalCollection();
    private StatusSignalCollection canivoreCollection = new StatusSignalCollection();
    private static BulkCANSignalUpdater singleton;

    private BulkCANSignalUpdater() {
    }

    public static BulkCANSignalUpdater getInstance() {
        if (singleton == null) {
            singleton = new BulkCANSignalUpdater();
        }

        return singleton;
    }

    public void registerSignals(CANBusType busType, BaseStatusSignal... signals) {
        (busType == CANBusType.RIO ? rioCollection : canivoreCollection).addSignals(signals);
    }

    public void optimizeDevices(ParentDevice... devices) {
        if (Constants.CANUpdateFrequencies.optimize) {
            ParentDevice.optimizeBusUtilizationForAll(devices);
        }
    }

    public void refreshAll() {
        rioCollection.refreshAll();
        canivoreCollection.refreshAll();
    }
}
