package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.lynx.LynxModule;
import java.util.List;

public class BulkReadSubsystem extends SubsystemBase {
    private final List<LynxModule> allHubs;

    public BulkReadSubsystem(final HardwareMap hMap){
        allHubs = hMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void periodic() {
        for (LynxModule hub: allHubs) {
            hub.clearBulkCache();
        }
    }
}