package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.limelight.LimeLight;

public final class S_LimeLight extends SubsystemBase {
    private LimeLight _limelight;

    public S_LimeLight() { 
        this._limelight = new LimeLight();
    }

    public LimeLight getLimeLight() { 
        return this._limelight;
    }
}