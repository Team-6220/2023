package frc.robot.subsystems;

import frc.robot.Constants.VisionConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem {
    private NetworkTable table;
    private NetworkTableEntry ty, tx, tv;
    public VisionSubsystem(){
        this.table = NetworkTableInstance.getDefault().getTable("limelight");
        this.ty = table.getEntry("ty");
        this.tx = table.getEntry("tx");
        this.tv = table.getEntry("tv");
    }

    public double getDistanceToTargetInches(){
        double verticalOffset = Math.toRadians(this.ty.getDouble(0.0));
        double angle = verticalOffset + VisionConstants.k_LIMELIGHT_ANGLE_RADIANS;
        return (VisionConstants.k_GOAL_HEIGHT_INCHES- VisionConstants.k_LIMELIGHT_HEIGHT_INCHES)/Math.tan(angle);
    }

    public double getHorizontalOffsetDegrees(){
        return this.tx.getDouble(0.0);
    }

    public boolean hasTarget(){
        return this.tv.getDouble(0) == 1;
    }
}
