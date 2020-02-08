package frc.robot.limelight;

import edu.wpi.first.networktables.*;

//import frc.robot.limelight.constants.*;

public final class LimeLight {
    private final String _tableName; 
	private final NetworkTable _table; 
	
	// highest pipeline currently set
	public static final int maxPipelineIds = 1;

	// adjust as needed
	private static final double _mountingAngle = 0;
	private static final double _heightOfCamera = 1; 

    /**
     * The LimeLight class constructor. 
     */
    public LimeLight() { 
        this._tableName = "limelight";
        this._table = NetworkTableInstance.getDefault().getTable(this._tableName);
    }

    /**
     * Determines whether the target is found or not. This is represented by the "tv" table in the Shuffleboard. 
     * @return true if the target is found, false otherwise. 
     */
    public boolean hasTarget() { 
        NetworkTableEntry tv = this._table.getEntry("tv");
        return tv.getDouble(0) == 1.0;
    }

    /**
     * Gets the horizontal offset from the crosshair to the target. This can be anywhere from -27 degrees to 27 degrees, inclusive. 
     * @return The horizontal offset. 
     */
    public double getHorizontalOffset() { 
        NetworkTableEntry tx = this._table.getEntry("tx");
        return tx.getDouble(0);
    }
    
    /**
     * Gets the vertical offset from the crosshair to the target. This can be anywhere from -20.5 degrees to 20.5 degrees. 
     * @return The verical offset. 
     */
    public double getVerticalOffset() { 
        NetworkTableEntry ty = this._table.getEntry("ty");
        return ty.getDouble(0);
    }

    /**
     * Gets the target area (0% of image to 100% of image). 
     * @return The target area (0 to 1, inclusive). 
     */
    public double getTargetArea() { 
        NetworkTableEntry ta = this._table.getEntry("ta");
        return ta.getDouble(0);
    }

    /**
     * Gets the current camera mode.
     * @return The camera mode (either VISION or DRIVER).
     */
    public String getCurrentCameraMode() { 
        NetworkTableEntry camMode = this._table.getEntry("camMode");
        return LimeLightConstants.getCamModeState((int)camMode.getDouble(0));
    }

    /**
     * Returns the current pipeline of the limelight.
     * @return The current pipeline. 
     */
    public double getCurrentPipelineDouble() { 
        NetworkTableEntry pipeline = this._table.getEntry("pipeline");
        return pipeline.getDouble(0.0);
    }

    /**
     * Returns the current pipeline of the limelight.
     * @return The current pipeline. 
     */
    public int getCurrentPipelineInt() { 
        return (int)this.getCurrentPipelineDouble();
	}
	
	/**
	 * Sets the pipeline.
	 * @param id The pipeline id. 
	 */
	public void setPipeline(int id) { 
		if (id > maxPipelineIds) {
			id = maxPipelineIds;
		}

		if (id < 0) {
			id = 0;
		}

        NetworkTableEntry pipeline = this._table.getEntry("pipeline");
		pipeline.forceSetNumber(id);	
	}

	/**
	 * Gets the next pipeline. Useful if you want to have a button or controller change the pipeline. 
	 */
	public void setNextPipeline() {
		Number currentPipelineId = this._table.getEntry("pipeline").getNumber(0);	
		int newPipeline;
		// if the next number will go past the max registered pipelines
		// then we want to switch back to 0.
		if (currentPipelineId.intValue() + 1 > maxPipelineIds) {
			newPipeline = 0;
		}
		else {
			newPipeline = currentPipelineId.intValue() + 1;
		}
		this._table.getEntry("pipeline").forceSetNumber(newPipeline);
	}

	/**
	 * Determines the distance from the limelight (or the robot) to the target. 
	 * @param heightOfTarget The height of the target.  
	 * @return If a target is found, the distance from the limelight to the target. Otherwise, -1. 
	 */
	public double getDistanceFromTarget(double heightOfTarget) { 
		if (this.hasTarget()) {	
			return (heightOfTarget - _heightOfCamera) / Math.tan(_mountingAngle + this.getVerticalOffset());
		}
		else {
			return -1; 
		}
	}
}