package oi.limelightvision.constants;

public class StreamingMode {
    /**
     * Standard - Side-by-side streams if a webcam is attached to Limelight
     */
    public static final int STANDARD = 0;

    /**
     * PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream
     */
    public static final int PIPMAIN = 1;
    
    /**
     * PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream
     */
    public static final int PIPSECONDARY = 2;
}