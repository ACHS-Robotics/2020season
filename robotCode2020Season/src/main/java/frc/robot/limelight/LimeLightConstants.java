package frc.robot.limelight;

public final class LimeLightConstants {
    private static String[] CamModeStates = { "VISION", "DRIVER" };
    /**
     * Gets the current camera mode. 
     * @param index The index. This should either be 0 or 1. 
     * @return The state (either VISION or DRIVER).
     */
    public static String getCamModeState(int index) {
        return CamModeStates[index];
    }

    /**
     * Gets the current index of the camera mode. 
     * @param name The state. This should eitherbe DRIVER or VISION. 
     * @return The index. 
     */
    public static double getCamModeIndex(String name) { 
        for (int i = 0; i < CamModeStates.length; i++) {
            if (CamModeStates[i].equals(name)) {
                return i;
            }
        }
        return -1; 
    }
}