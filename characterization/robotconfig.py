{
    # Ports for the left-side motors
    "leftMotorPorts": [14, 15],
    # Ports for the right-side motors
    "rightMotorPorts": [1, 2],
    # Note: Inversions of the slaves (i.e. any motor *after* the first on
    # each side of the drive) are *with respect to their master*.  This is
    # different from the other poject types
    # TrevNote: also in the update as of 1/21/20 you must type inversions as if
    # you allowed differentialDrive to automatically invert them (for us both
    # masters should have the same inversion.
    # Inversions for the left-side motors
    "leftMotorsInverted": [True, False],
    # Inversions for the right side motors
    "rightMotorsInverted": [True, False],
    # The total gear reduction between the motor and the wheels, expressed as
    # a fraction [motor turns]/[wheel turns]
    "gearing": 10.71,
    # Wheel diameter (in units of your choice - will dictate units of analysis)
    "wheelDiameter": 0.1524, #this is in meters
    # Your gyro type (one of "NavX", "Pigeon", "ADXRS450", "AnalogGyro", or "None")
    "gyroType": "NavX",
    # Whatever you put into the constructor of your gyro
    # Could be:
    # "SPI.Port.kMXP" (MXP SPI port for NavX or ADXRS450),
    # "SerialPort.Port.kMXP" (MXP Serial port for NavX),
    # "I2C.Port.kOnboard" (Onboard I2C port for NavX),
    # "0" (Pigeon CAN ID or AnalogGyro channel),
    # "new WPI_TalonSRX(3)" (Pigeon on a Talon SRX),
    # "" (NavX using default SPI, ADXRS450 using onboard CS0, or no gyro)
    "gyroPort": "SerialPort.Port.kUSB",
}

