package utilities;

public class SwerveModuleConfig {
    public int wheelMotorId;
    public int turnMotorId;
    public int turnEncoderChannelA;
    public int turnEncoderChannelB;
    public boolean invertWheelMotor;
    public boolean invertTurnMotor;

    public SwerveModuleConfig(int wheelMotorId, int turnMotorId, int turnEncoderChannelA, int turnEncoderChannelB, boolean invertWheelMotor, boolean invertTurnMotor) {
        this.wheelMotorId = wheelMotorId;
        this.turnMotorId = turnMotorId;
        this.turnEncoderChannelA = turnEncoderChannelA;
        this.turnEncoderChannelB = turnEncoderChannelB;
        this.invertWheelMotor = invertWheelMotor;
        this.invertTurnMotor = invertTurnMotor;
    }
}
