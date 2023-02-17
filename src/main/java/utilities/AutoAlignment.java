package utilities;

import frc.robot.subsystems.DriveTrainSub;
import frc.robot.subsystems.Vision;
import frc.robot.Constants;

public class AutoAlignment {
    // Pids.
    private ConfigurablePID horizontalPid;
    private ConfigurablePID verticalPid;

    private DriveTrainSub m_driveTrainSub;
    private Vision m_vision;

    private boolean atPosition = false;

    private double verticalSetpoint;
    private double horizontalSetpoint;
    
    public AutoAlignment(DriveTrainSub driveTrainSub, Vision vision, double verticalSetpoint, double horizontalSetpoint) {
        m_driveTrainSub = driveTrainSub;
        m_vision = vision;

        this.verticalSetpoint = verticalSetpoint;
        this.horizontalSetpoint = horizontalSetpoint;

        // Pids.
        horizontalPid = new ConfigurablePID(Constants.LIMELIGHT_HORIZONTAL_PID);
        verticalPid = new ConfigurablePID(Constants.LIMELIGHT_VERTICAL_PID);
    }

    public void reset() {
        horizontalPid.resetValues();
        verticalPid.resetValues();
        atPosition = false;
    }

    public boolean isAtPosition() {
        return atPosition;
    }

    // Runs alignment pids and drivetrain.
    // Returns true if at position.
    public boolean run() {
        // Get x and y.
        double x = m_vision.getAprilTagHorizontalOffset();
        double y = m_vision.getAprilTagVerticalOffset();

        // Run pids.
        double speed = verticalPid.runPID(verticalSetpoint, y);
        double strafe = horizontalPid.runPID(horizontalSetpoint, x);

        // At position.
        if (verticalPid.getError() == 0.0 && horizontalPid.getError() == 0.0) {
            atPosition = true;
            m_driveTrainSub.stop();
            return true;
        }

        m_driveTrainSub.setSwerveDrive(strafe, speed, 0.0, false);
        m_driveTrainSub.run();
        atPosition = false;
        return false;
    }
}
