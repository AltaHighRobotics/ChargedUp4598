package utilities;

import frc.robot.subsystems.DriveTrainSub;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private double verticalThreshold;
    private double horizontalThresold;
    
    public AutoAlignment(DriveTrainSub driveTrainSub, Vision vision, double verticalSetpoint, double horizontalSetpoint, boolean aprilTagMode) {
        m_driveTrainSub = driveTrainSub;
        m_vision = vision;

        this.verticalSetpoint = verticalSetpoint;
        this.horizontalSetpoint = horizontalSetpoint;

        // Pids.
        horizontalPid = new ConfigurablePID(Constants.LIMELIGHT_HORIZONTAL_PID);

        if (aprilTagMode) {
            verticalThreshold = Constants.AUTO_ALIGNMENT_VERTICAL_THRESHOLD;
            horizontalThresold = Constants.AUTO_ALIGNMENT_HORIZONTAL_THRESHOLD;
            verticalPid = new ConfigurablePID(Constants.LIMELIGHT_VERTICAL_PID);
        } else {
            verticalThreshold = Constants.AUTO_ALIGNMENT_REFLECTIVE_VERTICAL_THRESHOLD;
            horizontalThresold = Constants.AUTO_ALIGNMENT_REFLECTIVE_HORIZONTAL_THRESHOLD;
            verticalPid = new ConfigurablePID(Constants.REFLECTIVE_LIMELIGHT_VERTICAL_PID);
        }
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
        double x = m_vision.getHorizontalOffset();
        double y = m_vision.getVerticalOffset();

        // Run pids.
        double speed = verticalPid.runPID(verticalSetpoint, y);
        double strafe = horizontalPid.runPID(horizontalSetpoint, x);

        SmartDashboard.putNumber("Horizontal setpoint", horizontalSetpoint);
        SmartDashboard.putNumber("Vertical setpoint", verticalSetpoint);

        // At position.
        if (Math.abs(verticalPid.getError()) <= verticalThreshold 
        && Math.abs(horizontalPid.getError()) <= horizontalThresold) {
            atPosition = true;
            m_driveTrainSub.stop();
            return true;
        }

        //strafe = 0.0;

        m_driveTrainSub.setSwerveDrive(strafe, -speed, 0.0, false, false);
        m_driveTrainSub.run();
        atPosition = false;
        return false;
    }
}
