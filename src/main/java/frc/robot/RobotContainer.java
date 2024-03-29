// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.commands.ArmPositions.*;
import frc.robot.commands.PlacingCommands.CenterLeftPlaceCommand;
import frc.robot.commands.PlacingCommands.CenterMiddlePlaceCommand;
import frc.robot.commands.PlacingCommands.CenterRightPlaceCommand;
import frc.robot.commands.PlacingCommands.LowerLeftPlaceCommand;
import frc.robot.commands.PlacingCommands.LowerMiddlePlaceCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.PlacingCommands.LowerRightPlaceCommand;
import frc.robot.commands.PlacingCommands.UpperLeftPlaceCommand;
import frc.robot.commands.PlacingCommands.UpperMiddlePlaceCommand;
import frc.robot.commands.PlacingCommands.UpperRightPlaceCommand;
import frc.robot.subsystems.*;
import frc.robot.autonomous.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's sussystems and commands are defined here...

  // Controllers
  private final XboxController m_driveController = new XboxController(Constants.DRIVE_CONTROLLER);
  private final Joystick m_ButtonBox = new Joystick(Constants.BUTTON_BOX);

  // Subsystems
  private final DriveTrainSub m_driveTrainSub = new DriveTrainSub();
  private final ArmAndClawSub m_armAndClawSub = new ArmAndClawSub();
  private final Vision m_Vision = new Vision();

  // Commands
  private final DriveCommand m_driveCommand = new DriveCommand(m_driveTrainSub, m_driveController, m_armAndClawSub);
  private final ZeroAngleCommand m_zeroAngleCommand = new ZeroAngleCommand(m_driveTrainSub);
  private final TestArmCommad m_testArmCommand = new TestArmCommad(m_armAndClawSub);
  private final OpenClawCommand m_openClawCommand = new OpenClawCommand(m_armAndClawSub);
  private final CloseClawCommand m_CloseClawCommand = new CloseClawCommand(m_armAndClawSub);
  private final ClawOpenAndCloseCommand m_clawOpenAndCloseCommand = new ClawOpenAndCloseCommand(m_armAndClawSub);
  private final LowerSmallArmCommand m_lowerSmallArmCommand = new LowerSmallArmCommand(m_armAndClawSub);
  private final ArmRestPositionCommand m_armRestPositionCommand = new ArmRestPositionCommand(m_armAndClawSub);
  private final AprilTagTestCommand m_AprilTagTestCommand = new AprilTagTestCommand(m_Vision);
  private final AprilTagToggleCommand m_AprilTagToggleCommand = new AprilTagToggleCommand(m_Vision);
  private final TogglePipelineCommand m_TogglePipelineCommand = new TogglePipelineCommand(m_Vision);
  private final ArmGrabPositionCommand m_armGrabPositionCommand = new ArmGrabPositionCommand(m_armAndClawSub);
  private final ClawUpCommand m_ClawUpCommand = new ClawUpCommand(m_armAndClawSub);
  private final ClawDownCommand m_ClawDownCommand = new ClawDownCommand(m_armAndClawSub);
  private final ZeroClawCommand m_zeroClawCommand = new ZeroClawCommand(m_armAndClawSub);
  private final SpinClawCommand m_spinClawCommand = new SpinClawCommand(m_armAndClawSub);
  private final ClawIntakeCommand m_ClawIntakeCommand = new ClawIntakeCommand(m_armAndClawSub);
  private final ClawOutCommand m_clawOutCommand = new ClawOutCommand(m_armAndClawSub);

  //Placing Commands
  private final CenterLeftPlaceCommand m_CenterLeftPlaceCommand = new CenterLeftPlaceCommand(m_driveController, m_driveTrainSub, m_armAndClawSub, m_Vision);
  private final CenterMiddlePlaceCommand m_CenterMiddlePlaceCommand = new CenterMiddlePlaceCommand(m_driveController, m_driveTrainSub, m_armAndClawSub, m_Vision);
  private final CenterRightPlaceCommand m_CenterRightPlaceCommand = new CenterRightPlaceCommand(m_driveController, m_driveTrainSub, m_armAndClawSub, m_Vision);
  private final LowerLeftPlaceCommand m_LowerLeftPlaceCommand = new LowerLeftPlaceCommand(m_driveController, m_driveTrainSub, m_armAndClawSub, m_Vision);
  private final LowerMiddlePlaceCommand m_LowerMiddlePlaceCommand = new LowerMiddlePlaceCommand(m_driveController, m_driveTrainSub, m_armAndClawSub, m_Vision);
  private final LowerRightPlaceCommand m_LowerRightPlaceCommand = new LowerRightPlaceCommand(m_driveController, m_driveTrainSub, m_armAndClawSub, m_Vision);
  private final UpperLeftPlaceCommand m_UpperLeftPlaceCommand = new UpperLeftPlaceCommand(m_driveController, m_driveTrainSub, m_armAndClawSub, m_Vision);
  private final UpperMiddlePlaceCommand m_UpperMiddlePlaceCommand = new UpperMiddlePlaceCommand(m_driveController, m_driveTrainSub, m_armAndClawSub, m_Vision);
  private final UpperRightPlaceCommand m_UpperRightPlaceCommand = new UpperRightPlaceCommand(m_driveController, m_driveTrainSub, m_armAndClawSub, m_Vision);

  // Position commands.
  private final ArmHigherPositionCommand m_armHigherPositionCommand = new ArmHigherPositionCommand(m_armAndClawSub);
  private final ArmMiddlePositionCommand m_armMiddlePositionCommand = new ArmMiddlePositionCommand(m_armAndClawSub);
  private final ArmHigherConePositionCommand m_ArmHigherConePositionCommand = new ArmHigherConePositionCommand(m_armAndClawSub);
  private final ArmMiddleConeCommand m_ArmMiddleConeCommand = new ArmMiddleConeCommand(m_armAndClawSub);
  private final ArmHighGrabPositionCommand m_ArmHighGrabPositionCommand = new ArmHighGrabPositionCommand(m_armAndClawSub);
  private final ArmGrabPositionCommand m_ArmGrabPositionCommand = new ArmGrabPositionCommand(m_armAndClawSub);
  private final ArmGrabConeGroundCommand m_armGrabConeGroundCommand = new ArmGrabConeGroundCommand(m_armAndClawSub);



  // Autonomous.
  private final BalanceAutoCommand m_balanceAutoCommand = new BalanceAutoCommand(m_driveTrainSub);
  private final Autonomous1Command m_autonomous1Command = new Autonomous1Command(m_driveTrainSub, m_armAndClawSub, m_Vision);
  private final PlaceAndStayAutoCommand m_placeAndStayAutoCommand = new PlaceAndStayAutoCommand(m_driveTrainSub, m_armAndClawSub, m_Vision);
  private final PlaceAndBalanceAutoCommand m_placeAndBalanceAutoCommand = new PlaceAndBalanceAutoCommand(m_driveTrainSub, m_armAndClawSub);

  private final SendableChooser<Command> m_autonomousChooser = new SendableChooser<>();

  // Arm positions.
  //private final ArmMiddlePositionCommand

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings

    // Autonomous chooser.
    m_autonomousChooser.setDefaultOption("Place and drive back", m_autonomous1Command);
    m_autonomousChooser.addOption("Place and stay", m_placeAndStayAutoCommand);
    m_autonomousChooser.addOption("Balance", m_balanceAutoCommand);
    m_autonomousChooser.addOption("Place and balance", m_placeAndBalanceAutoCommand);
    SmartDashboard.putData(m_autonomousChooser);

    configureButtonBindings();
    CommandScheduler.getInstance().setDefaultCommand(m_driveTrainSub, m_driveCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Set buttons.
    final JoystickButton zeroAngleButton = new JoystickButton(m_driveController, Constants.XBOX_Y_BUTTON);
    final JoystickButton clawIntakeButton = new JoystickButton(m_driveController, Constants.XBOX_A_BUTTON);
    final JoystickButton clawOutButton = new JoystickButton(m_driveController, Constants.XBOX_B_BUTTON);
    final JoystickButton lowGrabButton = new JoystickButton(m_driveController, Constants.XBOX_LEFT_BUMPER);
    final JoystickButton highGrabButton = new JoystickButton(m_driveController, Constants.XBOX_RIGHT_BUMPER);
    final JoystickButton restButton = new JoystickButton(m_driveController, Constants.XBOX_X_BUTTON);
    final JoystickButton zeroClawCommand = new JoystickButton(m_driveController, 7);

    final JoystickButton wristDownButton = new JoystickButton(m_driveController, Constants.XBOX_RIGHT_BUMPER);
    final JoystickButton wristUpButton = new JoystickButton(m_driveController, Constants.XBOX_LEFT_BUMPER);
    //final JoystickButton limeLightTestButton = new JoystickButton(m_driveController, Constants.XBOX_LEFT_BUMPER);
    //final JoystickButton limeLightToggleButton = new JoystickButton(m_driveController, Constants.XBOX_RIGHT_BUMPER);
    //final JoystickButton spinClawButton = new JoystickButton(m_driveController, Constants.XBOX_X_BUTTON);
    //final JoystickButton ClawSpinButton = new JoystickButton(m_driveController, Constants.XBOX_B_BUTTON);


    zeroAngleButton.whenPressed(m_zeroAngleCommand);
    lowGrabButton.onTrue(m_armGrabPositionCommand);
    highGrabButton.onTrue(m_ArmHighGrabPositionCommand);
    restButton.onTrue(m_armRestPositionCommand);
    //wristDownButton.whileTrue(m_ClawDownCommand);
    //wristUpButton.whileTrue(m_ClawUpCommand);
    //limeLightTestButton.whileTrue(m_AprilTagTestCommand);
    //limeLightToggleButton.onTrue(m_AprilTagToggleCommand);
    //spinClawButton.toggleOnTrue(m_spinClawCommand);
    zeroClawCommand.onTrue(m_zeroClawCommand);
    //clawIntakeButton.whileTrue(m_ClawIntakeCommand);
    //clawOutButton.whileTrue(m_clawOutCommand);

    //Button Box Bindings
    final JoystickButton toggleButton = new JoystickButton(m_ButtonBox, Constants.TOGGLE_BUTTON);

    final JoystickButton topLeftButton = new JoystickButton(m_ButtonBox, Constants.TOP_LEFT_BUTTON);
    final JoystickButton topCenterButton = new JoystickButton(m_ButtonBox, Constants.TOP_CENTER_BUTTON);
    final JoystickButton topRightButton = new JoystickButton(m_ButtonBox, Constants.TOP_RIGHT_BUTTON);
    final JoystickButton middleLeftButton = new JoystickButton(m_ButtonBox, Constants.MIDDLE_LEFT_BUTTON);
    final JoystickButton middleCenterButton = new JoystickButton(m_ButtonBox, Constants.MIDDLE_CENTER_BUTTON);
    final JoystickButton middleRightButton = new JoystickButton(m_ButtonBox, Constants.MIDDLE_RIGHT_BUTTON);
    final JoystickButton bottomLeftButton = new JoystickButton(m_ButtonBox, Constants.BOTTOM_LEFT_BUTTON);
    final JoystickButton bottomCenterButton = new JoystickButton(m_ButtonBox, Constants.BOTTOM_CENTER_BUTTON);
    final JoystickButton bottomRightButton = new JoystickButton(m_ButtonBox, Constants.BOTTOM_RIGHT_BUTTON);

    topRightButton.onTrue(m_armHigherPositionCommand);
    middleRightButton.onTrue(m_armMiddlePositionCommand);
    topCenterButton.onTrue(m_ArmHigherConePositionCommand);
    middleCenterButton.onTrue(m_ArmMiddleConeCommand);
    bottomLeftButton.onTrue(m_armGrabConeGroundCommand);

    // Evil haha.

    /*
    toggleButton.onTrue(m_TogglePipelineCommand);

    topLeftButton.onTrue(m_UpperLeftPlaceCommand);
    topCenterButton.onTrue(m_UpperMiddlePlaceCommand);
    topRightButton.onTrue(m_UpperRightPlaceCommand);
    middleLeftButton.onTrue(m_CenterLeftPlaceCommand);
    middleCenterButton.onTrue(m_CenterMiddlePlaceCommand);
    middleRightButton.onTrue(m_CenterRightPlaceCommand);

    // Changed for arm postion commands (---:
    bottomLeftButton.onTrue(m_armHigherPositionCommand);
    bottomCenterButton.onTrue(m_armMiddlePositionCommand);
    //bottomRightButton.onTrue(m_LowerRightPlaceCommand);
    */

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autonomousChooser.getSelected();
  }
}
