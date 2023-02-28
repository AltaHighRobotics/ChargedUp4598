// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import limelightvision.limelight.frc.LimeLight;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  private final LimeLight limeLight;
  public int trackedID;

  private DriverStation.Alliance alliance;

  public Vision() {

    limeLight = new LimeLight();

    trackedID = -1;
  }

  public int getID() {
    int AprilTagID;
    AprilTagID = (int)NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0.0);
    return AprilTagID;
  }


  public void initializeTrackingID(){
    alliance = DriverStation.getAlliance();

    if (trackedID == -1){
      if (alliance == DriverStation.Alliance.Red){
      trackedID = 3;
      }
      else if (alliance == DriverStation.Alliance.Blue){
        trackedID = 6;
      }
    }
    SmartDashboard.putString("Alliance", alliance.toString());
  }

  public void cycleTrackingID(){

    trackedID = trackedID+1;

    if (alliance == DriverStation.Alliance.Red){
      if (trackedID == 4){
        trackedID = 1;
      }
    }
    
    else if (alliance == DriverStation.Alliance.Blue){
      if (trackedID == 9){
        trackedID = 6;
      }
    }
  }

  public void setTrackingID(int number) {
    trackedID = number;
  }

  public double getHorizontalOffset() {
    return limeLight.getdegRotationToTarget();
  }

  public double getVerticalOffset() {
    return limeLight.getdegVerticalToTarget();
  }

  public boolean isTrackedAprilTagFound() {
    int mainID = getID();
    if (mainID == trackedID){
      return true;
    }
    else {
      return false;
    }
  }

  public boolean isTargetFound(){
    double haveTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    if (haveTarget == 1){
      return true;
    }
    else {
      return false;
    }

    }

  public void setLimelightPipeline(int pipeline){
    limeLight.setPipeline(pipeline);
  }

  public void togglePipeline(){
    double currentPipeline = NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").getDouble(-1);

    if (currentPipeline == Constants.LIMELIGHT_APRIL_TAG_PIPELINE){
      setLimelightPipeline(Constants.LIMELIGHT_REFLECTIVE_TAPE_PIPELINE);
    }
    else if(currentPipeline == Constants.LIMELIGHT_REFLECTIVE_TAPE_PIPELINE){
      setLimelightPipeline(Constants.LIMELIGHT_APRIL_TAG_PIPELINE);
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
