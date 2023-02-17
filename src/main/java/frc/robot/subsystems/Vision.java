// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelightvision.limelight.frc.LimeLight;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.DriverStation;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  private final LimeLight limeLightAprilTag;
  private int trackedID;

  private DriverStation.Alliance alliance;

  public Vision() {

    limeLightAprilTag = new LimeLight("AprilTag");

    int trackedID = 0;

  }

  public int getID(String limelight) {
    int AprilTagID;
    AprilTagID = (int)NetworkTableInstance.getDefault().getTable(limelight).getEntry("tid").getDouble(0.0);
    return AprilTagID;
  }

  public void setLeftPipeline(String limelight){
    NetworkTableInstance.getDefault().getTable(limelight).getEntry("pipeline").setNumber(1);
  }

  public void setCenterPipeline(String limelight){
    NetworkTableInstance.getDefault().getTable(limelight).getEntry("pipeline").setNumber(2);
  }

  public void setRightPipeline(String limelight){
    NetworkTableInstance.getDefault().getTable(limelight).getEntry("pipeline").setNumber(3);
  }

  public void initializeTrackingID(){
    if (trackedID == 0){
      if (alliance == DriverStation.Alliance.Red){
      trackedID = 3;
      }
      else if (alliance == DriverStation.Alliance.Blue){
        trackedID = 6;
      }
    }
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

  public double getAprilTagHorizontalOffset() {
    return limeLightAprilTag.getdegRotationToTarget();
  }

  public double getAprilTagVerticalOffset() {
    return limeLightAprilTag.getdegVerticalToTarget();
  }

  public boolean isTrackedAprilTagFound() {
    int mainID = getID( "AprilTag");
    if (mainID == trackedID){
      return true;
    }
    else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
