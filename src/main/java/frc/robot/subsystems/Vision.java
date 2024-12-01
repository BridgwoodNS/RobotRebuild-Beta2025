// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private final CommandSwerveDrivetrain m_swerve;
  boolean useMegaTag2 = true; //set to false to use MegaTag1
  boolean doRejectUpdate = false;
  public Vision( CommandSwerveDrivetrain m_swerve) {
    this.m_swerve = m_swerve;
  }
    


  public void updateOdometry() {
    // Update odometry with new data from the encoders and the gyro
   
    if(!useMegaTag2)
    {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        m_swerve.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      
      }
    }
    else if (useMegaTag2 == true)
    {
      LimelightHelpers.SetRobotOrientation("limelight", m_swerve.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");


     // if(Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates

     //Not sure if my math checks here -  720 degrees per second is 12.5 radians per second
     if(Math.abs(m_swerve.getState().Speeds.omegaRadiansPerSecond ) > 12.5) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {

        m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_swerve.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
       
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}

