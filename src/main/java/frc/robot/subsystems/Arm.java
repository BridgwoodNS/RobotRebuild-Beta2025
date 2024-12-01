// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private final TalonFX armMotor;

  
  // PID coefficients
  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
//  private static final double kV = 0.0;

PositionDutyCycle postionController= new PositionDutyCycle(0);

  /** Creates a new Arm. */
  public Arm(int motorID) {
    armMotor = new TalonFX(motorID);
    
    // enable stator current limit
    var limitConfigs = new CurrentLimitsConfigs();

    limitConfigs.StatorCurrentLimit = 50;
    limitConfigs.StatorCurrentLimitEnable = true;


   // Configure the motor controller with PID coefficients
   // robot init, set slot 0 gains
    var slot0Configs = new Slot0Configs();
    // slot0Configs.kV = kV;
    slot0Configs.kP = kP;
    slot0Configs.kI = kI;
    slot0Configs.kD = kD;
    //slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    //slot0Configs.kG = 0.0;

    var hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
    //hardwareLimitSwitchConfigs.ForwardLimitRemoteSensorID = 0;
    hardwareLimitSwitchConfigs.ForwardLimitAutosetPositionEnable = true;
    hardwareLimitSwitchConfigs.ForwardLimitAutosetPositionValue = 0;

    armMotor.getConfigurator().apply(slot0Configs);
    armMotor.getConfigurator().apply(limitConfigs);
    postionController.withSlot(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //different feedforward types for lowering or raising. Should do different slots tbh
  public void setPosition(double position) {
    armMotor.setControl(postionController.withPosition(position).withFeedForward(0));
    if (position == 0) {
      armMotor.setControl(postionController.withPosition(0).withFeedForward(0));
    }
  }

  public double getCurrentPosition() {
    return armMotor.getRotorPosition().getValueAsDouble();
  }
  public void zeroSensor() {
    armMotor.setPosition(0);
  }


  Command setPositionCommand(double position) {
    return new InstantCommand(() -> armMotor.setPosition(position));
  }

  Command resetPositionCommand() {
    return new InstantCommand(() -> armMotor.setPosition(0));
  }

  Command zeroSensorCommand() {
    return new InstantCommand(() -> armMotor.setPosition(0));
  }

}





//ArmCommandFactory

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import frc.robot.subsystems.Arm;

// public class ArmCommandFactory {
//   private final Arm arm;

//   public ArmCommandFactory(Arm arm) {
//     this.arm = arm;
//   }

//   public Command setPositionCommand(double position) {
//     return new InstantCommand(() -> arm.setPosition(position), arm);
//   }

//   public Command holdPositionCommand() {
//     return new RunCommand(() -> arm.setPosition(arm.getCurrentPosition()), arm);
//   }
// }


// The ArmCommandFactory class is created with a constructor that takes an Arm subsystem instance.
// The setPositionCommand method returns an InstantCommand that sets the arm position to the specified value.
// The holdPositionCommand method returns a RunCommand that continuously sets the arm position to its current position, effectively holding it in place.
// You can use this command factory in your robot code to create commands for the Arm subsystem. For example:

// Arm arm = new Arm(1); // Assuming the motor ID is 1
// ArmCommandFactory armCommandFactory = new ArmCommandFactory(arm);

// // Create a command to set the arm position to 1000 encoder units
// Command setPositionCommand = armCommandFactory.setPositionCommand(1000);

// // Create a command to hold the arm position
// Command holdPositionCommand = armCommandFactory.holdPositionCommand();