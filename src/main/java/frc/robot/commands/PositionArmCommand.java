// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** An example command that uses an example subsystem. */
public class PositionArmCommand extends CommandBase {
  private final ElevatorSubsystem elevatorSubsystem;

  private Trigger aTrigger;
  private Trigger bTrigger;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PositionArmCommand(ElevatorSubsystem elevatorSubsystem, Trigger aTrigger, Trigger bTrigger) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.aTrigger = aTrigger;
    this.bTrigger = bTrigger;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(aTrigger.getAsBoolean()){
      elevatorSubsystem.setTargetHeight(20d);
    }else if(bTrigger.getAsBoolean()){
      elevatorSubsystem.setTargetHeight(10d);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
