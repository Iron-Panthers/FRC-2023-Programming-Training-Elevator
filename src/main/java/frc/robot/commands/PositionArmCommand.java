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
  public double targetHeight;  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

public PositionArmCommand(ElevatorSubsystem elevatorSubsystem, double targetHeight){
      this.targetHeight = targetHeight;
      this.elevatorSubsystem = elevatorSubsystem;

      addRequirements(elevatorSubsystem);

}
    // Use addRequirements() here to declare subsystem dependencies.

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.setTargetHeight(targetHeight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevatorSubsystem.nearTargetHeight();
  }
}
