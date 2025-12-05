// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Grabber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralIntake extends Command {
  Grabber s_Grabber;
  /** Creates a new CoralIntake. */
  public CoralIntake(Grabber s_Grabber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Grabber = s_Grabber;
    addRequirements(s_Grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Grabber.intakeCorral(s_Grabber.corralIntakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Grabber.intakeCorral(s_Grabber.corralIntakeSpeed);
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
