// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Cmd_Set_State extends Command {
  /** Creates a new Cmd_Set_State. */
  private final Sub_Swerve Swerve;
  private final double speed_y, speed_x,speed_giro;
  public Cmd_Set_State(Sub_Swerve Swerve, double speed_y,double speed_x, double speed_giro ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Swerve=Swerve;
    this.speed_y=speed_y;
    this.speed_x=speed_x;
    this.speed_giro=speed_giro;
    addRequirements(Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds Speeds;
    Speeds= new ChassisSpeeds(speed_y,speed_x,speed_giro);
    SwerveModuleState[] moduleStates = frc.robot.Constants.Swerve.swervekinematics.toSwerveModuleStates(Speeds);
    Swerve.setModuleStates(moduleStates);
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
