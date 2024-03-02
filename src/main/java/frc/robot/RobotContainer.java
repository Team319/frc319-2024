// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;

import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
//import frc.robot.subsystems.drive.TankIO;
//import frc.robot.subsystems.drive.TankIOReal;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.collector.CollectorIO;
import frc.robot.subsystems.collector.CollectorIOReal;
import frc.robot.subsystems.collector.CollectorIOSim;

//import java.util.Collection;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  public final Shooter shooter;
  public final Collector collector;
  public final Elevator elevator;
  // private final Flywheel flywheel;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
 private final LoggedDashboardChooser<Command> autoChooser;
 //private final LoggedDashboardNumber flywheelSpeedInput =
  //    new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
          new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(0),
            new ModuleIOTalonFX(1),
            new ModuleIOTalonFX(2),
            new ModuleIOTalonFX(3));
        
        shooter =
          new Shooter(
            new ShooterIOReal() 
          );

        collector =
          new Collector(
            new CollectorIOReal()
          );

        elevator =
          new Elevator(
            new ElevatorIOReal()
        );
          
        break;
      
        case BUSTER:
          drive=
            new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(0),
            new ModuleIOTalonFX(1),
            new ModuleIOTalonFX(2),
            new ModuleIOTalonFX(3));
        
        shooter =
          new Shooter(
            new ShooterIO() {}
          );

        collector =
          new Collector(
            new CollectorIO() {}
          );

        elevator =
          new Elevator(
            new ElevatorIO() {}
        );
        break;
        case TANK:
          drive = 
            new Drive(new GyroIO() {});
          
          shooter =
            new Shooter(
              new ShooterIO() {}
            );

          collector =
            new Collector(
              new CollectorIO() {}
            );

          elevator =
            new Elevator(
              new ElevatorIO() {}
            );
            
          break;
      
        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

          shooter =
            new Shooter(
              new ShooterIOSim()
            );
          
          collector =
            new Collector(
              new CollectorIOSim()
              );

          elevator =
            new Elevator(
              new ElevatorIO() {}
            );

          break;
        
      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // flywheel = new Flywheel(new FlywheelIO() {});

      shooter =
          new Shooter(new ShooterIO() {
            
          });

      collector = 
          new Collector(
            new CollectorIO() {}
          );

          elevator =
            new Elevator(
              new ElevatorIO() {}
            );

        break;
    }

    // Set up named commands for PathPlanner
    // NamedCommands.registerCommand(
    //    "exampleCommand",
    //     exampleSubsystem.exampleCommand());

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up FF characterization routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    switch (Constants.currentMode) {
      case REAL:
      case SIM:
      case REPLAY:

        drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> -driverController.getLeftY(),
              () -> -driverController.getLeftX(),
              () -> -driverController.getRightX(),
              () -> driverController.getLeftTriggerAxis()));
      
      
        driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
        
    /*    driverController
            .b()
            .onTrue(
                Commands.runOnce(
                        () ->
                            drive.setPose(
                                new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                        drive)
                    .ignoringDisable(true)); */

         driverController.y().onTrue(Commands.runOnce(
          () -> {
            shooter.setFeedPO(0.2);

            }
          )
        ); 

         driverController.y().onFalse(Commands.runOnce(
          () -> {
            shooter.setFeedPO(0.0);
            }
          )
        );
      

        // ============================= Collector Debugging ============================= 

        operatorController.rightBumper().onTrue(Commands.runOnce(
          () -> {
            collector.setRollersPO(1.0);
            }
          )
        );

        operatorController.rightBumper().onFalse(Commands.runOnce(
          () -> {
            collector.setRollersPO(0);
            }
          )
        );

        operatorController.leftBumper().onTrue(Commands.runOnce(
          () -> {
            collector.setRollersPO(-1.0);
            }
          )
        );

        operatorController.leftBumper().onFalse(Commands.runOnce(
          () -> {
            collector.setRollersPO(0);
            }
          )
        );

        /*  ============================= Elevator Debugging ============================= */

        operatorController.povUp().whileFalse(Commands.runOnce(
          () -> {
            elevator.setPO(0.0);
            }
          )
        );

        operatorController.povUp().whileTrue(Commands.runOnce(
          () -> {
            elevator.setPO(1.0);
            }
          )
        );

        operatorController.povDown().whileFalse(Commands.runOnce(
          () -> {
            elevator.setPO(0.0);
            }
          )
        );

        operatorController.povDown().whileTrue(Commands.runOnce(
          () -> {
            elevator.setPO(-1.0);
            }
          )
        );

        // ============================= Wrist Debugging ============================= 

        // Hot keys, motors are not happy tho :(
          /*      operatorController.a().whileTrue(Commands.runOnce(
          () -> {
            shooter.setWristPosition(0.0);
            }
          )
        );

        operatorController.y().whileTrue(Commands.run(
          () -> {
            shooter.setWristPosition(-24.8);
            }
          )  */
        
        operatorController.a().whileFalse(Commands.runOnce(
          () -> {
            shooter.setWristPO(0.0);
            }
          )
        );

        operatorController.a().whileTrue(Commands.run(
          () -> {
            shooter.setWristPO(0.1);
            }
          )
        );

        operatorController.y().whileFalse(Commands.runOnce(
          () -> {
            shooter.setWristPO(0.0);
            }
          )
        );

        operatorController.y().whileTrue(Commands.run(
          () -> {
            shooter.setWristPO(-0.1);
            }
          )
        );
        /*  ============================= Shooter Debugging ============================= */

         driverController.rightTrigger().whileTrue(Commands.run(
          ()-> {
            shooter.setVoltages(12, 12, 3);
          }
        )
        );

        driverController.rightTrigger().whileFalse(Commands.runOnce(
          ()-> {
            shooter.stop();
          }
        )
        ); 


        
        break;
    
      default:
      /* Do nothing */
        break;
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
