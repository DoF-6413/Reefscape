package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Climber.Climber;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Periscope.Periscope;

 public class OrchestraCommands {
    
     public static Command startMusicForAllMotors(Drive drive, Climber climber, Periscope periscope) {
        return Commands.run (() -> {
            drive.startMusic();
            climber.startMusic();
            periscope.startMusic();
        },
        drive, climber, periscope
        );
     }

     public static Command stopMusicForAllMotors(Drive drive, Climber climber, Periscope periscope) {
        return Commands.run (() -> {
            drive.stopMusic();
            climber.stopMusic();
            periscope.stopMusic();
        },
        drive, climber, periscope
        );
     }
 }

