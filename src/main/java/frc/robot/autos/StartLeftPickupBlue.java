package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Swerve;

public class StartLeftPickupBlue extends DriveSegment {
    public StartLeftPickupBlue(Swerve swerve){
            super(swerve, 
            List.of(WAYPOINT_START1.getTranslation(), WAYPOINT_RIGHT_OF_CS.getTranslation(),WAYPOINT_GP1.getTranslation()),
                WAYPOINT_START1.getRotation(),
                WAYPOINT_GP1.getRotation(),
                false,
                true);
       
      }
      protected static final Pose2d WAYPOINT_RIGHT_OF_CS = new Pose2d(2.0, 1.55, Rotation2d.fromDegrees(0));
}