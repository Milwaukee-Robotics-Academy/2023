package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Swerve;

public class LongSide  extends DriveSegment {
        public LongSide(Swerve swerve){
            super(swerve, 
            List.of(START3.getTranslation(), END3.getTranslation()),
                START3.getRotation(),
                END3.getRotation(),
                true,
                true);
          }
          protected static final Pose2d START3 = new Pose2d(0, 2.75, Rotation2d.fromDegrees(0));
          protected static final Pose2d END3 = new Pose2d(-4.65, 2.75, Rotation2d.fromDegrees(0));
    }    
