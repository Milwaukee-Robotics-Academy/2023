package frc.robot.autos;

import java.util.List;

import frc.robot.subsystems.SwerveBase;

public class startLeftPickup1 extends DriveSegment {
    public startLeftPickup1(SwerveBase swerve){
        super(swerve, 
        List.of(WAYPOINT_START1.getTranslation(), WAYPOINT_GP1.getTranslation()),
            WAYPOINT_START1.getRotation(),
            WAYPOINT_GP1.getRotation(),
            false,
            true);
      }
}