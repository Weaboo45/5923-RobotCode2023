/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class ArmConstants {
        public static int topSegMotorID = SpeedControllerCanPortConstants.driveTopSegMotor;
        public static int bottomSegMotorID = SpeedControllerCanPortConstants.driveBottomSegMotor;

        public static double rotationDeadband = 0;
    }

    public static class MecanumDrivetrianConstants {
        //Mecanum ID names
        public static int leftFrontVictorID = SpeedControllerCanPortConstants.driveFrontLeftVictor;
        public static int leftRearVictorID = SpeedControllerCanPortConstants.driveRearLeftVictor;
        
        public static int rightFrontTalonID = SpeedControllerCanPortConstants.driveFrontRightTalon;
        public static int rightRearTalonID = SpeedControllerCanPortConstants.driveRearRightTalon;

        public static double rotationDeadband = .25;
        public static double strafingDeadband = .25;  //.75
        public static double speedDeadband = .3;
        public static double m_maxOutput = .5;
        public static boolean isTankDriveSquared = false;
        public static boolean doesCheesyDrivePivot = false;
        public static boolean isArcadeDriveSquared = true;

    }

    public static class SpeedControllerCanPortConstants {
        //mecanum drive motor ID ports
        //left side
        private static int driveFrontLeftVictor = 9;
        private static int driveRearLeftVictor = 3;

        //right side
        private static int driveFrontRightTalon = 7;
        private static int driveRearRightTalon = 1;

        //intake arm motors
        private static int driveTopSegMotor = 5;
        private static int driveBottomSegMotor = 1;
    }

    public static final int CONTROLLER_PORT = 1;

    public static final int LEFT_STICK_X = 0;
    public static final int LEFT_STICK_Y = 1;
    public static final int RIGHT_STICK_X = 4;
    public static final int RIGHT_STICK_Y = 5;
}
