package frc.robot.subsystems.pose;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.intake.Intake;

class PoseCommands {

    static Command moveToStartingConfigPose() {
        return new SequentialCommandGroup(
                Commands.parallel(
                        Subsystems.intake.moveToStateCmd(Intake.IntakeState.StartingPosition),
                        Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.StartingPosition)
                ));
    }

    static Command moveToPickupPose() {
        return new ParallelCommandGroup(
                Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.FeedPosition),
                Subsystems.intake.moveToStateCmd(Intake.IntakeState.IntakeFromFloor),
                Commands.runOnce(Subsystems.shooter::stopFeeder)
        );
    }

    static Command feedNoteToShooterPose() {
        return new SequentialCommandGroup(
                Commands.parallel(
                        Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.FeedPosition),
                        Subsystems.intake.moveToStateCmd(Intake.IntakeState.FeedNote)
                ),
                new FeedNoteToShooterCommand().withTimeout(2.0),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.ReadyToShoot)
        );
    }

    static Command readyToShootPose() {
        return Commands.parallel(
                Commands.runOnce(Subsystems.pivot::moveToQueuedProfile),
                Commands.runOnce(Subsystems.shooter::stopFeeder),
                Subsystems.intake.moveToStateCmd(Intake.IntakeState.HoldNote)
        );
    }

    static Command moveToDrivePose() {
        return new ConditionalCommand(
                new ParallelCommandGroup(
                        Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.FeedPosition),
                        Subsystems.intake.moveToStateCmd(Intake.IntakeState.HoldNote)
                ),
                new PrintCommand("Ignoring drive request since we in transition with note"),
                () -> !(Intake.IntakeState.RotateUpWhileFeedingNote.equals(Subsystems.intake.getIntakeState())));

    }


    public static Command moveToNotePickedUpPose() {
        return new ParallelCommandGroup(
                Subsystems.intake.moveToStateCmd(Intake.IntakeState.HoldNote)
        );
    }

    public static Command positionForAmpPose() {
        return new SequentialCommandGroup(
                Subsystems.intake.moveToStateCmd(Intake.IntakeState.AmpAim)
        );
    }

    public static Command shooterAimVisionPose() {
        return Commands.parallel(
                Commands.runOnce(Subsystems.shooter::runShooter),
                Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.VisionAim)
        );
    }

    public static Command shortShotPose() {
        return Commands.parallel(
                Commands.runOnce(Subsystems.shooter::runShooter),
                Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.ShortShot)
        );
    }

    public static Command startClimbPose() {
        return Commands.parallel(
                Commands.runOnce(Subsystems.shooter::stopShooter),
                Commands.runOnce(Subsystems.shooter::stopFeeder),
                Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.Up),
                Subsystems.intake.moveToStateCmd(Intake.IntakeState.Climb)
        );
    }

    public static Command prepareBloopShotPose() {
        return Commands.parallel(
                Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.StartingPosition),
                Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(Subsystems.shooter.BloopProfile))
        );
    }
}
