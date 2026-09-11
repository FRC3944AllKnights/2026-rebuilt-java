package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.robot.Constants.IntakeConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IntakeSubsystemTest {
    private static IntakeSubsystem intake;

    @BeforeAll
    static void initializeSimulation() {
        assertTrue(HAL.initialize(500, 0));
        intake = new IntakeSubsystem();
    }

    @BeforeEach
    void resetTarget() {
        intake.setDeployTarget(IntakeConstants.INTAKE_START_POSITION);
    }

    @Test
    void successiveJogsPersistWhenDefaultCommandHoldsPosition() {
        intake.jogPosition(Degrees.of(1));
        intake.holdDeployPosition();
        intake.jogPosition(Degrees.of(1));
        intake.holdDeployPosition();

        assertEquals(2.0, intake.getTargetPosition().in(Degrees), 1e-9);
    }

    @Test
    void negativeJogRetractsFromExistingTarget() {
        intake.setDeployTarget(Degrees.of(10));
        intake.jogPosition(Degrees.of(-1));
        intake.holdDeployPosition();

        assertEquals(9.0, intake.getTargetPosition().in(Degrees), 1e-9);
    }

    @Test
    void jogsCannotAccumulateTargetsBeyondEitherTravelLimit() {
        intake.jogPosition(Degrees.of(1000));
        assertEquals(IntakeConstants.INTAKE_DEPLOYED_POSITION.in(Degrees),
                intake.getTargetPosition().in(Degrees), 1e-9);

        intake.jogPosition(Degrees.of(-1000));
        assertEquals(0.0, intake.getTargetPosition().in(Degrees), 1e-9);
    }

    @Test
    void invalidDashboardStepsDoNotPoisonSubsequentTargets() {
        intake.jogPosition(Degrees.of(1));
        intake.jogPosition(Degrees.of(Double.NaN));
        intake.jogPosition(Degrees.of(Double.POSITIVE_INFINITY));
        intake.jogPosition(Degrees.of(Double.NEGATIVE_INFINITY));
        intake.jogPosition(Degrees.of(1));

        assertEquals(2.0, intake.getTargetPosition().in(Degrees), 1e-9);
    }

    @Test
    void positionButtonsReplaceJogTarget() {
        intake.jogPosition(Degrees.of(1));
        intake.setIntakePosition(false);
        assertEquals(IntakeConstants.INTAKE_DEPLOYED_POSITION.in(Degrees),
                intake.getTargetPosition().in(Degrees), 1e-9);

        intake.setIntakePosition(true);
        assertEquals(0.0, intake.getTargetPosition().in(Degrees), 1e-9);
    }
}
