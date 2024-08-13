package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "teleop")
public class teleop extends OpMode {

    DcMotorEx rightWheel1, rightWheel2, leftWheel1, leftWheel2;

    DcMotor armLift1;
    DcMotor armLift2;

    DcMotor extendArm;
    boolean extended;

    double targetVelocityX;
    double targetVelocityY;

    double powerY;
    double powerX;

    PIDFController velocityPID;
    double kP, kI, kD, kF;

    double maxVelocity;

    PIDController liftPID;
    ArmFeedforward liftFeedForward;
    PIDController extendPID;

    double targetExtendPosition = 0;
    int targetLiftPosition;


    @Override
    public void init() {
        rightWheel1 = hardwareMap.get(DcMotorEx.class, "rightWheel1");
        rightWheel2 = hardwareMap.get(DcMotorEx.class, "rightWheel2");
        leftWheel1 = hardwareMap.get(DcMotorEx.class, "leftWheel1");
        leftWheel2 = hardwareMap.get(DcMotorEx.class, "leftWheel2");

        rightWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftWheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armLift1 = hardwareMap.get(DcMotor.class, "armLift1");
        armLift2 = hardwareMap.get(DcMotor.class, "armLift2");

        extendArm = hardwareMap.get(DcMotor.class, "extendArm");
        extended = false;


        // gain coefficients
        kP = 0.1;
        kI = 0.0;
        kD = 0.01;
        kF = 0.0005;
        velocityPID = new PIDFController(kP, kI, kD, kF);

        // assign max Velocity

        double kP_lift = 0.1;
        double kI_lift = 0.0;
        double kD_lift = 0.01;
        liftPID = new PIDController(kP_lift, kI_lift, kD_lift);

        // Initialize FeedForward controller
        double kS_lift = 0.2;  // Static gain
        double kV_lift = 0.1;  // Velocity gain
        double kA_lift = 0.05; // Acceleration gain
        liftFeedForward = new ArmFeedforward(kS_lift, kV_lift, kA_lift);

        // Set initial target positions
        targetLiftPosition = armLift1.getCurrentPosition();

        double kP_extend = 0.1;
        double kI_extend = 0.0;
        double kD_extend = 0.01;
        extendPID = new PIDController(kP_extend, kI_extend, kD_extend);

        // Set initial target positions

        targetExtendPosition = 0;


    }


    @Override
    public void loop() {

        targetVelocityY = gamepad1.left_stick_y * maxVelocity;
        targetVelocityX = gamepad1.left_stick_x * maxVelocity;

        powerY = velocityPID.calculate(rightWheel1.getVelocity(), targetVelocityY);
        powerX = velocityPID.calculate(leftWheel1.getVelocity(), targetVelocityX);

        // rotate right and left
        if (gamepad1.right_stick_x > 0) {
            rightWheel1.setPower(-gamepad1.right_stick_x); // - change to PID
            rightWheel2.setPower(-gamepad1.right_stick_x); // -
            leftWheel1.setPower(gamepad1.right_stick_x); // +
            leftWheel2.setPower(gamepad1.right_stick_x); // +
        }
        if (gamepad1.right_stick_x < 0) {
            rightWheel1.setPower(-gamepad1.right_stick_x); // + change to PID
            rightWheel2.setPower(-gamepad1.right_stick_x); // +
            leftWheel1.setPower(gamepad1.right_stick_x); // -
            leftWheel2.setPower(gamepad1.right_stick_x); // -
        }


        // Combine the power outputs based on the direction
        if (targetVelocityY != 0 || targetVelocityX != 0) {
            // Forward/Backward movement
            if (targetVelocityY > 0) { // Forward
                rightWheel1.setVelocity(powerY);
                rightWheel2.setVelocity(powerY);
                leftWheel1.setVelocity(powerY);
                leftWheel2.setVelocity(powerY);
            } else if (targetVelocityY < 0) { // Backward
                rightWheel1.setVelocity(-powerY);
                rightWheel2.setVelocity(-powerY);
                leftWheel1.setVelocity(-powerY);
                leftWheel2.setVelocity(-powerY);
            }

            // Strafing movement
            if (targetVelocityX > 0) { // Strafe Right
                rightWheel1.setVelocity(powerX);
                rightWheel2.setVelocity(-powerX);
                leftWheel1.setVelocity(-powerX);
                leftWheel2.setVelocity(powerX);
            } else if (targetVelocityX < 0) { // Strafe Left
                rightWheel1.setVelocity(-powerX);
                rightWheel2.setVelocity(powerX);
                leftWheel1.setVelocity(powerX);
                leftWheel2.setVelocity(-powerX);
            }
        }

        // Handle diagonal movement
        if (targetVelocityY != 0 && targetVelocityX != 0) {
            // Adjust the power to combine forward/backward and strafing movements
            double combinedPowerY = velocityPID.calculate(rightWheel1.getVelocity(), targetVelocityY);
            double combinedPowerX = velocityPID.calculate(leftWheel1.getVelocity(), targetVelocityX);

            if (targetVelocityY > 0 && targetVelocityX > 0) { // Top right
                rightWheel1.setVelocity(combinedPowerY);
                rightWheel2.setVelocity(combinedPowerY);
                leftWheel1.setVelocity(combinedPowerY);
                leftWheel2.setVelocity(-combinedPowerX);
            } else if (targetVelocityY > 0 && targetVelocityX < 0) { // Top left
                rightWheel1.setVelocity(-combinedPowerX);
                rightWheel2.setVelocity(combinedPowerX);
                leftWheel1.setVelocity(combinedPowerY);
                leftWheel2.setVelocity(-combinedPowerY);
            } else if (targetVelocityY < 0 && targetVelocityX < 0) { // Bottom left
                rightWheel1.setVelocity(-combinedPowerY);
                rightWheel2.setVelocity(-combinedPowerY);
                leftWheel1.setVelocity(-combinedPowerY);
                leftWheel2.setVelocity(combinedPowerX);
            } else if (targetVelocityY < 0 && targetVelocityX > 0) { // Bottom right
                rightWheel1.setVelocity(combinedPowerX);
                rightWheel2.setVelocity(-combinedPowerX);
                leftWheel1.setVelocity(-combinedPowerY);
                leftWheel2.setVelocity(combinedPowerY);
            }
        }

        // Arm lift
        if (gamepad2.dpad_up) {
            targetLiftPosition = 1; // Example target position when moving up
        } else if (gamepad2.dpad_down) {
            targetLiftPosition = -1; // Example target position when moving down
        } else {
            targetLiftPosition = armLift1.getCurrentPosition(); // Maintain current position
        }

        // Calculate PID output
        double currentLiftPosition = armLift1.getCurrentPosition();
        double liftPIDOutput = liftPID.calculate(currentLiftPosition, targetLiftPosition);

        // Calculate FeedForward term
        double desiredVelocity = calculateDesiredVelocity(); // Example calculation for velocity
        double desiredAcceleration = calculateDesiredAcceleration();
        double liftFeedForwardOutput = liftFeedForward.calculate(desiredVelocity, desiredAcceleration);

        // Combine PID output and FeedForward term
        double liftPower = liftPIDOutput + liftFeedForwardOutput;


        armLift1.setPower(liftPower);
        armLift2.setPower(liftPower);

        // Arm extend ADD PID
        if (gamepad2.a){
            if(!extended) {
                extendArm.setPower(0.5);
                extended = !extended; // switch to extended
            }else{
                extendArm.setPower(-0.5);
                extended = !extended; // switch to not extended
            }
        }

    }

    // Method to estimate or calculate desired velocity
    private double calculateDesiredVelocity() {
        // Example calculation or estimation
        return (targetLiftPosition - armLift1.getCurrentPosition()) * 5; // velocity factor (change)
    }

    // Method to estimate or calculate desired acceleration
    private double calculateDesiredAcceleration() {
        // Example calculation or estimation
        return calculateDesiredVelocity() * 5; // accelerationFactor;
    }
}
