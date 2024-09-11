package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "teleop")
public class teleop extends OpMode {

    Motor rightWheel1, rightWheel2, leftWheel1, leftWheel2;

    Motor armLift1;

    Motor extendArm;

    PIDFController liftPID;

    int targetLiftPosition;

    double targetPosition;

    @Override
    public void init() {

        rightWheel1 = new CRServo(hardwareMap, "front-right");
        rightWheel1.setRunMode(Motor.RunMode.VelocityControl);
        rightWheel1.setVeloCoefficients(0.05, 0,0);

        rightWheel2 = new Motor(hardwareMap, "back-right");
        rightWheel2.setRunMode(Motor.RunMode.VelocityControl);
        rightWheel2.setVeloCoefficients(0.05, 0,0);

        leftWheel1 = new CRServo(hardwareMap, "front-left");
        leftWheel1.setRunMode(Motor.RunMode.VelocityControl);
        leftWheel1.setVeloCoefficients(0.05, 0,0);

        leftWheel2 = new Motor(hardwareMap, "back-left");
        leftWheel2.setRunMode(Motor.RunMode.VelocityControl);
        leftWheel2.setVeloCoefficients(0.05, 0,0);


      //  armLift1 = hardwareMap.get(Motor.class, "erect-arm");

        // extend
      // extendArm = hardwareMap.get(Motor.class, "elbow-arm");
      // extendArm.setRunMode(Motor.RunMode.PositionControl);
      // extendArm.setPositionCoefficient(0.05);


        // lift PIDF values
        double kP_lift = 0.05;
        double kI_lift = 0;
        double kD_lift = 0;
        double kF_lift = 0.01;
       // liftPID = new PIDFController(kP_lift, kI_lift, kD_lift, kF_lift);


        // Set initial target positions
       // targetLiftPosition = armLift1.getCurrentPosition();
       //
        targetPosition = 0;


    }


    @Override
    public void loop() {


        double axial   = gamepad1.left_stick_x;  //Note: pushing stick forward gives negative value
        double lateral =  gamepad1.right_stick_y;
        double yaw     =  gamepad1.left_stick_y;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // set velocity for each wheel
        rightWheel1.set(rightFrontPower);
        rightWheel2.set(rightBackPower);
        leftWheel1.set(leftFrontPower);
        leftWheel2.set(leftBackPower);


        // Calculate PID output
       // double currentLiftPosition = armLift1.getCurrentPosition();


        // Arm lift
     //  if (gamepad2.dpad_up) {
     //      targetLiftPosition += 1;
     //  } else if (gamepad2.dpad_down) {
     //      targetLiftPosition -= 1;
     //  }

       // double liftPower = liftPID.calculate(currentLiftPosition, targetLiftPosition);
       // armLift1.set(liftPower);


        // arm extend
        targetPosition -= gamepad2.right_stick_y;

       // extendArm.setTargetPosition((int) targetPosition);
       // extendArm.set(0.75);
    }


}
