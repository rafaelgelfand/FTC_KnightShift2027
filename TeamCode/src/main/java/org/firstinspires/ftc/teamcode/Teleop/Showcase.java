package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Showcase")
public class Showcase extends LinearOpMode {

    // Vision Sensor & Scoring Servo
    private HuskyLens huskylens;
    private Servo shootServo;

    // Drivetrain Motors
    private DcMotor LB;
    private DcMotor LF;
    private DcMotor RB;
    private DcMotor RF;

    // Intake & Shooter Motors
    private DcMotor IR;
    private DcMotor IL;
    private DcMotor SR;
    private DcMotor SL;

    // Drive power values
    double lbPower;
    double lfPower;
    double rbPower;
    double rfPower;

    // Aim-assist tuning constants
    private final int TARGET_TAG_ID = 1;     // AprilTag ID to lock onto
    private final int CENTER_X_PIXEL = 160;  // Center X frame coordinate (320 / 2)
    private final int PIXEL_THRESHOLD = 5;   // Error margin for centering
    private final double AUTO_TURN_SPEED = 0.15; // Speed used during auto-aim assist / search

    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();

        // Hardware mapping for Vision & Servos
        huskylens = hardwareMap.get(HuskyLens.class, "huskylens");
        shootServo = hardwareMap.get(Servo.class, "shoot_servo");

        // Hardware mapping for Drivetrain
        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");

        // Hardware mapping for Intake and Shooter
        IR = hardwareMap.get(DcMotor.class, "IR");
        IL = hardwareMap.get(DcMotor.class, "IL");
        SR = hardwareMap.get(DcMotor.class, "SR");
        SL = hardwareMap.get(DcMotor.class, "SL");

        // Motor directions (Drivetrain)
        LB.setDirection(DcMotor.Direction.FORWARD);
        LF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.REVERSE);

        // Motor directions (Shooter & Intake opposition mapping)
        SR.setDirection(DcMotor.Direction.FORWARD);
        SL.setDirection(DcMotor.Direction.REVERSE);
        IR.setDirection(DcMotor.Direction.FORWARD);
        IL.setDirection(DcMotor.Direction.REVERSE);

        // Brake motors behavior
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Drive encoders setup
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize HuskyLens algorithm for AprilTags
        if (!huskylens.initialize()) {
            telemetry.addData("Error", "HuskyLens failed to initialize!");
        }
        huskylens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        shootServo.setPosition(0.0); // Home position

        telemetry.addData("Status", "Initialized & Ready");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // Mecanum Drive Controls (Gamepad 1)
            double forward = -gamepad1.left_stick_y;
            double strafe  =  gamepad1.left_stick_x;
            double turn    =  gamepad1.right_stick_x;

            // --- Auto-Aim & Automatic Search Assist ---
            // Holding Gamepad 1 B enables tracking. If the tag is visible it centers
            // If the tag is out of view, it automatically spins to find it
            if (gamepad1.b) {
                HuskyLens.Block[] blocks = huskylens.blocks();
                boolean targetFound = false;
                int targetX = 0;

                for (HuskyLens.Block block : blocks) {
                    if (block.id == TARGET_TAG_ID) {
                        targetFound = true;
                        targetX = block.x;
                        break;
                    }
                }

                if (targetFound) {
                    int error = targetX - CENTER_X_PIXEL;
                    if (Math.abs(error) > PIXEL_THRESHOLD) {
                        turn = (error > 0) ? AUTO_TURN_SPEED : -AUTO_TURN_SPEED;
                    } else {
                        turn = 0; // Perfectly centered on target tag
                        telemetry.addData("Aim Assist", "LOCKED ON TARGET!");
                    }
                } else {
                    // Tag is out of view: automatically spin to scan for it
                    turn = AUTO_TURN_SPEED; 
                    telemetry.addData("Aim Assist", "Searching for Tag ID %d...", TARGET_TAG_ID);
                }
            }

            lbPower = forward - strafe + turn;
            lfPower = forward + strafe + turn;
            rbPower = forward + strafe - turn;
            rfPower = forward - strafe - turn;

            double max = Math.max(
                    Math.max(Math.abs(lfPower), Math.abs(rfPower)),
                    Math.max(Math.abs(lbPower), Math.abs(rbPower))
            );

            if (max > 1.0) {
                lbPower /= max;
                lfPower /= max;
                rbPower /= max;
                rfPower /= max;
            }

            LB.setPower(lbPower);
            LF.setPower(lfPower);
            RB.setPower(rbPower);
            RF.setPower(rfPower);


            // Intake Controls (Gamepad 2)
            if (gamepad2.left_bumper) {
                IR.setPower(0.8); // Run intake inward
                IL.setPower(0.8);
            } else if (gamepad2.left_trigger > 0.1) {
                IR.setPower(-0.8); // Reverse intake outward
                IL.setPower(-0.8);
            } else {
                IR.setPower(0.0);
                IL.setPower(0.0);
            }

            // Shooter & Indexer Controls (Gamepad 2)
            if (gamepad2.right_bumper) {
                SR.setPower(0.85); 
                SL.setPower(0.85);
            } else {
                SR.setPower(0.0);
                SL.setPower(0.0);
            }

            if (gamepad2.a) {
                shootServo.setPosition(0.8); // Push forward
            } else {
                shootServo.setPosition(0.0); // Retract home position
            }

            telemetry.addData("Run Time", runtime.toString());
            telemetry.update();
        }
    }
}
