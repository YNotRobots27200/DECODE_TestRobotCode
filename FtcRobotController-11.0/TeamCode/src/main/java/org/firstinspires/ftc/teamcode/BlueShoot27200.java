/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="BlueShoot Auto 27200", group="Iterative OpMode")

public class BlueShoot27200 extends OpMode
{
    private ElapsedTime timer = new ElapsedTime();
    private DcMotor flywheelOutRight = null;
    private DcMotor flywheelOutLeft = null;
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    private Servo leftGateServo = null;
    private Servo rightGateServo = null;
    private DcMotor intakeRight = null;
    private DcMotor intakeLeft = null;
    private boolean isFlyWheel = false;
    private  boolean isServoOpen = false;
    private  boolean aWasPressed = false;
    private  boolean bWasPressed = false;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "frontLeftMotor");

        backLeftMotor   = hardwareMap.get(DcMotor.class, "backLeftMotor");

        frontRightMotor  = hardwareMap.get(DcMotor.class, "frontRightMotor");

        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        leftGateServo = hardwareMap.get(Servo.class,"leftGateServo");

        rightGateServo = hardwareMap.get(Servo.class,"rightGateServo");

        flywheelOutRight = hardwareMap.get(DcMotor.class, "flywheelOutRight") ;
        flywheelOutLeft = hardwareMap.get(DcMotor.class, "flywheelOutLeft") ;

        intakeRight = (DcMotorEx) hardwareMap.get(DcMotor.class, "intakeRight") ;
        intakeLeft = (DcMotorEx) hardwareMap.get(DcMotor.class, "intakeLeft") ;
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelOutLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelOutRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        timer.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop()
    {

        timer.reset();
        while ((timer.seconds() < 1)) {
            MoveRobot(0,-0.37,0);
        }
        MoveRobot(0,0,0);

        flywheelOutRight.setPower(-0.5);
        flywheelOutLeft.setPower(0.5);
        intakeRight.setPower(3);
        intakeLeft.setPower(3);


        timer.reset();
        while ((timer.seconds() < 2)) {

        }
        rightGateServo.setPosition(.35);
        leftGateServo.setPosition(.35);
        timer.reset();
        while ((timer.seconds() < 5)) {

        }
        timer.reset();
        while ((timer.seconds() < 0.6)) {
            MoveRobot(-0.9,-0.,0);
        }
        MoveRobot(0,0,0);
            timer.reset();

        while ((timer.seconds() < 3000)) {
            flywheelOutRight.setPower(0);
            flywheelOutLeft.setPower(0);
            intakeRight.setPower(0);
            intakeLeft.setPower(0);

        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void MoveRobot (double x,double y,double rx){
        frontLeftMotor.setPower(y + x + rx);
        backLeftMotor.setPower(y - x + rx);
        frontRightMotor.setPower(y - x - rx);
        backRightMotor.setPower(y + x - rx);

    }
}
