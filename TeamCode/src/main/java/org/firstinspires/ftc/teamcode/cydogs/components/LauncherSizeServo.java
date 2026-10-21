package org.firstinspires.ftc.teamcode.cydogs.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseServo;

    public class LauncherSizeServo extends BaseServo {

        private final static String hardwareName="LauncherSizeServo";
        private final static Servo.Direction direction= Servo.Direction.FORWARD;
        private final static double min=0.02;
        private final static double max=0.9;

        private final static double NECTAR_SIZE=0.08;
        private final static double POLLEN_SIZE=0.86;
        private final static double startingPosition=0.7;

        public LauncherSizeServo(OpMode opMode) {
            super(opMode, hardwareName, direction, min, max, startingPosition);

        }

        public void SetNectar(){SetPosition(NECTAR_SIZE);}
        public void SetPollen(){SetPosition(POLLEN_SIZE);}





























    }
