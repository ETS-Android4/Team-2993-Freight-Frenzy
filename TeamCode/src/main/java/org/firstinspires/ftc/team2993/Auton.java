package org.firstinspires.ftc.team2993;

public interface Auton {
    enum strafe{
        left("sLeft"),
        right("sRight");
        private String direction;
        strafe(String direction){
            this.direction = direction;
        }
        public String getDirection(){
            return direction;
        }
    }
}