package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

@TeleOp(name="TESTE ODO")

public class TesteOdometryAndPods extends OpMode {

    GoBildaPinpointDriver odo;  // Variável própria da GoBilda


    @Override

    public void init(){
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo"); //Pra aparecer e desbugar

// Configuração da odo e os OffSets etc... ------->
        odo.setOffsets(xxx, yyy);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);  //4 com ctz
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

// Pontos iniciais da odo  ------------>
        odo.resetPosAndIMU();
        Pode2D startingPosition = new Pose2D(DistanceUnit.MM, xxx, yyy, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("OffSet X:", odo.getXOffset());
        telemetry.addData("OffSet Y:", odo.getYOffset());
        telemetry.update();
    }

    public void moveRobot(){

// Medir a  algulatura
        Pose2D pos = odo.getPosition();
        double heading = pos.getHeading(AngleUnit.RADIANS);
// Calcular seno e cosseno com o valor de cima
        double cos = Math.cos((Math.PI / 2)-heading);
        double sin = Math.sin((Math.PI / 2)-heading);
// Usar os valores para calcular o starffe
        double globalStrafe = -forward * sin + strafe * cos;
        double globalForward = forward * cos + strafe * sin;
    }

    @Override
    public void loop(){
        moveRobot();
        odo.update();  // Atualiar a odo
    }

}
