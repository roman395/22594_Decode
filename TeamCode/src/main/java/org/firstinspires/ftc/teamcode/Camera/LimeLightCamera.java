package org.firstinspires.ftc.teamcode.Camera;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Configs.HardwareNames;

public class LimeLightCamera extends Camera {
  Limelight3A limelight3A;
  LLResultTypes.FiducialResult lastResult;
  
  public LimeLightCamera(int tagId, LinearOpMode linearOpMode) {
    super(tagId);
    limelight3A = linearOpMode.hardwareMap.get(Limelight3A.class, HardwareNames.LimeLight);
    switch (tagId) {
      case 20:
        limelight3A.pipelineSwitch(0);
        break;
      case 24:
        limelight3A.pipelineSwitch(1);
        break;
      default:
        limelight3A.pipelineSwitch(2);
        break;
    }
    limelight3A.reloadPipeline();
  }
  
  @Override
  public void update() {
    LLResult result = limelight3A.getLatestResult();
    if (result != null && result.isValid()) {
      for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
        if (fid.getFiducialId() == tagId)
          lastResult = fid;
      }
    }
  }
  
  @Override
  public double getDistance() {
    if(lastResult!=null)
      return lastResult.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.METER).z;
    return -404;
  }
  
  @Override
  public double getXHeading() {
    if(lastResult!=null)
      return lastResult.getTargetPoseCameraSpace().getOrientation().getYaw(AngleUnit.DEGREES);
    return -404;
  }
  
}
