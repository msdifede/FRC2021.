package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * 
 * Add your docs here.
 */
public class LimeLight extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  NetworkTable table;
  NetworkTableEntry tx, ty, tv, ta, ts;
  double x, y, v, area, s;

  public LimeLight(){
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tv = table.getEntry("tv");
    ta = table.getEntry("ta");
    ts = table.getEntry("ts");
  }


  public void readValues(){
     x = tx.getDouble(0.0);
     y = ty.getDouble(0.0);
     v = tv.getDouble(0.0);
     area = ta.getDouble(0.0);
     s = ts.getDouble(0.0);
    postToDashboard();
  }

  public void postToDashboard(){
    //SmartDashboard.putNumber("LimelightX", x);
    //SmartDashboard.putNumber("LimelightY", y);
    //SmartDashboard.putNumber("LimelightV", v);
    SmartDashboard.putNumber("LimelightArea", area);
    //SmartDashboard.putNumber("LimelightSkew", s);
    //SmartDashboard.putNumber("LEDMODE", (int)table.getEntry("ledMode").getDouble(0.0));
  }
  
  public boolean isAimed(){
    return getX()<=1 && getX()>=-1; 
  }


  public double getX(){
    readValues();
    return x;
  }

  public double getY(){
    readValues();
    return y;
  }

  public double getV(){
    readValues();  
    return v;
  }

  public double getArea(){
    readValues();
    return area;
  }

  public double getSkew(){
    readValues();
    return s;
  }

  public boolean seesTarget(){
    readValues(); 
    if( v == 0 )
      return false;
    else  
      return true;
  }

  //a value of 0 represents the vision processor
  //a value of 1 represents the driver camera
  public void setView(int n){
    table.getEntry("camMode").setNumber(n);
  }

  public int getView() {
    return (int)table.getEntry("camMode").getDouble(0.0);
  }

  //0 is side by side, 1 is PiP Main with limelight big
  //2 is PiP Secondary with Secondary big
  public void setStreamMode(int n ){
    table.getEntry("stream").setNumber(n);
  }

  public int getStreamMode() {
    return (int)table.getEntry("stream").getDouble(0.0);
  }

  //0: use pipeline default
  //1: force off, 2: force blink, 3: force on
  public void setLedMode(int n  ){
    table.getEntry("ledMode").setNumber(n);
  }

  public void setPipeline( int p){
    table.getEntry("pipeline").setNumber(p);
  }

  public double getTargetDistance() {
    
    return ( Constants.targetHeight - Constants.cameraHeight ) / Math.tan( Constants.cameraMountingAngle + getY());
  }
}