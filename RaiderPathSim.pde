import java.io.FileWriter;
import java.io.File;
import java.io.IOException;
Robot robot;
Path path;
PurePursuitController ppc;
PrintWriter writer;
PImage field;
void settings(){
  size((int)(648 * 1.5), (int)(319 * 1.5));
}
void setup(){
  field = loadImage("fields\\2019-Field.jpg");
  field.resize(width, height);
  path = new LeftHabToLeftFarRocket();
  robot = new Robot(new Pose(path.points.get(0).x, path.points.get(0).y, 0), 70, 50);
  ppc = new PurePursuitController(path, new DriveCharacterization(2, 0.01, 50), 5);
  writer = createWriter("output.csv");
  writer.println("x,y,velocity,curvature");
  writer.println(path.points.size());
  for(TrajPoint point: path.get_points())
    writer.println(String.valueOf(point.x) + ',' + String.valueOf(point.y) + ',' + String.valueOf(point.velocity));
  writer.flush();
  writer.close();
}
DriveCommand driveCommand;
ArrayList<Pose> poses = new ArrayList<Pose>();
void draw(){
    image(field, 0, 0);
    stroke(0);
    fill(0);
    for(Point point: path.get_points())
      rect((float)point.x * 1.5, height - (float)point.y * 1.5, 3, 3);
    stroke(#00ff00);
    fill(#00ff00);
    for(WayPoint waypoint : path.waypoints)
      rect((float)waypoint.x * 1.5, height - (float)waypoint.y * 1.5, 5, 5);
    poses.add(new Pose(robot.pos.x, robot.pos.y, robot.pos.theta));
    stroke(#ff0000);
    noFill();
    for(Pose pose : poses)
      rect((float)pose.x * 1.5, height - (float)pose.y * 1.5, 1, 1);
    stroke(#ffff00);
    if(!ppc.is_finished()){
       driveCommand = ppc.pursuit_path(robot.pos);
       robot.move((float)driveCommand.left_command, (float)driveCommand.right_command);
    }
   // else
     // System.out.println("Done");
    robot.upd();
}
