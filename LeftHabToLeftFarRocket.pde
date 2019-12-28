public class LeftHabToLeftFarRocket extends Path{
    @Override
    public ArrayList<WayPoint> get_waypoints(){
      ArrayList<WayPoint> points = new ArrayList<WayPoint>();
      points.add(new WayPoint(66, 202, 0));
      points.add(new WayPoint(200, 208, radians(14.8)));
      points.add(new WayPoint(287, 260, radians(55.7)));
      points.add(new WayPoint(259, 294, radians(150)));
      return points;
    }
}
