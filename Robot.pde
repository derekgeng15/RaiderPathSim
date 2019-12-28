public class Robot{
    Pose pos;
    float vel;
    int rlength;
    int rWidth;
    public Robot(Pose sPos, int l, int w){
      vel = 0;
      rlength = l;
      rWidth = w;
      pos = sPos;
  }
    public void move(float leftVel, float rightVel){
      if(leftVel == rightVel){
         rightVel+=0.0000001;
      }
      //float R = (rwidth/2)*(leftVel + rightVel)/(rightVel - leftVel) ;
      //float angVel = (rightVel - leftVel)/rwidth;
      //Point ICC = new Point(pos.x - R * sin((float)pos.theta), pos.y + R * cos((float)pos.theta));
      //double x = (pos.x - ICC.x)*cos(angVel) + (pos.y - ICC.y) * -sin(angVel) + ICC.x;
      //double y = (pos.x - ICC.x)*sin(angVel) + (pos.y - ICC.y) * cos(angVel) + ICC.y;
      //double z = angVel + pos.theta; 
      //pos.copy(x,y,z);
      pos.update(new DriveCharacterization(10.0, 10.0, rWidth), leftVel, rightVel);
    }
    public void upd(){
     noFill();
     rectMode(CENTER);
     translate((float)pos.x * 1.5, (float)(height - pos.y * 1.5));
     rotate(PI - (float)pos.theta); 
     stroke(#ffff00);
     rect(0, 0, rlength/1.5, rWidth/1.5);
     fill(255);
     translate((float)(-pos.x * 1.5), (float)(-(height - pos.y * 1.5)));
    }
}
