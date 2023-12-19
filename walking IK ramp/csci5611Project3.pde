void setup(){ // 430 - 530
  size(1000,1000);
  surface.setTitle("Inverse Kinematics [CSCI 5611 Example]");
}

//collisions
boolean pointLineCollision(Vec2 p, Vec2 v1, Vec2 v2) {
    float len = v1.distanceTo(v2);
    float d1 = p.distanceTo(v1);
    float d2 = p.distanceTo(v2);
    if (d1 + d2 >= len - 0.01 && d1 + d2 <= len + 0.01) {
        return true;
    }
    return false;
}

boolean lineBallCollision(Vec2 v1, Vec2 v2, Vec2 circlePos, float circleRad) { //https://www.jeffreythompson.org/collision-detection/line-circle.php
    if (v1.distanceTo(circlePos) < circleRad || v2.distanceTo(circlePos) < circleRad) { //check if line exists within circle
        return true;
    }
    float len = v1.distanceTo(v2);
    float t = (((circlePos.x - v1.x) * (v2.x - v1.x)) + ((circlePos.y - v1.y) * (v2.y - v1.y))) / pow(len, 2);
    Vec2 nearestPoint = new Vec2(v1.x + (t * (v2.x - v1.x)), v1.y + (t * (v2.y - v1.y)));
    if (nearestPoint.distanceTo(circlePos) < circleRad && pointLineCollision(nearestPoint, v1, v2)) {
        return true;
    }
    return false;
}

//Root
Vec2 endPointR = new Vec2(500,800);

//legs
float leglength = 200; 
float a0 = -0.7; //Shoulder joint
float a1 = -0.7; //connecting joint
float a2 = 0.3; //Wrist joint
float a3 = 0.3;
float old_a0, old_a1, old_a2, old_a3;


Vec2 start_l1, start_l2, start_l3, endPoint;
Vec2 start_ground = new Vec2(600, 800);
Vec2 end_ground = new Vec2(1000, 600);
int mode = 0;

void mousePressed() {
  if (mode == 0) {
    mode = 1;
  } else {
    mode = 0;
  }
}

void solve(){ 
  Vec2 goal = new Vec2(mouseX, mouseY);
  
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;
  
  old_a0 = a0;
  old_a1 = a1;
  old_a2 = a2;
  old_a3 = a3;
  
  if(mode == 0){
    //Update left knee joint
    startToGoal = goal.minus(start_l3);
    startToEndEffector = endPoint.minus(start_l3);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = acos(dotProd);
    do {
      a3 = old_a3;
      if (cross(startToGoal,startToEndEffector) < 0)
        a3 += angleDiff;
      else
        a3 -= angleDiff;
      if(a3 < 0){a3 = 0;}
      if(a3 > 3*PI/4){a3 = 3*PI/4;}
      fk();
      angleDiff *= 0.5;
    } while (lineBallCollision(start_ground, end_ground, endPoint, armW/2));
    //Update left joint
    startToGoal = goal.minus(start_l2);
    startToEndEffector = endPoint.minus(start_l2);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = acos(dotProd);
    do {
      a2 = old_a2;
      if (cross(startToGoal,startToEndEffector) < 0)
        a2 += angleDiff;
      else
        a2 -= angleDiff;
      if(a2 < PI/2){a2 = PI/2;}
      if(a2 > PI){a2 = PI;}
      fk(); //Update link positions with fk (e.g. end effector changed)
      angleDiff *= 0.5;
    } while (lineBallCollision(start_ground, end_ground, endPoint, armW/2));
    //Update right hip joint
    startToGoal = goal.minus(start_l1);
    startToEndEffector = endPoint.minus(start_l1);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = acos(dotProd);
    do {
      a1 = old_a1;
      if (cross(startToGoal,startToEndEffector) < 0)
        a1 += angleDiff;
      else
        a1 -= angleDiff;
      if(a1 > 0){a1 = 0;}
      if(a1 < -0.45){a1 = -0.45;}
      fk(); //Update link positions with fk (e.g. end effector changed)
      angleDiff *= 0.5;
    } while (lineBallCollision(start_ground, end_ground, endPoint, armW/2));
    //Update right knee joint
    startToGoal = goal.minus(endPointR);
    if (startToGoal.length() < .0001) return;
    startToEndEffector = endPoint.minus(endPointR);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = acos(dotProd);
    do {
      a0 = old_a0;
      if (cross(startToGoal,startToEndEffector) < 0)
        a0 += angleDiff;
      else
        a0 -= angleDiff;
      if(a0 > -1.3){a0 = -1.3;}
      if(a0 < -1.9){a0 = -1.9;} 
      fk();
      angleDiff *= 0.5;
    } while(lineBallCollision(start_ground, end_ground, endPoint, armW/2));
  }
  
  
  
  
  else{
    //Update right knee joint
    startToGoal = goal.minus(start_l3);
    startToEndEffector = endPointR.minus(start_l3);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = acos(dotProd);
    //do {
      a3 = old_a3;
      if (cross(startToGoal,startToEndEffector) < 0)
        a3 += angleDiff;
      else
        a3 -= angleDiff;
      //if(a1 < 0){a1 = 0;}
      //if(a1 > 3*PI/4){a1 = 3*PI/4;}
      fk();
      //angleDiff *= 0.5;
    //} while (lineBallCollision(start_ground, end_ground, root, armW/2));
    //Update right hip joint
    startToGoal = goal.minus(start_l2);
    startToEndEffector = endPointR.minus(start_l2);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = acos(dotProd);
    //do {
      a2 = old_a2;
      if (cross(startToGoal,startToEndEffector) < 0)
        a2 += angleDiff;
      else
        a2 -= angleDiff;
      //if(a2 < 0){a2 = 0;}
      //if(a2 > PI/1.2){a2 = PI/1.2;}
      fk(); //Update link positions with fk (e.g. end effector changed)
      //angleDiff *= 0.5;
    //} while (lineBallCollision(start_ground, end_ground, root, armW/2));
    //Update right hip joint
    startToGoal = goal.minus(start_l1);
    startToEndEffector = endPointR.minus(start_l1);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = acos(dotProd);
    //do {
      a1 = old_a3;
      if (cross(startToGoal,startToEndEffector) < 0)
        a1 += angleDiff;
      else
        a1 -= angleDiff;
      //if(a3 > 0){a3 = 0;}
      //if(a3 < -0.45){a3 = -0.45;}
      fk(); //Update link positions with fk (e.g. end effector changed)
      //angleDiff *= 0.5;
    //} while (lineBallCollision(start_ground, end_ground, root, armW/2));
    //Update right knee joint
    startToGoal = goal.minus(endPoint);
    if (startToGoal.length() < .0001) return;
    startToEndEffector = endPointR.minus(endPoint);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = acos(dotProd);
    //do {
      a0 = old_a0;
      if (cross(startToGoal,startToEndEffector) < 0)
        a0 += angleDiff;
      else
        a0 -= angleDiff;
      //if(a4 > -1.3){a4 = -1.3;}
      //if(a4 < -1.9){a4 = -1.9;} 
      fk();
      //angleDiff *= 0.5;
    //} while(lineBallCollision(start_ground, end_ground, root, armW/2));
  }
  
}

void fk(){
  if(mode == 0){
    start_l1 = new Vec2(cos(a0)*leglength,sin(a0)*leglength).plus(endPointR);
    start_l2 = new Vec2(cos(a0+a1)*leglength,sin(a0+a1)*leglength).plus(start_l1);
    start_l3 = new Vec2(cos(a0+a1+a2)*leglength,sin(a0+a1+a2)*leglength).plus(start_l2);
    endPoint = new Vec2(cos(a0+a1+a2+a3)*leglength,sin(a0+a1+a2+a3)*leglength).plus(start_l3);
  }
  else{
    start_l1 = new Vec2(cos(a0)*leglength,sin(a0)*leglength).plus(endPoint);
    start_l2 = new Vec2(cos(a0+a1)*leglength,sin(a0+a1)*leglength).plus(start_l1);
    start_l3 = new Vec2(cos(a0+a1+a2)*leglength,sin(a0+a1+a2)*leglength).plus(start_l2);
    endPointR = new Vec2(cos(a0+a1+a2+a3)*leglength,sin(a0+a1+a2+a3)*leglength).plus(start_l3);
  }
  
}

float armW = 20;
void draw(){
  fk();
  solve();
  
  background(250,250,250);
  
  fill(150, 0, 150);
  pushMatrix();
  translate(endPointR.x, endPointR.y);
  rect(-37.5, -20, 75, 75);
  popMatrix();
  
  fill(214,168,133);
  pushMatrix();
  translate(endPointR.x,endPointR.y);
  rotate(a0);
  rect(0, -armW/2, leglength, armW);
  popMatrix();
  
  
  pushMatrix();
  translate(start_l1.x,start_l1.y);
  rotate(a0+a1);
  rect(0, -armW/2, leglength, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l2.x,start_l2.y);
  rotate(a0+a1+a2);
  rect(0, -armW/2, leglength, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l3.x,start_l3.y);
  rotate(a0+a1+a2+a3);
  rect(0, -armW/2, leglength, armW);
  popMatrix();
  
  circle(endPoint.x, endPoint.y, armW);
  
  line(start_ground.x, start_ground.y, end_ground.x, end_ground.y);
}


public class Vec2 {
  public float x, y;
  
  public Vec2(float x, float y){
    this.x = x;
    this.y = y;
  }
  
  public String toString(){
    return "(" + x+ "," + y +")";
  }
  
  public float length(){
    return sqrt(x*x+y*y);
  }
  
  public Vec2 plus(Vec2 rhs){
    return new Vec2(x+rhs.x, y+rhs.y);
  }
  
  public void add(Vec2 rhs){
    x += rhs.x;
    y += rhs.y;
  }
  
  public Vec2 minus(Vec2 rhs){
    return new Vec2(x-rhs.x, y-rhs.y);
  }
  
  public void subtract(Vec2 rhs){
    x -= rhs.x;
    y -= rhs.y;
  }
  
  public Vec2 times(float rhs){
    return new Vec2(x*rhs, y*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
  }
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
    }
  }
  
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y);
    x *= newL/magnitude;
    y *= newL/magnitude;
  }
  
  public void normalize(){
    float magnitude = sqrt(x*x + y*y);
    x /= magnitude;
    y /= magnitude;
  }
  
  public Vec2 normalized(){
    float magnitude = sqrt(x*x + y*y);
    return new Vec2(x/magnitude, y/magnitude);
  }
  
  public float distanceTo(Vec2 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    return sqrt(dx*dx + dy*dy);
  }
}

Vec2 interpolate(Vec2 a, Vec2 b, float t){
  return a.plus((b.minus(a)).times(t));
}

float interpolate(float a, float b, float t){
  return a + ((b-a)*t);
}

float dot(Vec2 a, Vec2 b){
  return a.x*b.x + a.y*b.y;
}

float cross(Vec2 a, Vec2 b){
  return a.x*b.y - a.y*b.x;
}


Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(a.x*b.x + a.y*b.y);
}

float clamp(float f, float min, float max){
  if (f < min) return min;
  if (f > max) return max;
  return f;
}
