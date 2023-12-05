void setup(){
  size(1000,1000);
  surface.setTitle("Inverse Kinematics [CSCI 5611 Example]");
}

Vec2 goal;

//ARC
Vec2 arcCenter = new Vec2(500, 750);
float arcRadius = 250;
float arcStart = PI;
float arcEnd = TWO_PI;
float arcStep = 0.01; 

int lastUpdateTime = 0;
int updateInterval = 250; // 0.25 seconds in milliseconds
float currentAngle = arcStart;

//Root
Vec2 root = new Vec2(500,300);

//Upper Arm
float l0 = 200; 
float a0 = 0.3; //Shoulder joint

//Lower Arm
float l1 = 200;
float a1 = 0.3; //connecting joint

Vec2 start_l1, endPoint;

float old_a0, old_a1, old_a2;

void solve(){  
  currentAngle += arcStep;
  if (currentAngle > arcEnd) {
    currentAngle = arcStart;
  }

  float x = arcCenter.x + arcRadius * cos(currentAngle);
  float y = arcCenter.y + arcRadius * sin(currentAngle);
  goal = new Vec2(x, y);
  Vec2 rightgoal = new Vec2(mouseX, mouseY);
  
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;
  
  //Update elbow joint
  startToGoal = goal.minus(start_l1);
  startToEndEffector = endPoint.minus(start_l1);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a1 += angleDiff;
  else
    a1 -= angleDiff;
  if(a1 < 0){ a1 = 0; } // Prevents bending backwards
  if(a1 > 3*PI/4){ a1 = 3*PI/4; } // Limits bending to 135 degrees
  fk();
    
  //Update shoulder joint
  startToGoal = goal.minus(root);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = endPoint.minus(root);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a0 += angleDiff;
  else
    a0 -= angleDiff;
  if(a0 < 0){ a0 = 0; } // Prevents moving downwards past straight
  if(a0 > PI/1.2){ a0 = PI/1.2; } // Limits upward movement to 90 degrees
  fk();
}

void fk(){
  start_l1 = new Vec2(cos(a0)*l0,sin(a0)*l0).plus(root);
  endPoint = new Vec2(cos(a0+a1)*l1,sin(a0+a1)*l1).plus(start_l1);
}

float armW = 20;
void draw(){
  fk();
  solve();
  
  background(250,250,250);
  
  fill(150, 0, 150);
  pushMatrix();
  translate(root.x, root.y);
  rect(-37.5, -20, 75, 75);
  popMatrix();
  
  fill(214,168,133);
  pushMatrix();
  translate(root.x,root.y);
  rotate(a0);
  rect(0, -armW/2, l0, armW);
  popMatrix();
  
  
  pushMatrix();
  translate(start_l1.x,start_l1.y);
  rotate(a0+a1);
  rect(0, -armW/2, l1, armW);
  popMatrix();
  arc(500, 750, 500, 300, PI, TWO_PI);
  circle(goal.x, goal.y, 20);
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
