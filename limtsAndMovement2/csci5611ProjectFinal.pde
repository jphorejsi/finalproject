void setup(){
  size(1000,1000);
  surface.setTitle("Inverse Kinematics [CSCI 5611 Example]");
}

Vec2 goal;

//Root
Vec2 root = new Vec2(500,300);

//Upper legs
float l0 = 200; 
float a0 = 0.3; //knee joint
float a0r = 0.3;

//Lower legs
float l1 = 200;
float a1 = 0.3; //connecting joint
float a1r = 0.3; //right connecting joint

int trigger = 0;
float t = 0;
float speed = 0.05;

Vec2 start_l1, endPoint, start_l1r, endPointr;
Vec2 v1, v2, v3, v4, v5, v6;

void createPath(Vec2 start, float magnitude){
  v1 = start;
  v2 = start.plus(new Vec2(-50.0, -50.0).times(magnitude));
  v3 = v2.plus(new Vec2(50, -50).times(magnitude));
  v4 = v3.plus(new Vec2(250, 0).times(magnitude));
  v5 = v4.plus(new Vec2(50, 50).times(magnitude));
  v6 = v5.plus(new Vec2(-50, 50).times(magnitude));
}

Vec2 lerp(Vec2 start, Vec2 end, float t) {
  float newX = start.x + t * (end.x - start.x);
  float newY = start.y + t * (end.y - start.y);
  return new Vec2(newX, newY);
}

void solve(){
  //create only once when its legs turn
  if(trigger == 0){
    float mag = 1;
    createPath(new Vec2(375, 700), mag);
    t = 0;
    trigger = 1;
  }
  
  if(t < 1){
    goal = lerp(v1, v2, t);
    t += speed;
  }
  if(t >= 1 && t < 2){
    goal = lerp(v2, v3, t - 1);
    t += speed;
  }
  if(t >= 2 && t < 3){
    goal = lerp(v3, v4, t - 2);
    t += speed/2;
  }
  if(t >= 3 && t < 4){
    goal = lerp(v4, v5, t - 3);
    t += speed;
  }
  if(t >= 4 && t < 5){
    goal = lerp(v5, v6, t - 4);
    t += speed;
  }
  if(t >= 5 && t < 6){  //on ground
    goal = lerp(v6, v1, t - 5);
    t += speed/5;
  }
  if(t > 6){
    t = 0;
  }
  
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;

  //Update left knee joint
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
    
  //Update left hip joint
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
  
  //update right knee joint
  startToGoal = goal.minus(start_l1r);
  startToEndEffector = endPointr.minus(start_l1r);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a1r += angleDiff;
  else
    a1r -= angleDiff;
  if(a1r < 0){ a1r = 0; } // Prevents bending backwards
  if(a1r > 3*PI/4){ a1r = 3*PI/4; } // Limits bending to 135 degrees
  fkr();
  
  //update right hip joint
  startToGoal = goal.minus(root);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = endPointr.minus(root);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a0r += angleDiff;
  else
    a0r -= angleDiff;
  if(a0r < 0){ a0r = 0; } // Prevents moving downwards past straight
  if(a0r > PI/1.2){ a0r = PI/1.2; } // Limits upward movement to 90 degrees
  fkr();
}

void fk(){
  start_l1 = new Vec2(cos(a0)*l0,sin(a0)*l0).plus(root);
  endPoint = new Vec2(cos(a0+a1)*l1,sin(a0+a1)*l1).plus(start_l1);
}

void fkr(){
  start_l1r = new Vec2(cos(a0r)*l0,sin(a0r)*l0).plus(root);
  endPointr = new Vec2(cos(a0r+a1r)*l1,sin(a0r+a1r)*l1).plus(start_l1r);
}

float armW = 20;
void draw(){
  fk();
  fkr();
  solve();
  
  background(250,250,250);
  
    //root
  fill(150, 0, 150);
  pushMatrix();
  translate(root.x, root.y);
  rect(-37.5, -20, 75, 75);
  popMatrix();
  
  //leg
  fill(214,168,133);
  pushMatrix();
  translate(root.x,root.y);
  rotate(a0);
  rect(0, -armW/2, l0, armW);
  popMatrix();
  
  //knee
  pushMatrix();
  translate(start_l1.x,start_l1.y);
  rotate(a0+a1);
  rect(0, -armW/2, l1, armW);
  popMatrix();
  
  //right leg
  fill(214,168,133);
  pushMatrix();
  translate(root.x,root.y);
  rotate(a0r);
  rect(0, -armW/2, l0, armW);
  popMatrix();
  
  // right knee
  pushMatrix();
  translate(start_l1r.x,start_l1r.y);
  rotate(a0r+a1r);
  rect(0, -armW/2, l1, armW);
  popMatrix();
  
  circle(goal.x, goal.y, 20);
  line(v1.x, v1.y, v2.x, v2.y);
  line(v2.x, v2.y, v3.x, v3.y);
  line(v3.x, v3.y, v4.x, v4.y);
  line(v4.x, v4.y, v5.x, v5.y);
  line(v5.x, v5.y, v6.x, v6.y);
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
