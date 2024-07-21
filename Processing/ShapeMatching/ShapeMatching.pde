final float MAX_MATCH_DISTANCE = 100;


ArrayList<PVector> initialShape = new ArrayList<PVector>();
ArrayList<PVector> transformedShape = new ArrayList<PVector>();
int matching[] = null;

// Normalized versions of initial and target shapes, in order to perform gradient descent:
float normalizationFactor;
ArrayList<PVector> normalizedInitial;
ArrayList<PVector> normalizedTarget;

// Current transformation applied to the initial shape (using gradient descent):
float currentTransformX, currentTransformY, currentTransformTheta;

enum DrawMode {
  INITIAL_SHAPE,
  TRANSFORMED_SHAPE,
  DONE
}

DrawMode drawMode = DrawMode.INITIAL_SHAPE;


void setup() {
  size(2000, 1200, P2D);
  
  textSize(32);
  strokeWeight(2);
}

void draw() {
  background(255);
  
  // Build the current shape from the initial shape and the parameters:
  ArrayList<PVector> currentShape = transformShape(initialShape, currentTransformX * normalizationFactor, currentTransformY * normalizationFactor, currentTransformTheta);
  
  // Draw the shapes:
  drawShape(currentShape, color(0, 255, 0));
  drawShape(transformedShape, color(0, 0, 255));
  
  // Draw the matching between the shapes:
  if(matching != null) {
    stroke(255, 0, 0);
    
    for(int i = 0; i < currentShape.size(); i++) {
      if(matching[i] == -1)
        continue;
        
      PVector from = currentShape.get(i);
      PVector to = transformedShape.get(matching[i]);
      
      line(from.x, from.y, to.x, to.y);
    }
  }
  
  // Draw text info:
  fill(0);
  if(drawMode == DrawMode.INITIAL_SHAPE)
    text("Draw initial shape", 10, 40);
    
  else if(drawMode == DrawMode.TRANSFORMED_SHAPE)
    text("Draw transformed shape", 10, 40);
}

void keyPressed() {
  if(key == ' ') {
    if(drawMode == DrawMode.INITIAL_SHAPE)
      initialShape.add(new PVector(mouseX, mouseY));
      
    else if(drawMode == DrawMode.TRANSFORMED_SHAPE)
      transformedShape.add(new PVector(mouseX, mouseY));
  }
  
  else if(key == 't' && drawMode == DrawMode.INITIAL_SHAPE && initialShape.size() > 1) {
    drawMode = DrawMode.TRANSFORMED_SHAPE;
  }
    
  else if(key == ENTER && matching == null && ((drawMode == DrawMode.INITIAL_SHAPE && initialShape.size() > 1) || transformedShape.size() > 1)) {
    
    // If the transformed shape wasn't created, create it by shifting and rotating randomly the initial shape:
    if(transformedShape.size() == 0) {
      float transformX = random(-40, 40);
      float transformY = random(-40, 40);
      float transformTheta = radians(random(-10, 10));
      
      transformedShape = transformShape(initialShape, transformX, transformY, transformTheta);
      println("Real Transformation: (dX: " + transformX + "; dY: " + transformY + "; dTheta: " + transformTheta);
    }
    
    drawMode = DrawMode.DONE;
    matching = buildMatching(initialShape, transformedShape);
    
    // Create the normalized versions of initial and target shapes:
    normalizedInitial = new ArrayList<PVector>();
    normalizedTarget = new ArrayList<PVector>();
    
    PVector center = computeCenter(initialShape);
    
    // Maximum distance of a point from the initial or target shapes from the center:
    float maxDistance = 0;
    for(PVector point : initialShape) {
      PVector centered = PVector.sub(point, center, new PVector());
      normalizedInitial.add(centered);
      
      float distance = centered.mag();
      if(distance > maxDistance)
        maxDistance = distance;
    }
    
    for(PVector point : transformedShape) {
      PVector centered = PVector.sub(point, center, new PVector());
      normalizedTarget.add(centered);
      
      float distance = centered.mag();
      if(distance > maxDistance)
        maxDistance = distance;
    }
    
    normalizationFactor = maxDistance;
    for(PVector point : normalizedInitial) point.div(normalizationFactor);
    for(PVector point : normalizedTarget) point.div(normalizationFactor);
  }
  
  else if(key == 'd' && drawMode == DrawMode.DONE) {    // Perform one gradient descent step
    // Perform multiple steps per iteration:
    final int STEPS_COUNT = 100;
    
    for(int i = 0; i < STEPS_COUNT; i++) {
      float[] gradient = computeGradient(normalizedInitial, normalizedTarget, matching, currentTransformX, currentTransformY, currentTransformTheta);
    
      final float timestep = 0.1f;
      currentTransformX -= timestep * gradient[0];
      currentTransformY -= timestep * gradient[1];
      currentTransformTheta -= timestep * gradient[2];
    }
    
    println("Transformation estimate:\n(dX: " + currentTransformX * normalizationFactor + "; dY: " + currentTransformY * normalizationFactor + "; dTheta: " + currentTransformTheta);
  }
  
  else if(key == 's') {    // Print data about the error function into a file
    PrintWriter writer = createWriter("data.csv");
    
    writer.println("x;errorX;;y;errorY;;theta;errorTheta");
    int n = 1000;
    for(int i = 0; i < n; i++) {
      float x = map(i, 0, n-1, -1, 1);
      float y = map(i, 0, n-1, -1, 1);
      float theta = map(i, 0, n-1, 0, TWO_PI);
      
      float errorX =     errorBetween(normalizedInitial, normalizedTarget, matching, x, 0, 0);
      float errorY =     errorBetween(normalizedInitial, normalizedTarget, matching, 0, y, 0);
      float errorTheta = errorBetween(normalizedInitial, normalizedTarget, matching, 0, 0, theta);
      
      writer.println(x + ";" + errorX + ";;" + y + ";" + errorY + ";;" + theta + ";" + errorTheta);
    }
    
    writer.flush();
    writer.close();
    
    println("Data written to file !");
  }
  
  else if(key == 'c') {    // Clear
    initialShape.clear();
    transformedShape.clear();
    matching = null;
    drawMode = DrawMode.INITIAL_SHAPE;
    
    currentTransformX = 0;
    currentTransformY = 0;
    currentTransformTheta = 0;
  }
}

public void drawShape(ArrayList<PVector> shape, color shapeColor) {
  if(shape == null || shape.size() == 0)
    return;
    
  stroke(shapeColor);
  PVector previous = shape.get(0);
  for(int i = 1; i < shape.size(); i++) {
    PVector current = shape.get(i);
    line(previous.x, previous.y, current.x, current.y);
    previous = current;
  }
}

public ArrayList<PVector> transformShape(ArrayList<PVector> shape, float transformX, float transformY, float transformTheta) {
  ArrayList<PVector> result = new ArrayList<PVector>();
  
  PVector center = computeCenter(shape);
  
  // Translate and rotate the shape around its center to get the new shape:
  for(PVector point : shape) {
    float dX = point.x - center.x;
    float dY = point.y - center.y;
    
    float x = center.x + transformX + dX*cos(transformTheta) - dY*sin(transformTheta);
    float y = center.y + transformY + dY*cos(transformTheta) + dX*sin(transformTheta);
    result.add(new PVector(x, y));
  }
  
  return result;
}

public float errorBetween(ArrayList<PVector> normalizedInitial, ArrayList<PVector> normalizedTarget, int matching[], float x, float y, float theta) {
  final float costheta = cos(theta);
  final float sintheta = sin(theta);
  
  float error = 0;
  
  int count = 0;
  for(int i = 0; i < normalizedInitial.size(); i++) {
    if(matching[i] == -1)
      continue;
      
    PVector u = normalizedInitial.get(i);
    PVector v = normalizedTarget.get(matching[i]);
    
    float dX = x + u.x * costheta - u.y * sintheta - v.x;
    float dY = y + u.y * costheta + u.x * sintheta - v.y;
    
    error += dX*dX + dY*dY;
    count++;
  }
  
  return error / count;
}

public float[] computeGradient(ArrayList<PVector> normalizedInitial, ArrayList<PVector> normalizedTarget, int matching[], float x, float y, float theta) {
  final float costheta = cos(theta);
  final float sintheta = sin(theta);
  
  float df_dx = 0, df_dy = 0, df_dtheta = 0;
  
  int count = 0;
  for(int i = 0; i < normalizedInitial.size(); i++) {
    if(matching[i] == -1)
      continue;
      
    PVector u = normalizedInitial.get(i);
    PVector v = normalizedTarget.get(matching[i]);
    
    float dX = x + u.x * costheta - u.y * sintheta - v.x;
    float dY = y + u.y * costheta + u.x * sintheta - v.y;
    
    df_dx += dX;
    df_dy += dY;
    df_dtheta += -dX * (u.x*sintheta + u.y*costheta) + dY * (u.x*costheta - u.y*sintheta);
    count++;
  }
  
  // Divide by count, to use the average error:
  df_dx = 2 * df_dx / count;
  df_dy = 2 * df_dy / count;
  df_dtheta = 2 * df_dtheta / count;
  
  return new float[]{df_dx, df_dy, df_dtheta};
}

public int[] buildMatching(ArrayList<PVector> initial, ArrayList<PVector> target) {
  int[] matching = new int[initial.size()];
  
  for(int i = 0; i < initial.size(); i++) {
    float minDistance = 0;
    int index = -1;
    
    for(int j = 0; j < target.size(); j++) {
      float distance = PVector.dist(initial.get(i), target.get(j));
      
      if(index == -1 || distance < minDistance) {
        index = j;
        minDistance = distance;
      }
    }
    
    matching[i] = minDistance <= MAX_MATCH_DISTANCE ? index : -1;
  }
  
  return matching;
}

public PVector computeCenter(ArrayList<PVector> shape) {
  float sumX = 0, sumY = 0;
  
  for(PVector point : shape) {
    sumX += point.x;
    sumY += point.y;
  }
  
  int n = shape.size();
  return new PVector(sumX / n, sumY / n);
}

public float angleBetween(PVector u, PVector v) {
  float angle = atan2(u.y, u.x) - atan2(v.y, v.x);
  while(angle > PI) angle -= TWO_PI;
  while(angle < -PI) angle += TWO_PI;
  
  return angle;
}
