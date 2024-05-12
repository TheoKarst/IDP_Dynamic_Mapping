final float MAX_MATCH_DISTANCE = 20;

public int[] computeMatching(ArrayList<PVector> initial, ArrayList<PVector> target) {
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

public void transformShape(PVector center, ArrayList<PVector> shape, float transformX, float transformY, float transformTheta) {
  // Translate and rotate the shape around its center to get the new shape:
  for(int i = 0; i < shape.size(); i++) {
    PVector point = shape.get(i);
    float dX = point.x - center.x;
    float dY = point.y - center.y;
    
    float x = center.x + transformX + dX*cos(transformTheta) - dY*sin(transformTheta);
    float y = center.y + transformY + dY*cos(transformTheta) + dX*sin(transformTheta);
    shape.set(i, new PVector(x, y));
  }
}

public float[] estimateTransform(ArrayList<PVector> initialShape, ArrayList<PVector> targetShape, PVector center, int matching[], int iterations) {
  
  // Build normalized versions of the initial and target shapes:
  ArrayList<PVector> normalizedInitial = centerShape(initialShape, center);
  ArrayList<PVector> normalizedTarget = centerShape(targetShape, center);
  
  float normalizationFactor = max(maxMagnitude(normalizedInitial), maxMagnitude(normalizedTarget));
  
  for(PVector point : normalizedInitial) point.div(normalizationFactor);
  for(PVector point : normalizedTarget) point.div(normalizationFactor);
  
  float transformX = 0, transformY = 0, transformTheta = 0;
  
  for(int i = 0; i < iterations; i++) {
    float[] gradient = computeGradient(normalizedInitial, normalizedTarget, matching, transformX, transformY, transformTheta);
    
    final float timestep = 0.1f;
    transformX -= timestep * gradient[0];
    transformY -= timestep * gradient[1];
    transformTheta -= timestep * gradient[2];
  }
  
  return new float[]{transformX * normalizationFactor, transformY * normalizationFactor, transformTheta};
}

public ArrayList<PVector> centerShape(ArrayList<PVector> shape, PVector center) {
  ArrayList<PVector> centered = new ArrayList<PVector>(shape.size());
  
  for(PVector point : shape)
    centered.add(PVector.sub(point, center, new PVector()));
    
  return centered;
}

public float maxMagnitude(ArrayList<PVector> vectors) {
  float maxMagnitude = 0;
  
  for(PVector vector : vectors) {
    float magnitude = vector.mag();
    
    if(magnitude > maxMagnitude)
      maxMagnitude = magnitude;
  }
  
  return maxMagnitude;
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

public PVector computeCenter(ArrayList<PVector> shape) {
  float sumX = 0, sumY = 0;
  
  for(PVector point : shape) {
    sumX += point.x;
    sumY += point.y;
  }
  
  int n = shape.size();
  return new PVector(sumX / n, sumY / n);
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

ArrayList<PVector> deepcopy(ArrayList<PVector> shape) {
  ArrayList<PVector> copy = new ArrayList<PVector>(shape.size());
  
  for(PVector point : shape)
    copy.add(new PVector().set(point));
    
  return copy;
}
