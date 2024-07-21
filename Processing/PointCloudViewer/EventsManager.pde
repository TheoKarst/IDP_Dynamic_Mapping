import java.util.HashSet;

public interface EventHandler<T> {
  public void handle(T event);
}

public enum EventType {
  KEY_PRESS, KEY_RELEASE, MOUSE_WHEEL, MOUSE_PRESS, MOUSE_DRAG, MOUSE_RELEASE
}

public static class EventsManager {
  // List the keyCodes of all the keys that are currently pressed:
  private static HashSet<Integer> keysPressed = new HashSet<Integer>();
  
  private static ArrayList<EventHandler<KeyEvent>> keyPressListeners = new ArrayList<EventHandler<KeyEvent>>();
  private static ArrayList<EventHandler<KeyEvent>> keyReleaseListeners = new ArrayList<EventHandler<KeyEvent>>();
  
  private static ArrayList<EventHandler<MouseEvent>> mouseWheelListeners = new ArrayList<EventHandler<MouseEvent>>();
  private static ArrayList<EventHandler<MouseEvent>> mousePressListeners = new ArrayList<EventHandler<MouseEvent>>();
  private static ArrayList<EventHandler<MouseEvent>> mouseDragListeners = new ArrayList<EventHandler<MouseEvent>>();
  private static ArrayList<EventHandler<MouseEvent>> mouseReleaseListeners = new ArrayList<EventHandler<MouseEvent>>();
  
  public static void addEventHandler(EventType type, EventHandler handler){
    switch(type){
      case KEY_PRESS:     keyPressListeners.add(handler); break;
      case KEY_RELEASE:   keyReleaseListeners.add(handler); break;
      
      case MOUSE_WHEEL:   mouseWheelListeners.add(handler); break;
      case MOUSE_PRESS:   mousePressListeners.add(handler); break;
      case MOUSE_DRAG:    mouseDragListeners.add(handler); break;
      case MOUSE_RELEASE: mouseReleaseListeners.add(handler); break;
    }
  }
  
  public static boolean isKeyPressed(int code){
    return keysPressed.contains(code);
  }
  
  private static void onKeyPressed(KeyEvent event){
    keysPressed.add(event.getKeyCode());
    for(EventHandler<KeyEvent> handlers : keyPressListeners)
      handlers.handle(event);
  }
  
  private static void onKeyReleased(KeyEvent event){
    keysPressed.remove(event.getKeyCode());
    for(EventHandler<KeyEvent> handlers : keyReleaseListeners)
      handlers.handle(event);
  }
  
  private static void onMouseWheel(MouseEvent event){
    for(EventHandler<MouseEvent> handlers : mouseWheelListeners)
      handlers.handle(event);
  }
  
  private static void onMousePressed(MouseEvent event){
    for(EventHandler<MouseEvent> handlers : mousePressListeners)
      handlers.handle(event);
  }
  
  private static void onMouseDragged(MouseEvent event){
    for(EventHandler<MouseEvent> handlers : mouseDragListeners)
      handlers.handle(event);
  }
  
  private static void onMouseReleased(MouseEvent event){
    for(EventHandler<MouseEvent> handlers : mouseReleaseListeners)
      handlers.handle(event);
  }
}

void keyPressed(KeyEvent event){
  EventsManager.onKeyPressed(event);
  
  if(key == 'n')
    pointCloud.nextFrame(100);
}

void keyReleased(KeyEvent event){
  EventsManager.onKeyReleased(event);
}

void mouseWheel(MouseEvent event){
  EventsManager.onMouseWheel(event);
}

void mousePressed(MouseEvent event){
  EventsManager.onMousePressed(event);
}

void mouseDragged(MouseEvent event){
  EventsManager.onMouseDragged(event);
}

void mouseReleased(MouseEvent event){
  EventsManager.onMouseReleased(event);
}
