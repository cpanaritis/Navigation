package ca.mcgill.ecse211.navigationlab;

/**
 * @author Christos Panaritis Kevin Chuong
 *Abstract class
 */
public interface UltrasonicController {

  /**
 * @param distance
 * abstract method
 */
public void processUSData(int distance);

  /**
 * @return
 * abstract method
 */
public int readUSDistance();
}
