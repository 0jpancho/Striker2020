package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

public class DPad extends Button {
    private XboxController joystick;
    private int direction;
    
    /**
     * Class for dealing with the D-Pad
     * aka PoV hat using enum
     * @param m_operator {@link Joystick} in
     * question
     * @param dir {@link Direction} enum
     * to use
     */
    public DPad(XboxController m_operator, Direction dir) {
    	this(m_operator, dir.getDirection());
    }
    		
    /**
     * Class for dealing with the D-Pad aka PoV hat
     * 
     * @param m_operator the joystick
     * @param direction  angle the hat must be equal to
     */
    public DPad(XboxController m_operator, int direction) {
        joystick = m_operator;
    	this.direction = direction;
    }
    
    /**
     * Enumerates all of the eight possible
     * directions for a d-pad. The all-caps
     * version may fix a problem that popped
     * up out-of-the-blue, so don't touch them!
     */
    public enum Direction {
    	right(90),
    	upRight(45),
    	up(0),
    	upLeft(315),
    	left(270),
    	downLeft(225),
    	down(180),
    	downRight(135),
    	UP(0),
    	UP_LEFT(45),
    	LEFT(90),
    	DOWN_LEFT(135),
    	DOWN(180),
    	DOWN_RIGHT(225),
    	RIGHT(270),
    	UP_RIGHT(315);
    	
    	private int dir;
    	
    	private Direction(int direction) {
    	    dir = direction;
    	}
    	
    	public int getDirection() {
    	    return dir;
    	}
    }
    
    /**
     * Gets whether or not the hat's angle
     * is equal to the given angle
     * @return true if the PoV hat is
     * equal to the given angle, false
     * otherwise
     */
    public boolean get() {
    	return joystick.getPOV() == direction;
    }
}
