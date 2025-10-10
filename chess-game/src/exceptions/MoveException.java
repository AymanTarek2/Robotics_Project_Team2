package exceptions;

public class MoveException extends GameException{
	public MoveException() {
	}

	public MoveException(String message) {
		super(message);
	}
}
