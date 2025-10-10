package pieces;

import engine.Game;
import exceptions.MoveException;

public class Rook extends Piece{
	private boolean moved = false;
	public Rook(int x, int y, PieceColor color) {
		super(x,y,color);

	}
	
	public boolean isMoved() {
		return moved;
	}

	public void move(int x, int y) throws MoveException {
		super.move(x, y);
		moved = true;
	}

	public void setAvailableMoves() {
		this.resetAvailableMoves();
		int x = this.getPoint().x;
		int y = this.getPoint().y;
		longRangeCheckX(x, y, true);
		longRangeCheckX(x, y, false);
		longRangeCheckY(x, y, true);
		longRangeCheckY(x, y, false);
	}
}
