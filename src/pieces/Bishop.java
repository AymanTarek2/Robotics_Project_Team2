package pieces;
import java.awt.Point;

import engine.Game;
import exceptions.MoveException;

public class Bishop extends Piece{
	public Bishop(int x, int y, PieceColor color) {
		super(x,y,color);
	}
	
	public void setAvailableMoves() {
		this.resetAvailableMoves();
		int x = this.getPoint().x;
		int y = this.getPoint().y;
		longRangeDiagonalCheck(x, y, true, true);
		longRangeDiagonalCheck(x, y, true, false);
		longRangeDiagonalCheck(x, y, false, true);
		longRangeDiagonalCheck(x, y, false, false);
	}
	
}