package pieces;

import engine.Game;
import exceptions.MoveException;

public class Queen extends Piece{
	public Queen(int x, int y, PieceColor color) {
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
		
		longRangeCheckX(x, y, true);
		longRangeCheckX(x, y, false);
		longRangeCheckY(x, y, true);
		longRangeCheckY(x, y, false);
	}	
}
