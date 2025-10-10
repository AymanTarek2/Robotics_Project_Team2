package pieces;

import engine.Game;
import exceptions.MoveException;

public class Knight extends Piece{
	public Knight(int x, int y, PieceColor color) {
		super(x,y,color);
	}
	
	public void setAvailableMoves() {
		this.resetAvailableMoves();
		int x = getPoint().x;
		int y = getPoint().y;
		for(int i=-2; i<=2; i++) {
			for(int j=-2; j<=2; j++) {
				if(i==0 || j==0 || Math.abs(i)==Math.abs(j) || x+i<1 || x+i>8 || y+j<1 || y+j>8)
					continue;
				if(Game.board[x+i][y+j].getPiece() == null || this.getColor() != Game.board[x+i][y+j].getPiece().getColor())
					movesForCheck(x+i,y+j);
			}
		}
	}
}
