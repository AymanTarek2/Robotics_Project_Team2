package pieces;

import java.awt.Point;
import engine.Game;
import exceptions.MoveException;
import views.az.chessGUI;

public class King extends Piece{
	private boolean moved = false;
	
	public King(int x, int y, PieceColor color) {
		super(x,y,color);
	}

	public boolean threatCell(int x, int y) {
		return pawnInRange(x-1,y) || pawnInRange(x+1,y) || kingInRange(x,y) || knightsInRange(x,y) || longRangeDiagonal(x,y,true,true) 
		|| longRangeDiagonal(x,y,true,false) || longRangeDiagonal(x,y,false,true) || longRangeDiagonal(x,y,false,false) || longRangeX(x,y,true) 
		|| longRangeX(x,y,false) || longRangeY(x,y,true) || longRangeY(x,y,false);
			
	}
	
	public boolean pawnInRange(int x, int y) {
		int y1 = (this.getColor() == PieceColor.WHITE ? y+1 : y-1);
		if(y1<= 1 || y1>=8 || x<1 || x > 8)
			return false;
		if(Game.board[x][y1].getPiece() != null && this.getColor() != Game.board[x][y1].getPiece().getColor() 
			&& Game.board[x][y1].getPiece() instanceof Pawn) {
			return true;
		}
		return false;
	}
	
	public boolean kingInRange(int x, int y) {
		for(int i=-1; i<=1; i++) {
			for(int j=-1; j<=1; j++) {
				if((i==0 && j==0) || x+i<1 || x+i>8 || y+j<1 || y+j>8)
					continue;
				
				if(Game.board[x+i][y+j].getPiece() != null && getColor() != Game.board[x+i][y+j].getPiece().getColor()
				&& Game.board[x+i][y+j].getPiece() instanceof King) {
					return true;
				}
			}
		}
		return false;
	}
	
	public boolean knightsInRange(int x, int y) {
		for(int i=-2; i<=2; i++) {
			for(int j=-2; j<=2; j++) {
				if(i==0 || j==0 || Math.abs(i)==Math.abs(j) || x+i<1 || x+i>8 || y+j<1 || y+j>8)
					continue;
				if(Game.board[x+i][y+j].getPiece() != null && getColor() != Game.board[x+i][y+j].getPiece().getColor()
					&& Game.board[x+i][y+j].getPiece() instanceof Knight) {
					return true;
				}
			}
		}
		return false;
	}
	public boolean longRangeDiagonal(int x, int y, boolean incrementX, boolean incrementY) {
		while(true) {
			x += (incrementX ? 1 : -1);
			y += (incrementY ? 1 : -1);
			if(x<1 || x>8 || y<1 || y>8)
				return false;
			
			if(Game.board[x][y].getPiece() == null)
				continue;
			else {
				if(getColor() != Game.board[x][y].getPiece().getColor()
				&& (Game.board[x][y].getPiece() instanceof Bishop || Game.board[x][y].getPiece() instanceof Queen)) {
					return true;
				}
				return false;
			}
		}
	}
	
	public boolean longRangeX(int x, int y, boolean increment) {
		while(true) {
			x += (increment ? 1 : -1);
			if(x<1 || x>8)
				return false;
			if(Game.board[x][y].getPiece() == null)
				continue;
			
			if(getColor() != Game.board[x][y].getPiece().getColor() 
			&& (Game.board[x][y].getPiece() instanceof Rook || Game.board[x][y].getPiece() instanceof Queen)) {
				return true;
			}
			return false;
		}
		
	}
	public boolean longRangeY(int x, int y, boolean increment) {
		while(true) {
			y += (increment ? 1 : -1);
			if(y<1 || y>8)
				return false;
			
			if(Game.board[x][y].getPiece() == null)
				continue;
			else {
				if(Game.board[x][y].getPiece() != null && this.getColor() != Game.board[x][y].getPiece().getColor() 
				&& (Game.board[x][y].getPiece() instanceof Rook || Game.board[x][y].getPiece() instanceof Queen)) {
					return true;
				}
				return false;
			}
		}
	}
	
	public void setAvailableMoves() {
		this.resetAvailableMoves();
		int x = getPoint().x;
		int y = getPoint().y;
		for(int i=-1; i<=1; i++) {
			for(int j=-1; j<=1; j++) {
				if((i==0 && j==0) || x+i<1 || x+i>8 || y+j<1 || y+j>8)
					continue;
				if(!threatCell(x+i,y+j) && (Game.board[x+i][y+j].getPiece() == null || this.getColor() != Game.board[x+i][y+j].getPiece().getColor()))
					addMove(x+i,y+j);
			}
		}
		if(getColor() == PieceColor.WHITE) {
			castleConditions(7, 8, 1);
			castleConditions(3, 1, 1);
		}
		else {
			castleConditions(7, 8, 8);
			castleConditions(3, 1, 8);
		}
	}
	
	private void castleConditions(int x, int rookX , int y) {
		int median = (5 + x)/2;
		if(!moved && Game.board[rookX][y].getPiece() != null && Game.board[rookX][y].getPiece() instanceof Rook 
				&& !((Rook)Game.board[rookX][y].getPiece()).isMoved() && !threatCell(5 ,y) && !threatCell(median ,y) && !threatCell(x ,y)
				&& Game.board[median][y].getPiece() == null && Game.board[x][y].getPiece() == null) {
					addMove(x,y);
				}
	}
	
	public boolean inCheck() {
		return threatCell(getPoint().x,getPoint().y);
	}
	
	public void move(int x, int y) throws MoveException{
		int prevX = getPoint().x;
		super.move(x, y);
		moved = true;
		castleTheRook(x, prevX, y);
	}
	
	private void castleTheRook(int x, int prevX, int y) {
		if(Math.abs(x - prevX) < 2)
			return;
		int originalX = (x > prevX ? 8 : 1);
		int targetX = (x > prevX ? 6 : 4);
		
		Piece rook = Game.board[originalX][y].getPiece();
		Game.board[originalX][y].setPiece(null);
		Game.board[targetX][y].setPiece(rook);
		rook.setPoint(new Point(targetX,y));
		chessGUI.castleTheRook(originalX,targetX,y);
		
	}
}
