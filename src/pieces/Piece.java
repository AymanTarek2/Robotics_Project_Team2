package pieces;
import java.awt.*;
import java.util.*;
import engine.Game;
import exceptions.MoveException;
public abstract class Piece {
	private Point point;
	private PieceColor color;
	private ArrayList<Point> availableMoves;
	
	public Piece(int x, int y, PieceColor color) {
		point = new Point(x,y);
		this.color = color;
		availableMoves = new ArrayList<Point>();
	}
	
	public Point getPoint() {
		return point;
	}
	public void setPoint(Point point) {
		this.point = point;
	}
	
	public PieceColor getColor() {
		return color;
	}

	public void setColor(PieceColor color) {
		this.color = color;
	}
	
	public ArrayList<Point> getAvailableMoves() {
		return availableMoves;
	}
	public void addMove(int x, int y) {
		availableMoves.add(new Point(x,y));
	}
	
	public void move(int x, int y) throws MoveException {
		if(Game.whiteMove && this.getColor() == PieceColor.BLACK)
			throw new MoveException();
		if(!Game.whiteMove && this.getColor() == PieceColor.WHITE)
			throw new MoveException();
		
		for(Point p: availableMoves) {
			if(x == p.x && y == p.y) {
				Game.whitePieces.remove(Game.board[x][y].getPiece());  
				Game.blackPieces.remove(Game.board[x][y].getPiece());
				Game.board[x][y].setPiece(this);
				Game.board[point.x][point.y].setPiece(null);
				this.setPoint(new Point(x,y));
				return;
			}
		}
		throw new MoveException();
	}
	
	public void resetAvailableMoves() {
		this.availableMoves = new ArrayList<Point>();
	}
	
	public void movesForCheck(int x,int y) {
		Piece piece;
		King king = getKing();
		boolean whiteFlag = false, blackFlag = false;
		piece = Game.board[x][y].getPiece();
		if(piece != null && Game.whitePieces.contains(piece)) {
			Game.whitePieces.remove(piece);
			whiteFlag = true;
		}
		else if(piece != null && Game.blackPieces.contains(piece)) {
			Game.blackPieces.remove(piece);
			blackFlag = true;
		}
		
		Game.board[x][y].setPiece(this);
		Game.board[getPoint().x][getPoint().y].setPiece(null);
		if(!king.inCheck())
			this.availableMoves.add(new Point(x,y));
		
		Game.board[getPoint().x][getPoint().y].setPiece(this);
		Game.board[x][y].setPiece(piece);
		if(whiteFlag)
			Game.whitePieces.add(piece);
		if(blackFlag)
			Game.blackPieces.add(piece);
	}
	
	public King getKing() {
    	if(getColor() == PieceColor.WHITE)
    		return Game.whiteKing;
    	else
    		return Game.blackKing;
	}
	
	public void longRangeDiagonalCheck(int x, int y, boolean incrementX, boolean incrementY) {
		while(true) {
			x += (incrementX ? 1 : -1);
			y += (incrementY ? 1 : -1);
			if(x<1 || x>8 || y<1 || y>8)
				return;
			
			if(Game.board[x][y].getPiece() == null)
				this.movesForCheck(x, y);
			else {
				if(this.getColor() != Game.board[x][y].getPiece().getColor()) //capture
					movesForCheck(x,y);
				return;	
			}
		}
	}
	
	public void longRangeCheckX(int x, int y, boolean increment) {
		while(true) {  
			x += (increment ? 1 : -1);
			if(x<1 || x>8)
				return;
			
			if(Game.board[x][y].getPiece() == null)
				movesForCheck(x, y);
			else {
				if(this.getColor() != Game.board[x][y].getPiece().getColor()) //capture
					movesForCheck(x,y);
				return;	
			}
		}
	}
	public void longRangeCheckY(int x, int y, boolean increment) {
		while(true) {  
			y += (increment ? 1 : -1);
			if(y<1 || y>8)
				return;
			
			if(Game.board[x][y].getPiece() == null)
				movesForCheck(x, y);
			else {
				if(this.getColor() != Game.board[x][y].getPiece().getColor()) //capture
					movesForCheck(x,y);
				return;	
			}
		}
	}
	
	public abstract void setAvailableMoves();
}
