package engine;
import pieces.*;


import java.awt.Point;
import java.lang.reflect.Constructor;
import java.util.*;
import Cells.*;
import exceptions.MoveException;
public class Game {
	public static Cell[][] board = new Cell[9][9];
	public static ArrayList<Piece> whitePieces = new ArrayList<Piece>();
	public static ArrayList<Piece> blackPieces = new ArrayList<Piece>();
	public static boolean whiteMove = false;
	public static King whiteKing;
	public static King blackKing;
	
	public static void startGame() {
		for(int i=1; i<=8; i++)
			for(int j=1; j<=8; j++) 
				board[i][j] = new Cell();
		
		whiteKing = new King(5,1,PieceColor.WHITE);
		whitePieces.add(whiteKing);
		board[5][1].setPiece(whiteKing);	
		
		blackKing = new King(5,8,PieceColor.BLACK);
		blackPieces.add(blackKing);
		board[5][8].setPiece(blackKing);
		
		Piece p;
			
		for(int i=1; i<=8;i++) { //adding pawns
			p = new Pawn(i,2,PieceColor.WHITE);
			whitePieces.add(p);
			board[i][2].setPiece(p);
			p = new Pawn(i,7,PieceColor.BLACK);
			blackPieces.add(p);
			board[i][7].setPiece(p);
		}
		p = new Rook(1,1,PieceColor.WHITE);
		whitePieces.add(p);
		board[1][1].setPiece(p);
		
		p = new Rook(8,1,PieceColor.WHITE);
		whitePieces.add(p);
		board[8][1].setPiece(p);
		
		p = new Rook(8,8,PieceColor.BLACK);
		blackPieces.add(p);
		board[8][8].setPiece(p);
		
		p = new Rook(1,8,PieceColor.BLACK);
		blackPieces.add(p);
		board[1][8].setPiece(p);
		
		p = new Knight(2,1,PieceColor.WHITE);
		whitePieces.add(p);
		board[2][1].setPiece(p);
		
		p = new Knight(7,1,PieceColor.WHITE);
		whitePieces.add(p);
		board[7][1].setPiece(p);
		
		p = new Knight(2,8,PieceColor.BLACK);
		blackPieces.add(p);
		board[2][8].setPiece(p);
		
		p = new Knight(7,8,PieceColor.BLACK);
		blackPieces.add(p);
		board[7][8].setPiece(p);
		
		p = new Bishop(3,1,PieceColor.WHITE);
		whitePieces.add(p);
		board[3][1].setPiece(p);
		
		p = new Bishop(6,1,PieceColor.WHITE);
		whitePieces.add(p);
		board[6][1].setPiece(p);
		
		p = new Bishop(3,8,PieceColor.BLACK);
		blackPieces.add(p);
		board[3][8].setPiece(p);
		
		p = new Bishop(6,8,PieceColor.BLACK);
		blackPieces.add(p);
		board[6][8].setPiece(p);
		
		p = new Queen(4,1,PieceColor.WHITE);
		whitePieces.add(p);
		board[4][1].setPiece(p);
		
		p = new Queen(4,8,PieceColor.BLACK);
		blackPieces.add(p);
		board[4][8].setPiece(p);
		
		endTurn();
	}
	
	public static void makePiece(int x, int y, Piece p, PieceColor color, String pieceType) {
        try {
            Class<?> clazz = Class.forName("pieces.King");
            
            // Get the constructor of the class with no parameters
            Constructor<?> constructor = clazz.getConstructor(int.class, int.class, PieceColor.class);
            
            // Create an instance of the class
            p = (Piece) constructor.newInstance(x,y,color);
            
    		whitePieces.add(p);
    		board[x][y].setPiece(p);
            
        } catch (Exception e) {
            e.printStackTrace();
        }
	}
	
	public static void endTurn() { //if endTurn() is called after move()
		if(whiteMove)
			whiteMove = false;
		else
			whiteMove = true;
		
		for(int i=0; whiteMove && i<whitePieces.size(); i++) 
			whitePieces.get(i).setAvailableMoves();
			
		for(int i=0; !whiteMove && i<blackPieces.size(); i++)
			blackPieces.get(i).setAvailableMoves();
	}
	
	public static boolean noMovesForWhite() {
			for(Piece piece: whitePieces) 
				if(!piece.getAvailableMoves().isEmpty())
					return false;
		
		return true;
	}
	
	public static boolean noMovesForBlack() {
			for(Piece piece: blackPieces) 
				if(!piece.getAvailableMoves().isEmpty())
					return false;
			
			return true;
	}
	
	public static boolean checkWhiteWin() {
		if(noMovesForBlack() && blackKing.inCheck())
			return true;
		return false;
	}
	
	public static boolean checkBlackWin() {
		if(noMovesForWhite() && whiteKing.inCheck())
			return true;
		return false;
	}
	
	public static boolean drawByInsufficientMaterial() {
		if(whitePieces.size()<=2 && blackPieces.size() <=2) {  
			if(whitePieces.size() == 2 && !(whitePieces.get(1) instanceof Knight) && !(whitePieces.get(1) instanceof Bishop))
				return false;
			if(blackPieces.size() == 2 && !(blackPieces.get(1) instanceof Knight) && !(blackPieces.get(1) instanceof Bishop))
				return false;
			return true;
		}
		return false;
	}
	
	public static boolean drawByStalemate() {
		if(whiteMove && noMovesForWhite() && !whiteKing.inCheck()) 
			return true;
		if(!whiteMove && noMovesForBlack() && !blackKing.inCheck())
			return true;
		return false;
	}
	
}
