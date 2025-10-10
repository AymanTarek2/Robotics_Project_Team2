package pieces;
import engine.*;
import exceptions.*;
import javafx.geometry.Pos;
import javafx.scene.control.Button;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.layout.HBox;
import views.az.chessGUI;

public class Pawn extends Piece{
	private boolean justMovedTwoSquares;
	private boolean moved;
	public Pawn(int x,int y,PieceColor color) {
		super(x,y,color);
	}
	
	public boolean isJustMovedTwoSquares() {
		return justMovedTwoSquares;
	}

	public void move(int x, int y) throws MoveException{
		int prevx = this.getPoint().x;
		int prevy = this.getPoint().y;
		boolean enPassant = false;
		if(Math.abs(x - prevx) == 1 && Game.board[x][y].getPiece() == null)
			enPassant = true;
		
		super.move(x, y);
		moved = true;
		
		if(enPassant) { //took en passant
			Game.whitePieces.remove(Game.board[x][prevy].getPiece());  
			Game.blackPieces.remove(Game.board[x][prevy].getPiece());
			Game.board[x][prevy].setPiece(null);
			chessGUI.boardSquares[x][prevy].getChildren().removeIf(node -> node instanceof ImageView);
		}
		else if(Math.abs(getPoint().y - prevy) == 2)
			justMovedTwoSquares = true;
		
		else if(y == 8) 
			promote();
		
		else if(y == 1)
			promote();
	}
	
	public void setAvailableMoves() {
		this.resetAvailableMoves();
		justMovedTwoSquares = false;
		int x = this.getPoint().x;
		int y = this.getPoint().y;
		
		int y1 = (this.getColor() == PieceColor.WHITE ? y+1 : y-1);
		int y2 = (this.getColor() == PieceColor.WHITE ? y+2 : y-2);
		
		if(!moved && Game.board[x][y1].getPiece() == null && Game.board[x][y2].getPiece() == null) 
			movesForCheck(x,y2);
		
		if(Game.board[x][y1].getPiece() == null)
			movesForCheck(x,y1);
		if(x<8 && Game.board[x+1][y1].getPiece() != null && this.getColor() != Game.board[x+1][y1].getPiece().getColor()) //capture
			movesForCheck(x+1,y1);
		if(x>1 && Game.board[x-1][y1].getPiece() != null && this.getColor() != Game.board[x-1][y1].getPiece().getColor()) //capture
			movesForCheck(x-1,y1);
			
		if(x>1 && Game.board[x-1][y].getPiece() != null && Game.board[x-1][y].getPiece() instanceof Pawn  //en passant
		&& ((Pawn)Game.board[x-1][y].getPiece()).justMovedTwoSquares)
			movesForCheck(x-1,y1);
		if(x<8 && Game.board[x+1][y].getPiece() != null && Game.board[x+1][y].getPiece() instanceof Pawn
		&& ((Pawn)Game.board[x+1][y].getPiece()).justMovedTwoSquares)
			movesForCheck(x+1,y1);
	}
	
	public void promote() {
        HBox hbox = new HBox();
        hbox.setAlignment(Pos.CENTER);
        Button queenButton;
        Button rookButton;
        Button bishopButton; 
        Button knightButton;
        if(getColor() == PieceColor.WHITE) {
        	queenButton = createButton("Queen", "w_queen_png_shadow_128px.png");
        	rookButton = createButton("Rook", "w_rook_png_shadow_128px.png");
        	bishopButton = createButton("Bishop", "w_bishop_png_shadow_128px.png");
        	knightButton = createButton("Knight", "w_knight_png_shadow_128px.png");
        }
        else {
        	queenButton = createButton("Queen", "b_queen_png_shadow_128px.png");
            rookButton = createButton("Rook", "b_rook_png_shadow_128px.png");
            bishopButton = createButton("Bishop", "b_bishop_png_shadow_128px.png");
            knightButton = createButton("Knight", "b_knight_png_shadow_128px.png");
        }

        queenButton.setOnAction(event -> handleButtonClick(queenButton, "Queen"));
        rookButton.setOnAction(event -> handleButtonClick(rookButton,"Rook"));
        bishopButton.setOnAction(event -> handleButtonClick(bishopButton,"Bishop"));
        knightButton.setOnAction(event -> handleButtonClick(knightButton,"Knight"));
        
        hbox.getChildren().addAll(queenButton, rookButton, bishopButton, knightButton);
        chessGUI.chessBoardRoot.getChildren().add(hbox);
        
    }
	
	
    private Button createButton(String buttonText, String imageName) {
        Button button = new Button(buttonText);
        ImageView imageView = new ImageView(new Image(imageName));
        imageView.setFitWidth(45);
        imageView.setFitHeight(45);
        button.setGraphic(imageView);
        return button;
    }

    private void handleButtonClick(Button button, String pieceType) {
        Piece piece = null;
        switch(pieceType){
            case "Knight": piece = new Knight(getPoint().x, getPoint().y,getColor()); break;
            case "Bishop": piece = new Bishop(getPoint().x, getPoint().y,getColor()); break;
            case "Rook": piece = new Rook(getPoint().x, getPoint().y,getColor()); break;
            default: piece = new Queen(getPoint().x, getPoint().y,getColor()); break;
        }
        
        promoteBackend(piece, (ImageView) button.getGraphic());
        chessGUI.chessBoardRoot.getChildren().removeIf(node -> node instanceof HBox);
        Game.endTurn();
        Game.endTurn();
    }
	private void promoteBackend(Piece piece, ImageView promotedImg) {
		if(Game.whitePieces.contains(this)) {
			Game.whitePieces.remove(this);
			Game.whitePieces.add(piece);
			
		}
		else if(Game.blackPieces.contains(this)) {
			Game.blackPieces.remove(this);
			Game.blackPieces.add(piece);
			
		}
		Game.board[getPoint().x][getPoint().y].setPiece(piece);
		
		chessGUI.boardSquares[getPoint().x][getPoint().y].getChildren().removeIf(node -> node instanceof ImageView);
		chessGUI.boardSquares[getPoint().x][getPoint().y].getChildren().add(promotedImg);
		
	}
	
}
