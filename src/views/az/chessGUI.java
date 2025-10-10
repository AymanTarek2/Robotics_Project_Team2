package views.az;
	
import javafx.application.Application;
import javafx.stage.Stage;
import javafx.scene.Scene;
import javafx.scene.image.Image;
import javafx.scene.layout.StackPane;
import java.awt.Point;
import java.util.ArrayList;
import engine.Game;
import exceptions.MoveException;
import javafx.event.Event;
import javafx.event.EventHandler;
import javafx.geometry.Pos;
import javafx.scene.control.*;
import javafx.scene.image.ImageView;
import javafx.scene.layout.*;
import javafx.scene.paint.Color;
import javafx.scene.shape.*;
import javafx.stage.*;
import pieces.*;

public class chessGUI extends Application {
    public static Scene s;
    public static Stage stage;
    public static StackPane chessBoardRoot;
    public static GridPane chessboard;
    public static StackPane[][] boardSquares;
    
    public void start(Stage primaryStage) throws Exception {
        StackPane root = new StackPane();
        Image img = new Image("gold-silver-chess-chess-board-game-business-metaphor-leadership-concept (1).jpg");
        ImageView imageView = new ImageView(img);
        imageView.setFitWidth(Screen.getPrimary().getBounds().getWidth());
        imageView.setFitHeight(Screen.getPrimary().getBounds().getHeight());

        Button b = new Button("Start Game");
        b.setTranslateY(300);
        b.setTextFill(javafx.scene.paint.Color.WHITE);
        b.setStyle("-fx-font: 32 serif;");
        b.setBackground(null);

        Button b2 = new Button("");
        b2.setPrefSize(1920, 1080);
        b2.setBackground(null);
        
        root.getChildren().addAll(imageView, b, b2);
        s = new Scene(root, 1000, 600);
        primaryStage.setScene(s);
        stage = primaryStage;
        stage.show();
        
        b2.setOnMouseClicked(event -> Scene2());
    }
	
	public static void Scene2() {
		chessboard = new GridPane();
		boardSquares = new StackPane[9][9];
		for (int row = 0; row < 8; row++) {
            for (int col = 0; col < 8; col++) {
                StackPane square = new StackPane();
                square.setMinSize(75, 75); // Set the size of each square
                if ((row + col) % 2 == 0) 
                	square.setStyle("-fx-background-color: #DDB88C;"); // Light wooden color
                else 
                	square.setStyle("-fx-background-color: #A66F4E;"); // Dark wooden color
                
                chessboard.add(square, row, col);
                boardSquares[1+row][8-col] = square;
            }
		}
        
        createAllPieces();
    	
    	ImageView background = new ImageView(new Image("monochrome-pieces-chess-board-game.jpg"));
    	background.setFitWidth(Screen.getPrimary().getBounds().getWidth());
    	background.setFitHeight(Screen.getPrimary().getBounds().getHeight());
        
        chessBoardRoot = new StackPane();
        chessBoardRoot.getChildren().addAll(background, chessboard);
        chessboard.setAlignment(Pos.CENTER);
        
		s.setRoot(chessBoardRoot);
		stage.setScene(s);
		stage.show();
		
		writeNotations();
		
		Game.startGame();
		gameplay(boardSquares);
	}  
	
	public static void createAllPieces() {
		for(int i=1; i<=8; i++) {
            createPieces("w_pawn_png_shadow_128px.png",i,2);
            createPieces("b_pawn_png_shadow_128px.png",i,7);
        }
		createPieces("w_rook_png_shadow_128px.png", 1, 1);
        createPieces("w_rook_png_shadow_128px.png", 8, 1);
    	createPieces("w_knight_png_shadow_128px.png",2,1);
    	createPieces("w_knight_png_shadow_128px.png",7,1);
        createPieces("w_bishop_png_shadow_128px.png",3,1);
    	createPieces("w_bishop_png_shadow_128px.png",6,1);
        createPieces("w_queen_png_shadow_128px.png",4,1);
        createPieces("w_king_png_shadow_128px.png",5,1);
        createPieces("b_rook_png_shadow_128px.png",1,8);
        createPieces("b_rook_png_shadow_128px.png",8,8);
        createPieces("b_knight_png_shadow_128px.png",2,8);
    	createPieces("b_knight_png_shadow_128px.png",7,8);
        createPieces("b_bishop_png_shadow_128px.png",3,8);
    	createPieces("b_bishop_png_shadow_128px.png",6,8);
        createPieces("b_queen_png_shadow_128px.png",4,8);
        createPieces("b_king_png_shadow_128px.png",5,8);
	}
	
	public static void writeNotations() {
		for (int col = 0; col < 8; col++) {
            Label fileLabel = new Label(Character.toString((char) ('a' + col)));
            fileLabel.setMinSize(70, 75);
            fileLabel.setStyle("-fx-text-fill: white;");
            fileLabel.setAlignment(Pos.TOP_CENTER);
            chessboard.add(fileLabel, col, 8); // Add labels to the bottom row
        }

        // Add rank (1-8) notations
        for (int row = 0; row < 8; row++) {
            Label rankLabel = new Label(Integer.toString(8 - row));
            rankLabel.setMinSize(75, 75);
            rankLabel.setStyle("-fx-text-fill: white;");
            rankLabel.setAlignment(Pos.BASELINE_LEFT);
            rankLabel.setTranslateX(5);
            chessboard.add(rankLabel, 8, row); // Add labels to the rightmost column
        }
	}
	
	public static void createPieces(String img, int x, int y) {
        ImageView imageView = new ImageView(new Image(img));
        imageView.setFitWidth(45);
        imageView.setFitHeight(45);
        boardSquares[x][y].getChildren().add(imageView);
	}
	
	public static void gameplay(StackPane[][] boardSquares) {
		ArrayList<Piece> pieceArray = (Game.whiteMove ? Game.whitePieces : Game.blackPieces);
		
	    for (int i = 0; i < pieceArray.size() ; i++) {
	        int x = pieceArray.get(i).getPoint().x , y = pieceArray.get(i).getPoint().y;
	        boardSquares[x][y].setOnMouseClicked(new EventHandler<Event>() {
	            public void handle(Event event) {
	                ArrayList<Point> moves = Game.board[x][y].getPiece().getAvailableMoves();
	                highlightMoves(boardSquares, moves);
	                handleMovement(boardSquares, x, y, moves);
	            }
	        });
	    }
	    
	}
	
	private static void highlightMoves(StackPane[][] boardSquares, ArrayList<Point> moves) {
	    for (Point p : moves) {
	        Circle circle = new Circle(7.5, Color.LIGHTGREY);
	        if (p.x > 0 && p.x <= 8 && p.y > 0 && p.y <= 8 && boardSquares[p.x][p.y] != null) {
	            boardSquares[p.x][p.y].getChildren().add(circle);
	        }
	    }
	}

	private static void handleMovement(StackPane[][] boardSquares, int x, int y, ArrayList<Point> moves) {
	    for (int i = 1; i <= 8; i++) {
	        for (int j = 1; j <= 8; j++) {
	            int a = i, b = j;
	            if (a > 0 && a <= 8 && b > 0 && b <= 8 && boardSquares[a][b] != null) {
	            	if(a == x && b == y)
	            		continue;
	                boardSquares[a][b].setOnMouseClicked(new EventHandler<Event>() {
	                    public void handle(Event event) {
	                        removeCircles(boardSquares);
	                        movePiece(boardSquares, x, y, a, b);
	                    }
	                });
	            }
	        }
	    }
	}

	private static void movePiece(StackPane[][] boardSquares, int x, int y, int a, int b) {
	    try {
	    	Piece piece = Game.board[x][y].getPiece();
	        piece.move(a, b);
	        
	        translatePiece(boardSquares[x][y], boardSquares[a][b]);
	        Game.endTurn();
	        checkGameEndConditions();
	        gameplay(boardSquares);
	        
	    } catch (MoveException e) {
	        e.printStackTrace();
	        gameplay(boardSquares);
	    }
	}
	
	public static void translatePiece(StackPane stackPane1, StackPane stackPane2) {
        stackPane2.getChildren().removeIf(node -> node instanceof ImageView);
        stackPane2.getChildren().add(stackPane1.getChildren().remove(0));
	}
	
	
	public static void checkGameEndConditions() {
        if (!Game.whiteMove && Game.checkWhiteWin()) 
            gameEnded("White wins by Checkmate!");
        else if (Game.whiteMove && Game.checkBlackWin())
        	gameEnded("Black wins by Checkmate!");
        else if (Game.drawByInsufficientMaterial())
        	gameEnded("Draw by Insufficient Material");
        else if (Game.drawByStalemate())
        	gameEnded("Draw by Stalemate");
	}
	
	public static void castleTheRook(int x,int a, int y) {
		ImageView img = null;
		for(int i=0; i<chessGUI.boardSquares[x][y].getChildren().size(); i++)
			if(chessGUI.boardSquares[x][y].getChildren().get(i) instanceof ImageView)
				img = (ImageView)chessGUI.boardSquares[x][y].getChildren().remove(i);
		
		chessGUI.boardSquares[a][y].getChildren().add(img);
	}
	
	public static void gameEnded(String message) {
		Label l = new Label(message);
        l.setStyle("-fx-background-color: white; -fx-padding: 10px; -fx-border-color: black; -fx-border-width: 2px; -fx-font-size: 20px;");
	    chessBoardRoot.getChildren().add(l);
	    l.setPrefSize(300, 100);
	    l.setVisible(true);
	    l.setAlignment(Pos.CENTER);
	}
	
	
	private static void removeCircles(StackPane[][] boardSquares) {
	    for (StackPane[] row : boardSquares) {
	        for (StackPane cell : row) {
	            if (cell != null) 
	                cell.getChildren().removeIf(node -> node instanceof Circle);
	        }
	    }
	}


	
	public static void main(String[] args) {
		launch(args);
		
	}
}
