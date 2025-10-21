package solver;

import java.util.HashSet;
import java.util.Set;

public class SokoBot {
  //Properties
  private final Set<Point> goals = new HashSet<>();

  //Methods
  public String solveSokobanPuzzle(int width, int height, char[][] mapData, char[][] itemsData) {
    findGoals(mapData, itemsData, width, height);
    State start = parseInitialState(itemsData, width, height);
  }


  /** Represents a grid coordinate (row, col) */
  static class Point {
    final int row, col;

    Point(int row, int col) {
        this.row = row;
        this.col = col;
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof Point)) return false;
        Point p = (Point) o;
        return row == p.row && col == p.col;
    }

    @Override
    public int hashCode() {
        return Objects.hash(row, col);
    }
  }

  /** Represents a full Sokoban configuration */
  static class State {
    final int playerRow, playerCol;  // player position
    final Set<Point> boxes;          // box positions

    State(int playerRow, int playerCol, Set<Point> boxes) {
        this.playerRow = playerRow;
        this.playerCol = playerCol;
        this.boxes = boxes;
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof State)) return false;
        State s = (State) o;
        return playerRow == s.playerRow && playerCol == s.playerCol && boxes.equals(s.boxes);
    }

    @Override
    public int hashCode() {
        return Objects.hash(playerRow, playerCol, boxes);
    }
  }

  /** Node for the priority queue (GBFS frontier) */
static class Node implements Comparable<Node> {
    final State state;
    final String path; // movement sequence so far
    final int f;       // heuristic cost (lower = better)

    Node(State s, String p, int f) {
        this.state = s;
        this.path = p;
        this.f = f;
    }

    @Override
    public int compareTo(Node o) {
        return Integer.compare(this.f, o.f);
    }
}

  /** Represents a single movement (up, down, left, right, with/without push) */
static class Move {
    final int dRow, dCol; // movement offset
    final char symbol;    // movement symbol ('u','d','l','r')
    final boolean push;   // true if pushing a box

    Move(int dRow, int dCol, char sym, boolean push) {
        this.dRow = dRow;
        this.dCol = dCol;
        this.symbol = sym;
        this.push = push;
    }
}
  
  //map out points of the goals
  private void findGoals(char[][] mapData, char[][] itemsData, int width, int height){
    for (int i = 0; i < height; i++){
      for (int j = 0; j < width; j++){
        if (mapData[i][j] == '.'){
          goals.add(new Point(i, j));
        }
        if (itemsData[i][j] == '*' || itemsData[i][j] == '+'){
          goals.add(new Point(i, j));
        }
      }
    }
  }

  private State parseInitialState(char[][] itemsData, int width, int height){
    int row = 0, col = 0; //player pos
    Set<Point> box_pos = new HashSet<>();

    for (int i = 0; i < height; i++){
      for (int j = 0; j < width; j++){
        if (itemsData[i][j] == '@' || itemsData[i][j] == '+'){ //Find player
          row = i;
          col = j;
        }
        if (itemsData[i][j] == '$' || itemsData[i][j] == '*'){
          box_pos.add(new Point(i, j));
        }
      }
    }

    return new State(row, col, box_pos);
  }

  private boolean isWall(int row, int col, char[][] mapData){
    return mapData[row][col] == '#'; //add bounds checking if needed
  }
}
