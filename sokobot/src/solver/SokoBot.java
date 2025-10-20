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


  class Point{

  }

  class State{

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
