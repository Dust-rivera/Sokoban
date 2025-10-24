package solver;

import java.util.HashSet;
import java.util.Set;

import java.util.Objects;
import java.util.PriorityQueue;
import java.util.ArrayDeque;
import java.util.Queue;


public class SokoBot {
  //Properties
  private final Set<Point> goals = new HashSet<>();
  private int[][] distToGoal;
  private final int MAX_DIST = Integer.MAX_VALUE; // an integer used when initializing distances in distToGoal.

  //Methods
  public String solveSokobanPuzzle(int width, int height, char[][] mapData, char[][] itemsData) {
		findGoals(mapData, itemsData, width, height);

		//precompute the distance-to-goal for every tile
		computeGoalDistances(mapData);
    computeGoalDistancesAll(mapData); // mini

		//parse the starting state (player + boxes)
		State start = parseInitialState(itemsData, width, height);

		//priority queue (GBFS) — expands the state with the lowest heuristic f
    // and make a hash set to track visited states
		PriorityQueue<Node> pq = new PriorityQueue<>();
		pq.add(new Node(start, "", heuristic(start))); 
		Set<State> visited = new HashSet<>();

		// search loop
  while (!pq.isEmpty()) {
    Node cur = pq.poll(); // pop best node (lowest heuristic)
    if (isGoal(cur.state)) {
        return cur.path; // success condition
           }
    if (!visited.add(cur.state)) continue;  // skip if already explored

    for (Move mv : legalMoves(cur.state, mapData)) {
        State nxt = applyMove(cur.state, mv); // simulate move


        // Skip states that are either:
        // - deadlocked (cannot be solved)
        // - already visited
        if (isFailed(nxt, mapData)) continue;
        if (visited.contains(nxt)) continue;


        // Compute heuristic and add to priority queue
        int f2 = heuristic(nxt);
        pq.add(new Node(nxt, cur.path + mv.symbol, f2));
    }


    if(diediedie){
        return "";
      }
    }
    // If no path found, return empty string 
    return "";
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

  /** Check if current state satisfies the goal condition (all boxes on goals) */
  private boolean isGoal(State s) {
      for (Point b : s.boxes)
          if (!goals.contains(b)) return false;
      return true;
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

  // compute distance to nearest goal for every tile using bfs
  private void computeGoalDistances(char[][] mapData){
      int height = mapData.length; //counts how many rows
      int width = mapData[0].length; //counts how many columns
      distToGoal = new int[height][width];

      for (int i=0;i<height;i++){
          for(int j=0;j<width;j++){
              distToGoal[i][j] = MAX_DIST;
          }
      }

      Queue<Point> boxQueue = new ArrayDeque<>(); // A deque (double ended queue). used as a queue in these purposes

      for (Point goal: goals){
          distToGoal[goal.row][goal.col] = 0 ; // shortest distance from any goal to itself is 0
          boxQueue.add(goal);  // add goals to queue for the bfs search later
      }

      int [][] moves = {
              //{row,col}
              {-1,0}, // up
              {1,0},  // down
              {0,-1}, // left
              {0,1}  // right
      };

      //bfs search to find the distance from any tile to the goal. ignores boxes and treats them as tiles
      while(!boxQueue.isEmpty()){
          Point curr = boxQueue.poll(); //dequeue from the queue
          int dist = distToGoal[curr.row][curr.col];

          for(int[] move : moves){
              int neighborRow = curr.row + move[0];
              int neighborCol = curr.col + move[1];

              //walls are not passable
              if (isWall(neighborRow,neighborCol,mapData))
                  continue;
              //some dont have walls on borders, so as a safety measure dont explore past mapData array bounds
              if (neighborRow < 0 || neighborCol < 0 || neighborRow >= height || neighborCol >= width )
                  continue;
              //if we have a shorter path to the neighbor, update the distances array and add this tile to the queue
              if (distToGoal[neighborRow][neighborCol] > dist + 1){
                  distToGoal[neighborRow][neighborCol] = dist + 1;
                  boxQueue.add(new Point(neighborRow,neighborCol));
              }
          }
      }
  }
    //now that we have information about the distances from each tile to their nearest goals we can write a simple heuristic based off that
    //this heuristic is used to help the game choose which game state to work from, in other words we prioritize game states where boxes are close to goals
  private int heuristic(State s){
      int sum = 0;
      for (Point box : s.boxes){
          // similar logic to the computeGoalDistances function, but this time instead of ignoring out of bound entries we apply a penalty
          if (box.row < 0
                  || box.col < 0
                  || box.row >= distToGoal.length
                  || box.col >= distToGoal[0].length)
              sum+=10000;
          else{
              int dist = distToGoal[box.row][box.col];
              if (dist == MAX_DIST){
                  sum+=10000; // also penalize if distToGoal is max_dist, as this also means the box cannot reach the goal.
              }
              else{
                  sum+= dist; // else, the box can reach the goal!
              }
          }
      }
      return sum;
  }

  private void computeGoalDistancesAll(char[][] map){
  // function to be made
        }




    }


  //isDeadlocked is given a game state and determines if it's impossible to clear the stage by checking the surrounding 8 tiles on every box
  //due to the nature of this it doesn't 'predict' if the game would reach a softlock, but rather detects if the game is already in a softlock
  //in simple words it just checks if there's any boxes that can't be moved anymore and isn't on a goal
  private boolean isDeadlocked(State s, char[][] mapData){
        /*
        For each box do deadlock check
        - if box is on goal, do not deadlock check
        - else use deadlock patterns
         */
        //For all boxes

        for(Point b : s.boxes){
            //only deadlock check if box is not on goal
            if (!goals.contains(b)){
                int bRow = b.row;
                int bCol = b.col; // current position of box

                //represent the 3x3 area around the box (including box)
                //sRow should be  {bRow+1,bRow+1,bRow+1,bRow,bRow,bRow,bRow-1,bRow-1,bRow-1}
                //sCol should be  {bCol-1,bCol,bCol+1,bCol-1,bCol,bCol+1,bCol-1,bCol,bCol+1}
                //to get the coordinates of any area around the box, simply
                int[] sRow = {bRow+1,bRow+1,bRow+1,bRow,bRow,bRow,bRow-1,bRow-1,bRow-1};
                int[] sCol = {bCol-1,bCol,bCol+1,bCol-1,bCol,bCol+1,bCol-1,bCol,bCol+1};

                //now have arrays that check if the surrounding boxes are walls or boxes!

                boolean[] wallStatus = new boolean[9];
                boolean[] boxStatus  = new boolean[9];

                for(int i = 0; i<9;i++){
                    wallStatus[i] = isWall(sRow[i],sCol[i],mapData);
                    boxStatus[i]  = s.boxes.contains(new Point(sRow[i],sCol[i]));
                }

                //now we write every single deadlock pattern!
                //Pattern 1: box cornered by 2 walls
                if (
                        (wallStatus[1] && wallStatus[5])
                                || (wallStatus[5] && wallStatus[7])
                                || (wallStatus[7] && wallStatus[3])
                                || (wallStatus[3] && wallStatus[1])
                )
                    return true;
                //Pattern 2: adjacent boxes that are both adjacent to walls
                if(
                        (boxStatus[1] && wallStatus[2] && wallStatus[5]) //normal
                                || (boxStatus[1] && wallStatus[0] && wallStatus[3]) // flip
                                || (boxStatus[5] && wallStatus[7] && wallStatus[8]) // normal (rotated right)
                                || (boxStatus[5] && wallStatus[1] && wallStatus[2]) // flip
                                || (boxStatus[7] && wallStatus[3] && wallStatus[6])
                                || (boxStatus[7] && wallStatus[5] && wallStatus[8])
                                || (boxStatus[3] && wallStatus[0] && wallStatus[1])
                                || (boxStatus[3] && wallStatus[6] && wallStatus[7])
                )
                    return true;
                //Pattern 3: 3 boxes around a wall in a corner (similar to a 2x2 deadlock)
                if(
                        (boxStatus[1] && boxStatus[5] && wallStatus[2])
                                || (boxStatus[5] && boxStatus[7] && wallStatus[8])
                                || (boxStatus[7] && boxStatus[3] && wallStatus[6])
                                || (boxStatus[3] && boxStatus[1] && wallStatus[0])
                )
                    return true;
                //Pattern 4: 4 boxes. simply just 4 boxes all adjacent to each other. I like calling this a 2x2 deadlock
                if(
                        (boxStatus[1] && boxStatus[2] && boxStatus[5] )
                                || (boxStatus[5] && boxStatus[8] && boxStatus[7] )
                                || (boxStatus[7] && boxStatus[6] && boxStatus[3] )
                                || (boxStatus[3] && boxStatus[0] && boxStatus[1] )
                )
                    return true;
                //Pattern 5: 2 adjacent boxes each adjacent to walls on opposite sides (similar to pattern 2)
                if (
                        (boxStatus[1] && wallStatus[2] && wallStatus[3])
                                || (boxStatus[1] && wallStatus[0] && wallStatus[5])
                                || (boxStatus[5] && wallStatus[1] && wallStatus[8])
                                || (boxStatus[5] && wallStatus[2] && wallStatus[7])
                                || (boxStatus[7] && wallStatus[3] && wallStatus[8])
                                || (boxStatus[7] && wallStatus[5] && wallStatus[6])
                                || (boxStatus[3] && wallStatus[1] && wallStatus[6])
                                || (boxStatus[3] && wallStatus[0] && wallStatus[7])
                )
                    return true;
            }
        }
        return false;
    }

    
}
