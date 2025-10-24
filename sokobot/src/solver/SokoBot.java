package solver;

import java.util.HashSet;
import java.util.Set;

import java.util.Objects;
import java.util.PriorityQueue;
import java.util.ArrayDeque;
import java.util.Queue;
import java.util.Map;
import java.util.HashMap;
import java.util.List;
import java.util.ArrayList;

public class SokoBot {
    //Properties
    private final Set<Point> goals = new HashSet<>();
    private int[][] distToGoal;
    private final int MAX_DIST = Integer.MAX_VALUE; // an integer used when initializing distances in distToGoal.

    private int[][][] pushesToGoal; //an array of distance (in number of pushes) to get to a goal. considers deadlocks
    //stored values calculated using the second heuristic function
    private final Map<State, Integer> heuristicDP = new HashMap<>();

    //these are the 4 possible directions the player can move.
    //note that we follow the representation of the map as an array,
    // so the 'top left' would be [0][0]
    // the 'bottom right' would be [height][width]
    private static final int[][] DIRECTIONS = {
            {-1, 0, 'u'},  // move up    (row - 1)
            { 1, 0, 'd'},  // move down  (row + 1)
            { 0,-1, 'l'},  // move left  (col - 1)
            { 0, 1, 'r'}   // move right (col + 1)
    };



    //implementation of GBFS
    public String solveSokobanPuzzle(int width, int height, char[][] mapData, char[][] itemsData) {

        findGoals(mapData, itemsData, width, height);

        //do computations for the 2 heuristics
        computeGoalDistances(mapData);
        computeGoalPushes(mapData);

        //get initial state for bfs queue
        State start = parseInitialState(itemsData, width, height);

        PriorityQueue<Node> pq = new PriorityQueue<>();
        pq.add(new Node(start, "", heuristic(start))); // initial node

        Set<State> visited = new HashSet<>();


        while (!pq.isEmpty()) {
            Node cur = pq.poll();
            //^ this is the best state according to heuristics and we pop this
            if (isGoal(cur.state)) {
                //end solving if we reached the goal
                return cur.path; // success condition
            }
            if (!visited.add(cur.state)) continue;  //to avoid repetition

            //try all directions
            for (Move mv : possibleMoves(cur.state, mapData)) {
                State nxt = applyMove(cur.state, mv, mapData); // simulate move

                // we skip states that are either:
                // - deadlocked (cannot be solved)
                // - already visited
                if (isDeadlocked(nxt, mapData)) continue;
                if (visited.contains(nxt)) continue;

                //get heuristic value of the possible states
                int value = heuristic(nxt);
                if (!nxt.isTunnel){
                    pq.add(new Node(nxt, cur.path + mv.symbol, value));
                }else{
                    pq.add(new Node(nxt, cur.path + nxt.tunnelPath, value));
                }
            }
        }
        System.out.println("failed to find goal");
        return "";
    }

    static class State {
        final int playerRow, playerCol; // player position
        final Set<Point> boxes;   // box positions

        boolean isTunnel = false;
        String tunnelPath = "";

        State(int playerRow, int playerCol, Set<Point> boxes) {
            this.playerRow = playerRow;
            this.playerCol = playerCol;
            this.boxes = boxes;
        }

        @Override public boolean equals(Object o) {
            if (!(o instanceof State)) return false;
            State s = (State)o;
            return playerRow == s.playerRow && playerCol == s.playerCol && boxes.equals(s.boxes);
        }

        @Override public int hashCode() {
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

        @Override public int compareTo(Node o) {
            return Integer.compare(this.f, o.f);
        }
    }
    /** Represents a single movement (up, down, left, right, with/without push) */
    static class Move {
        final int dRow, dCol;   // movement offset
        final char symbol;  // movement symbol ('u','d','l','r')
        final boolean push; // true if pushing a box

        Move(int dRow, int dCol, char sym, boolean push) {
            this.dRow = dRow;
            this.dCol = dCol;
            this.symbol = sym;
            this.push = push;
        }
    }

    /** Represents a grid coordinate (x=row, y=col) */
    static class Point {
        final int row, col;
        Point(int row, int col) {
            this.row = row; this.col = col;
        }

        @Override public boolean equals(Object o) {
            if (!(o instanceof Point)) return false;
            Point p = (Point)o;
            return row == p.row && col == p.col;
        }

        @Override public int hashCode() {
            return Objects.hash(row, col);
        }
    }


    //FUNCTIONS ON OUT ARE FOR INITIALIZING THE GAME


    //map out points of the goals
    private void findGoals(char[][] mapData, char[][] itemsData, int width, int height){
        for (int i = 0; i < height; i++){
            for (int j = 0; j < width; j++){
                if (mapData[i][j] == '.'){ //Goals
                    goals.add(new Point(i, j));
                }
                if (itemsData[i][j] == '*' || itemsData[i][j] == '+'){ //Boxes or Players on goals
                    goals.add(new Point(i, j));
                }
            }
        }
    }


    private State parseInitialState(char[][] itemsData, int width, int height){
        int row = 0, col = 0; //player pos
        Set<Point> box_pos = new HashSet<>(); //boxes pos

        for (int i = 0; i < height; i++){
            for (int j = 0; j < width; j++){
                if (itemsData[i][j] == '@' || itemsData[i][j] == '+'){ //Find player
                    row = i;
                    col = j;
                }
                if (itemsData[i][j] == '$' || itemsData[i][j] == '*'){ //Find boxes
                    box_pos.add(new Point(i, j));
                }
            }
        }
        return new State(row, col, box_pos);
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

        //bfs search to find the distance from any tile to the goal. ignores boxes and treats them as tiles
        while(!boxQueue.isEmpty()){
            Point curr = boxQueue.poll(); //dequeue from the queue
            int dist = distToGoal[curr.row][curr.col];

            for(int[] move : DIRECTIONS){
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


    //this is similar to commputeGoalDistances but instead of distances it computes PUSHES it takes to get to a goal
    private void computeGoalPushes(char[][] map) {

        int gcount = goals.size();
        int h = map.length;
        int w = map[0].length;
        pushesToGoal = new int [gcount][h][w];

        //inital fill
        for(int i = 0; i <gcount ; i++){
            for(int j = 0; j <h ; j++){
                for(int k = 0; k <w ; k++){
                    pushesToGoal[i][j][k] = Integer.MAX_VALUE;
                }
            }
        }

        int currGoal = 0;
        for (Point g : goals) {
            //do bfs for each goal
            Queue<Point> q = new ArrayDeque<>(); // your queue!

            // costs nothing if the box is already at goal
            pushesToGoal[currGoal][g.row][g.col] = 0;
            q.add(g);

            while (!q.isEmpty()) {
                Point currentBoxPos = q.poll();
                int row = currentBoxPos.row; // Row
                int col = currentBoxPos.col; // Col
                int dist = pushesToGoal[currGoal][row][col];

                //try all directions
                for (int[] dir : DIRECTIONS) {

                    //we are simulating "reverse" pushes so we call where we expand from here as the  "previous" state
                    int prevBoxRow = row + dir[0];
                    int prevBoxCol = col + dir[1];

                    //the player also need to be behind the box, so we call this the push spot
                    int pushSpotX = row + (2 * dir[0]);
                    int pushSpotY = col + (2 * dir[1]);


                    //is the box still within bounds?
                    if (prevBoxRow < 0 || prevBoxCol < 0 || prevBoxRow >= h || prevBoxCol >= w||map[prevBoxRow][prevBoxCol] == '#') continue;

                    //is the player still within bounds?
                    if (pushSpotX < 0 || pushSpotY < 0 || pushSpotX >= h || pushSpotY >= w || map[pushSpotX][pushSpotY] == '#') continue;

                    //acts similar to a minimum function, the distance we found is smaller, if yes, then update it
                    if (pushesToGoal[currGoal][prevBoxRow][prevBoxCol] > dist + 1) {
                        pushesToGoal[currGoal][prevBoxRow][prevBoxCol] = dist + 1;
                        q.add(new Point(prevBoxRow, prevBoxCol));
                    }
                }
            }
            currGoal++;//go to the next goal
        }
    }



    //THESE FUNCTIONS ARE USED FOR CHECKING CONDITIONS

    private boolean isWall(int row, int col, char[][] mapData){
        return mapData[row][col] == '#'; //add bounds checking if needed
    }

    //check if all boxes are contained by a goal
    private boolean isGoal(State s) {
        for (Point b : s.boxes)
            if (!goals.contains(b)) return false;
        return true;
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




    //THIS IS OUR HEURISTIC
    private int heuristic(State s) {
        int sum = 0;
        int box_count = s.boxes.size();
        int goal_count = goals.size();
        Integer storedHeuristic = heuristicDP.get(s);
        if (storedHeuristic != null) {
            return storedHeuristic; // done to avoid recalculation for the O(!n) heuristic
        }

        //PICKING WHICH HEURISTIC TO USE

        //Case 1: Simple heuristic
        //this heuristic is used to help the game choose which game state to work from, in other words we prioritize game states where boxes are close to goals
        if (box_count <= 6) {
            for (Point box : s.boxes) {
                // similar logic to the computeGoalDistances function, but this time instead of ignoring out of bound entries we apply a penalty
                if (box.row < 0
                        || box.col < 0
                        || box.row >= distToGoal.length
                        || box.col >= distToGoal[0].length)
                    sum += 10000;
                else {
                    int dist = distToGoal[box.row][box.col];
                    if (dist == MAX_DIST) {
                        sum += 10000; // also penalize if distToGoal is max_dist, as this also means the box cannot reach the goal.
                    } else {
                        sum += dist; // else, the box can reach the goal!
                    }
                }
            }
        } else { // Case 2: Minimum Matching

            //Create a cost matrix (what is the cost to get box n to goal n?)
            int[][] costMatrix = new int[box_count][box_count];
            int i = 0;
            for (Point b : s.boxes) {
                int brow = b.row;
                int bcol = b.col;
                for (int j=0; j<goal_count;j++) {

                    costMatrix[i][j] = pushesToGoal[j][brow][bcol];

                    if (costMatrix[i][j] == Integer.MAX_VALUE) {
                        costMatrix[i][j] = 100000; // cost matrix is related to heuristic, so this tells the algorithm to avoid states with boxes that cant reach goal
                    }
                }
                i++;
            }

            boolean[] usedGoals = new boolean[goal_count]; // number of goals = boxes

            int minimumMatch = bruteMinimumMatch(0, costMatrix, usedGoals);
            heuristicDP.put(s, minimumMatch);
            sum = minimumMatch;
        }
        return sum;
    }

    //this will brute force all n! combinations for n number of boxes and goals.
    //Solves the assignment problem with brute force!
    //the sokoban wiki actually says that there's a n^3 algorithm for this, but it's a bit too complex in my opinion so we stil with this while it works!
    //so yes this is a recursive function
    //This works by recursively doing assignments, like box 0 assigned to goal 0, then 1 to 1, all until box n is passed and the recursion goes back up
    private int bruteMinimumMatch(int currentBox, int[][] costMatrix, boolean[] usedGoals) {
        int N = costMatrix.length;

        //we follow 0-indexing so this means that there are no more boxes to assign
        if (currentBox == N) {
            return 0;
        }

        //initialize it as this
        int minCost = Integer.MAX_VALUE;

        //for loop to assign the current box to different goals
        for (int goalIndex = 0; goalIndex < N; goalIndex++) {

            //cannot take goals that have boxes
            if (!usedGoals[goalIndex]) {
                usedGoals[goalIndex] = true;

                int currCost = costMatrix[currentBox][goalIndex];

                //if the cost is greater than this, this is DEFINITELY not a minimum.
                if (currCost >= 100000) {
                    usedGoals[goalIndex] = false; //we mark off this combination of box and goal
                    continue;
                }

                //this is added to the current cost since we're looking for the sums of all goal box assignments
                //we call minimum match again but for the next box, which will have less choices
                int everythingElse = bruteMinimumMatch(currentBox + 1, costMatrix, usedGoals);

                // aonce again, we avoid values that are not minumums
                if (everythingElse != Integer.MAX_VALUE) {
                    //add it all together. once you get here everythingElse is a set of sums of minimum costs
                    int totalCost = currCost + everythingElse;
                    minCost = Math.min(minCost, totalCost);
                    //minimum is updated in the highest recursion stack for n times
                }

                //unmark the goal so that future boxes can use it
                usedGoals[goalIndex] = false;
            }
        }

        return minCost;
    }





    //THESE FUNCTIONS ARE FOR SIMULATION OF MOVES
    private List<Move> possibleMoves(State s, char[][] map) {
        //possible moves
        List<Move> possMoves = new ArrayList<>();

        for (int[] d : DIRECTIONS) {
            //find where the player would be after trying that move
            int newRow = s.playerRow + d[0];
            int newCol = s.playerCol + d[1];
            Point next = new Point(newRow, newCol);

            // is the player in a wall? not possible
            if (isWall(newRow, newCol, map)) continue;

            // is the player in a box? then check if that box can be pushed
            if (s.boxes.contains(next)) {
                int boxRow = newRow + d[0];
                int boxCol = newCol + d[1];
                Point dest = new Point(boxRow, boxCol);

                //if it cant be pushed, then not a possible move
                if (isWall(boxRow, boxCol, map) || s.boxes.contains(dest)) continue;

                //otherwise, lets add it to the possible moves!
                //note that the moves also track if it was pushing a box or not
                possMoves.add(new Move(d[0], d[1], (char)d[2], true));
            } else {
                //is the player in a valid spot? then goods!
                possMoves.add(new Move(d[0], d[1], (char)d[2], false));
            }
        }
        return possMoves;
    }

    //this function simulates a move and returns the resulting state
    //this also considers tunnelling
    private State applyMove(State s, Move move, char[][] map) {

        //try
        int playerRowAfterMove = s.playerRow + move.dRow;
        int playerColAfterMove = s.playerCol + move.dCol;
        Set<Point> newBoxes = new HashSet<>(s.boxes);

        if (move.push) {
            Point from = new Point(playerRowAfterMove, playerColAfterMove);
            Point to = new Point(playerRowAfterMove + move.dRow, playerColAfterMove + move.dCol);
            newBoxes.remove(from);
            newBoxes.add(to);
        }


        //TUNNELLING LOGIC
        State currentState = new State(playerRowAfterMove, playerColAfterMove, newBoxes);
        StringBuilder sb = new StringBuilder();
        sb.append(move.symbol);

        //before we try to tunnel, check if we already goaled. if yes, then no need to tunnel
        if (isGoal(currentState)) {
            currentState.isTunnel = false;
            currentState.tunnelPath = sb.toString();
            return currentState;
        }

        //now we try to tunnel
        Move nextMove;
        do {
            char lastMoveMade = sb.charAt(sb.length() - 1); //this is the last move we made
            nextMove = keepMoving(currentState, lastMoveMade, map);
            //if a move is returned, that means that the player is still in a tunnel


            if (nextMove != null) {
                sb.append(nextMove.symbol);
                int nextPx = currentState.playerRow + nextMove.dRow;
                int nextPy = currentState.playerCol + nextMove.dCol;
                Set<Point> nextBoxes = new HashSet<>(currentState.boxes);
                currentState = new State(nextPx, nextPy, nextBoxes); //update the state

                //we try to simulate the player moving deeper into the tunnel

                //if the player goals by going into the tunnel, then we stop there
                if (isGoal(currentState)) {
                    currentState.isTunnel = true;
                    currentState.tunnelPath = sb.toString();
                    return currentState;
                }
            }
        } while (nextMove != null);


        //if we wrote something into the tunnel path, then that means we tunnelled. this lets the GBFS loop know that we tunnelled
        currentState.isTunnel = (sb.length() > 1);
        currentState.tunnelPath = sb.toString();
        return currentState;
    }

    //Tries to keep moving the player in a tunnel.
    private Move keepMoving(State s, char mv, char[][] map){


        List<Move> whereCanGo = possibleMoves(s, map);

        //keep moving will very much just try to keep moving, no logic for pushes themselves so we return null and leave box logic to some other function
        for (Move m : whereCanGo) {
            if (m.push) {
                return null;
            }
        }


        if (whereCanGo.size() == 2) {
            //either go back to whence you came or find what's out there.
            for (Move m : whereCanGo){
                if (!isReverseMove(m.symbol, mv)) {
                    return m; // go deeper.
                }
            }
        }
        //if there's 1,3,or 4 possible moves to be made, it's not a tunnel. well for 1 possible move it's usually the the dead end of a tunnel
        return null;
    }

    //helper for keepMoving
    private boolean isReverseMove(char to, char from) {
        if (to == 'u' && from == 'd') return true;
        if (to == 'd' && from == 'u') return true;
        if (to == 'l' && from == 'r') return true;
        if (to == 'r' && from == 'l') return true;
        return false;
    }

}





