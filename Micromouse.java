package src;
import src.API;
import java.util.LinkedList;
import java.util.Queue;
class Cell {
    public int x, y;

    public Cell(int x, int y) {
        this.x = x;
        this.y = y;
    }
}

public class Micromouse {

    public static final int MAZE_SIZE = 16;
    public static int[][] maze = new int[MAZE_SIZE][MAZE_SIZE];  // 2D array to represent walls
    public static int[][] distances = new int[MAZE_SIZE][MAZE_SIZE];  // 2D array to store distances
    // Define target cells
    public static int[][] targetCells = {
        {7, 7}, {7, 8}, {8, 7}, {8, 8}
    };

    private static void log(String text) {
        System.err.println(text);
    }

    private static void log1(int num) {
        System.err.printf("%3d", num);
    }

    public static void main(String[] args) {
        Micromouse.log("Running...");

        API.setColor(0, 0, 'G');
        //API.setText(0, 0, "abc");

        // Initialize distances to a large number
        for (int i = 0; i < MAZE_SIZE; i++) {
            for (int j = 0; j < MAZE_SIZE; j++) {
                distances[i][j] = 255; // A large number representing infinity
            }
        }

        // for (int i = 0; i < MAZE_SIZE; i++) {
        //     for (int j = 0; j < MAZE_SIZE; j++) {
        //          API.setText(j, MAZE_SIZE-1-i, String.valueOf(distances[i][j]));
        //     }
        // }

        Cell currentPosition = new Cell(15, 0);
        int orientation = 4;

        floodFill();
        for (int i = 0; i < MAZE_SIZE; i++) {
            for (int j = 0; j < MAZE_SIZE; j++) {
                 API.setText(j, MAZE_SIZE-1-i, String.valueOf(distances[j][i]));
            }
        }
        navigate(currentPosition, orientation);

    }


    private static void navigate(Cell currentPosition, int orientation) {
        // Define the direction vectors for forward, right, backward, left
        int[][] directions = {
            {0, 1},   // Right (1)
            {0, -1},  // Left (2)
            {-1, 0},  // Forward (4)
            {1, 0}    // Backward (8)
        };

        while (true) {
            // Check surroundings
            boolean right = API.wallRight();
            boolean left = API.wallLeft();
            boolean front = API.wallFront();
            boolean back = false;  // The robot cannot directly sense back

            // Adjust walls according to the current orientation
            boolean adjustedFront, adjustedRight, adjustedLeft, adjustedBack;

            switch (orientation) {
                case 4: // Facing forward
                    adjustedFront = front;
                    adjustedRight = right;
                    adjustedLeft = left;
                    adjustedBack = back;
                    break;
                case 1: // Facing right
                    adjustedFront = left;
                    adjustedRight = front;
                    adjustedLeft = back;
                    adjustedBack = right;
                    break;
                case 8: // Facing backward
                    adjustedFront = back;
                    adjustedRight = left;
                    adjustedLeft = right;
                    adjustedBack = front;
                    break;
                case 2: // Facing left
                    adjustedFront = right;
                    adjustedRight = back;
                    adjustedLeft = front;
                    adjustedBack = left;
                    break;
                default:
                    throw new IllegalStateException("Unexpected value: " + orientation);
            }

            // Update the maze with adjusted walls
            updateMaze(adjustedLeft, adjustedRight, adjustedFront, adjustedBack, currentPosition);

            // Recalculate distances with flood fill
            floodFill();
            for (int i = 0; i < MAZE_SIZE; i++) {
                for (int j = 0; j < MAZE_SIZE; j++) {
                     API.setText(j, MAZE_SIZE-1-i, String.valueOf(distances[i][j]));
                }
            }
            
            for (int i = 0; i < MAZE_SIZE; i++) {
                for (int j = 0; j < MAZE_SIZE; j++) {
                    Micromouse.log1(maze[i][j]);
                }
                Micromouse.log("");
            }
            Micromouse.log("");
            

            // Find the next position to move to based on the distances
            int minDistance = Integer.MAX_VALUE;
            Cell nextPosition = null;
            int nextOrientation = orientation;

            for (int i = 0; i < 4; i++) {
                int newX = currentPosition.x + directions[i][0];
                int newY = currentPosition.y + directions[i][1];

                if (newX >= 0 && newX < MAZE_SIZE && newY >= 0 && newY < MAZE_SIZE) {
                    if (distances[newX][newY] < minDistance && (maze[currentPosition.x][currentPosition.y] & (1 << i)) == 0) {
                        minDistance = distances[newX][newY];
                        nextPosition = new Cell(newX, newY);
                        nextOrientation = 1 << i;  // Update the orientation based on the movement direction
                    }
                }
            }

            // If nextPosition is null, it means the robot is stuck
            if (nextPosition == null) {
                System.out.println("Robot is stuck!");
                return;
            }

            // Move to the next position
            moveToNextPosition(orientation, nextOrientation);
            currentPosition.x = nextPosition.x;
            currentPosition.y = nextPosition.y;
            orientation = nextOrientation;

            // Check if the current position is a target cell
            for (int[] targetCell : targetCells) {
                if (currentPosition.x == targetCell[0] && currentPosition.y == targetCell[1]) {
                    System.out.println("Robot has reached a target cell!");
                    return;
                }
            }

        }
    }

    private static void moveToNextPosition(int currentOrientation, int nextOrientation) {
        // Determine the direction to turn based on the orientation
        if (nextOrientation == 4) {  // Forward
            if (currentOrientation == 2) {  // Left
                API.turnRight();
            } else if (currentOrientation == 8) {  // Backward
                API.turnRight();
                API.turnRight();
            } else if (currentOrientation == 1) {  // Right
                API.turnLeft();
            }
        } else if (nextOrientation == 1) {  // Right
            if (currentOrientation == 2) {  // Left
                API.turnRight();
                API.turnRight();
            } else if (currentOrientation == 4) {  // Forward
                API.turnRight();
            } else if (currentOrientation == 8) {  // Backward
                API.turnLeft();
            }
        } else if (nextOrientation == 8) {  // Backward
            if (currentOrientation == 2) {  // Left
                API.turnLeft();
            } else if (currentOrientation == 4) {  // Forward
                API.turnRight();
                API.turnRight();
            } else if (currentOrientation == 1) {  // Right
                API.turnRight();
            }
        } else if (nextOrientation == 2) {  // Left
            if (currentOrientation == 1) {  // Right
                API.turnRight();
                API.turnRight();
            } else if (currentOrientation == 4) {  // Forward
                API.turnLeft();
            } else if (currentOrientation == 8) {  // Backward
                API.turnRight();
            }
        }

        // Move forward to the next position
        API.moveForward();
    }

    private static void updateMaze(boolean left, boolean right, boolean front, boolean back, Cell currentPosition){
        int x = currentPosition.x;
        int y = currentPosition.y;

        // Set walls for the current cell and the adjacent cells
        if (left) {
            if ((maze[x][y] & (1 << 1)) == 0) {
                maze[x][y] += 1 << 1;
            }
            if (y > 0 && (maze[x][y - 1] & (1)) == 0) {
                maze[x][y - 1] += 1;
            }
            API.setWall(y, MAZE_SIZE - 1 - x, 'w');
        }
        if (right) {
            if ((maze[x][y] & (1)) == 0) {
                maze[x][y] += 1;
            }
            if (y < MAZE_SIZE - 1 && (maze[x][y + 1] & (1 << 1)) == 0) {
                maze[x][y + 1] += 1 << 1;
            }
            API.setWall(y, MAZE_SIZE - 1 - x, 'e');
        }
        if (front) {
            if ((maze[x][y] & (1 << 2)) == 0) {
                maze[x][y] += 1 << 2;
            }
            if (x > 0 && (maze[x - 1][y] & (1 << 3)) == 0) {
                maze[x - 1][y] += 1 << 3;
            }
            API.setWall(y, MAZE_SIZE - 1 - x, 'n');
        }
        if (back) {
            if ((maze[x][y] & (1 << 3)) == 0) {
                maze[x][y] += 1 << 3;
            }
            if (x < MAZE_SIZE - 1 && (maze[x + 1][y] & (1 << 2)) == 0) {
                maze[x + 1][y] += 1 << 2;
            }
            API.setWall(y, MAZE_SIZE - 1 - x, 's');
        }
    }


    private static void floodFill(){

        // Initialize the queue
        Queue<Cell> queue = new LinkedList<>();

        for (int i = 0; i < MAZE_SIZE; i++) {
            for (int j = 0; j < MAZE_SIZE; j++) {
                distances[i][j] = 255; // A large number representing infinity
            }
        }

        // Add all four central target cells to the queue
        for (int[] targetCell : targetCells) {
            int tx = targetCell[0];
            int ty = targetCell[1];
            distances[tx][ty] = 0;  // Distance to target cells is 0
            queue.add(new Cell(tx, ty));
        }


        // Process the queue
        while (!queue.isEmpty()) {
            Cell current = queue.poll();
            int currentDist = distances[current.x][current.y];

            // Up
            if (current.x > 0 && (maze[current.x][current.y] & (1 << 2)) == 0 && distances[current.x - 1][current.y] > currentDist + 1) {
                distances[current.x - 1][current.y] = currentDist + 1;
                queue.add(new Cell(current.x - 1, current.y));
            }
            // Down
            if (current.x < MAZE_SIZE - 1 && (maze[current.x][current.y] & (1 << 3)) == 0 && distances[current.x + 1][current.y] > currentDist + 1) {
                distances[current.x + 1][current.y] = currentDist + 1;
                queue.add(new Cell(current.x + 1, current.y));
            }
            // Left
            if (current.y > 0 && (maze[current.x][current.y] & (1 << 1)) == 0 && distances[current.x][current.y - 1] > currentDist + 1) {
                distances[current.x][current.y - 1] = currentDist + 1;
                queue.add(new Cell(current.x, current.y - 1));
            }
            // Right
            if (current.y < MAZE_SIZE - 1 && (maze[current.x][current.y] & (1)) == 0 && distances[current.x][current.y + 1] > currentDist + 1) {
                distances[current.x][current.y + 1] = currentDist + 1;
                queue.add(new Cell(current.x, current.y + 1));
            }
        }
    }

    // private static void floodFill(int[][] targetCells, Queue<Cell> queue){
    //     // Clear the queue before starting
    //     queue.clear();

    //     // for (int i = 0; i < MAZE_SIZE; i++) {
    //     //     for (int j = 0; j < MAZE_SIZE; j++) {
    //     //         distances[i][j] = 255; // A large number representing infinity
    //     //     }
    //     // }
    
    //     // Add all four central target cells to the queue
    //     for (int[] targetCell : targetCells) {
    //         int tx = targetCell[0];
    //         int ty = targetCell[1];
    //         distances[tx][ty] = 0;  // Distance to target cells is 0
    //         queue.add(new Cell(tx, ty));
    //     }
    
    //     // Process the queue
    //     while (!queue.isEmpty()) {
    //         Cell current = queue.poll();
    //         int currentDist = distances[current.x][current.y];
    
    //         // Up
    //         if (current.x > 0 && (maze[current.x][current.y] & (1 << 2)) == 0 && distances[current.x - 1][current.y] > currentDist + 1) {
    //             distances[current.x - 1][current.y] = currentDist + 1;
    //             queue.add(new Cell(current.x - 1, current.y));
    //         }
    //         // Down
    //         if (current.x < MAZE_SIZE - 1 && (maze[current.x][current.y] & (1 << 3)) == 0 && distances[current.x + 1][current.y] > currentDist + 1) {
    //             distances[current.x + 1][current.y] = currentDist + 1;
    //             queue.add(new Cell(current.x + 1, current.y));
    //         }
    //         // Left
    //         if (current.y > 0 && (maze[current.x][current.y] & (1 << 1)) == 0 && distances[current.x][current.y - 1] > currentDist + 1) {
    //             distances[current.x][current.y - 1] = currentDist + 1;
    //             queue.add(new Cell(current.x, current.y - 1));
    //         }
    //         // Right
    //         if (current.y < MAZE_SIZE - 1 && (maze[current.x][current.y] & (1)) == 0 && distances[current.x][current.y + 1] > currentDist + 1) {
    //             distances[current.x][current.y + 1] = currentDist + 1;
    //             queue.add(new Cell(current.x, current.y + 1));
    //         }
    //     }
    // }
    
}
