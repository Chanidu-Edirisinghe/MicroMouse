const int sensorPin[] = {A0, A1, A2};
float distance[3];
const int AVERAGE_OF = 200;
const float MCU_VOLTAGE = 5.0;

int enA = 9;
int in1 = 4;
int in2 = 5;

int enB = 6;
int in3 = 7;
int in4 = 8;

float kp = 1.0;  // Proportional gain
float kd = 0.0;  // Derivative gain
float ki = 0.0; // Integral gain

float ePrevious = 0;
float eIntegral = 0;
long previousTime = 0;

const int MAZE_SIZE = 8;
int maze[MAZE_SIZE][MAZE_SIZE] = {0};  // 2D array to represent walls
int distances[MAZE_SIZE][MAZE_SIZE];   // 2D array to store distances

int currentX = 7;
int currentY = 0;
int currentOrientation = 4;

struct Cell {
  int x, y;
};

Cell start = { currentX, currentY };

const int targetCells[4][2] = {
    {3, 3}, {3, 4}, {4, 3}, {4, 4}
};

struct Node {
  Cell data;
  Node* next;
};

class Queue {
private:
  Node* front;
  Node* rear;

public:
  // Constructor
  Queue() : front(nullptr), rear(nullptr) {}

  // Destructor
  ~Queue() {
    while (!isEmpty()) {
      dequeue();
    }
  }

  // Function to check if the queue is empty
  bool isEmpty() {
    return front == nullptr;
  }

  // Function to add an element to the queue
  void enqueue(Cell c) {
    Node* newNode = new Node();
    newNode->data = c;
    newNode->next = nullptr;
    if (isEmpty()) {
      front = newNode;
      rear = newNode;
    } else {
      rear->next = newNode;
      rear = newNode;
    }
  }

  // Function to remove an element from the queue
  Cell dequeue() {
    Cell c = { -1, -1 }; // Default value for an empty queue
    if (!isEmpty()) {
      Node* temp = front;
      c = front->data;
      front = front->next;
      if (front == nullptr) {
        rear = nullptr;
      }
      delete temp;
    }
    return c;
  }

  // Function to get the front element
  Cell peek() {
    if (!isEmpty()) {
      return front->data;
    } else {
      return { -1, -1 }; // Default value for an empty queue
    }
  }
};

void turnRight() {
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  digitalWrite(in2, HIGH);
  digitalWrite(in1, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in3, LOW);
  delay(655);
  stop();
}

void turnLeft() {
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(655);
  stop();
}

void moveForward() {
  analogWrite(enB, 255);
  analogWrite(enA, 255);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in3, LOW);
  delay(1400);
  stop();
}

void stop() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

bool wallFront(){
  if (distance[2] > 6) {
    return false;
  }
  else{
    return true;
  }
}

bool wallRight(){
  if (distance[0] > 20) {
    return false;
  }
  else{
    return true;
  }
}

bool wallLeft(){
  if (distance[1] > 20) {
    return false;
  }
  else{
    return true;
  }
}

void floodFill() {
  Queue queue;

  // Initialize distances to a large number
  for (int i = 0; i < MAZE_SIZE; i++) {
    for (int j = 0; j < MAZE_SIZE; j++) {
      distances[i][j] = 255; // A large number representing infinity
    }
  }

  if (MAZE_SIZE % 2 == 0) {
    // Add all four central target cells to the queue
    int n = MAZE_SIZE/2;
    int targetCells[4][2] = {
      {n-1, n-1}, {n-1, n}, {n, n-1}, {n, n}
    };

    for (int i = 0; i < 4; i++) {
      int tx = targetCells[i][0];
      int ty = targetCells[i][1];
      distances[tx][ty] = 0;  // Distance to target cells is 0
      queue.enqueue({tx, ty});
    }
  }
  else{
    int n = MAZE_SIZE/2;
    distances[n][n] = 0;
    queue.enqueue({n, n});
  }
  
  
  while (!queue.isEmpty()) {
    Cell current = queue.dequeue();
    int currentDist = distances[current.x][current.y];
    
    if (current.x > 0 && !(maze[current.x][current.y] & (1 << 2)) && distances[current.x - 1][current.y] > currentDist + 1) {  // Up
      distances[current.x - 1][current.y] = currentDist + 1;
      queue.enqueue({current.x - 1, current.y});
    }
    if (current.x < MAZE_SIZE - 1 && !(maze[current.x][current.y] & (1 << 3)) && distances[current.x + 1][current.y] > currentDist + 1) {  // Down
      distances[current.x + 1][current.y] = currentDist + 1;
      queue.enqueue({current.x + 1, current.y});
    }
    if (current.y > 0 && !(maze[current.x][current.y] & (1 << 1)) && distances[current.x][current.y - 1] > currentDist + 1) {  // Left
      distances[current.x][current.y - 1] = currentDist + 1;
      queue.enqueue({current.x, current.y - 1});
    }
    if (current.y < MAZE_SIZE - 1 && !(maze[current.x][current.y] & (1 << 0)) && distances[current.x][current.y + 1] > currentDist + 1) {  // Right
      distances[current.x][current.y + 1] = currentDist + 1;
      queue.enqueue({current.x, current.y + 1});
    }
  }
}

// void PID(){
//   // convert the values to distances
//   int frontDistance = analogRead(frontSensorPin);
//   int leftDistance = analogRead(leftSensorPin);
//   int rightDistance = analogRead(rightSensorPin);

//   currentTime = millis();
//   float elapsedTime = (currentTime - previousTime) / 1000.0;
  
//   float errorLeft = setPoint - leftDistance;
//   float errorRight = setPoint - rightDistance;

//   float PoutLeft = Kp * errorLeft;
//   float PoutRight = Kp * errorRight;

//   float derivativeLeft = (errorLeft - previousErrorLeft) / elapsedTime;
//   float derivativeRight = (errorRight - previousErrorRight) / elapsedTime;
//   float DoutLeft = Kd * derivativeLeft;
//   float DoutRight = Kd * derivativeRight;

//   float PIDoutLeft = PoutLeft + DoutLeft;
//   float PIDoutRight = PoutRight + DoutRight;

//   int baseSpeed = 100;  
//   int leftMotorSpeed = baseSpeed + PIDoutLeft;
//   int rightMotorSpeed = baseSpeed + PIDoutRight;

//   leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
//   rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

//   analogWrite(leftMotorPin, leftMotorSpeed);
//   analogWrite(rightMotorPin, rightMotorSpeed);

//   previousErrorLeft = errorLeft;
//   previousErrorRight = errorRight;
//   previousTime = currentTime;

//   if (frontDistance < setPoint) {  
//     analogWrite(leftMotorPin, 0);
//     analogWrite(rightMotorPin, 0);
//   }

//   delay(50);
// }

void readFrontDistance(int sensor) {
  float voltage_temp_average = 0;
  for (int i = 0; i < AVERAGE_OF; i++) {
    int sensorValue = analogRead(sensorPin[sensor]);
    voltage_temp_average += sensorValue * (MCU_VOLTAGE / 1023.0);
  }
  voltage_temp_average /= AVERAGE_OF;

  distance[sensor] = 13 * pow(voltage_temp_average, -1);
}

void readDistance(int sensor) {
  float voltage_temp_average = 0;
  for (int i = 0; i < AVERAGE_OF; i++) {
    int sensorValue = analogRead(sensorPin[sensor]);
    voltage_temp_average += sensorValue * (MCU_VOLTAGE / 1023.0);
  }
  voltage_temp_average /= AVERAGE_OF;

  distance[sensor] = 34.6 + -69.5 * (voltage_temp_average) + 62.3 * pow(voltage_temp_average, 2) + -25.4 * pow(voltage_temp_average, 3) + 3.83 * pow(voltage_temp_average, 4);
}

void updateMaze(bool left, bool right, bool front, bool back, Cell currentPosition) {
    int x = currentPosition.x;
    int y = currentPosition.y;

    if (left) {
        if ((maze[x][y] & (1 << 1)) == 0) {
            maze[x][y] += 1 << 1;
        }
        if (y > 0 && (maze[x][y - 1] & (1)) == 0) {
            maze[x][y - 1] += 1;
        }
    }
    if (right) {
        if ((maze[x][y] & (1)) == 0) {
            maze[x][y] += 1;
        }
        if (y < MAZE_SIZE - 1 && (maze[x][y + 1] & (1 << 1)) == 0) {
            maze[x][y + 1] += 1 << 1;
        }
    }
    if (front) {
        if ((maze[x][y] & (1 << 2)) == 0) {
            maze[x][y] += 1 << 2;
        }
        if (x > 0 && (maze[x - 1][y] & (1 << 3)) == 0) {
            maze[x - 1][y] += 1 << 3;
        }
    }
    if (back) {
        if ((maze[x][y] & (1 << 3)) == 0) {
            maze[x][y] += 1 << 3;
        }
        if (x < MAZE_SIZE - 1 && (maze[x + 1][y] & (1 << 2)) == 0) {
            maze[x + 1][y] += 1 << 2;
        }
    }
}

void moveToNextPosition(int currentOrientation, int nextOrientation) {
    // Determine the direction to turn based on the orientation
    if (nextOrientation == 4) {  // Forward
        if (currentOrientation == 2) {  // Left
            turnRight();
        } else if (currentOrientation == 8) {  // Backward
            turnRight();
            turnRight();
        } else if (currentOrientation == 1) {  // Right
            turnLeft();
        }
    } 
    else if (nextOrientation == 1) {  // Right
        if (currentOrientation == 2) {  // Left
            turnRight();
            turnRight();
        } else if (currentOrientation == 4) {  // Forward
            turnRight();
        } else if (currentOrientation == 8) {  // Backward
            turnLeft();
        }
    } 
    else if (nextOrientation == 8) {  // Backward
        if (currentOrientation == 2) {  // Left
            turnLeft();
        } else if (currentOrientation == 4) {  // Forward
            turnRight();
            turnRight();
        } else if (currentOrientation == 1) {  // Right
            turnRight();
        }
    } 
    else if (nextOrientation == 2) {  // Left
        if (currentOrientation == 1) {  // Right
            turnRight();
            turnRight();
        } else if (currentOrientation == 4) {  // Forward
            turnLeft();
        } else if (currentOrientation == 8) {  // Backward
            turnRight();
        }
    }

    // Move forward to the next position
    moveForward();
}

void navigate(Cell currentPosition, int orientation) {
    // Define the direction vectors for forward, right, backward, left
    int directions[4][2] = {
        {0, 1},   // Right (1)
        {0, -1},  // Left (2)
        {-1, 0},  // Forward (4)
        {1, 0}    // Backward (8)
    };
    
    while (true) {
      readDistance(0);
      readDistance(1);
      readFrontDistance(2);
  
        // Check surroundings
      bool right = wallRight();
      bool left = wallLeft();
      bool front = wallFront();
      bool back = false;  // The robot cannot directly sense back
      Serial.println(right);
      Serial.println(left);
      Serial.println(front);
      // Adjust walls according to the current orientation
      bool adjustedFront, adjustedRight, adjustedLeft, adjustedBack;

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
              Serial.println("Unexpected orientation!");
              return;
        }

        // Update the maze with adjusted walls
        updateMaze(adjustedLeft, adjustedRight, adjustedFront, adjustedBack, currentPosition);
        Serial.println("Maze updated");
        // Recalculate distances with flood fill
        floodFill();
        // for (int i = 0; i < MAZE_SIZE; i++) {
        //     for (int j = 0; j < MAZE_SIZE; j++) {
        //         API::setText(j, MAZE_SIZE - 1 - i, String(distances[i][j]));
        //     }
        // }

        // Log the maze status for debugging
        // for (int i = 0; i < MAZE_SIZE; i++) {
        //     for (int j = 0; j < MAZE_SIZE; j++) {
        //         log1(maze[i][j]);
        //     }
        //     Serial.println();
        // }
        // Serial.println();

        // Find the next position to move to based on the distances
        int minDistance = 1000;
        Cell nextPosition = { -1, -1 };
        int nextOrientation = orientation;
        // Serial.println(currentPosition.x);
        // Serial.println(currentPosition.y);
        for (int i = 0; i < 4; i++) {
            int newX = currentPosition.x + directions[i][0];
            int newY = currentPosition.y + directions[i][1];
            Serial.println(newX);
            Serial.println(newY);
            if (newX >= 0 && newX < MAZE_SIZE && newY >= 0 && newY < MAZE_SIZE) {
                if (distances[newX][newY] < minDistance && (maze[currentPosition.x][currentPosition.y] & (1 << i)) == 0) {
                    minDistance = distances[newX][newY];
                    Serial.println("next Position updated");
                    nextPosition = { newX, newY };
                    nextOrientation = 1 << i;  // Update the orientation based on the movement direction
                }
            }
        }
        Serial.println(nextPosition.x);
        Serial.println(nextPosition.y);

        // If nextPosition is invalid, it means the robot is stuck
        if (nextPosition.x == -1 && nextPosition.y == -1) {
            Serial.println("Robot is stuck!");
            return;
        }

        // Move to the next position
        moveToNextPosition(orientation, nextOrientation);
        currentPosition.x = nextPosition.x;
        currentPosition.y = nextPosition.y;
        orientation = nextOrientation;

        // Check if the current position is a target cell
        for (int i = 0; i < sizeof(targetCells) / sizeof(targetCells[0]); i++) {
            if (currentPosition.x == targetCells[i][0] && currentPosition.y == targetCells[i][1]) {
                Serial.println("Robot has reached a target cell!");
                // Add led to show target reached
                return;
            }
        }
    }
}


void setup() {
  Serial.begin(9600);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  floodFill();
  for (int i = 0; i < MAZE_SIZE; i++) {
    for (int j = 0; j < MAZE_SIZE; j++) {
        Serial.print(distances[i][j]);
        Serial.print(" ");
    }
    Serial.println("");
  }

  navigate(start, currentOrientation);
  // for (int row = 0; row < MAZE_SIZE; row++) {
  //   for (int col = 0; col < MAZE_SIZE; col++) {
  //     Serial.print(distances[row][col]);
  //     if(distances[row][col] > 9){
  //       Serial.print(" "); 
  //     }
  //     else{
  //       Serial.print("  ");
  //     }
      
  //   }
  //   Serial.println(); // Move to the next line after printing all columns in the row
  // }
}

void loop() {
  
  
}

