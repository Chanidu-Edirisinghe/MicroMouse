const int frontSensorPin = A0;
const int leftSensorPin = A1;
const int rightSensorPin = A2;

const int leftMotorPin = 9; 
const int rightMotorPin = 10;

float Kp = 0;
float Kd = 0;

float previousErrorLeft = 0;
float previousErrorRight = 0;
float setPoint = 4.6;

unsigned long previousTime = 0;
unsigned long currentTime;

const int MAZE_SIZE = 16;
int maze[MAZE_SIZE][MAZE_SIZE] = {0};  // 2D array to represent walls
int distances[MAZE_SIZE][MAZE_SIZE];   // 2D array to store distances

int currentX = 0;
int currentY = 0;

struct Cell {
  int x, y;
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

void PID(){
  // convert the values to distances
  int frontDistance = analogRead(frontSensorPin);
  int leftDistance = analogRead(leftSensorPin);
  int rightDistance = analogRead(rightSensorPin);

  currentTime = millis();
  float elapsedTime = (currentTime - previousTime) / 1000.0;
  
  float errorLeft = setPoint - leftDistance;
  float errorRight = setPoint - rightDistance;

  float PoutLeft = Kp * errorLeft;
  float PoutRight = Kp * errorRight;

  float derivativeLeft = (errorLeft - previousErrorLeft) / elapsedTime;
  float derivativeRight = (errorRight - previousErrorRight) / elapsedTime;
  float DoutLeft = Kd * derivativeLeft;
  float DoutRight = Kd * derivativeRight;

  float PIDoutLeft = PoutLeft + DoutLeft;
  float PIDoutRight = PoutRight + DoutRight;

  int baseSpeed = 100;  
  int leftMotorSpeed = baseSpeed + PIDoutLeft;
  int rightMotorSpeed = baseSpeed + PIDoutRight;

  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  analogWrite(leftMotorPin, leftMotorSpeed);
  analogWrite(rightMotorPin, rightMotorSpeed);

  previousErrorLeft = errorLeft;
  previousErrorRight = errorRight;
  previousTime = currentTime;

  if (frontDistance < setPoint) {  
    analogWrite(leftMotorPin, 0);
    analogWrite(rightMotorPin, 0);
  }

  delay(50);
}


void setup() {
  Serial.begin(9600);

  pinMode(leftMotorPin, OUTPUT);
  pinMode(rightMotorPin, OUTPUT);

  floodFill();
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
  PID();
  
}

