

/*
//Глобальная переменная должна отмечаться ключевым словом _global_
int mainMode _global_;

//Флеш-переменная должна отмечаться ключевым словом _flash_
float sideDistorsion _flash_;

//В качестве переменных могут быть также массивы ...
int stepPositions[10] _global_;
float leftK[5] _flash_;

//... и структуры
struct Point {
  float x;
  float y;
};

Point origin _global_;

Point offset _flash_;
*/

int jump_mode _global_; // 0 - jump at spot, 1-jump forward, 2- backward, 3 - left, 4 - right, 5 - rotate CCW, 6 - rotate CW, 
						// Power (tens): 0 - 100%, 10 - 10%, 20 - 20%, 30- 30%, etc.  
						// Number of jumps (hundreds): 100 - 1 time, 200 - 2 times, etc.
						// Example: 101 - jump forward one time 100%, 253 - jump left 2 times with 50% of power
int robot_Serial_Number _global_;
int splits_Mode _global_;
int lowest_ntc _global_;