// 低八度音符
#define note_L1       3822  // C
#define note_L1_high  3608  // C#
#define note_L2       3405  // D
#define note_L2_high  3214  // D#
#define note_L3       3034  // E
#define note_L4       2863  // F
#define note_L4_high  2703  // F#
#define note_L5       2551  // G
#define note_L5_high  2408  // G#
#define note_L6       2272  // A
#define note_L6_high  2145  // A#
#define note_L7       2025  // B

// 中八度音符
#define note_M1       1911  // C
#define note_M1_high  1804  // C#
#define note_M2       1703  // D
#define note_M2_high  1607  // D#
#define note_M3       1517  // E
#define note_M4       1432  // F
#define note_M4_high  1351  // F#
#define note_M5       1276  // G
#define note_M5_high  1204  // G#
#define note_M6       1136  // A
#define note_M6_high  1073  // A#
#define note_M7       1012  // B

// 高八度音符
#define note_H1       956   // C
#define note_H1_high  902   // C#
#define note_H2       851   // D
#define note_H2_high  804   // D#
#define note_H3       758   // E
#define note_H4       716   // F
#define note_H4_high  676   // F#
#define note_H5       638   // G
#define note_H5_high  602   // G#
#define note_H6       568   // A
#define note_H6_high  536   // A#
#define note_H7       506   // B

#define note_REST  0       // 休止符

const int hetangyeushe_melody[] = {
	note_M1,note_M1,note_L6,note_L5,note_L6,note_M1,note_M1,note_M2,note_M3,note_REST,
	
	note_M2,note_M2,note_M1,note_M2,note_M2,note_M5,    note_M3,note_M3,note_M2,note_M3,

	
};

// 音符时长数组
const int hetangyeushe_Durations[] = {
	250,500,250,500,500,500,250,250,500, 500,
	
	250,500,250,500 ,250,250,250,250,250,500,
};

const int hetangyeushe_Durations_0[] = {
	100,100,100,100,100,100,100,100,100,500,
	
	100,100,100,100 ,0,0,0,0,0,100,
};

const int hetangyeushe_music[][3]= {

// {note_M1, 250, 50},   
// {note_M1, 500, 50},   
// {note_L6, 250, 50},   
// {note_L5, 500, 50},   
// {note_L6, 500, 50},   
// {note_M1, 500, 50},   
// {note_M1, 250, 50},   
// {note_M2, 250, 50},   
// {note_M3, 500, 0},   
// {note_REST, 500, 50}, 

// 
// 
// 
// {note_M2, 250, 50},   
// {note_M2, 500, 50},   
// {note_M1, 250, 50},   
// {note_M2, 500, 50},   
// {note_M2, 250, 0},   
// {note_M5, 250, 0},   
// {note_M5, 250, 0},    
// {note_M3, 250, 0},     
// {note_M3, 250, 0},     
// {note_M2, 250, 50},     
// {note_M3, 600, 0} ,
// {note_REST, 500, 50}, 

// 
// 
// //弹一首小河
// {note_M1, 250, 50},   
// {note_M1, 500, 50},   
// {note_L6, 250, 50},   
// {note_L5, 500, 50},   
// {note_M5, 500, 50},  
// 
// {note_M3, 250, 0},   
// {note_M2, 250, 0},   
// {note_M3, 250, 0},   
// {note_M2, 250, 0},   
// {note_M1, 500, 0},   
// {note_REST, 500, 50}, 
// 
// 
// //美丽的琴声
// {note_M2, 250, 50},   
// {note_M2, 500, 50},   
// {note_M1, 250, 50},   
// {note_M2, 250, 50},   
// {note_M2, 500, 50},   
// {note_M3, 250, 50},   
// {note_M2, 250, 0},    
// {note_M1, 250, 0},     
// {note_L6, 250, 0},     
// {note_M2, 250, 50},     
// {note_M1, 600, 0} ,
// {note_REST, 500, 50}, 

// 
// 
//  //萤火虫点亮
// {note_M1, 250, 50},   
// {note_M1, 500, 50},   
// {note_L6, 250, 50},   
// {note_L5, 500, 50},   
// {note_L6, 500, 50},  
// 
// {note_M1, 250, 50},   
// {note_M1, 500, 50},   
// {note_M2, 250, 50},   
// {note_M3, 500, 0},   
// {note_REST, 500, 50}, 
// 
// 
// //谁为我添
// {note_M2, 250, 50},   
// {note_M2, 500, 50},   
// {note_M1, 250, 50},   
// {note_M2, 500, 50},   
// {note_M2, 250, 0},   
// {note_M5, 250, 0},   
// {note_M5, 250, 0},    
// {note_M3, 250, 0},     
// {note_M3, 250, 0},     
// {note_M2, 250, 50},     
// {note_M3, 600, 0} ,
// {note_REST, 500, 50}, 

 
//  //挪开那
// {note_M1, 250, 0},   
// {note_M1, 250, 0},   
// {note_M1, 250, 0},   
// {note_L6, 250, 0},   
// {note_L5, 500, 50},   
// {note_M5, 500, 50},  
// 
// {note_M3, 250, 0},   
// {note_M2, 250, 0},   
// {note_M3, 250, 0},   
// {note_M2, 250, 0},   
// {note_M1, 500, 0},   
// {note_REST, 500, 50}, 

// 
//  //谁采下
// {note_M2, 250, 50},   
// {note_M2, 500, 50},   
// {note_M1, 250, 50},   
// {note_M2, 250, 50},   
// {note_M2, 500, 50},   
// {note_M3, 250, 50},   
// {note_M2, 250, 0},    
// {note_M1, 250, 0},     
// {note_L6, 250, 0},     
// {note_M2, 250, 50},     
// {note_M1, 600, 0} ,
// {note_REST, 500, 50}, 


  //我像只鱼儿
 {note_M3, 250, 50},   
 {note_M5, 500, 50},   
 {note_M5, 250, 50},   
 {note_M5, 500, 50},   
 {note_M5, 500, 50},  
 
 {note_M6, 250, 0},   
 {note_M5, 250, 0},    
 {note_M3, 250, 0},     
 {note_M2, 250, 50},     
 {note_M1, 500, 50},     
 {note_REST, 500, 50}, 
 
 
 
	//只为和你
 {note_M6, 250, 0},   
 {note_H1, 250, 50},   
 {note_M6, 250, 0},   
 {note_M5, 250, 50},   
 {note_M3, 250, 0},  
 {note_M2, 250, 50},   
 {note_M1, 250, 0},    
 {note_L6, 250, 50},   
 
 {note_M2, 500, 50},     
 {note_M2, 250, 0},     
 {note_M3, 250, 50},     
 {note_M3, 250, 0},     
 {note_M2, 500, 50},     
 {note_REST, 500, 50}, 
 
 
   //游过了四
 {note_M3, 250, 50},   
 {note_M5, 500, 50},   
 {note_M5, 250, 50},   
 {note_M5, 500, 50},   
 {note_M5, 500, 50},  
 
 {note_M6, 250, 0},   
 {note_M5, 250, 0},    
 {note_M3, 250, 0},     
 {note_M2, 250, 50},     
 {note_M1, 500, 50},     
 {note_REST, 500, 50}, 


 
 	//等你玩在水中
 {note_L6, 250, 0},   
 {note_M1, 250, 50},   
 {note_L6, 250, 0},   
 {note_L5, 250, 50}, 
 
 {note_M2, 500, 50},  
 {note_M3, 250, 0},   
 {note_M2, 250, 50},    
 {note_M1, 1000, 50},   
 {note_REST, 500, 50}, 

 }; 

 /*
 {note_M1, 250, 50},   
 {note_M1, 500, 50},   
 {note_L6, 250, 50},   
 {note_L5, 500, 50},   
 {note_L6, 500, 50},   
 {note_M1, 500, 50},   
 {note_M1, 250, 50},   
 {note_M2, 250, 50},   
 {note_M3, 500, 0},   
 {note_REST, 500, 50}, 

 
 
 
 {note_M2, 250, 50},   
 {note_M2, 500, 50},   
 {note_M1, 250, 50},   
 {note_M2, 500, 50},   
 {note_M2, 250, 0},   
 {note_M5, 250, 0},   
 {note_M5, 250, 0},    
 {note_M3, 250, 0},     
 {note_M3, 250, 0},     
 {note_M2, 250, 50},     
 {note_M3, 600, 0} ,
 {note_REST, 500, 50}, 

 
 
 //弹一首小河
 {note_M1, 250, 50},   
 {note_M1, 500, 50},   
 {note_L6, 250, 50},   
 {note_L5, 500, 50},   
 {note_M5, 500, 50},  
 
 {note_M3, 250, 0},   
 {note_M2, 250, 0},   
 {note_M3, 250, 0},   
 {note_M2, 250, 0},   
 {note_M1, 500, 0},   
 {note_REST, 500, 50}, 
 
 
 //美丽的琴声
 {note_M2, 250, 50},   
 {note_M2, 500, 50},   
 {note_M1, 250, 50},   
 {note_M2, 250, 50},   
 {note_M2, 500, 50},   
 {note_M3, 250, 50},   
 {note_M2, 250, 0},    
 {note_M1, 250, 0},     
 {note_L6, 250, 0},     
 {note_M2, 250, 50},     
 {note_M1, 600, 0} ,
 {note_REST, 500, 50}, 

 
 
  //萤火虫点亮
 {note_M1, 250, 50},   
 {note_M1, 500, 50},   
 {note_L6, 250, 50},   
 {note_L5, 500, 50},   
 {note_L6, 500, 50},  
 
 {note_M1, 250, 50},   
 {note_M1, 500, 50},   
 {note_M2, 250, 50},   
 {note_M3, 500, 0},   
 {note_REST, 500, 50}, 
 
 
 //谁为我添
 {note_M2, 250, 50},   
 {note_M2, 500, 50},   
 {note_M1, 250, 50},   
 {note_M2, 500, 50},   
 {note_M2, 250, 0},   
 {note_M5, 250, 0},   
 {note_M5, 250, 0},    
 {note_M3, 250, 0},     
 {note_M3, 250, 0},     
 {note_M2, 250, 50},     
 {note_M3, 600, 0} ,
 {note_REST, 500, 50}, 

 
  //挪开那
 {note_M1, 250, 0},   
 {note_M1, 250, 0},   
 {note_M1, 250, 0},   
 {note_L6, 250, 0},   
 {note_L5, 500, 50},   
 {note_M5, 500, 50},  
 
 {note_M3, 250, 0},   
 {note_M2, 250, 0},   
 {note_M3, 250, 0},   
 {note_M2, 250, 0},   
 {note_M1, 500, 0},   
 {note_REST, 500, 50}, 

 
  //谁采下
 {note_M2, 250, 50},   
 {note_M2, 500, 50},   
 {note_M1, 250, 50},   
 {note_M2, 250, 50},   
 {note_M2, 500, 50},   
 {note_M3, 250, 50},   
 {note_M2, 250, 0},    
 {note_M1, 250, 0},     
 {note_L6, 250, 0},     
 {note_M2, 250, 50},     
 {note_M1, 600, 0} ,
 {note_REST, 500, 50}, 


  //我像只鱼儿
 {note_M3, 250, 50},   
 {note_M5, 500, 50},   
 {note_M5, 250, 50},   
 {note_M5, 500, 50},   
 {note_M5, 500, 50},  
 
 {note_M6, 250, 0},   
 {note_M5, 250, 0},    
 {note_M3, 250, 0},     
 {note_M2, 250, 50},     
 {note_M1, 500, 50},     
 {note_REST, 500, 50}, 
 
 
 
	//只为和你
 {note_M6, 250, 0},   
 {note_H1, 250, 50},   
 {note_M6, 250, 0},   
 {note_M5, 250, 50},   
 {note_M3, 250, 0},  
 {note_M2, 250, 50},   
 {note_M1, 250, 0},    
 {note_L6, 250, 50},   
 
 {note_M2, 500, 50},     
 {note_M2, 250, 0},     
 {note_M3, 250, 50},     
 {note_M3, 250, 0},     
 {note_M2, 500, 50},     
 {note_REST, 500, 50}, 

 */