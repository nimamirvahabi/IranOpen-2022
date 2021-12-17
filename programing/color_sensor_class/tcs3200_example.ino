#include <tcs3200.h>


int red, green, blue, white;

int limit_YELLOW[3] ={17,10,32};
int limit_GREEN[3] ={5,6,19};
#define S0_1 4
#define S1_1 5
#define S2_1 6
#define S3_1 7
#define output_1 8

#define S0_2 8
#define S1_2 9
#define S2_2 10
#define S3_2 11
#define output_2 12

tcs3200 tcs(S0_1, S1_1, S2_1, S3_1, output_1); 
tcs3200 tcs2(S0_2, S1_2, S2_2, S3_2, output_2);
class color_sen{
  public:
    int get_value_R(int mode){
      if (mode==1){
        return tcs.colorRead('r');   
      }
      else{
        return tcs2.colorRead('r');
      }
      
      
    }
    int get_value_B(int mode){
      if (mode==1){
        return tcs.colorRead('b');   
      }
      else{
        return tcs2.colorRead('b');
      }
    }
    int get_value_G(int mode){
      if (mode==1){
        return tcs.colorRead('g');   
      }
      else{
        return tcs2.colorRead('g');
      }
    }
    int get_value_C(int mode){
      if (mode==1){
        return tcs.colorRead('c');   
      }
      else{
        return tcs2.colorRead('c');
      }
    }
    char color(int mode){
      if (mode==1){
        if(( tcs.colorRead('r') <= limit_YELLOW[0]+2 &&  tcs.colorRead('r') >= limit_YELLOW[0]-2)&&
         ( tcs.colorRead('g') <= limit_YELLOW[1]+2 &&  tcs.colorRead('g') >= limit_YELLOW[1]-2)&&
         ( tcs.colorRead('b') <= limit_YELLOW[2]+2 &&  tcs.colorRead('b') >= limit_YELLOW[2]-2)){
            return 'y';
         }
        else if(( tcs.colorRead('r') <= limit_GREEN[0]+2 &&  tcs.colorRead('r') >= limit_GREEN[0]-2)&&
              ( tcs.colorRead('g') <= limit_GREEN[1]+2 &&  tcs.colorRead('g') >= limit_GREEN[1]-2)&&
              ( tcs.colorRead('b') <= limit_GREEN[2]+2 &&  tcs.colorRead('b') >= limit_GREEN[2]-2)){
            return 'g';
         } 
         else{
          return '!';
        } 
      }
      else{
        if(( tcs2.colorRead('r') <= limit_YELLOW[0]+2 &&  tcs2.colorRead('r') >= limit_YELLOW[0]-2)&&
         ( tcs2.colorRead('g') <= limit_YELLOW[1]+2 &&  tcs2.colorRead('g') >= limit_YELLOW[1]-2)&&
         ( tcs2.colorRead('b') <= limit_YELLOW[2]+2 &&  tcs2.colorRead('b') >= limit_YELLOW[2]-2)){
            return 'y';
         }
        else if(( tcs2.colorRead('r') <= limit_GREEN[0]+2 &&  tcs2.colorRead('r') >= limit_GREEN[0]-2)&&
              ( tcs2.colorRead('g') <= limit_GREEN[1]+2 &&  tcs2.colorRead('g') >= limit_GREEN[1]-2)&&
              ( tcs2.colorRead('b') <= limit_GREEN[2]+2 &&  tcs2.colorRead('b') >= limit_GREEN[2]-2)){
            return 'g';
        }
        else{
          return '!';
        }
      }
      
    } 
};
void setup() {
  Serial.begin(9600);
}

void loop() {
  color_sen bot;
  Serial.println(bot.color(1));
  
}
