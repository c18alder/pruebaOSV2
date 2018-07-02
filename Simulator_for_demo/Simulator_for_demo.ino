#include <stdint.h>
#define SEND_DATA 1
#define NOT_SEND_DATA 2


int con_status = NOT_SEND_DATA;
uint8_t incomingData[4];
int line_sent_counter=0;
int battery_level=100;
bool battery_charging=false;
int aux_km=0;
int aux_speed=0;
uint8_t aux[15];
unsigned long previous_sent=0;
static bool flag_send_speed=false;
unsigned long previous_time=0;

void setup() {
   // Serial.begin(38400);
   Serial.begin(9600);
    while (!Serial) {
      ;
    }
}
void loop(){

    while (Serial.available() > 0) {   //read the commands sent
      Serial.readBytes(incomingData, 64);  //cont = 
    }


    


    if (incomingData[0] == '0' && incomingData[1] == '1'){  // && cont == 2) {   // read a start
       incomingData[0] = 0;
       incomingData[1] = 0;
       con_status = SEND_DATA;
     } else if (incomingData[0] == '0' && incomingData[1] == '0' ){  //&& cont == 2) {  //read a stop
       incomingData[0] = 0;
       incomingData[1] = 0;
       con_status = NOT_SEND_DATA;
     }




       // //// TO SIMULATE RESPONSE TO SPEED REQUEST 
       
//     } else if(incomingData[0] == '3' && incomingData[1] == '3'){
//     
//        incomingData[0] = 0;
//       incomingData[1] = 0;
//         flag_send_speed=true; 
//     }


//         if(flag_send_speed){
//                 aux[0]='5';
//                 aux[1]='4';
//                 if(!battery_charging){
//                   aux_speed=aux_speed + (random(-35,35));
//                   if(aux_speed>170){
//                      aux_speed= 100;
//                  }else if(aux_speed< 10 && aux_speed != 0){
//                      aux_speed=50;
//                  }
//                   aux[2]=(aux_speed/100)+'0';                                  //stupid way to get the number jee.
//                   aux[3]=( (aux_speed-((aux_speed/100)*100) ) / 10 )+'0';
//                   aux[4]= (aux_speed%((aux_speed/100)*100) ) -  ( ( (aux_speed%((aux_speed/100)*100) ) /10 )*10 ) +'0';
//        
//                 }else if(battery_charging){
//                   aux[2]='0';
//                  aux[3]='0';
//                  aux[4]='0';
//        
//                 }
//        
//                 aux[5]='\n';
//                Serial.write(aux,6);     
//                  delay(50);
//                flag_send_speed=false;
//        }

     if (con_status == SEND_DATA) {
       line_sent_counter++;
      if(line_sent_counter>=1 && line_sent_counter<=24){ //prepare cell voltages
              aux[0]=(line_sent_counter/10) + '0';
              aux[1]=( line_sent_counter-((line_sent_counter/10)*10)  ) + '0';
              aux[2]='6';          //always  3. ish
              aux[3]=random(48, 58);
              aux[4]=random(48, 58);
      }else{
          switch(line_sent_counter){
              case 25: case 26: case 27: case 28:   // prepare temperatures
                  aux[0]='7';
                    aux[1]='4';
                    aux[2]=line_sent_counter - 25 + '0';
                    aux[3]=random(50, 52);     //always between 20 and 40;
                    aux[4]=random(48, 58);
                    break;
              case 29:         // prepare current (amperes ) TODO
                    aux[0]='8';
                    aux[1]='4';
                    aux[2]=random(48, 58);
                    aux[3]=random(48, 58);
                    aux[4]=random(48, 58);
                    break;
              case 30:   //PREPARE battery state of charge
                    aux[0]='9';
                    aux[1]='4';


                    if (battery_charging){

                      battery_level=battery_level+ 6 ;   //random(30, 50);
                      if ( battery_level >= 100){
                      aux[2]='1';
                      aux[3]='0';
                      aux[4]='0';
                      battery_level=100;
                      battery_charging=false;
                      }else{
                      aux[2]='0';
                        aux[3]=   (battery_level / 10) + '0';
                        aux[4]=  battery_level - ((battery_level / 10) * 10) + '0';
                      }
                    }else if( !battery_charging){

                      battery_level=battery_level-10;
                      if(battery_level <= 0){
                          aux[2]='0';
                          aux[3]='0';
                          aux[4]='0';
                          battery_level=0;
                          battery_charging=true;
                      }else{
                      aux[2]='0';
                      aux[3]=(battery_level / 10) + '0';
                      aux[4]=battery_level - ((battery_level / 10) * 10) + '0';
                      }

                    }else{ }
                    break;

              case 31:   //PWM TO BATTERY CHArger prepare
                          aux[0]='9';
                          aux[1]='4';
                          aux[2]='3';
                          if(battery_charging){
                            aux[3]='9';
                            aux[4]='5';
                          }else if(!battery_charging){
                            aux[3]='1';
                            aux[4]='5';
                          }
                          break;
              case 32:   //PWM TO Engine controller prepare
                    aux[0]='9';
                    aux[1]='9';
                    aux[2]='0';
                    aux[3]=random(48, 58);
                    aux[4]=random(48, 58);
                    break;


              default:
                if(line_sent_counter>32 && line_sent_counter<=65){   //prepare uselees ( for now ) data.
                      aux[0]='5';
                      aux[1]='5';
                      aux[2]='5';
                      aux[3]=random(48, 58);
                      aux[4]=random(48, 58);
                }else{
                  line_sent_counter=0;
                }



          }  //end switch
      }//end else

      aux[5]='\n';
      Serial.write(aux,6);
      delay(50);
     }//end if send data




     //DIFFERENT WAY OF SIMULATING SPEED. NOT COMPATIBLE WITH BMS REQ
//unsigned long current_time=millis();
//         //PREPARE SPEED TO SEND ( SIMULATED FROM BMS )
//    if(con_status != SEND_DATA && (current_time > (previous_time + 1500) ) ){
//                delay ( 50); 
//    
//    
//                     aux[0]='5';
//                     aux[1]='4';
//                     if(!battery_charging){
//                       aux_speed=aux_speed + (random(-35,35));
//                       if(aux_speed>170){
//                          aux_speed= 100;
//                      }else if(aux_speed< 10 && aux_speed != 0){
//                          aux_speed=50;
//                      }
//                       aux[2]=(aux_speed/100)+'0';                                  //stupid way to get the number jee.
//                       aux[3]=( (aux_speed-((aux_speed/100)*100) ) / 10 )+'0';
//                       aux[4]= (aux_speed%((aux_speed/100)*100) ) -  ( ( (aux_speed%((aux_speed/100)*100) ) /10 )*10 ) +'0';
//            
//                     }else if(battery_charging){
//                       aux[2]='0';
//                      aux[3]='0';
//                      aux[4]='0';
//            
//                     }
//            
//                     aux[5]='\n';
//                    Serial.write(aux,6);     
//                      delay(50);
//              previous_time=millis();
//    
//    
//    
//      
//    }


     
}

