#include <iostream>
#include<bits/stdc++.h>
using namespace std;

    int dir = 1;
    int X = 0;
    int Y = 1;
    int x = 0;
    int y = 0;


int main(){
    int rightDistance = RightDistance();
    int leftDistance = LeftDistance();
    int frontDistance = FrontDistance();
    vector<vector<char>> Mapa;
    int MAX_WAL_DIST = 15;
    int DistBetweenBlock = 23;

    //getNext(1);
    /*
            0

    1              -1
            
            2
    
    */

    bool rightFree = rightDistance > MAX_WAL_DIST ;
    bool leftFree = leftDistance > MAX_WAL_DIST;
    bool frontFree = frontDistance > MAX_WAL_DIST;

/*
    Modificar el GoFront();
    Interrumpible cuando variable ON o creando una nueva
    if(DetectLine()){
        GoBack();
    }
    GoBack(){
        Objective = rightPulses;
        rightPulses = 0;
        while(rightPulses < Objective){
            Drive(-0.5, -0.5);
        }
        Mark('N');
        CoordMinus();

        
    }
*/
    

    if((rightFree && leftFree) || (rightFree && frontFree) || (leftFree&&frontFree)){
        Mark('I');
    }
    if(Mapa[x][y] == 'I'){
        
        if(rightFree && verNextBlock(-1)){
            turn('r');
            dir -= 1;
        }else if(leftFree && verNextBlock(1)){
            turn('l')
            dir +=1;
        }else if(frontFree && verNextBlock(0)){
    
        }else{
            Mark('N');
            UTurn();
            dir +=2;
        }
        Drive(0,0);
        delay(500);
        CoordPlus();
        Mark('P')
        GoFront(DistBetweenBlock);
    }else if(frontFree){
        CoordPlus();
        Mark('P')
        GoFront(DistBetweenBlock);
    }else if(rightFree){
        Turn('r');
        dir -=1;
    }else if(leftFree){
        Turn('l');
        dir +=1;
    }
    Drive(0,0);
    delay(500);
    
}

void Mark(char state){
    if (Mapa[x][y] == 'P' && state == 'P'){
        state = 'N';
    }
    Mapa[x][y] = state;
}

void CoordPlus(){
    setVectorAngle(0);
    x +=X;
    y += Y;
}

void CoordMinus(){
    setVectorAngle(0);
    x +=X;
    y += Y;
}

bool verNextBlock(int side){
    setVectorAngle(side);
    return Mapa[x+X][y+Y] != 'N';
}

void setVectorAngle(int plus){
    X = cos((float)(dir+plus)*PI/2);
    Y = sin((float)(dir+plus) * PI/2);
}

void Simple(){
    int rightDistance = RightDistance();
    int leftDistance = LeftDistance();
    int frontDistance = FrontDistance();
    int MAX_WAL_DIST = 15;
    int MAX_FRONT_DIST = 8;
    int DistBetweenBlock = 23;
    if(rightDistance < MAX_WAL_DIST && frontDistance > MAX_FRONT_DIST){
        FollowWall('r', rightDistance);
    }
    else if(rightDistance > MAX_WAL_DIST){
        turn('r');
        GoFront(DistBetweenBlock);
    }else if (leftDistance > MAX_WAL_DIST){
        turn('l');
        GoFront(DistBetweenBlock);
    }else{
        UTurn();
    }
    
}