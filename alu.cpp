#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <typeinfo>

#include <cstdint>
#include <cmath>
#include <stdexcept>

using namespace std;

class ALU{

    private:
        struct ALUFlags {
            bool zero=false;
            bool sign=false;
            bool overflow=false;
            bool catrry=false;
            bool sizematch=true;
        }

    public:
        ALU();
        ~ALU();

        // ALU operations
        int add(int a, int b){
            return a+b;
        };


        int sub(int a, int b){
            return a-b;
        };


        int mul(int a, int b){
            return a*b;
        };


        int div(int a, int b){
            if(b!=0){
                return a/b;
            }
            else{
                zero=true;
                cerr<<"Division by zero error"<<endl;
                return -1;  
            }
        };



        int mod(int a, int b){
            if(b!=0){
                return a%b;
                
            }


            else{
                cerr<<"Division by zero error"<<endl;
                zero=true;
                return -1;  
        };


        int and_op(int a, int b){
            return a & b;
        };


        int or_op(int a, int b){
            return a | b;
        };

        int xor_op(int a, int b){
            return a ^ b;
        };


        int not_op(int a){
            return ~a;
        };


        uint_32_t lsl(int a, int b){
            return a << b;
        };


        uint_32_t lsr(int a, int b){
            return (uint_32_t)a >> b;
        };


        int asr(int a, int b){
            return (uint_32_t)a >> b;
        };


        uint_32_t asl(int a, int b){
            return (uint_32_t)a << b;
        };


        float sin(float a){
            return sin(a);
        };


        float cos(float a){
            return cos(a);
        };


        float log(float a){
            return log(a);
        };


        float exp(float a){
            return exp(a);
        };


        float pow(float a, float b){
            return pow(a, b);
        };
}