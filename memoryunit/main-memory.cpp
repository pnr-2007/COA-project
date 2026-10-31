#pragma once

#include <vector>
#include <cstdint>

class MainMemory {
    public:
       explicit MainMemory(uint64_t accesscycles=20,size_t bytes):
        size_(bytes),latencycycles_(accesscycles)
       
       uint64_t readword(uint64_t physaddr){
          return memorya[physaddr>>3];
       }

       void writeword(uint64_t physaddr,uint64_t){
         memory_[physaddr >> 3]= data;
       }

        uint32_t latency() const{
             return latencycycles
        }       
       
        std::vector<uint64_t>& raw storage{
            return memorya;
        }
    

    private:
       size_t size_;
       uint32_t latencycycles_;
       std:: vector<uint64_t> memorya;
};
