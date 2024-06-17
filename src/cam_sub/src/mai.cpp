#include "iostream"
#include <stdio.h>
#include "reserved_mem.hpp"
#include "ximage_processing.h"

#define BUFFER_LENGTH 100000

int main(){
    Reserved_Mem res_mem=Reserved_Mem();
    int buffer[BUFFER_LENGTH];
    for(int i=0;i<BUFFER_LENGTH;i++){
        buffer[i]=1;
    }
    res_mem.transfer<int>(buffer,BUFFER_LENGTH,BUFFER_LENGTH);
    for(int i=0;i<BUFFER_LENGTH;i++){
        buffer[i]=i;
    }
    res_mem.transfer<int>(buffer,0x0,BUFFER_LENGTH);


    XImage_processing xip;
    int status=XImage_processing_Initialize(&xip,"image_processing");
    if (status!= XST_SUCCESS){
        std::cout << "Cannot Init IP" << std::endl;
        exit(1);
    }

    while(!XImage_processing_IsReady(&xip)){}

    XImage_processing_Set_in_r(&xip,0x70000000);
    XImage_processing_Set_out_r(&xip,0x70000000+BUFFER_LENGTH*sizeof(int));
    XImage_processing_Start(&xip);

    while(!XImage_processing_IsDone(&xip)){}



    res_mem.gather<int>(buffer,BUFFER_LENGTH,BUFFER_LENGTH);
    for(int i=0;i<1000;i++){
        std::cout << std::to_string(i) << ": "<<std::to_string(buffer[i]) << std::endl;
    }
    std::cout << "I'd say it works" << std::endl;
    return 0;
}