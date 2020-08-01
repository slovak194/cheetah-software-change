/*!
 * @file main.cpp
 * @brief Main Function for the WBC Controller
 * @brief WBC控制器的主函数
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

//#define PRINTF_TO_FILE

#include <main_helper.h>
#include "stdio.h"
#include "time.h"
#include "MIT_Controller.hpp"
#include "WebotsBridge.hpp"
// main函数解析命令行参数并启动相应的驱动程序。
int main(int argc, char** argv) {
    
  #ifdef PRINTF_TO_FILE
    time_t t = time(0); 
    char tmp[64]; 
    strftime( tmp, sizeof(tmp), "%Y-%m-%d-%H-%M-runlog.txt",localtime(&t) ); 
    FILE* fp = freopen(tmp, "w" ,stdout);  //将printf输出重定向到log.txt文件中
  #endif
  printf("run mit_ctrl, argv: %c\n",argv[1][0]);
  
  main_helper(argc, argv, new MIT_Controller());

  #ifdef PRINTF_TO_FILE
    fclose(fp);
  #endif

  return 0;
}
