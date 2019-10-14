#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>

#define DEVICE "/dev/ttyUSB0"
#define input
#ifndef input
  #define output
#endif
int
open_port(void)
{
  int fd; 
  fd = open(DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1)
  {
    perror("open_port: Unable to open /dev/ttyS0 - ");
    return -1;
  }
  else
    fcntl(fd, F_SETFL, 0);
  return (fd);
}
void set_output(int fd,int speed)
{
  struct termios opt;
  tcgetattr(fd,&opt);
  cfsetospeed(&opt,speed);
  opt.c_cflag     |= (CLOCAL | CREAD);
  opt.c_lflag     &= ~(ICANON | ECHO | ECHOE | ISIG);
  opt.c_oflag     &= ~OPOST;
  opt.c_cc[VMIN]  = 0;
  opt.c_cc[VTIME] = 10;
  tcsetattr(fd, TCSANOW,&opt);
}
void set_input(int fd, int speed)
{
  struct termios opt;
  tcgetattr(fd,&opt);
  cfsetospeed(&opt,speed);
  opt.c_cflag &= ~CSIZE; 
  opt.c_cflag |= CS8;
  opt.c_cflag &= ~PARENB; 
  opt.c_iflag &= ~INPCK;
  opt.c_cflag &= ~CSTOPB; 
  opt.c_cc[VTIME] = 150; 
  opt.c_cc[VMIN] = 0;
  tcsetattr(fd,TCSANOW,&opt);

}
int
main()
{
  int fd = open_port();
  #ifdef output
    set_output(fd,B19200);
  #endif
  #ifdef input
    set_input(fd,B19200);
  #endif
  std::string str;
  
  while(1)
  {
    #ifdef output;
      std::cout<<">> ";
      std::getline(std::cin,str);
      str+='\n';
      const char*op=str.c_str();
      std::cout<<str.size()<<" "<<str<<std::endl;
      write(fd,op,str.size());
    #endif

    #ifdef input
      char buf[256];
      read(fd,buf,256);
      //tcflush(fd,TCIOFLUSH);
      printf("%s\n",buf);
    #endif
    sleep(1);
  }
  close(fd);
}
