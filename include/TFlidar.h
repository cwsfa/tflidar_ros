#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string>

namespace benewake
{
  class TFlidar
  {
    public:
      TFlidar(std::string _name, int _baudRate);
      ~TFlidar(){};
      float getDist();
      void closePort();

      unsigned char dataBuf[7];

    private:
      std::string portName_;
      std::string model_;
      int baudRate_;
      int serial_;

      bool readData(unsigned char *_buf, int _nRead);
  };
}
