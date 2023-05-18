#include <stdlib.h>

//This code opens the html page in a web browser. (tested on firefox)

int main(int argc, char** argv)
{
    system("xdg-open ~/catkin_ws/src/remote_autobot/www/remote_control_autobot.html");
    return 0;
}
