#include <iostream>
#include <unistd.h>


#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <stdio.h>

#include <signal.h>
#include <stdio.h>
void sigint(int a)
{
    printf("^C caught\n");
}

  


int main()
{
    pid_t pid;

    pid = fork();
    if (pid == 0)
    { // child process
        setpgid(getpid(), getpid());
        system("roslaunch flir_one_node flir_data_record.launch");
    }
    else
    { // parent process
        // sleep(10);
        unsigned int microsecond = 1000000;
        usleep(5 * microsecond); //sleeps for 3 second
        printf("Sleep returned\n");
        kill(-pid, SIGKILL);
        printf("killed process group %d\n", pid);
    }
    exit(0);
}

// int main()
// {
//     system("gnome-terminal -- sh -c 'cd ../../../..; rosbag play flir_data_2021-04-24-23-26-17.bag -l;exec bash'");

//     unsigned int microsecond = 1000000;
//     usleep(5 * microsecond); //sleeps for 3 second
//     system("gnome-terminal -- sh -c 'exit'");
// }


