import os
import subprocess
import time
import signal

def start_ros ():
    # ros_start = subprocess.run("roscore")
    start_ros = os.system("roscore")


def terminate(self):
    # process.terminate()
   os.kill(self.p.pid, signal.CTRL_C_EVENT)


def main():
    # test = start_ros()
    # print("waiting start")
    # time.sleep(3)
    # print("Killing Now")
    # terminate(test)
    # print("\nTerminated")

    proc = subprocess.Popen(
        ['roslaunch flir_one_node flir_data_vis.launch'], stdout=subprocess.PIPE,
        shell=True, preexec_fn=os.setsid)


    time.sleep(20)  # <-- sleep for 12''
    # # proc.send_signal(signal.SIGKILL)
    # # proc.kill()
    # proc.terminate()  # <-- terminate the process
    # os.system("killall -9 master")
    # Send the signal to all the process groups
    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)

    time.sleep(20)
   

    proc = subprocess.Popen(
    ['roslaunch flir_one_node flir_data_vis.launch'], stdout=subprocess.PIPE,
    shell=True, preexec_fn=os.setsid)

    time.sleep(20)  # <-- sleep for 12''
# # proc.send_signal(signal.SIGKILL)
# # proc.kill()
# proc.terminate()  # <-- terminate the process
# os.system("killall -9 master")
# Send the signal to all the process groups
    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    # terminate_all = os.system('killall -9 rosmaster')
    # proc = subprocess.run(["roslaunch", "flir_one_node", "flir_data_vis.launch"])
    # time.sleep(20)
    # proc.kill() 
  
    

if __name__=="__main__":
    main()



