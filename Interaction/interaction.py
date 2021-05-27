import os
import subprocess
import time
import signal



   
def main():

    visual = 'roslaunch flir_one_node flir_data_vis.launch'
    record = 'roslaunch flir_one_node flir_data_record.launch'

############## Starts process######
    proc = subprocess.Popen(
        [record], stdout=subprocess.PIPE,
        shell=True, preexec_fn=os.setsid)


######################
    time.sleep(20)  # <-- sleep for 12''



###################terminates##########################
    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
   


  

if __name__ == "__main__":
    main()
