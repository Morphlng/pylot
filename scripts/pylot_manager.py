"""
    This file is used along with macad_gym to manage the Pylot docker container
"""
import argparse
import os
import signal
import subprocess
import time

import redis


class PylotManager:
    '''
    This class is used to manage the Pylot docker container

    When macad env_config contains "use_redis": True, we will use this class to manage the Pylot docker container
    '''

    def __init__(self, args):
        self.ue_host = args.ue_host
        self.ue_port = args.ue_port
        self.redis_host = args.redis_host
        self.redis_port = args.redis_port
        self.map = args.map

        self.conn = redis.Redis(host=self.redis_host,
                                port=self.redis_port, decode_responses=True)

        START_PYLOT_BASE_COMMAND = ["python3", "/home/erdos/workspace/pylot/pylot.py",
                                    "--flagfile=/home/erdos/workspace/pylot/configs/scenarios/person_avoidance_frenet.conf", 
                                    f"--simulator_host={self.ue_host}", f"--simulator_port={self.ue_port}"]

        # Map goal matching
        goal_map = {
            "Town01": "--goal_location=-2.128490,317.256927,0.556630",
            "Town03": "--goal_location=217.043533,62.721680,0.773467",
            "Town05": "--goal_location=-3.752541,-84.496841,0.546758",
            "Town11": "--goal_location=-238.007019,-4065.660645,174.704025",
            "Afghani": "--goal_location=521.621337890625,496.4139404296875,-0.051280900835990906"
            # "Afghani": "--goal_location=811.485168,6.489622,0.006774"
        }

        self.START_PYLOT_COMMAND = START_PYLOT_BASE_COMMAND + [goal_map[self.map]]

        # The docker container of pylot should name "pylot"
        self.CHECK_PYLOT_ALIVE = "ps -ef | grep pylot.py | grep -v grep"
        self.KILL_PYLOT = "ps -ef | grep /home/erdos/workspace/pylot/configs/ | awk \'{print $2}\' | xargs -r kill -9"
        self.TIMEOUT = 60

    def run(self):
        pylot_process = None
        while True:
            try:
                if self.conn.get('reset') == 'yes':
                    print('get reset yes')
                    self.conn.set('reset', "no")
                    print('set reset no')

                    # Kill previous pylot process
                    print("kill pylot")
                    if pylot_process is not None and pylot_process.poll() is None:
                        pylot_process.terminate()

                    subprocess.run(self.KILL_PYLOT, shell=True,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    self.conn.set('START_EGO', '0')
                    print("pylot killed")

                    # Start new pylot process
                    time.sleep(1)
                    print('start pylot')
                    pylot_process = subprocess.Popen(self.START_PYLOT_COMMAND,
                                                     shell=False, preexec_fn=os.setsid)
                    print('pylot started')

                if pylot_process is not None:
                    ret_code = pylot_process.poll()
                    if ret_code in [-2, 0, 130]:
                        raise KeyboardInterrupt
                    elif ret_code is not None:
                        raise RuntimeError(
                            "Pylot process exited with code {}".format(ret_code))

                time.sleep(0.1)
            except KeyboardInterrupt:
                print("KeyboardInterrupt received, terminating the script...")
                subprocess.run(self.KILL_PYLOT, shell=True,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                if pylot_process is not None and pylot_process.poll() is None:
                    os.killpg(os.getpgid(pylot_process.pid), signal.SIGTERM)
                    pylot_process.terminate()
                    pylot_process.wait(timeout=5)
                break
            except Exception:
                pylot_process = None

        self.conn.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--ue_host', type=str, default='172.17.0.1')
    parser.add_argument('--ue_port', type=int, default=2000)
    parser.add_argument('--redis_host', type=str, default='172.17.0.1')
    parser.add_argument('--redis_port', type=int, default=6379)
    parser.add_argument('--map', type=str, default='Town01')
    parser.add_argument('--flagfile', type=str, default='/home/erdos/workspace/pylot/configs/scenarios/person_avoidance_frenet.conf')
    args = parser.parse_args()

    pylot_manager = PylotManager(args)
    pylot_manager.run()
