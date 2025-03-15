import time
import Board
import ActionGroupControl as AGC


# action groups are saved under the path /home/pi/ArmPi/ActionGroups
def run_action_group(group_number):
    try: 
        while True:
            AGC.runAction(group_number) 
    except KeyboardInterrupt:
        print("Stopped.")
        
        
if __name__ == "__main__":
    group_number = input("Action to run:")
    run_action_group(group_number)