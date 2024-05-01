
import sys
sys.path.append('Utilities')
from find_trans import find_trans_main, output

def main():
    find_trans_main()
    print(f"Testing output function")
    goal, helpers =output()

    print(f"Output function Detected goal point at x: {goal.x}, z: {goal.z}")
    for id_, helper in helpers.items():
        print(f"Output function Detected helper {id_} point at x: {helper.x}, z: {helper.z}")

if __name__ == '__main__':
    main()
