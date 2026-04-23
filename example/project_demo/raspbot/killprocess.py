import subprocess

def kill_process():
        try:
            # 使用pgrep查找脚本
            result = subprocess.run(['pgrep', '-f', '/home/pi/project_demo/raspbot/raspbot.pyc'], capture_output=True, text=True, check=True)
            pids = result.stdout.strip().split('\n')
            # 遍历找到的所有PID
            for pid in pids:
                try:
                    # 终止进程
                    subprocess.run(['kill', pid], check=True)
                    print(f"Process {pid} has been terminated.")
                except subprocess.CalledProcessError:
                    print(f"Failed to terminate process {pid}.")
        except subprocess.CalledProcessError:
            print("No matching processes found.")

if __name__ == "__main__":
    kill_process()
