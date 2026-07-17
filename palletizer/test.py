import subprocess

x = 0.0
y = 0.2175

cmd = f'''
ssh er@192.168.1.10 "
python3 /home/er/git/rcll/palletizer/test.py picking {x} {y}
"
'''

subprocess.run(cmd, shell=True)
