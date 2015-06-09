
import os
import signal
import subprocess


def process_get_children(node):
    ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % int(node), shell=True, stdout=subprocess.PIPE)
    ps_output = ps_command.stdout.read()
    retcode = ps_command.wait()
    return ps_output.split("\n")[:-1]
    

def process_get_children_recursively(node, leafs):
    if node is not None:
        children = process_get_children(node)
        if len(children) == 0:
            leafs.append(node)
        for n in children:
            process_get_children_recursively(n, leafs)

            
def terminate_process_and_children(p):
    plist = []
    try:
        pnum = p.pid
    except AttributeError:
        pnum = p
    process_get_children_recursively(pnum, plist)
    for pid_str in plist:
        os.kill(int(pid_str), signal.SIGINT)
    try:
        p.kill()
    except AttributeError:
        os.kill(int(p), signal.SIGINT)
    return
