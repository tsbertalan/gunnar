from subprocess import Popen, PIPE

baudRate = 9600


def systemOut(cmdList, sayCmd=True, giveStatus=False):
    '''Run a command and capture the output.'''
    if sayCmd:
        print "cmdList is", cmdList
        print "$", " ".join(cmdList)
    process = Popen(cmdList, stdout=PIPE)
    if giveStatus:
        return process.communicate()[0], process.returncode
    return process.communicate()[0]

