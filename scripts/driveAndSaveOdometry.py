from sys import stdin
import curses


class Gunnar(object):
    
    def __init__(self):
        self._spds = [0, 0]
        self.robotSpeedsStr = ''
    
    def forward(self, inc=4):
        self.incSpds(inc, inc)
        
    def reverse(self, inc=4):
        self.incSpds(-inc, -inc)
        
    def left(self, inc=4):
        self.incSpds(-inc, inc) 
        
    def right(self, inc=4):
        self.incSpds(inc, -inc)
        
    def incSpds(self, a, b):
        self.spds = [
            self.spds[0] + a,
            self.spds[1] + b,
            ]
    
    def stop(self):
        self.spds = [0, 0]
    
    @property
    def spds(self):
        return list(self._spds)
        
    @spds.setter
    def spds(self, twoList):
        self._spds = twoList
        self.cmdSetSpeeds(twoList[0], twoList[1])

    def cmdSetSpeeds(self, a, b):
        self.robotSpeedsStr = "(%.1f, %.1f)." % (a, b)
        
    def checkForKeyboardInput(self):
        from select import select
        from sys import stdin
        if select([stdin], [], [], 0)[0]:  # stdin has a line to read.
            line = stdin.readline().strip().lower()
            #if line in cmds:
            print 'Got the command (hex %s).' % ' '.join(
                [
                '0x%03x' % ord(c) for c in line
                ])


class Controller(object):
        
    def __init__(self):
        self.stdscr = curses.initscr()
        curses.cbreak()
        self.stdscr.keypad(1)
        self.stdscr.addstr(0, 10, 'Hit "q" to quit.')
        self.stdscr.refresh()
        self.gunnar = Gunnar()
        for s in self.textLocations:
            self.updateText(s)
        
    textLocations = {
        'Up': [2, 20],
        'Left': [3, 10],
        'Right': [3, 30],
        'Down': [4, 20],
        'Space': [3, 20],
        'speeds': [10, 0],
        }
    
    def updateText(self, s):
        if s in self.textLocations:
            r, c = self.textLocations[s]
            if s == 'speeds':
                text = 'Speeds: ' + self.gunnar.robotSpeedsStr
            else:
                text = s
            self.stdscr.addstr(r, c, text)
        
    def run(self):
        key = ''
        while key != ord('q'):
            try:
                key = self.stdscr.getch()
                self.stdscr.refresh()
                if key == curses.KEY_UP:
                    self.gunnar.forward()
                    self.updateText('Up')
                elif key == curses.KEY_LEFT: 
                    self.gunnar.left()
                    self.updateText('Left')
                elif key == curses.KEY_RIGHT: 
                    self.gunnar.right()
                    self.updateText('Right')
                elif key == curses.KEY_DOWN: 
                    self.gunnar.reverse()
                    self.updateText('Down')
                elif key == ord(' '):
                    self.gunnar.stop()
                    self.updateText('Space')
                self.updateText('speeds');
            except KeyboardInterrupt:
                break
        self.gunnar.stop()
        curses.endwin()

            
if __name__ == "__main__":
    controller = Controller()
    controller.run()
    