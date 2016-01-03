'''
Created on Dec 29, 2015

@author: tsbertalan
'''
class VirtualClassError(NotImplementedError):
    
    def __init__(self, obj):
        text = "Class %s is virtual." % type(obj).__name__
        super(VirtualClassError, self).__init__(text)


def print_function(*args):
    for arg in args:
        print arg,
    print
