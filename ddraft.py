import time
class hele:
    def __init__(self):
        self.var = 0
        self.print = 0
        self.x = 0

    def while_loop(self):
        while(self.var == 0):
            print(self.x)
            if self.print == 1:
                self.x = 1
                self.print = 0
    
    def stop_loop(self):
        self.var = 1

    def print_other(self):
        self.print = 1

if __name__ == "__main__":
    hel = hele()
    hel.while_loop()

    hel.print_other()