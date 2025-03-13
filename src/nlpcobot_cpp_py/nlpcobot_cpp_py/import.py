#!/usr/bin/env python3
class MyClass:
    def __init__(self, x=1.0, y=1.0):
        self.x = x
        self.y = y
    
    def area(self):
        print(f"Area= {self.x * self.y}")

# create object of Room class
myclass = MyClass()

# assign values to all the properties 
myclass.x = 3.0
myclass.y = 4.0

# access method inside class
myclass.area()